/**
 * @file chassis_usb_protocol.c
 * @brief USB stream framing, command dispatch and chassis telemetry.
 */

#include "chassis_usb_protocol.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "battery_config.h"
#include "battery_monitor.h"
#include "chassis_controller.h"
#include "chassis_runtime.h"
#include "chassis_runtime_test.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "fault_manager.h"
#include "imu_static_test.h"
#include "imu_service.h"
#include "imu_config.h"
#include "motor_driver.h"
#include "motor_open_loop_test.h"
#include "task.h"
#include "usb_cdc_transport.h"
#include "watchdog_service.h"
#include "wheel_speed_estimator.h"

#define CHASSIS_USB_SOF_0                    0xA5U
#define CHASSIS_USB_SOF_1                    0x5AU
#define CHASSIS_USB_HEADER_SIZE                 8U
#define CHASSIS_USB_CRC_SIZE                    2U
#define CHASSIS_USB_MAX_PAYLOAD_SIZE           52U
#define CHASSIS_USB_MAX_FRAME_SIZE             62U
#define CHASSIS_USB_HOST_SESSION_TIMEOUT_MS  1200U
#define CHASSIS_USB_VELOCITY_PAYLOAD_SIZE       8U
#define CHASSIS_USB_ACK_PAYLOAD_SIZE            4U
#define CHASSIS_USB_STATUS_PAYLOAD_SIZE         52U
#define CHASSIS_USB_WHEEL_STATE_PAYLOAD_SIZE    52U
#define CHASSIS_USB_BATTERY_STATE_PAYLOAD_SIZE  28U
#define CHASSIS_USB_IMU_STATE_PAYLOAD_SIZE      46U
#define CHASSIS_USB_IMU_RAW_V1_PAYLOAD_SIZE     52U
#define CHASSIS_USB_RX_BYTE_BUDGET            128U
#define CHASSIS_USB_RX_FRAME_BUDGET             2U
/* No new source sample: still report sensor health, but do not manufacture a
 * 100 Hz stream by repeatedly transmitting one cached acquisition. */
#define CHASSIS_USB_IMU_HEALTH_PERIOD_MS       100U

#define CHASSIS_USB_BATTERY_FLAG_VALID              (1U << 0)
#define CHASSIS_USB_BATTERY_FLAG_PROTECTION_ENABLED (1U << 1)
#define CHASSIS_USB_BATTERY_FLAG_LOW                (1U << 2)
#define CHASSIS_USB_BATTERY_FLAG_CRITICAL           (1U << 3)
#define CHASSIS_USB_BATTERY_FLAG_ADC_FAULT          (1U << 4)

#define CHASSIS_USB_IMU_FLAG_VALID                   (1U << 0)
#define CHASSIS_USB_IMU_FLAG_GYRO_BIAS_VALID         (1U << 1)
#define CHASSIS_USB_IMU_FLAG_INITIALIZED             (1U << 2)
#define CHASSIS_USB_IMU_FLAG_CALIBRATION_ACTIVE      (1U << 3)
#define CHASSIS_USB_IMU_FLAG_CALIBRATION_RESULT_VALID (1U << 4)
#define CHASSIS_USB_IMU_FLAG_CALIBRATION_LAST_SUCCESS (1U << 5)
#define CHASSIS_USB_IMU_FLAG_CALIBRATION_LAST_MOVED  (1U << 6)
#define CHASSIS_USB_IMU_FLAG_CALIBRATION_LAST_RANGE  (1U << 7)

#define CHASSIS_USB_FLAG_COMMAND_VALID       (1U << 0)
#define CHASSIS_USB_FLAG_DIAGNOSTICS_ACTIVE  (1U << 1)
#define CHASSIS_USB_FLAG_ESTOP_LATCHED       (1U << 2)
#define CHASSIS_USB_FLAG_ESTOP_INPUT         (1U << 3)
#define CHASSIS_USB_FLAG_USB_CONFIGURED      (1U << 4)
#define CHASSIS_USB_FLAG_CONTROLLER_ACTIVE   (1U << 5)
#define CHASSIS_USB_FLAG_HOST_PORT_OPEN       (1U << 6)
#define CHASSIS_USB_FLAG_HEALTH_FAULT         (1U << 7)
#define CHASSIS_USB_FLAG_IWDG_REFRESH_ACTIVE  (1U << 8)

typedef struct
{
  uint8_t data[CHASSIS_USB_MAX_FRAME_SIZE];
  uint16_t index;
  uint16_t expected_length;
} ChassisUsbParser;

static ChassisUsbParser s_parser;
static ChassisUsbProtocolStats s_stats;
static bool s_link_was_configured;
static bool s_host_session_active;
static ChassisUsbImuStream s_imu_stream;
static uint32_t s_last_host_rx_ms;
static uint32_t s_next_status_ms;
static uint32_t s_next_wheel_state_ms;
static uint32_t s_next_battery_state_ms;
static uint32_t s_next_imu_state_ms;
/* USB task-owned delivery cursor, not an acquisition FIFO or a calibration
 * state. Advance only after TX accepted a coherent snapshot. */
static struct
{
  bool present;
  uint8_t flags;
  ImuServiceState state;
  uint32_t sample_count;
  uint32_t sample_time_ms;
  uint32_t acquisition_epoch;
  uint32_t read_error_count;
  uint32_t consecutive_errors;
  int32_t bias_milli_raw[3];
} s_last_imu_sent;
static uint16_t s_device_sequence;
static uint32_t s_next_tx_diagnostic_ms;
static uint32_t s_reported_tx_full;
static uint32_t s_reported_tx_deferred;
static uint32_t s_reported_log_drops;
static uint32_t s_reported_tx_busy;
static uint32_t s_reported_tx_errors;

static void ChassisUsbProtocol_LogLinkSnapshot(
  const char *event, uint32_t task_time_ms, const UsbCdcLinkSnapshot *link)
{
#if defined(DEBUG) && DEBUG_LOG_ENABLE_DEBUG
  /* Event-only diagnostics. Never log from an ISR or inside the snapshot's
   * critical section. Keep the public STATUS payload and flag semantics intact. */
  DEBUG_LOG_DEBUG("USB SNAP %s task=%lu now=%lu last=%lu age=%lu sof=%lu",
                  event, (unsigned long)task_time_ms,
                  (unsigned long)link->sample_time_ms,
                  (unsigned long)link->last_sof_ms,
                  (unsigned long)link->sof_age_ms,
                  (unsigned long)link->sof_count);
  DEBUG_LOG_DEBUG("USB SNAP cfg=%u mon=%u seen=%u alive=%u ctrl=%u dtr=%u open=%u events=%lu/%lu/%lu",
                  (unsigned)link->configured,
                  (unsigned)link->sof_monitor_enabled, (unsigned)link->sof_seen,
                  (unsigned)link->link_alive,
                  (unsigned)link->control_line_state_seen,
                  (unsigned)link->dtr_asserted, (unsigned)link->host_port_open,
                  (unsigned long)link->configurations,
                  (unsigned long)link->deconfigurations,
                  (unsigned long)link->control_line_events);
#else
  (void)event;
  (void)task_time_ms;
  (void)link;
#endif
}

static bool ChassisUsbProtocol_TimeReached(uint32_t now_ms,
                                           uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static uint16_t ChassisUsbProtocol_GetU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int32_t ChassisUsbProtocol_GetI32(const uint8_t *data)
{
  const uint32_t value = (uint32_t)data[0] |
    ((uint32_t)data[1] << 8U) |
    ((uint32_t)data[2] << 16U) |
    ((uint32_t)data[3] << 24U);
  return (int32_t)value;
}

static void ChassisUsbProtocol_PutU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void ChassisUsbProtocol_PutU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  data[2] = (uint8_t)((value >> 16U) & 0xFFU);
  data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void ChassisUsbProtocol_PutU64(uint8_t *data, uint64_t value)
{
  for (uint32_t byte = 0U; byte < 8U; ++byte)
  {
    data[byte] = (uint8_t)(value >> (byte * 8U));
  }
}

static int32_t ChassisUsbProtocol_Round(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t ChassisUsbProtocol_SaturateI16(int32_t value)
{
  if (value > 32767)
  {
    return 32767;
  }
  if (value < -32768)
  {
    return -32768;
  }
  return (int16_t)value;
}

static uint16_t ChassisUsbProtocol_Crc16(const uint8_t *data,
                                         uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U) ?
        (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static bool ChassisUsbProtocol_AreWheelsStopped(void)
{
  WheelSpeedState speeds[ENCODER_WHEEL_COUNT] = { 0 };

  if (!WheelSpeedEstimator_GetSnapshot(speeds))
  {
    return false;
  }
  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    if ((!speeds[wheel].valid) || (!speeds[wheel].stopped))
    {
      return false;
    }
  }
  return true;
}

static bool ChassisUsbProtocol_CanStartImuCalibration(void)
{
  ChassisRuntimeStatus runtime = { 0 };
  ImuServiceSnapshot imu = { 0 };
  MotorDriverStatus motor = { 0 };

  if ((!ChassisRuntime_GetStatus(&runtime)) ||
      (!ImuService_GetSnapshot(&imu)) ||
      (!MotorDriver_GetStatus(&motor)))
  {
    return false;
  }
  if ((runtime.state != CHASSIS_RUNTIME_STATE_IDLE) ||
      runtime.command_valid || runtime.diagnostics_active ||
      ChassisController_IsActive() ||
      (!EmergencyStop_CanEnableMotor()) ||
      FaultManager_HasCriticalFault() ||
      MotorOpenLoopTest_IsActive() ||
      ChassisRuntimeTest_IsActive() ||
      ImuStaticTest_IsActive())
  {
    return false;
  }
  if ((!motor.initialized) || motor.standby_enabled)
  {
    return false;
  }
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (motor.command_permille[wheel] != 0)
    {
      return false;
    }
  }
  return imu.initialized && imu.valid &&
         (imu.state == IMU_SERVICE_STATE_READY) &&
         ChassisUsbProtocol_AreWheelsStopped();
}

static bool ChassisUsbProtocol_SendFrame(uint8_t message_type,
                                         uint16_t sequence,
                                         const uint8_t *payload,
                                         uint16_t payload_length)
{
  uint8_t frame[CHASSIS_USB_MAX_FRAME_SIZE];
  uint16_t crc;
  uint16_t frame_length;

  if ((payload_length > CHASSIS_USB_MAX_PAYLOAD_SIZE) ||
      ((payload_length > 0U) && (payload == NULL)))
  {
    return false;
  }

  frame[0] = CHASSIS_USB_SOF_0;
  frame[1] = CHASSIS_USB_SOF_1;
  frame[2] = CHASSIS_USB_PROTOCOL_VERSION;
  frame[3] = message_type;
  ChassisUsbProtocol_PutU16(&frame[4], sequence);
  ChassisUsbProtocol_PutU16(&frame[6], payload_length);
  if (payload_length > 0U)
  {
    (void)memcpy(&frame[8], payload, payload_length);
  }
  crc = ChassisUsbProtocol_Crc16(&frame[2],
                                  (uint16_t)(6U + payload_length));
  ChassisUsbProtocol_PutU16(&frame[8U + payload_length], crc);
  frame_length = (uint16_t)(CHASSIS_USB_HEADER_SIZE + payload_length +
                            CHASSIS_USB_CRC_SIZE);
  if ((message_type == CHASSIS_USB_MSG_ACK) ||
      (message_type == CHASSIS_USB_MSG_STATUS))
  {
    return UsbCdcTransport_QueueControlTransmit(frame, frame_length);
  }
  /* Backpressure is not a discarded frame: leave its deadline due, regenerate
   * a fresh snapshot later and do not consume a device sequence number. */
  if (!UsbCdcTransport_CanQueueTelemetry())
  {
    UsbCdcTransport_RecordTelemetryDeferred();
    return false;
  }
  return UsbCdcTransport_QueueTransmit(frame, frame_length);
}

static void ChassisUsbProtocol_SendAck(uint8_t request_type,
                                       uint16_t sequence,
                                       ChassisUsbResult result)
{
  ChassisRuntimeStatus runtime = { 0 };
  uint8_t payload[CHASSIS_USB_ACK_PAYLOAD_SIZE];

  (void)ChassisRuntime_GetStatus(&runtime);
  payload[0] = request_type;
  payload[1] = (uint8_t)result;
  payload[2] = (uint8_t)runtime.state;
  payload[3] = (uint8_t)runtime.fault;
  (void)ChassisUsbProtocol_SendFrame(CHASSIS_USB_MSG_ACK,
                                     sequence,
                                     payload,
                                     sizeof(payload));
}

static bool ChassisUsbProtocol_SendStatus(uint32_t now_ms)
{
  ChassisRuntimeStatus runtime = { 0 };
  WheelSpeedState speeds[ENCODER_WHEEL_COUNT] = { 0 };
  UsbCdcTransportStats transport = { 0 };
  WatchdogServiceStatus watchdog = { 0 };
  UsbCdcLinkSnapshot link = { 0 };
  uint8_t payload[CHASSIS_USB_STATUS_PAYLOAD_SIZE] = { 0 };
  uint16_t flags = 0U;

  if ((!ChassisRuntime_GetStatus(&runtime)) ||
      (!WheelSpeedEstimator_GetSnapshot(speeds)))
  {
    return false;
  }
  (void)UsbCdcTransport_GetStats(&transport);
  (void)WatchdogService_GetStatus(&watchdog);
  (void)UsbCdcTransport_GetLinkSnapshot(&link);

  if (runtime.command_valid)
  {
    flags |= CHASSIS_USB_FLAG_COMMAND_VALID;
  }
  if (runtime.diagnostics_active)
  {
    flags |= CHASSIS_USB_FLAG_DIAGNOSTICS_ACTIVE;
  }
  if (EmergencyStop_IsLatched())
  {
    flags |= CHASSIS_USB_FLAG_ESTOP_LATCHED;
  }
  if (EmergencyStop_IsInputActive())
  {
    flags |= CHASSIS_USB_FLAG_ESTOP_INPUT;
  }
  if (link.link_alive)
  {
    flags |= CHASSIS_USB_FLAG_USB_CONFIGURED;
  }
  if (ChassisController_IsActive())
  {
    flags |= CHASSIS_USB_FLAG_CONTROLLER_ACTIVE;
  }
  if (link.host_port_open)
  {
    flags |= CHASSIS_USB_FLAG_HOST_PORT_OPEN;
  }
  if (!link.link_alive)
  {
    ChassisUsbProtocol_LogLinkSnapshot("STATUS-not-alive", now_ms, &link);
  }
  if (FaultManager_HasCriticalFault())
  {
    flags |= CHASSIS_USB_FLAG_HEALTH_FAULT;
  }
  if (watchdog.armed && watchdog.health_eligible &&
      (!watchdog.refresh_failed))
  {
    flags |= CHASSIS_USB_FLAG_IWDG_REFRESH_ACTIVE;
  }

  ChassisUsbProtocol_PutU32(&payload[0], now_ms);
  ChassisUsbProtocol_PutU32(&payload[4], runtime.command_sequence);
  ChassisUsbProtocol_PutU32(&payload[8], runtime.accepted_commands);
  ChassisUsbProtocol_PutU32(&payload[12], runtime.rejected_commands);
  ChassisUsbProtocol_PutU16(&payload[16], s_stats.last_rx_sequence);
  payload[18] = (uint8_t)runtime.state;
  payload[19] = (uint8_t)runtime.fault;
  payload[20] = (uint8_t)runtime.last_stop_reason;
  payload[21] = (uint8_t)runtime.controller_fault;
  ChassisUsbProtocol_PutU16(&payload[22], flags);
  ChassisUsbProtocol_PutU16(
    &payload[24],
    (uint16_t)ChassisUsbProtocol_SaturateI16(
      ChassisUsbProtocol_Round(runtime.requested_command.linear_x_m_s *
                               1000.0f)));
  ChassisUsbProtocol_PutU16(
    &payload[26],
    (uint16_t)ChassisUsbProtocol_SaturateI16(
      ChassisUsbProtocol_Round(runtime.requested_command.angular_z_rad_s *
                               1000.0f)));
  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    const int16_t speed_mm_s = ChassisUsbProtocol_SaturateI16(
      ChassisUsbProtocol_Round(speeds[wheel].wheel_linear_velocity_m_s *
                               1000.0f));
    ChassisUsbProtocol_PutU16(&payload[28U + (wheel * 2U)],
                              (uint16_t)speed_mm_s);
  }
  ChassisUsbProtocol_PutU32(&payload[36], s_stats.valid_frames);
  ChassisUsbProtocol_PutU32(&payload[40], s_stats.crc_errors);
  ChassisUsbProtocol_PutU32(&payload[44], transport.rx_overflow_packets);
  ChassisUsbProtocol_PutU32(&payload[48], transport.tx_dropped_frames);

  const bool queued = ChassisUsbProtocol_SendFrame(CHASSIS_USB_MSG_STATUS,
                                     s_device_sequence, payload, sizeof(payload));
  if (queued) { ++s_device_sequence; }
  return queued;
}

static bool ChassisUsbProtocol_SendWheelState(void)
{
  WheelSpeedState speeds[ENCODER_WHEEL_COUNT] = { 0 };
  uint8_t payload[CHASSIS_USB_WHEEL_STATE_PAYLOAD_SIZE] = { 0 };

  if (!WheelSpeedEstimator_GetSnapshot(speeds))
  {
    return false;
  }

  ChassisUsbProtocol_PutU32(&payload[0], speeds[0].sample_time_ms);
  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    ChassisUsbProtocol_PutU64(
      &payload[4U + (wheel * 8U)],
      (uint64_t)speeds[wheel].accumulated_count);
    ChassisUsbProtocol_PutU32(
      &payload[36U + (wheel * 4U)],
      (uint32_t)ChassisUsbProtocol_Round(
        speeds[wheel].filtered_counts_per_second));
  }

  const bool queued = ChassisUsbProtocol_SendFrame(CHASSIS_USB_MSG_WHEEL_STATE,
                                     s_device_sequence, payload, sizeof(payload));
  if (queued) { ++s_device_sequence; }
  return queued;
}

static bool ChassisUsbProtocol_SendBatteryState(void)
{
  BatteryMonitorSnapshot battery = { 0 };
  uint8_t payload[CHASSIS_USB_BATTERY_STATE_PAYLOAD_SIZE] = { 0 };
  uint8_t flags = 0U;

  if (!BatteryMonitor_GetSnapshot(&battery))
  {
    return false;
  }

  if (battery.valid)
  {
    flags |= CHASSIS_USB_BATTERY_FLAG_VALID;
  }
  if (battery.protection_enabled)
  {
    flags |= CHASSIS_USB_BATTERY_FLAG_PROTECTION_ENABLED;
  }
  if (battery.state == BATTERY_MONITOR_STATE_LOW)
  {
    flags |= CHASSIS_USB_BATTERY_FLAG_LOW;
  }
  if (battery.state == BATTERY_MONITOR_STATE_CRITICAL)
  {
    flags |= CHASSIS_USB_BATTERY_FLAG_CRITICAL;
  }
  if (battery.state == BATTERY_MONITOR_STATE_ADC_FAULT)
  {
    flags |= CHASSIS_USB_BATTERY_FLAG_ADC_FAULT;
  }

  ChassisUsbProtocol_PutU32(&payload[0], battery.sample_time_ms);
  ChassisUsbProtocol_PutU32(&payload[4], battery.sample_count);
  ChassisUsbProtocol_PutU32(&payload[8], battery.adc_error_count);
  ChassisUsbProtocol_PutU16(&payload[12], battery.raw_average);
  ChassisUsbProtocol_PutU16(&payload[14], battery.filtered_raw);
  ChassisUsbProtocol_PutU16(&payload[16], battery.adc_input_mv);
  ChassisUsbProtocol_PutU16(&payload[18], battery.supply_mv);
  payload[20] = (uint8_t)battery.state;
  payload[21] = flags;
  ChassisUsbProtocol_PutU16(&payload[22], battery.consecutive_adc_errors);
  ChassisUsbProtocol_PutU16(&payload[24], BATTERY_LOW_THRESHOLD_MV);
  ChassisUsbProtocol_PutU16(&payload[26], BATTERY_CRITICAL_THRESHOLD_MV);

  const bool queued = ChassisUsbProtocol_SendFrame(CHASSIS_USB_MSG_BATTERY_STATE,
                                     s_device_sequence, payload, sizeof(payload));
  if (queued) { ++s_device_sequence; }
  return queued;
}

static void ChassisUsbProtocol_RecordImuSent(const ImuServiceSnapshot *imu,
                                            uint8_t flags, uint32_t now_ms)
{
  s_last_imu_sent.present = true;
  s_last_imu_sent.flags = flags;
  s_last_imu_sent.state = imu->state;
  s_last_imu_sent.sample_count = imu->sample_count;
  s_last_imu_sent.sample_time_ms = imu->sample_time_ms;
  s_last_imu_sent.acquisition_epoch = imu->acquisition_epoch;
  s_last_imu_sent.read_error_count = imu->read_error_count;
  s_last_imu_sent.consecutive_errors = imu->consecutive_errors;
  s_last_imu_sent.bias_milli_raw[0] = imu->gyro_bias_x_milli_raw;
  s_last_imu_sent.bias_milli_raw[1] = imu->gyro_bias_y_milli_raw;
  s_last_imu_sent.bias_milli_raw[2] = imu->gyro_bias_z_milli_raw;
  s_next_imu_state_ms = now_ms + CHASSIS_USB_IMU_HEALTH_PERIOD_MS;
}

static bool ChassisUsbProtocol_SendImuState(uint32_t now_ms, bool force)
{
  ImuServiceSnapshot imu = { 0 };
  ImuStaticTestStatus calibration = { 0 };
  uint8_t payload[CHASSIS_USB_IMU_RAW_V1_PAYLOAD_SIZE] = { 0 };
  uint8_t flags = 0U;

  if (!ImuService_GetSnapshot(&imu))
  {
    return false;
  }
  (void)ImuStaticTest_GetStatus(&calibration);

  if (imu.valid)
  {
    flags |= CHASSIS_USB_IMU_FLAG_VALID;
  }
  if (imu.gyro_bias_valid)
  {
    flags |= CHASSIS_USB_IMU_FLAG_GYRO_BIAS_VALID;
  }
  if (imu.initialized)
  {
    flags |= CHASSIS_USB_IMU_FLAG_INITIALIZED;
  }
  if (calibration.result == IMU_STATIC_RESULT_RUNNING)
  {
    flags |= CHASSIS_USB_IMU_FLAG_CALIBRATION_ACTIVE;
  }
  else if (calibration.result != IMU_STATIC_RESULT_NOT_RUN)
  {
    flags |= CHASSIS_USB_IMU_FLAG_CALIBRATION_RESULT_VALID;
    if (calibration.result == IMU_STATIC_RESULT_PASS)
    {
      flags |= CHASSIS_USB_IMU_FLAG_CALIBRATION_LAST_SUCCESS;
    }
    else if (calibration.result == IMU_STATIC_RESULT_FAIL_MOVED)
    {
      flags |= CHASSIS_USB_IMU_FLAG_CALIBRATION_LAST_MOVED;
    }
    else if (calibration.result == IMU_STATIC_RESULT_FAIL_BIAS_RANGE)
    {
      flags |= CHASSIS_USB_IMU_FLAG_CALIBRATION_LAST_RANGE;
    }
  }

  /* Follow the producer's sample identity, not an independent 10 ms timer.
   * DRDY and USB task clocks are not phase locked. A timed latest-snapshot
   * resend can duplicate N, then skip N+1 when the source edge shifts phase.
   * State/error changes bypass the gate even when sample_count is unchanged.
   * Explicit GET_STATUS/calibration replies may deliberately repeat a sample.
   * This remains latest-only: a task/host stall can still lose acquisitions;
   * never invent/interpolate missing sensor data. */
  if ((!force) && s_last_imu_sent.present &&
      (imu.sample_count == s_last_imu_sent.sample_count) &&
      (imu.sample_time_ms == s_last_imu_sent.sample_time_ms) &&
      (imu.acquisition_epoch == s_last_imu_sent.acquisition_epoch) &&
      (imu.state == s_last_imu_sent.state) &&
      (flags == s_last_imu_sent.flags) &&
      (imu.read_error_count == s_last_imu_sent.read_error_count) &&
      (imu.consecutive_errors == s_last_imu_sent.consecutive_errors) &&
      (imu.gyro_bias_x_milli_raw == s_last_imu_sent.bias_milli_raw[0]) &&
      (imu.gyro_bias_y_milli_raw == s_last_imu_sent.bias_milli_raw[1]) &&
      (imu.gyro_bias_z_milli_raw == s_last_imu_sent.bias_milli_raw[2]) &&
      (!ChassisUsbProtocol_TimeReached(now_ms, s_next_imu_state_ms)))
  {
    return false;
  }

  ChassisUsbProtocol_PutU32(&payload[0], imu.sample_time_ms);
  ChassisUsbProtocol_PutU32(&payload[4], imu.sample_count);
  if ((s_imu_stream == CHASSIS_USB_IMU_STREAM_RAW_V1) ||
      (s_imu_stream == CHASSIS_USB_IMU_STREAM_RAW_V2))
  {
    const bool precise = (s_imu_stream == CHASSIS_USB_IMU_STREAM_RAW_V2);
    /* Every raw value and applied baseline comes from this one snapshot.
     * No adaptive correction, calibration or Flash operation occurs here. */
    ChassisUsbProtocol_PutU16(&payload[8], (uint16_t)imu.accel_x_raw);
    ChassisUsbProtocol_PutU16(&payload[10], (uint16_t)imu.accel_y_raw);
    ChassisUsbProtocol_PutU16(&payload[12], (uint16_t)imu.accel_z_raw);
    ChassisUsbProtocol_PutU16(&payload[14], (uint16_t)imu.gyro_x_raw);
    ChassisUsbProtocol_PutU16(&payload[16], (uint16_t)imu.gyro_y_raw);
    ChassisUsbProtocol_PutU16(&payload[18], (uint16_t)imu.gyro_z_raw);
    ChassisUsbProtocol_PutU32(&payload[20],
      imu.gyro_bias_valid ? (uint32_t)(precise ? imu.gyro_bias_x_milli_raw :
                                                imu.gyro_bias_x_raw) : 0U);
    ChassisUsbProtocol_PutU32(&payload[24],
      imu.gyro_bias_valid ? (uint32_t)(precise ? imu.gyro_bias_y_milli_raw :
                                                imu.gyro_bias_y_raw) : 0U);
    ChassisUsbProtocol_PutU32(&payload[28],
      imu.gyro_bias_valid ? (uint32_t)(precise ? imu.gyro_bias_z_milli_raw :
                                                imu.gyro_bias_z_raw) : 0U);
    ChassisUsbProtocol_PutU32(&payload[32],
      (uint32_t)ChassisUsbProtocol_Round(imu.temperature_c * 1000.0f));
    ChassisUsbProtocol_PutU32(&payload[36], imu.read_error_count);
    ChassisUsbProtocol_PutU32(&payload[40], imu.consecutive_errors);
    ChassisUsbProtocol_PutU16(&payload[44],
      (uint16_t)(IMU_GYRO_LSB_PER_DEGREE_PER_SECOND * 10.0f + 0.5f));
    ChassisUsbProtocol_PutU16(&payload[46], (uint16_t)IMU_ACCEL_LSB_PER_G);
    payload[48] = (uint8_t)imu.state;
    payload[49] = flags;
    /* Format 1 retains the coarse integer view. Format 2 exposes the exact
     * applied baseline in milli-RAW; all other fields/scales are identical. */
    payload[50] = precise ? 2U : 1U; /* Byte 51 remains reserved zero. */
    const bool queued = ChassisUsbProtocol_SendFrame(
      precise ? CHASSIS_USB_MSG_IMU_RAW_V2 : CHASSIS_USB_MSG_IMU_RAW_V1,
      s_device_sequence, payload, sizeof(payload));
    if (queued)
    {
      ++s_device_sequence;
      ChassisUsbProtocol_RecordImuSent(&imu, flags, now_ms);
    }
    return queued;
  }
  ChassisUsbProtocol_PutU32(
    &payload[8],
    (uint32_t)ChassisUsbProtocol_Round(imu.accel_x_m_s2 * 1000.0f));
  ChassisUsbProtocol_PutU32(
    &payload[12],
    (uint32_t)ChassisUsbProtocol_Round(imu.accel_y_m_s2 * 1000.0f));
  ChassisUsbProtocol_PutU32(
    &payload[16],
    (uint32_t)ChassisUsbProtocol_Round(imu.accel_z_m_s2 * 1000.0f));
  ChassisUsbProtocol_PutU32(
    &payload[20],
    (uint32_t)ChassisUsbProtocol_Round(
      imu.angular_velocity_x_rad_s * 1000.0f));
  ChassisUsbProtocol_PutU32(
    &payload[24],
    (uint32_t)ChassisUsbProtocol_Round(
      imu.angular_velocity_y_rad_s * 1000.0f));
  ChassisUsbProtocol_PutU32(
    &payload[28],
    (uint32_t)ChassisUsbProtocol_Round(
      imu.angular_velocity_z_rad_s * 1000.0f));
  ChassisUsbProtocol_PutU32(
    &payload[32],
    (uint32_t)ChassisUsbProtocol_Round(imu.temperature_c * 1000.0f));
  ChassisUsbProtocol_PutU32(&payload[36], imu.read_error_count);
  ChassisUsbProtocol_PutU32(&payload[40], imu.consecutive_errors);
  payload[44] = (uint8_t)imu.state;
  payload[45] = flags;

  const bool queued = ChassisUsbProtocol_SendFrame(CHASSIS_USB_MSG_IMU_STATE,
    s_device_sequence, payload, CHASSIS_USB_IMU_STATE_PAYLOAD_SIZE);
  if (queued)
  {
    ++s_device_sequence;
    ChassisUsbProtocol_RecordImuSent(&imu, flags, now_ms);
  }
  return queued;
}

static void ChassisUsbProtocol_Dispatch(const uint8_t *frame,
                                        uint16_t payload_length,
                                        uint32_t now_ms)
{
  const uint8_t message_type = frame[3];
  const uint16_t sequence = ChassisUsbProtocol_GetU16(&frame[4]);
  const uint8_t *payload = &frame[8];
  ChassisUsbResult result = CHASSIS_USB_RESULT_OK;

  /* Also cover the first frame after a session gap when no task iteration
   * happened at the expiry instant. This only resets telemetry selection. */
  if ((!s_host_session_active) || ChassisUsbProtocol_TimeReached(
        now_ms, s_last_host_rx_ms + CHASSIS_USB_HOST_SESSION_TIMEOUT_MS))
  {
    s_imu_stream = CHASSIS_USB_IMU_STREAM_LEGACY;
    s_last_imu_sent.present = false;
  }
  s_stats.last_rx_sequence = sequence;
  s_last_host_rx_ms = now_ms;
  s_host_session_active = true;

  switch (message_type)
  {
    case CHASSIS_USB_MSG_CMD_VELOCITY:
      if (payload_length != CHASSIS_USB_VELOCITY_PAYLOAD_SIZE)
      {
        result = CHASSIS_USB_RESULT_BAD_LENGTH;
      }
      else if (ImuStaticTest_IsActive())
      {
        result = CHASSIS_USB_RESULT_STATE_REJECTED;
      }
      else
      {
        const int32_t linear_mm_s = ChassisUsbProtocol_GetI32(&payload[0]);
        const int32_t angular_mrad_s = ChassisUsbProtocol_GetI32(&payload[4]);

        if ((linear_mm_s < -200) || (linear_mm_s > 200) ||
            (angular_mrad_s < -1500) || (angular_mrad_s > 1500))
        {
          (void)ChassisRuntime_SetVelocity((float)linear_mm_s / 1000.0f,
                                           (float)angular_mrad_s / 1000.0f,
                                           now_ms);
          result = CHASSIS_USB_RESULT_BAD_VALUE;
        }
        else if (!ChassisRuntime_SetVelocity(
                   (float)linear_mm_s / 1000.0f,
                   (float)angular_mrad_s / 1000.0f,
                   now_ms))
        {
          result = CHASSIS_USB_RESULT_STATE_REJECTED;
        }
      }
      ChassisUsbProtocol_SendAck(message_type, sequence, result);
      break;

    case CHASSIS_USB_MSG_CMD_STOP:
      if (payload_length != 0U)
      {
        result = CHASSIS_USB_RESULT_BAD_LENGTH;
      }
      else
      {
        ImuStaticTest_Cancel();
        MotorOpenLoopTest_StopAll(now_ms);
        ChassisRuntime_Stop(now_ms);
        UsbCdcLinkSnapshot link = { 0 };
        (void)UsbCdcTransport_GetLinkSnapshot(&link);
        ChassisUsbProtocol_LogLinkSnapshot("STOP", now_ms, &link);
      }
      ChassisUsbProtocol_SendAck(message_type, sequence, result);
      break;

    case CHASSIS_USB_MSG_CMD_CLEAR_FAULT:
      if (payload_length != 0U)
      {
        result = CHASSIS_USB_RESULT_BAD_LENGTH;
      }
      else if (!ChassisRuntime_ClearFault(now_ms))
      {
        result = CHASSIS_USB_RESULT_STATE_REJECTED;
      }
      ChassisUsbProtocol_SendAck(message_type, sequence, result);
      break;

    case CHASSIS_USB_MSG_CMD_GET_STATUS:
      if (payload_length != 0U)
      {
        result = CHASSIS_USB_RESULT_BAD_LENGTH;
      }
      ChassisUsbProtocol_SendAck(message_type, sequence, result);
      if (result == CHASSIS_USB_RESULT_OK)
      {
        s_next_status_ms = ChassisUsbProtocol_SendStatus(now_ms) ?
          now_ms + CHASSIS_USB_STATUS_PERIOD_MS : now_ms;
        s_next_battery_state_ms = ChassisUsbProtocol_SendBatteryState() ?
          now_ms + CHASSIS_USB_BATTERY_STATE_PERIOD_MS : now_ms;
        (void)ChassisUsbProtocol_SendImuState(now_ms, true);
      }
      break;

    case CHASSIS_USB_MSG_CMD_CALIBRATE_IMU:
      if (payload_length != 0U)
      {
        result = CHASSIS_USB_RESULT_BAD_LENGTH;
      }
      else if (!ChassisUsbProtocol_CanStartImuCalibration())
      {
        result = CHASSIS_USB_RESULT_STATE_REJECTED;
      }
      else if (!ImuStaticTest_Start(now_ms,
                                    false,
                                    IMU_STATIC_SOURCE_USB))
      {
        result = ImuStaticTest_IsActive() ?
          CHASSIS_USB_RESULT_STATE_REJECTED : CHASSIS_USB_RESULT_INTERNAL;
      }
      ChassisUsbProtocol_SendAck(message_type, sequence, result);
      if (result == CHASSIS_USB_RESULT_OK)
      {
        (void)ChassisUsbProtocol_SendImuState(now_ms, true);
      }
      break;

    case CHASSIS_USB_MSG_CMD_SET_IMU_STREAM:
      if (payload_length != 1U)
      {
        result = CHASSIS_USB_RESULT_BAD_LENGTH;
      }
      else if (payload[0] > CHASSIS_USB_IMU_STREAM_RAW_V2)
      {
        result = CHASSIS_USB_RESULT_BAD_VALUE;
      }
      else
      {
        s_imu_stream = (ChassisUsbImuStream)payload[0];
        s_last_imu_sent.present = false;
      }
      /* Telemetry-only: never call runtime/motor/calibration functions here
       * and never refresh the 250 ms motion-command watchdog. */
      ChassisUsbProtocol_SendAck(message_type, sequence, result);
      break;

    default:
      ++s_stats.unsupported_messages;
      ChassisUsbProtocol_SendAck(message_type,
                                 sequence,
                                 CHASSIS_USB_RESULT_UNSUPPORTED);
      break;
  }
}

static void ChassisUsbProtocol_ResetParser(void)
{
  s_parser.index = 0U;
  s_parser.expected_length = 0U;
}

static void ChassisUsbProtocol_ConsumeByte(uint8_t byte, uint32_t now_ms)
{
  uint16_t payload_length;
  uint16_t received_crc;
  uint16_t calculated_crc;

  if (s_parser.index == 0U)
  {
    if (byte == CHASSIS_USB_SOF_0)
    {
      s_parser.data[0] = byte;
      s_parser.index = 1U;
    }
    return;
  }
  if (s_parser.index == 1U)
  {
    if (byte == CHASSIS_USB_SOF_1)
    {
      s_parser.data[1] = byte;
      s_parser.index = 2U;
    }
    else if (byte != CHASSIS_USB_SOF_0)
    {
      ChassisUsbProtocol_ResetParser();
    }
    return;
  }

  s_parser.data[s_parser.index++] = byte;
  if (s_parser.index == CHASSIS_USB_HEADER_SIZE)
  {
    payload_length = ChassisUsbProtocol_GetU16(&s_parser.data[6]);
    if (payload_length > CHASSIS_USB_MAX_PAYLOAD_SIZE)
    {
      ++s_stats.length_errors;
      ChassisUsbProtocol_ResetParser();
      return;
    }
    s_parser.expected_length = (uint16_t)(CHASSIS_USB_HEADER_SIZE +
      payload_length + CHASSIS_USB_CRC_SIZE);
  }

  if ((s_parser.expected_length == 0U) ||
      (s_parser.index < s_parser.expected_length))
  {
    return;
  }

  payload_length = ChassisUsbProtocol_GetU16(&s_parser.data[6]);
  received_crc = ChassisUsbProtocol_GetU16(
    &s_parser.data[CHASSIS_USB_HEADER_SIZE + payload_length]);
  calculated_crc = ChassisUsbProtocol_Crc16(
    &s_parser.data[2], (uint16_t)(6U + payload_length));
  if (received_crc != calculated_crc)
  {
    ++s_stats.crc_errors;
  }
  else if (s_parser.data[2] != CHASSIS_USB_PROTOCOL_VERSION)
  {
    ++s_stats.version_errors;
    ChassisUsbProtocol_SendAck(s_parser.data[3],
                               ChassisUsbProtocol_GetU16(&s_parser.data[4]),
                               CHASSIS_USB_RESULT_UNSUPPORTED);
  }
  else
  {
    ++s_stats.valid_frames;
    ChassisUsbProtocol_Dispatch(s_parser.data, payload_length, now_ms);
  }
  ChassisUsbProtocol_ResetParser();
}

void ChassisUsbProtocol_Init(uint32_t now_ms)
{
  (void)memset(&s_stats, 0, sizeof(s_stats));
  ChassisUsbProtocol_ResetParser();
  s_link_was_configured = false;
  s_host_session_active = false;
  s_imu_stream = CHASSIS_USB_IMU_STREAM_LEGACY;
  s_last_host_rx_ms = now_ms;
  s_next_status_ms = now_ms + CHASSIS_USB_STATUS_PERIOD_MS;
  s_next_wheel_state_ms = now_ms + CHASSIS_USB_WHEEL_STATE_PERIOD_MS;
  s_next_battery_state_ms =
    now_ms + CHASSIS_USB_BATTERY_STATE_PERIOD_MS;
  s_next_imu_state_ms = now_ms + CHASSIS_USB_IMU_HEALTH_PERIOD_MS;
  (void)memset(&s_last_imu_sent, 0, sizeof(s_last_imu_sent));
  s_device_sequence = 0U;
  s_next_tx_diagnostic_ms = now_ms + 1000U;
  s_reported_tx_full = 0U;
  s_reported_tx_deferred = 0U;
  s_reported_log_drops = 0U;
  s_reported_tx_busy = 0U;
  s_reported_tx_errors = 0U;
#if BATTERY_MONITOR_ENABLED
  DEBUG_LOG_INFO("USB CDC protocol v1 ready: CRC16, heartbeat 250 ms, wheel 20 Hz, battery 2 Hz, IMU 100 Hz, DTR sessions");
#else
  DEBUG_LOG_INFO("USB CDC protocol v1 ready: CRC16, heartbeat 250 ms, wheel 20 Hz, battery deferred, IMU 100 Hz, DTR sessions");
#endif
  DEBUG_LOG_INFO("IMU stream: SI default; CMD 0x06: 1=coarse RAW v1/0x85, 2=precise RAW v2/0x86");
}

static void ChassisUsbProtocol_ProcessInternal(uint32_t now_ms)
{
  UsbCdcLinkSnapshot link = { 0 };
  const uint32_t task_time_ms = now_ms;
  (void)UsbCdcTransport_GetLinkSnapshot(&link);
  const bool configured = link.link_alive;
  uint8_t byte;

  /* Scheduling and session checks start with current task-context time, not
   * the possibly old value captured before this task was preempted. */
  now_ms = link.sample_time_ms;
  if (configured != s_link_was_configured)
  {
    s_link_was_configured = configured;
    ChassisUsbProtocol_ResetParser();
    s_host_session_active = false;
    s_imu_stream = CHASSIS_USB_IMU_STREAM_LEGACY;
    s_last_imu_sent.present = false;
    if (configured)
    {
      ChassisUsbProtocol_LogLinkSnapshot("LINK-configured", task_time_ms, &link);
      DEBUG_LOG_INFO("USB CDC protocol link configured");
    }
    else
    {
      (void)ImuStaticTest_CancelIfSource(IMU_STATIC_SOURCE_USB);
      UsbCdcTransport_HandleLinkTimeout();
      if (ChassisRuntime_IsActive())
      {
        ChassisRuntime_StopForTransportLoss(now_ms);
      }
      ChassisUsbProtocol_LogLinkSnapshot("LINK-lost", task_time_ms, &link);
      DEBUG_LOG_WARN("USB CDC protocol link lost; runtime stopped");
    }
  }

  if (!configured)
  {
    return;
  }

  if (!link.host_port_open)
  {
    const bool session_was_active = s_host_session_active;

    s_host_session_active = false;
    s_imu_stream = CHASSIS_USB_IMU_STREAM_LEGACY;
    s_last_imu_sent.present = false;
    (void)ImuStaticTest_CancelIfSource(IMU_STATIC_SOURCE_USB);
    ChassisUsbProtocol_ResetParser();
    UsbCdcTransport_HandleHostPortClosed();
    if (ChassisRuntime_IsActive())
    {
      ChassisRuntime_StopForTransportLoss(now_ms);
    }
    if (session_was_active)
    {
      ChassisUsbProtocol_LogLinkSnapshot("DTR-closed", task_time_ms, &link);
      DEBUG_LOG_INFO("USB CDC host port closed; session ended");
    }
    return;
  }

  UsbCdcTransport_ProcessTransmit();
  const uint32_t before_frames = s_stats.valid_frames + s_stats.crc_errors +
                                s_stats.version_errors + s_stats.length_errors;
  uint32_t bytes_processed = 0U;
  uint32_t last_frames = before_frames;
  while ((bytes_processed < CHASSIS_USB_RX_BYTE_BUDGET) &&
         UsbCdcTransport_ReadByte(&byte))
  {
    ++bytes_processed;
    ChassisUsbProtocol_ConsumeByte(byte, now_ms);
    const uint32_t frames = s_stats.valid_frames + s_stats.crc_errors +
                            s_stats.version_errors + s_stats.length_errors;
    if (frames != last_frames)
    {
      UsbCdcTransport_ProcessTransmit();
      last_frames = frames;
    }
    if ((uint32_t)(frames - before_frames) >= CHASSIS_USB_RX_FRAME_BUDGET)
    { break; }
  }

  if (s_host_session_active &&
      ChassisUsbProtocol_TimeReached(
        now_ms, s_last_host_rx_ms + CHASSIS_USB_HOST_SESSION_TIMEOUT_MS))
  {
    s_host_session_active = false;
    s_imu_stream = CHASSIS_USB_IMU_STREAM_LEGACY;
    s_last_imu_sent.present = false;
    (void)ImuStaticTest_CancelIfSource(IMU_STATIC_SOURCE_USB);
  }
  if (s_host_session_active &&
      ChassisUsbProtocol_TimeReached(now_ms, s_next_status_ms))
  {
    if (ChassisUsbProtocol_SendStatus(now_ms))
    { s_next_status_ms = now_ms + CHASSIS_USB_STATUS_PERIOD_MS; }
  }
  if (s_host_session_active &&
      ChassisUsbProtocol_TimeReached(now_ms, s_next_wheel_state_ms))
  {
    if (ChassisUsbProtocol_SendWheelState())
    { s_next_wheel_state_ms = now_ms + CHASSIS_USB_WHEEL_STATE_PERIOD_MS; }
  }
  if (s_host_session_active &&
      ChassisUsbProtocol_TimeReached(now_ms, s_next_battery_state_ms))
  {
    if (ChassisUsbProtocol_SendBatteryState())
    { s_next_battery_state_ms = now_ms + CHASSIS_USB_BATTERY_STATE_PERIOD_MS; }
  }
  if (s_host_session_active)
  {
    (void)ChassisUsbProtocol_SendImuState(now_ms, false);
  }
  UsbCdcTransport_ProcessTransmit();
  if (ChassisUsbProtocol_TimeReached(now_ms, s_next_tx_diagnostic_ms))
  {
    UsbCdcTransportStats tx;
    (void)UsbCdcTransport_GetStats(&tx);
    const uint32_t log_drops = DebugLog_GetDeferredDroppedCount();
    if ((tx.tx_full_events != s_reported_tx_full) ||
        (tx.tx_telemetry_deferred != s_reported_tx_deferred) ||
        (tx.tx_busy_returns != s_reported_tx_busy) ||
        (tx.tx_submit_errors != s_reported_tx_errors) ||
        (log_drops != s_reported_log_drops))
    {
      DEBUG_LOG_WARN("USB TX peak=%lu retry_peak=%lu full=%lu drop=%lu critical_drop=%lu last_type=0x%02lX deferred=%lu busy=%lu submit_error=%lu log_drop=%lu",
        (unsigned long)tx.tx_queue_peak, (unsigned long)tx.tx_control_backlog_peak,
        (unsigned long)tx.tx_full_events, (unsigned long)tx.tx_dropped_frames,
        (unsigned long)tx.tx_control_drops, (unsigned long)tx.tx_last_dropped_type,
        (unsigned long)tx.tx_telemetry_deferred, (unsigned long)tx.tx_busy_returns,
        (unsigned long)tx.tx_submit_errors,
        (unsigned long)log_drops);
      s_reported_tx_full = tx.tx_full_events;
      s_reported_tx_deferred = tx.tx_telemetry_deferred;
      s_reported_log_drops = log_drops;
      s_reported_tx_busy = tx.tx_busy_returns;
      s_reported_tx_errors = tx.tx_submit_errors;
    }
    s_next_tx_diagnostic_ms = now_ms + 1000U;
  }
}

void ChassisUsbProtocol_Process(uint32_t now_ms)
{
  const bool deferred = DebugLog_BeginDeferred();
  ChassisUsbProtocol_ProcessInternal(now_ms);
  if (deferred) { DebugLog_EndDeferred(); }
}

bool ChassisUsbProtocol_GetStats(ChassisUsbProtocolStats *stats)
{
  if (stats == NULL)
  {
    return false;
  }
  taskENTER_CRITICAL();
  *stats = s_stats;
  taskEXIT_CRITICAL();
  return true;
}
