/* Production parser/encoder/session + production transport; other services
 * mocked. No COM ports, Flash writes or physical motor operations. */
#include <limits.h>
#define main ExistingStopRegressionMain
#define CDC_Transmit_FS FixtureCDC_Transmit_FS
#include "test_usb_protocol_stop.c"
#undef main
#undef CDC_Transmit_FS

static uint8_t last_raw[52], last_legacy[46], ack_request, ack_result;
static uint8_t last_precise[52];
static unsigned precise_frames;
static unsigned raw_frames, legacy_frames, acks;
static uint16_t last_raw_sequence;

uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
  if (data[3] == CHASSIS_USB_MSG_IMU_RAW_V2)
  {
    assert(length == 62U);
    memcpy(last_precise, &data[8], sizeof(last_precise));
    ++precise_frames;
  }
  if (data[3] == CHASSIS_USB_MSG_IMU_RAW_V1)
  {
    assert(length == 62U);
    memcpy(last_raw, &data[8], sizeof(last_raw));
    last_raw_sequence = ChassisUsbProtocol_GetU16(&data[4]);
    ++raw_frames;
  }
  if (data[3] == CHASSIS_USB_MSG_IMU_STATE)
  {
    assert(length == 56U);
    memcpy(last_legacy, &data[8], sizeof(last_legacy));
    ++legacy_frames;
  }
  if (data[3] == CHASSIS_USB_MSG_ACK)
  {
    ++acks; ack_request = data[8]; ack_result = data[9];
  }
  return FixtureCDC_Transmit_FS(data, length);
}

static void SetupRawTest(void)
{
  test_tick = 1000U;
  host_sequence = 0U;
  raw_frames = legacy_frames = acks = 0U;
  precise_frames = 0U;
  transport_stops = stop_commands = 0U;
  test_full_telemetry = test_custom_imu = true;
  memset(&test_imu, 0, sizeof(test_imu));
  memset(&test_imu_calibration, 0, sizeof(test_imu_calibration));
  test_imu.initialized = test_imu.valid = test_imu.gyro_bias_valid = true;
  test_imu.state = IMU_SERVICE_STATE_READY;
  test_imu.sample_time_ms = 0x12345678U;
  test_imu.sample_count = 0xFEDCBA98U;
  test_imu.accel_x_raw = INT16_MIN;
  test_imu.accel_y_raw = INT16_MAX;
  test_imu.accel_z_raw = 8192;
  test_imu.gyro_x_raw = -57;
  test_imu.gyro_y_raw = 25;
  test_imu.gyro_z_raw = 151;
  test_imu.gyro_bias_x_raw = -57;
  test_imu.gyro_bias_y_raw = 25;
  test_imu.gyro_bias_z_raw = 152;
  test_imu.gyro_bias_x_milli_raw = -57000;
  test_imu.gyro_bias_y_milli_raw = 25000;
  test_imu.gyro_bias_z_milli_raw = 152000;
  test_imu.temperature_c = -10.25f;
  test_imu.read_error_count = 123456U;
  test_imu.consecutive_errors = 2U;
  test_imu.angular_velocity_x_rad_s = 0.1234f;
  test_imu.angular_velocity_z_rad_s = -0.1234f;
  SetIdle(CHASSIS_RUNTIME_STOP_NONE);
  UsbCdcTransport_Init(); UsbCdcTransport_EnableSofMonitor();
  UsbCdcTransport_OnConfigured(); UsbCdcTransport_OnControlLineState(1U);
  ChassisUsbProtocol_Init(test_tick); ChassisUsbProtocol_Process(test_tick);
}

static void SendPayload(uint8_t type, const uint8_t *payload, uint16_t size)
{
  uint8_t frame[62] = { 0xA5, 0x5A, 1U, type };
  ChassisUsbProtocol_PutU16(&frame[4], ++host_sequence);
  ChassisUsbProtocol_PutU16(&frame[6], size);
  if (size) { memcpy(&frame[8], payload, size); }
  ChassisUsbProtocol_PutU16(&frame[8U + size],
    ChassisUsbProtocol_Crc16(&frame[2], (uint16_t)(6U + size)));
  assert(UsbCdcTransport_OnReceive(frame, (uint16_t)(10U + size)));
  ChassisUsbProtocol_Process(test_tick); Drain();
}

static void Mode(uint8_t mode)
{
  SendPayload(CHASSIS_USB_MSG_CMD_SET_IMU_STREAM, &mode, 1U);
  assert(ack_request == CHASSIS_USB_MSG_CMD_SET_IMU_STREAM);
  assert(ack_result == CHASSIS_USB_RESULT_OK);
  assert(s_imu_stream == mode);
}

static void Emit(void)
{
  test_imu_snapshot_reads = 0U;
  assert(ChassisUsbProtocol_SendImuState(test_tick, true));
  assert(test_imu_snapshot_reads == 1U);
  Drain();
}

int main(void)
{
  assert(ExistingStopRegressionMain() == 0);
  SetupRawTest();
  SendPayload(CHASSIS_USB_MSG_CMD_GET_STATUS, NULL, 0U);
  assert(raw_frames == 0U && legacy_frames > 0U);
  assert(ChassisUsbProtocol_GetI32(&last_legacy[20]) == 123);
  assert(ChassisUsbProtocol_GetI32(&last_legacy[28]) == -123);
  Mode(1U); Emit();
  assert(raw_frames > 0U);
  assert(ChassisUsbProtocol_GetI32(&last_raw[0]) == 0x12345678L);
  assert((uint32_t)ChassisUsbProtocol_GetI32(&last_raw[4]) == 0xFEDCBA98U);
  assert(last_raw[8] == 0U && last_raw[9] == 0x80U);
  assert(last_raw[10] == 0xFFU && last_raw[11] == 0x7FU);
  assert((int16_t)ChassisUsbProtocol_GetU16(&last_raw[12]) == 8192);
  assert((int16_t)ChassisUsbProtocol_GetU16(&last_raw[14]) == -57);
  assert((int16_t)ChassisUsbProtocol_GetU16(&last_raw[16]) == 25);
  assert((int16_t)ChassisUsbProtocol_GetU16(&last_raw[18]) == 151);
  assert(ChassisUsbProtocol_GetI32(&last_raw[20]) == -57);
  assert(ChassisUsbProtocol_GetI32(&last_raw[24]) == 25);
  assert(ChassisUsbProtocol_GetI32(&last_raw[28]) == 152);
  assert(ChassisUsbProtocol_GetI32(&last_raw[32]) == -10250);
  assert(ChassisUsbProtocol_GetI32(&last_raw[36]) == 123456);
  assert(ChassisUsbProtocol_GetI32(&last_raw[40]) == 2);
  assert(ChassisUsbProtocol_GetU16(&last_raw[44]) == 655U);
  assert(ChassisUsbProtocol_GetU16(&last_raw[46]) == 8192U);
  assert(last_raw[48] == 2U && last_raw[49] == 7U);
  assert(last_raw[50] == 1U && last_raw[51] == 0U);
  puts("PASS: signed RAW, scales, baseline, temperature, state/flags and 52-byte layout; legacy unchanged");

  const uint8_t invalid[] = {3U, 0U};
  SendPayload(CHASSIS_USB_MSG_CMD_SET_IMU_STREAM, NULL, 0U);
  assert(ack_result == CHASSIS_USB_RESULT_BAD_LENGTH && s_imu_stream == 1U);
  SendPayload(CHASSIS_USB_MSG_CMD_SET_IMU_STREAM, invalid, 2U);
  assert(ack_result == CHASSIS_USB_RESULT_BAD_LENGTH && s_imu_stream == 1U);
  SendPayload(CHASSIS_USB_MSG_CMD_SET_IMU_STREAM, invalid, 1U);
  assert(ack_result == CHASSIS_USB_RESULT_BAD_VALUE && s_imu_stream == 1U);
  test_imu.gyro_bias_valid = false; Emit();
  assert(ChassisUsbProtocol_GetI32(&last_raw[20]) == 0);
  assert(ChassisUsbProtocol_GetI32(&last_raw[24]) == 0);
  assert(ChassisUsbProtocol_GetI32(&last_raw[28]) == 0 && last_raw[49] == 5U);
  test_imu.gyro_bias_valid = true;
  test_imu.gyro_bias_z_raw = 150;
  test_imu_calibration.result = IMU_STATIC_RESULT_PASS; Emit();
  assert(ChassisUsbProtocol_GetI32(&last_raw[28]) == 150 && last_raw[49] == 0x37U);
  const unsigned before_raw = raw_frames;
  Mode(0U); Emit(); assert(raw_frames == before_raw);
  puts("PASS: malformed selectors rejected; invalid bias zeroed; fresh calibration baseline and flags visible");

  AssertFreshMotion();
  const uint32_t command_time = test_runtime.last_command_ms;
  const uint32_t accepted = test_runtime.accepted_commands;
  Mode(1U);
  assert(ChassisRuntime_IsActive() && test_runtime.last_command_ms == command_time);
  assert(test_runtime.accepted_commands == accepted && transport_stops == 0U);
  UsbCdcTransport_OnControlLineState(0U); ChassisUsbProtocol_Process(test_tick);
  assert(s_imu_stream == 0U && !ChassisRuntime_IsActive() && transport_stops == 1U);
  UsbCdcTransport_OnControlLineState(1U); ChassisUsbProtocol_Process(test_tick);
  SendPayload(CHASSIS_USB_MSG_CMD_GET_STATUS, NULL, 0U);
  assert(s_imu_stream == 0U && !ChassisRuntime_IsActive());
  Mode(1U); AssertFreshMotion();
  test_tick += 51U; ChassisUsbProtocol_Process(test_tick);
  assert(s_imu_stream == 0U && !ChassisRuntime_IsActive() && transport_stops == 2U);
  UsbCdcTransport_OnSof(test_tick); ChassisUsbProtocol_Process(test_tick);
  Mode(1U); AssertFreshMotion();
  UsbCdcTransport_OnDeconfigured(); ChassisUsbProtocol_Process(test_tick);
  assert(s_imu_stream == 0U && !ChassisRuntime_IsActive() && transport_stops == 3U);
  puts("PASS: selector does not refresh motion command; RAW mode retains real SOF/DTR/deconfigure stops");

  SetupRawTest(); Mode(1U);
  test_tick += CHASSIS_USB_HOST_SESSION_TIMEOUT_MS;
  UsbCdcTransport_OnSof(test_tick); ChassisUsbProtocol_Process(test_tick);
  assert(s_imu_stream == 0U && !s_host_session_active);
  Mode(1U);
  test_tick += CHASSIS_USB_HOST_SESSION_TIMEOUT_MS;
  UsbCdcTransport_OnSof(test_tick);
  SendPayload(CHASSIS_USB_MSG_CMD_GET_STATUS, NULL, 0U);
  assert(s_imu_stream == 0U); /* Expired before parsing the first new frame. */
  Mode(1U);
  for (unsigned i = 0; i < 12U; ++i) { assert(ChassisUsbProtocol_SendImuState(test_tick, true)); }
  const uint16_t before_sequence = s_device_sequence;
  assert(!ChassisUsbProtocol_SendImuState(test_tick, true) && s_device_sequence == before_sequence);
  Drain();
  assert(last_raw_sequence != before_sequence);
  SendHost(CHASSIS_USB_MSG_CMD_VELOCITY);
  SendHost(CHASSIS_USB_MSG_CMD_STOP);
  SendHost(CHASSIS_USB_MSG_CMD_GET_STATUS);
  ChassisUsbProtocol_Process(test_tick); ChassisUsbProtocol_Process(test_tick); Drain();
  assert(test_runtime.last_stop_reason == CHASSIS_RUNTIME_STOP_EXPLICIT);
  assert(s_imu_stream == 1U && last_flags == 0x0150U);
  UsbCdcTransportStats tx;
  assert(UsbCdcTransport_GetStats(&tx) && tx.tx_dropped_frames == 0U);
  assert(critical_depth == 0U);
  puts("PASS: session expiry before/after parsing; RAW backpressure/STOP burst/sequence/0x0150; single snapshot per IMU frame");

  SetupRawTest();
  test_imu.gyro_bias_x_milli_raw = -57125;
  test_imu.gyro_bias_y_milli_raw = 25625;
  test_imu.gyro_bias_z_milli_raw = 150500;
  test_imu.gyro_bias_z_raw = 150;
  test_imu.angular_velocity_z_rad_s = (151000.0f - 150500.0f) /
    (1000.0f * IMU_GYRO_LSB_PER_DEGREE_PER_SECOND) * IMU_DEGREES_TO_RADIANS;
  Mode(2U); Emit();
  assert(precise_frames >= 1U);
  assert(ChassisUsbProtocol_GetI32(&last_precise[20]) == -57125);
  assert(ChassisUsbProtocol_GetI32(&last_precise[24]) == 25625);
  assert(ChassisUsbProtocol_GetI32(&last_precise[28]) == 150500);
  assert((int16_t)ChassisUsbProtocol_GetU16(&last_precise[18]) == 151);
  assert(ChassisUsbProtocol_GetU16(&last_precise[44]) == 655U);
  assert(last_precise[50] == 2U && last_precise[51] == 0U);
  Mode(1U); Emit();
  assert(ChassisUsbProtocol_GetI32(&last_raw[28]) == 150 && last_raw[50] == 1U);
  Mode(0U); Emit();
  assert(ChassisUsbProtocol_GetI32(&last_legacy[28]) ==
    ChassisUsbProtocol_Round(test_imu.angular_velocity_z_rad_s * 1000.0f));
  Mode(2U); test_imu.gyro_bias_valid = false; Emit();
  assert(ChassisUsbProtocol_GetI32(&last_precise[20]) == 0);
  assert(ChassisUsbProtocol_GetI32(&last_precise[24]) == 0);
  assert(ChassisUsbProtocol_GetI32(&last_precise[28]) == 0 && last_precise[49] == 5U);
  UsbCdcTransport_OnControlLineState(0U); ChassisUsbProtocol_Process(test_tick);
  assert(s_imu_stream == CHASSIS_USB_IMU_STREAM_LEGACY);
  assert(critical_depth == 0U);
  puts("PASS: RAW v2 exact signed milli-baseline/format/scale, coarse RAW v1 compatibility, precise SI and DTR fallback");
  return 0;
}
