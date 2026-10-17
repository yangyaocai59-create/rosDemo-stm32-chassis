/* Host integration: actual protocol parser/dispatch/STATUS/session + transport.
 * Other subsystems are mocks, so this is not a motor or hardware safety test. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
#ifdef TEST_REFERENCE_BASELINE
#include "../../../usb_status_flags_review_20261011/sources/rosDemo2/Application/Communication/chassis_usb_protocol.c"
#else
#include "../../../Application/Communication/chassis_usb_protocol.c"
#endif

TestUsbRegisters test_usb_registers;
static uint32_t test_tick;
static unsigned critical_depth;
static ChassisRuntimeStatus test_runtime;
static unsigned transport_stops;
static unsigned stop_commands;
static unsigned status_count;
static uint16_t last_flags;
static uint16_t host_sequence;
static bool test_full_telemetry;
static bool test_custom_imu;
static ImuServiceSnapshot test_imu;
static unsigned test_imu_snapshot_reads;
static ImuStaticTestStatus test_imu_calibration;

void Test_EnterCritical(void) { ++critical_depth; }
void Test_ExitCritical(void)
{
  assert(critical_depth > 0U);
  --critical_depth;
}
uint32_t HAL_GetTick(void) { return test_tick; }
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
  assert(length == 10U + ChassisUsbProtocol_GetU16(&buffer[6]));
  assert(ChassisUsbProtocol_GetU16(&buffer[length - 2U]) ==
         ChassisUsbProtocol_Crc16(&buffer[2], (uint16_t)(length - 4U)));
  if (buffer[3] == CHASSIS_USB_MSG_STATUS)
  {
    ++status_count;
    last_flags = ChassisUsbProtocol_GetU16(&buffer[8U + 22U]);
    assert(buffer[8U + 19U] == 0U && buffer[8U + 21U] == 0U);
    if (test_runtime.state == CHASSIS_RUNTIME_STATE_IDLE)
    {
      assert(buffer[8U + 18U] == CHASSIS_RUNTIME_STATE_IDLE);
      assert(ChassisUsbProtocol_GetI32(&buffer[8U + 24U]) == 0);
    }
  }
  return USBD_OK;
}
bool DebugLog_Printf(DebugLog_Level level, const char *format, ...)
{
  (void)level;
  (void)format;
  assert(critical_depth == 0U); /* No blocking UART inside snapshot. */
  return true;
}
bool DebugLog_BeginDeferred(void) { return true; }
void DebugLog_EndDeferred(void) {}
uint32_t DebugLog_GetDeferredDroppedCount(void) { return 0U; }
bool ChassisRuntime_GetStatus(ChassisRuntimeStatus *status)
{ *status = test_runtime; return true; }
bool ChassisRuntime_IsActive(void)
{ return test_runtime.state == CHASSIS_RUNTIME_STATE_ACTIVE; }
bool ChassisController_IsActive(void) { return ChassisRuntime_IsActive(); }
bool ChassisRuntime_SetVelocity(float vx, float wz, uint32_t now_ms)
{
  test_runtime.state = CHASSIS_RUNTIME_STATE_ACTIVE;
  test_runtime.command_valid = true;
  test_runtime.requested_command.linear_x_m_s = vx;
  test_runtime.requested_command.angular_z_rad_s = wz;
  test_runtime.last_command_ms = now_ms;
  ++test_runtime.accepted_commands;
  return true;
}
static void SetIdle(ChassisRuntimeStopReason reason)
{
  test_runtime.state = CHASSIS_RUNTIME_STATE_IDLE;
  test_runtime.command_valid = false;
  test_runtime.requested_command.linear_x_m_s = 0.0f;
  test_runtime.requested_command.angular_z_rad_s = 0.0f;
  test_runtime.last_stop_reason = reason;
}
void ChassisRuntime_Stop(uint32_t now_ms)
{
  (void)now_ms;
  ++stop_commands;
  SetIdle(CHASSIS_RUNTIME_STOP_EXPLICIT);
  /* Model the STOP path's UART wait/preemption: caller time becomes old,
   * while real SOF IRQs continue updating the production transport. */
  test_tick += 20U;
  UsbCdcTransport_OnSof(test_tick);
}
void ChassisRuntime_StopForTransportLoss(uint32_t now_ms)
{
  assert(now_ms == test_tick);
  ++transport_stops;
  SetIdle(CHASSIS_RUNTIME_STOP_TRANSPORT_LOST);
}
bool ChassisRuntime_ClearFault(uint32_t now_ms)
{ (void)now_ms; return true; }
bool WheelSpeedEstimator_GetSnapshot(
  WheelSpeedState states[ENCODER_WHEEL_COUNT])
{
  memset(states, 0, sizeof(WheelSpeedState) * ENCODER_WHEEL_COUNT);
  for (unsigned i = 0U; i < ENCODER_WHEEL_COUNT; ++i)
  { states[i].valid = true; states[i].stopped = true; }
  return true;
}
bool WatchdogService_GetStatus(WatchdogServiceStatus *status)
{
  memset(status, 0, sizeof(*status));
  status->armed = true;
  status->health_eligible = true;
  return true;
}
bool BatteryMonitor_GetSnapshot(BatteryMonitorSnapshot *snapshot)
{
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->valid = test_full_telemetry;
  return test_full_telemetry;
}
bool ImuService_GetSnapshot(ImuServiceSnapshot *snapshot)
{
  ++test_imu_snapshot_reads;
  if (test_custom_imu) { *snapshot = test_imu; return true; }
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->valid = test_full_telemetry;
  snapshot->initialized = test_full_telemetry;
  return test_full_telemetry;
}
bool MotorDriver_GetStatus(MotorDriverStatus *status)
{ memset(status, 0, sizeof(*status)); return true; }
bool EmergencyStop_IsLatched(void) { return false; }
bool EmergencyStop_IsInputActive(void) { return false; }
bool EmergencyStop_CanEnableMotor(void) { return true; }
bool FaultManager_HasCriticalFault(void) { return false; }
bool MotorOpenLoopTest_IsActive(void) { return false; }
void MotorOpenLoopTest_StopAll(uint32_t now_ms) { (void)now_ms; }
bool ChassisRuntimeTest_IsActive(void) { return false; }
bool ImuStaticTest_IsActive(void) { return false; }
void ImuStaticTest_Cancel(void) {}
bool ImuStaticTest_CancelIfSource(ImuStaticSource source)
{ (void)source; return false; }
bool ImuStaticTest_Start(uint32_t now, bool active, ImuStaticSource source)
{ (void)now; (void)active; (void)source; return false; }
bool ImuStaticTest_GetStatus(ImuStaticTestStatus *status)
{ *status = test_imu_calibration; return true; }

static void SendHost(uint8_t type)
{
  uint8_t frame[18] = { 0xA5U, 0x5AU, 1U, type };
  const uint16_t payload_length =
    (type == CHASSIS_USB_MSG_CMD_VELOCITY) ? 8U : 0U;
  ChassisUsbProtocol_PutU16(&frame[4], ++host_sequence);
  ChassisUsbProtocol_PutU16(&frame[6], payload_length);
  if (payload_length != 0U) { ChassisUsbProtocol_PutU32(&frame[8], 30U); }
  ChassisUsbProtocol_PutU16(&frame[8U + payload_length],
    ChassisUsbProtocol_Crc16(&frame[2], (uint16_t)(6U + payload_length)));
  assert(UsbCdcTransport_OnReceive(frame, 10U + payload_length));
}
static void Drain(void)
{
  for (unsigned i = 0U; i < 8U; ++i)
  {
    UsbCdcTransport_OnTransmitComplete();
    ChassisUsbProtocol_Process(test_tick);
  }
}
static void AssertFreshMotion(void)
{
  UsbCdcTransport_OnSof(test_tick);
  SendHost(CHASSIS_USB_MSG_CMD_VELOCITY);
  ChassisUsbProtocol_Process(test_tick);
  Drain();
  assert(ChassisRuntime_IsActive());
}
int main(void)
{
  test_tick = 1000U;
  SetIdle(CHASSIS_RUNTIME_STOP_NONE);
  UsbCdcTransport_Init();
  UsbCdcTransport_EnableSofMonitor();
  UsbCdcTransport_OnConfigured();
  UsbCdcTransport_OnControlLineState(1U);
  ChassisUsbProtocol_Init(test_tick);
  ChassisUsbProtocol_Process(test_tick);

  for (unsigned cycle = 0U; cycle < 50U; ++cycle)
  {
    AssertFreshMotion();
    const uint32_t old_time = test_tick;
    const unsigned previous_status = status_count;
    SendHost(CHASSIS_USB_MSG_CMD_STOP);
    SendHost(CHASSIS_USB_MSG_CMD_GET_STATUS);
    ChassisUsbProtocol_Process(old_time);
    Drain();
    assert(status_count > previous_status);
    if (last_flags != 0x0150U)
    {
      fprintf(stderr, "STOP regression: actual STATUS flags=0x%04X, expected=0x0150\n",
              (unsigned)last_flags);
    }
    assert(last_flags == 0x0150U);
    assert(test_runtime.last_stop_reason == CHASSIS_RUNTIME_STOP_EXPLICIT);
    assert(s_link_was_configured && s_host_session_active);
    assert(transport_stops == 0U);
  }
  assert(stop_commands == 50U);

  AssertFreshMotion();
  test_tick += 50U;
  ChassisUsbProtocol_Process(test_tick);
  assert(ChassisRuntime_IsActive());
  ++test_tick;
  ChassisUsbProtocol_Process(test_tick);
  assert(!ChassisRuntime_IsActive() && transport_stops == 1U);
  assert(!s_host_session_active && s_parser.index == 0U);
  UsbCdcTransport_OnSof(test_tick);
  ChassisUsbProtocol_Process(test_tick);
  assert(!ChassisRuntime_IsActive()); /* Link recovery never replays motion. */

  AssertFreshMotion();
  UsbCdcTransport_OnControlLineState(0U);
  SendHost(CHASSIS_USB_MSG_CMD_VELOCITY); /* Must be discarded, not replayed. */
  ChassisUsbProtocol_Process(test_tick);
  assert(!ChassisRuntime_IsActive() && transport_stops == 2U);
  assert(!s_host_session_active);
  UsbCdcTransport_OnControlLineState(1U);
  ChassisUsbProtocol_Process(test_tick);
  assert(!ChassisRuntime_IsActive());

  AssertFreshMotion();
  UsbCdcTransport_OnDeconfigured();
  ChassisUsbProtocol_Process(test_tick);
  assert(!ChassisRuntime_IsActive() && transport_stops == 3U);
  UsbCdcTransport_OnConfigured();
  UsbCdcTransport_OnControlLineState(1U);
  ChassisUsbProtocol_Process(test_tick);
  assert(!ChassisRuntime_IsActive());
  AssertFreshMotion();
  assert(critical_depth == 0U);
  puts("PASS: 50 production parser STOP/GET_STATUS/restart cycles; 0x0150 preserved; real SOF 51ms/DTR/deconfigure stop; no automatic replay");
  return 0;
}
