/* Production protocol/transport with a DRDY-like source clock independent of
 * the USB task. No serial devices, motors, calibration or Flash operations. */
#define main ExistingStopRegressionMain
#define CDC_Transmit_FS FixtureCDC_Transmit_FS
#include "test_usb_protocol_stop.c"
#undef main
#undef CDC_Transmit_FS

static uint32_t received_count[4096];
static uint32_t received_time[4096];
static unsigned received;

uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
  if ((data[3] == CHASSIS_USB_MSG_IMU_STATE) ||
      (data[3] == CHASSIS_USB_MSG_IMU_RAW_V1) ||
      (data[3] == CHASSIS_USB_MSG_IMU_RAW_V2))
  {
    assert(received < 4096U);
    received_time[received] = (uint32_t)ChassisUsbProtocol_GetI32(&data[8]);
    received_count[received++] = (uint32_t)ChassisUsbProtocol_GetI32(&data[12]);
  }
  return FixtureCDC_Transmit_FS(data, length);
}

static void SetupSchedule(uint32_t start)
{
  test_tick = start;
  received = 0U;
  test_full_telemetry = test_custom_imu = true;
  memset(&test_imu, 0, sizeof(test_imu));
  memset(&test_imu_calibration, 0, sizeof(test_imu_calibration));
  test_imu.initialized = test_imu.valid = true;
  test_imu.state = IMU_SERVICE_STATE_READY;
  test_imu.sample_count = 1U;
  test_imu.sample_time_ms = start;
  SetIdle(CHASSIS_RUNTIME_STOP_NONE);
  UsbCdcTransport_Init();
  UsbCdcTransport_EnableSofMonitor();
  UsbCdcTransport_OnConfigured();
  UsbCdcTransport_OnControlLineState(1U);
  ChassisUsbProtocol_Init(test_tick);
  SendHost(CHASSIS_USB_MSG_CMD_GET_STATUS);
  ChassisUsbProtocol_Process(test_tick);
  Drain();
}

static void PollAt(uint32_t time)
{
  test_tick = time;
  UsbCdcTransport_OnSof(test_tick);
  ChassisUsbProtocol_Process(test_tick);
  Drain();
}

static uint32_t EdgeTime(uint32_t count)
{
  /* An independent 100 Hz device with occasional 1 ms timestamp phase shift.
   * Acquisition remains contiguous. USB polls every 2 ms. */
  return count * 10U + ((count % 31U) == 2U ? 1U : 0U);
}

static void RunIndependentClock(uint32_t start, ChassisUsbImuStream stream)
{
  SetupSchedule(start);
  s_imu_stream = stream;
  uint32_t source = 0U;
  for (uint32_t elapsed = 2U; elapsed <= 802U; elapsed += 2U)
  {
    while (EdgeTime(source + 1U) <= elapsed)
    {
      ++source;
      test_imu.sample_count = source + 1U;
      test_imu.sample_time_ms = start + EdgeTime(source);
    }
    PollAt(start + elapsed);
  }
  unsigned duplicates = 0U, skipped = 0U;
  for (unsigned i = 1U; i < received; ++i)
  {
    const uint32_t step = received_count[i] - received_count[i - 1U];
    if (step == 0U) { ++duplicates; }
    else if (step != 1U) { skipped += step - 1U; }
  }
  if ((duplicates != 0U) || (skipped != 0U))
  {
    fprintf(stderr, "Independent source: received=%u duplicate=%u skipped=%u\n",
            received, duplicates, skipped);
  }
  assert(duplicates == 0U && skipped == 0U);
  assert(received == source + 1U);
  assert(critical_depth == 0U);
}

static void RunHealthAndRetry(void)
{
  SetupSchedule(1000U);
  assert(received == 1U);
  for (uint32_t t = 1002U; t < 1100U; t += 2U) { PollAt(t); }
  assert(received == 1U); /* Unchanged source does not pretend to run at 100 Hz. */
  PollAt(1100U);
  assert(received == 2U && received_count[1] == 1U); /* 100 ms health heartbeat. */

  test_imu.valid = false;
  test_imu.state = IMU_SERVICE_STATE_IO_FAULT;
  ++test_imu.read_error_count;
  ++test_imu.consecutive_errors;
  PollAt(1102U);
  assert(received == 3U); /* Fault immediately visible without new acquisition. */
  test_imu_calibration.result = IMU_STATIC_RESULT_RUNNING;
  PollAt(1104U);
  assert(received == 4U); /* Calibration flag changes must not be hidden. */
  test_imu_calibration.result = IMU_STATIC_RESULT_PASS;
  PollAt(1106U);
  assert(received == 5U);
  test_imu.valid = true;
  test_imu.state = IMU_SERVICE_STATE_READY;
  test_imu.consecutive_errors = 0U;
  ++test_imu.acquisition_epoch;
  PollAt(1108U);
  assert(received == 6U);

  /* Explicit queries retain their immediate reply even for the same sample. */
  SendHost(CHASSIS_USB_MSG_CMD_GET_STATUS);
  PollAt(1110U);
  assert(received == 7U);
  PollAt(1112U);
  assert(received == 7U);

  for (unsigned i = 0U; i < 12U; ++i)
  { assert(ChassisUsbProtocol_SendImuState(test_tick, true)); }
  const uint16_t before_sequence = s_device_sequence;
  const uint32_t before_count = s_last_imu_sent.sample_count;
  ++test_imu.sample_count;
  test_imu.sample_time_ms += 10U;
  assert(!ChassisUsbProtocol_SendImuState(test_tick, false));
  assert(s_device_sequence == before_sequence);
  assert(s_last_imu_sent.sample_count == before_count);
  Drain(); /* Same unsent sample is retried after the queue has space. */
  assert(s_last_imu_sent.sample_count == test_imu.sample_count);

  test_imu.sample_count = UINT32_MAX;
  PollAt(1114U);
  test_imu.sample_count = 0U;
  test_imu.sample_time_ms += 10U;
  PollAt(1116U);
  assert(received_count[received - 1U] == 0U);
  const unsigned before = received;
  PollAt(1118U);
  assert(received == before); /* Source counter wrap is not a sentinel. */
  assert(critical_depth == 0U);
  puts("PASS: immediate fault/calibration/recovery state, 100 ms stalled health, explicit query, TX retry cursor and source-count wrap");
}

int main(void)
{
  RunIndependentClock(1000U, CHASSIS_USB_IMU_STREAM_LEGACY);
  RunIndependentClock(1000U, CHASSIS_USB_IMU_STREAM_RAW_V1);
  RunIndependentClock(1000U, CHASSIS_USB_IMU_STREAM_RAW_V2);
  RunIndependentClock(UINT32_MAX - 400U, CHASSIS_USB_IMU_STREAM_LEGACY);
  puts("PASS: independent DRDY/USB clocks, 9/10/11 ms sample timestamps, SI/RAW and tick wrap; no artificial duplicate/skip");
  RunHealthAndRetry();
  return 0;
}
