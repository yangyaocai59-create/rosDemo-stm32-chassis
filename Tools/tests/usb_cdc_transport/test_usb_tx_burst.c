/* Production protocol+transport. Hardware/services use the existing fixture. */
#define main ExistingStopRegressionMain
#define CDC_Transmit_FS FixtureCDC_Transmit_FS
#include "test_usb_protocol_stop.c"
#undef main
#undef CDC_Transmit_FS
static unsigned seen, ack_seen;
static uint16_t last_device_sequence;
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
  if(data[3]==CHASSIS_USB_MSG_ACK) { ++ack_seen; }
  else
  {
    uint16_t sequence=ChassisUsbProtocol_GetU16(&data[4]);
    if(seen) { assert((uint16_t)(sequence-last_device_sequence)==1); }
    last_device_sequence=sequence; ++seen;
  }
  return FixtureCDC_Transmit_FS(data,length);
}
static void Setup(void)
{
  test_tick=1000; test_full_telemetry=true; seen=ack_seen=0;
  SetIdle(CHASSIS_RUNTIME_STOP_NONE);
  UsbCdcTransport_Init(); UsbCdcTransport_EnableSofMonitor();
  UsbCdcTransport_OnConfigured(); UsbCdcTransport_OnControlLineState(1);
  ChassisUsbProtocol_Init(test_tick); ChassisUsbProtocol_Process(test_tick);
}
static void DrainAll(void)
{
  for(unsigned i=0;i<64;i++)
  { UsbCdcTransport_OnTransmitComplete(); ChassisUsbProtocol_Process(test_tick); }
}
int main(void)
{
  UsbCdcTransportStats tx;
  Setup();
  assert(ChassisUsbProtocol_SendStatus(test_tick));
  assert(ChassisUsbProtocol_SendBatteryState());
  assert(ChassisUsbProtocol_SendImuState(test_tick, true));
  SendHost(CHASSIS_USB_MSG_CMD_VELOCITY); SendHost(CHASSIS_USB_MSG_CMD_STOP);
  SendHost(CHASSIS_USB_MSG_CMD_GET_STATUS);
  ChassisUsbProtocol_Process(test_tick);
  assert(test_runtime.last_stop_reason==CHASSIS_RUNTIME_STOP_EXPLICIT);
  assert(s_stats.valid_frames==2); /* Command batch is bounded, not starved. */
  ChassisUsbProtocol_Process(test_tick); DrainAll();
  assert(s_stats.valid_frames==3 && ack_seen==3);
  assert(UsbCdcTransport_GetStats(&tx) && tx.tx_dropped_frames==0);
  assert(last_flags==0x0150 && transport_stops==0);
  puts("PASS: full battery/IMU + preexisting burst + VELOCITY/STOP/GET_STATUS; ACKs/sequence/0x0150, zero drop");

  Setup();
  for(unsigned i=0;i<12;i++) { assert(ChassisUsbProtocol_SendImuState(test_tick, true)); }
  const uint16_t before=s_device_sequence;
  assert(!ChassisUsbProtocol_SendImuState(test_tick, true) && s_device_sequence==before);
  SendHost(CHASSIS_USB_MSG_CMD_VELOCITY); SendHost(CHASSIS_USB_MSG_CMD_STOP);
  SendHost(CHASSIS_USB_MSG_CMD_GET_STATUS);
  ChassisUsbProtocol_Process(test_tick); ChassisUsbProtocol_Process(test_tick);
  assert(test_runtime.last_stop_reason==CHASSIS_RUNTIME_STOP_EXPLICIT);
  assert(UsbCdcTransport_GetStats(&tx) && tx.tx_dropped_frames==0 && tx.tx_telemetry_deferred>0);
  DrainAll(); assert(ack_seen==3 && last_flags==0x0150);
  assert(UsbCdcTransport_GetStats(&tx) && tx.tx_dropped_frames==0);
  puts("PASS: near-full telemetry yields reserved replies and deferred fresh samples; no sequence consumption on deferral");

  Setup();
  for(unsigned i=0;i<25;i++) { SendHost(CHASSIS_USB_MSG_CMD_VELOCITY); }
  SendHost(CHASSIS_USB_MSG_CMD_STOP);
  for(unsigned i=0;i<13;i++) { ChassisUsbProtocol_Process(test_tick); }
  assert(test_runtime.state==CHASSIS_RUNTIME_STATE_IDLE && test_runtime.last_stop_reason==CHASSIS_RUNTIME_STOP_EXPLICIT);
  assert(UsbCdcTransport_GetStats(&tx) && tx.tx_control_drops>0 && tx.tx_dropped_frames>=tx.tx_control_drops);
  assert(critical_depth==0);
  puts("PASS: excessive stalled-host burst remains bounded, records real control drops and still executes STOP");
  return 0;
}
