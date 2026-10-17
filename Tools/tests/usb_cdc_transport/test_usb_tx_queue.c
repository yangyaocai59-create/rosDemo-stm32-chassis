/* Actual transport: capacity/reserve, bounded retry, FIFO and buffer ownership. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../Application/Communication/usb_cdc_transport.c"
TestUsbRegisters test_usb_registers;
static unsigned depth, accepted, busy;
static uint32_t tick;
static uint8_t labels[128];
static uint8_t *active;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
uint32_t HAL_GetTick(void) { return tick; }
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length)
{
  assert(depth && length == 8U);
  if (busy) { return 1U; }
  assert(accepted < sizeof(labels));
  labels[accepted++] = data[4]; active = data;
  return USBD_OK;
}
static void Init(void)
{
  tick = 1000; accepted = busy = 0; active = NULL;
  UsbCdcTransport_Init(); UsbCdcTransport_EnableSofMonitor();
  UsbCdcTransport_OnConfigured(); UsbCdcTransport_OnControlLineState(1);
}
static bool Queue(bool control, uint8_t label)
{
  const uint8_t frame[8] = {0xA5,0x5A,1, control ? 0x80 : 0x82, label,0,0,0};
  return control ? UsbCdcTransport_QueueControlTransmit(frame,8) :
                   UsbCdcTransport_QueueTransmit(frame,8);
}
static void Drain(void)
{
  for (unsigned i=0;i<64;i++)
  { UsbCdcTransport_OnTransmitComplete(); UsbCdcTransport_ProcessTransmit(); }
}
int main(void)
{
  UsbCdcTransportStats stats;
  Init();
  for (uint8_t i=0;i<12;i++) { assert(Queue(false,i)); }
  assert(accepted==1 && active[4]==0 && !UsbCdcTransport_CanQueueTelemetry());
  UsbCdcTransport_RecordTelemetryDeferred();
  assert(UsbCdcTransport_GetStats(&stats) && stats.tx_dropped_frames==0 && stats.tx_telemetry_deferred==1);
  assert(!Queue(false,99)); /* An actual unguarded enqueue failure is a drop. */
  for (uint8_t i=12;i<19;i++) { assert(Queue(true,i)); }
  assert(!Queue(true,99)); /* FIFO and 3-record retry backlog both full. */
  assert(active[4]==0); /* Never overwrite submitted data while busy. */
  assert(UsbCdcTransport_GetStats(&stats));
  assert(stats.tx_queue_peak==15 && stats.tx_control_backlog_peak==3);
  assert(stats.tx_dropped_frames==2 && stats.tx_control_drops==1 && stats.tx_last_dropped_type==0x80);
  Drain(); assert(accepted==19);
  for(unsigned i=0;i<19;i++) { assert(labels[i]==i); }
  assert(UsbCdcTransport_CanQueueTelemetry());
  puts("PASS: telemetry reserve, 15 FIFO + 3 retry + 1 in-flight; exact overflow accounting and FIFO order");

  Init(); busy=1; assert(Queue(true,1)); UsbCdcTransport_ProcessTransmit();
  assert(accepted==0 && !s_tx_in_flight);
  assert(UsbCdcTransport_GetStats(&stats) && stats.tx_busy_returns==1 && stats.tx_dropped_frames==0);
  busy=0; UsbCdcTransport_ProcessTransmit(); assert(accepted==1 && labels[0]==1);
  UsbCdcTransport_OnTransmitComplete();
  puts("PASS: USBD_BUSY keeps frame queued for retry without false drop");

  Init(); for(uint8_t i=0;i<19;i++) { assert(Queue(true,i)); }
  assert(active[4]==0 && s_control_head!=s_control_tail);
  UsbCdcTransport_OnControlLineState(0); UsbCdcTransport_HandleHostPortClosed();
  assert(s_control_head==s_control_tail && s_tx_head==s_tx_tail);
  UsbCdcTransport_OnControlLineState(1); assert(Queue(true,40));
  UsbCdcTransport_ProcessTransmit(); assert(accepted==1 && active[4]==0);
  UsbCdcTransport_OnTransmitComplete(); UsbCdcTransport_ProcessTransmit();
  assert(accepted==2 && labels[1]==40); /* No buffered old reply is replayed. */
  puts("PASS: DTR flush removes FIFO/retry data but preserves in-flight buffer ownership");

  Init(); assert(Queue(true,0)); assert(Queue(true,1));
  assert(accepted==1 && active[4]==0);
  tick+=51; UsbCdcTransport_HandleLinkTimeout();
  assert(!Queue(true,2)); assert(active[4]==0 && s_tx_in_flight);
  UsbCdcTransport_OnSof(tick); assert(Queue(true,3));
  assert(active[4]==0); UsbCdcTransport_OnTransmitComplete();
  UsbCdcTransport_ProcessTransmit(); assert(accepted==2 && labels[1]==3);
  UsbCdcTransport_OnDeconfigured(); assert(s_control_head==s_control_tail && !s_tx_in_flight);
  assert(depth==0);
  puts("PASS: SOF timeout guard, buffer ownership, deconfiguration and no queued replay");
  return 0;
}
