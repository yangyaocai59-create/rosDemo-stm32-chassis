/* Compile the actual production transport, not a copied link predicate.
 * IRQs and tick are mocked; physical USB/STOP acceptance is a separate test. */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include "usb_cdc_transport.h"
#include "usbd_cdc_if.h"

static uint32_t test_tick;
static unsigned critical_depth;
static unsigned snapshots;
static unsigned transmitted;
static int sof_before_enter;
static int sof_during_tick;
static int sof_pending;
static int require_critical_tick;
TestUsbRegisters test_usb_registers;

void Test_EnterCritical(void)
{
  /* An IRQ can win the race immediately before BASEPRI is raised. */
  if (sof_before_enter)
  {
    assert(critical_depth == 0U);
    sof_before_enter = 0;
    ++test_tick;
    UsbCdcTransport_OnSof(test_tick);
  }
  ++critical_depth;
}

void Test_ExitCritical(void)
{
  assert(critical_depth > 0U);
  --critical_depth;
  /* Simulate deferred tick + USB IRQ delivery after the snapshot. */
  if ((critical_depth == 0U) && sof_pending)
  {
    sof_pending = 0;
    ++test_tick;
    UsbCdcTransport_OnSof(test_tick);
  }
}

uint32_t HAL_GetTick(void)
{
  if (require_critical_tick)
  {
    assert(critical_depth > 0U);
  }
  if (sof_during_tick)
  {
    assert(critical_depth > 0U);
    sof_during_tick = 0;
    sof_pending = 1;
  }
  return test_tick;
}

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
  assert(buffer != NULL && length > 0U);
  ++transmitted;
  return USBD_OK;
}

static UsbCdcLinkSnapshot Snapshot(void)
{
  UsbCdcLinkSnapshot link;
  require_critical_tick = 1;
  assert(UsbCdcTransport_GetLinkSnapshot(&link));
  require_critical_tick = 0;
  assert(critical_depth == 0U);
  ++snapshots;
  return link;
}

int main(void)
{
  UsbCdcLinkSnapshot link;
  const uint8_t bytes[] = { 0xA5U, 0x5AU, 0x01U };
  uint8_t byte;
  UsbCdcTransportStats stats;

  UsbCdcTransport_Init();
  assert(!UsbCdcTransport_GetLinkSnapshot(NULL));
  link = Snapshot();
  assert(!link.configured && !link.link_alive && !link.host_port_open);

  test_tick = 1000U;
  UsbCdcTransport_OnConfigured();
  link = Snapshot();
  assert(link.link_alive && link.host_port_open);
  assert(!link.control_line_state_seen && !link.dtr_asserted);
  assert(link.configurations == 1U);
  /* No-monitor compatibility remains unchanged. */
  test_tick = 2000U;
  assert(UsbCdcTransport_IsLinkAlive());
  UsbCdcTransport_EnableSofMonitor();
  assert(test_usb_registers.GINTMSK & USB_OTG_GINTMSK_SOFM);
  assert(!UsbCdcTransport_IsLinkAlive());

  UsbCdcTransport_OnSof(2000U);
  test_tick = 2050U;
  link = Snapshot();
  assert(link.sof_age_ms == 50U && link.link_alive);
  test_tick = 2051U;
  link = Snapshot();
  assert(link.sof_age_ms == 51U && !link.link_alive);

  test_tick = 3000U;
  UsbCdcTransport_OnSof(test_tick);
  const uint32_t stale_task_time = test_tick;
  sof_before_enter = 1;
  link = Snapshot();
  assert(link.sample_time_ms == 3001U && link.last_sof_ms == 3001U);
  assert(link.sof_age_ms == 0U && link.link_alive);
  /* This is the old implementation's independently reproduced false loss. */
  assert((uint32_t)(stale_task_time - link.last_sof_ms) == UINT32_MAX);
  assert((uint32_t)(stale_task_time - link.last_sof_ms) > 50U);

  sof_during_tick = 1;
  link = Snapshot();
  assert(link.sample_time_ms == 3001U && link.last_sof_ms == 3001U);
  assert(link.link_alive); /* IRQ cannot tear the local snapshot. */
  link = Snapshot();
  assert(link.sample_time_ms == 3002U && link.last_sof_ms == 3002U);
  assert(link.sof_count == 4U && link.link_alive);

  UsbCdcTransport_OnSof(0xFFFFFFF0U);
  test_tick = 0x00000022U;
  link = Snapshot();
  assert(link.sof_age_ms == 50U && link.link_alive);
  test_tick = 0x00000023U;
  link = Snapshot();
  assert(link.sof_age_ms == 51U && !link.link_alive);

  test_tick = 4000U;
  UsbCdcTransport_OnSof(test_tick);
  UsbCdcTransport_OnControlLineState(1U);
  link = Snapshot();
  assert(link.control_line_state_seen && link.dtr_asserted);
  assert(link.host_port_open && link.link_alive);
  assert(UsbCdcTransport_OnReceive(bytes, sizeof(bytes)));
  assert(UsbCdcTransport_QueueTransmit(bytes, sizeof(bytes)));
  UsbCdcTransport_OnControlLineState(0U);
  link = Snapshot();
  assert(link.link_alive && !link.host_port_open);
  assert(link.control_line_events == 2U);
  assert(!UsbCdcTransport_QueueTransmit(bytes, sizeof(bytes)));
  UsbCdcTransport_HandleHostPortClosed();
  assert(!UsbCdcTransport_ReadByte(&byte));
  UsbCdcTransport_ProcessTransmit();
  assert(transmitted == 0U);
  UsbCdcTransport_OnControlLineState(1U);
  assert(UsbCdcTransport_IsHostPortOpen());
  assert(UsbCdcTransport_QueueTransmit(bytes, sizeof(bytes)));
  UsbCdcTransport_ProcessTransmit();
  assert(transmitted == 1U);
  UsbCdcTransport_OnTransmitComplete();

  assert(UsbCdcTransport_OnReceive(bytes, sizeof(bytes)));
  assert(UsbCdcTransport_QueueTransmit(bytes, sizeof(bytes)));
  test_tick += 51U;
  assert(!UsbCdcTransport_IsLinkAlive());
  UsbCdcTransport_HandleLinkTimeout();
  assert(!UsbCdcTransport_ReadByte(&byte));
  UsbCdcTransport_ProcessTransmit();
  assert(transmitted == 1U); /* Timed-out session cannot replay queued TX. */

  UsbCdcTransport_OnDeconfigured();
  UsbCdcTransport_OnSof(test_tick);
  link = Snapshot();
  assert(!link.configured && !link.link_alive && !link.host_port_open);
  assert(link.deconfigurations == 1U);
  assert(!UsbCdcTransport_OnReceive(bytes, sizeof(bytes)));
  UsbCdcTransport_OnConfigured();
  link = Snapshot();
  assert(link.configurations == 2U && link.link_alive);
  assert(!link.control_line_state_seen && link.host_port_open);

  UsbCdcTransport_Init();
  UsbCdcTransport_EnableSofMonitor();
  link = Snapshot();
  assert(!link.sof_seen && !link.link_alive);
  assert(UsbCdcTransport_GetStats(&stats));
  assert(stats.configurations == 0U);
  printf("PASS: %u coherent production-transport snapshots; stale caller, IRQ interleavings, 50/51ms, tick wrap, DTR, deconfigure, queue flush\n", snapshots);
  return 0;
}
