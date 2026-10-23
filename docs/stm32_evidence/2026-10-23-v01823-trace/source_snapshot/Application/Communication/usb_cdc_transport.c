/**
 * @file usb_cdc_transport.c
 * @brief Interrupt-safe USB CDC RX ring and task-owned TX queue.
 */

#include "usb_cdc_transport.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"

#define USB_CDC_RX_RING_SIZE       512U
#define USB_CDC_RX_RING_MASK       (USB_CDC_RX_RING_SIZE - 1U)
#define USB_CDC_TX_QUEUE_DEPTH      16U
#define USB_CDC_TX_CONTROL_RESERVE   4U
#define USB_CDC_TX_CONTROL_BACKLOG  4U
#define USB_CDC_SOF_TIMEOUT_MS      50U

#if ((USB_CDC_RX_RING_SIZE & USB_CDC_RX_RING_MASK) != 0U)
#error "USB CDC RX ring size must be a power of two"
#endif

typedef struct
{
  uint16_t length;
  uint8_t data[USB_CDC_TRANSPORT_MAX_FRAME_SIZE];
} UsbCdcTxSlot;

static uint8_t s_rx_ring[USB_CDC_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

static UsbCdcTxSlot s_tx_queue[USB_CDC_TX_QUEUE_DEPTH];
static UsbCdcTxSlot s_control_backlog[USB_CDC_TX_CONTROL_BACKLOG];
static volatile uint8_t s_control_head;
static volatile uint8_t s_control_tail;
static uint8_t s_tx_active[USB_CDC_TRANSPORT_MAX_FRAME_SIZE];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile bool s_tx_in_flight;
static volatile bool s_configured;
static volatile bool s_initialized;
static volatile bool s_sof_monitor_enabled;
static volatile bool s_sof_seen;
static volatile bool s_control_line_state_seen;
static volatile bool s_host_port_open;
static volatile uint32_t s_last_sof_ms;
static volatile uint32_t s_sof_count;
static volatile uint32_t s_deconfigurations;
static volatile uint32_t s_control_line_events;
static volatile UsbCdcTransportStats s_stats;

void UsbCdcTransport_Init(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_tx_head = 0U;
  s_tx_tail = 0U;
  s_tx_in_flight = false;
  s_control_head = 0U;
  s_control_tail = 0U;
  s_configured = false;
  s_sof_monitor_enabled = false;
  s_sof_seen = false;
  s_control_line_state_seen = false;
  s_host_port_open = false;
  s_last_sof_ms = 0U;
  s_sof_count = 0U;
  s_deconfigurations = 0U;
  s_control_line_events = 0U;
  (void)memset((void *)&s_stats, 0, sizeof(s_stats));
  s_initialized = true;
}

void UsbCdcTransport_EnableSofMonitor(void)
{
  /* CubeMX keeps VBUS sensing disabled for this self-powered product. SOF is
   * therefore used as the physical-host heartbeat without changing .ioc. */
  USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_SOFM;
  s_sof_monitor_enabled = true;
}

void UsbCdcTransport_OnConfigured(void)
{
  if (!s_initialized)
  {
    return;
  }
  s_rx_tail = s_rx_head;
  s_tx_tail = s_tx_head;
  s_control_tail = s_control_head;
  s_tx_in_flight = false;
  s_last_sof_ms = HAL_GetTick();
  s_sof_seen = true;
  s_control_line_state_seen = false;
  s_host_port_open = false;
  s_configured = true;
  ++s_stats.configurations;
}

void UsbCdcTransport_OnDeconfigured(void)
{
  ++s_deconfigurations;
  s_configured = false;
  s_sof_seen = false;
  s_control_line_state_seen = false;
  s_host_port_open = false;
  s_rx_tail = s_rx_head;
  s_tx_in_flight = false;
  s_tx_tail = s_tx_head;
  s_control_tail = s_control_head;
}

bool UsbCdcTransport_OnReceive(const uint8_t *data, uint32_t length)
{
  uint16_t head;
  uint16_t tail;
  uint16_t free_bytes;

  if ((!s_initialized) || (!s_configured) || (data == NULL) ||
      (length == 0U))
  {
    return false;
  }

  head = s_rx_head;
  tail = s_rx_tail;
  free_bytes = (uint16_t)((tail - head - 1U) & USB_CDC_RX_RING_MASK);
  ++s_stats.rx_packets;
  s_stats.rx_bytes += length;

  if (length > free_bytes)
  {
    ++s_stats.rx_overflow_packets;
    s_stats.rx_dropped_bytes += length;
    return false;
  }

  for (uint32_t index = 0U; index < length; ++index)
  {
    s_rx_ring[head] = data[index];
    head = (uint16_t)((head + 1U) & USB_CDC_RX_RING_MASK);
  }
  __DMB();
  s_rx_head = head;
  return true;
}

void UsbCdcTransport_OnTransmitComplete(void)
{
  s_tx_in_flight = false;
}

void UsbCdcTransport_OnSof(uint32_t now_ms)
{
  s_last_sof_ms = now_ms;
  s_sof_seen = true;
  ++s_sof_count;
}

void UsbCdcTransport_OnControlLineState(uint16_t control_line_state)
{
  if (!s_initialized)
  {
    return;
  }
  s_host_port_open = ((control_line_state & 0x0001U) != 0U);
  __DMB();
  s_control_line_state_seen = true;
  ++s_control_line_events;
}

bool UsbCdcTransport_IsConfigured(void)
{
  return s_configured;
}

bool UsbCdcTransport_GetLinkSnapshot(UsbCdcLinkSnapshot *snapshot)
{
  if (snapshot == NULL)
  {
    return false;
  }

  /* Task context only. OTG_FS priority=5 and TIM7 HAL tick priority=15 are
   * masked by configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5. Capture both
   * timestamps inside the same short critical section. No HAL I/O or logs.
   * Merely passing HAL_GetTick() from the caller still leaves an ISR race. */
  taskENTER_CRITICAL();
  snapshot->configured = s_configured;
  snapshot->sof_monitor_enabled = s_sof_monitor_enabled;
  snapshot->sof_seen = s_sof_seen;
  snapshot->last_sof_ms = s_last_sof_ms;
  snapshot->sample_time_ms = HAL_GetTick();
  snapshot->sof_count = s_sof_count;
  snapshot->configurations = s_stats.configurations;
  snapshot->deconfigurations = s_deconfigurations;
  snapshot->control_line_events = s_control_line_events;
  snapshot->control_line_state_seen = s_control_line_state_seen;
  snapshot->dtr_asserted = s_host_port_open;
  taskEXIT_CRITICAL();

  /* Unsigned subtraction is intentional and handles the 32-bit tick wrap;
   * it is now applied to timestamps from one coherent, fresh snapshot. */
  snapshot->sof_age_ms = snapshot->sample_time_ms - snapshot->last_sof_ms;
  snapshot->link_alive = snapshot->configured &&
    ((!snapshot->sof_monitor_enabled) ||
      (snapshot->sof_seen &&
       (snapshot->sof_age_ms <= USB_CDC_SOF_TIMEOUT_MS)));
  snapshot->host_port_open = snapshot->configured &&
    ((!snapshot->control_line_state_seen) || snapshot->dtr_asserted);
  return true;
}

bool UsbCdcTransport_IsLinkAlive(void)
{
  UsbCdcLinkSnapshot snapshot;

  return UsbCdcTransport_GetLinkSnapshot(&snapshot) && snapshot.link_alive;
}

bool UsbCdcTransport_IsHostPortOpen(void)
{
  UsbCdcLinkSnapshot snapshot;

  /* Preserve compatibility with hosts that never issue the optional CDC
   * control-line request. Once the request is seen, DTR is authoritative. */
  return UsbCdcTransport_GetLinkSnapshot(&snapshot) && snapshot.host_port_open;
}

void UsbCdcTransport_HandleLinkTimeout(void)
{
  taskENTER_CRITICAL();
  s_rx_tail = s_rx_head;
  s_tx_tail = s_tx_head;
  s_control_tail = s_control_head;
  taskEXIT_CRITICAL();
  /* A missed SOF is not an endpoint reset. Keep an already submitted buffer
   * owned until completion/deconfiguration, just as for DTR close. */
}

void UsbCdcTransport_HandleHostPortClosed(void)
{
  taskENTER_CRITICAL();
  s_rx_tail = s_rx_head;
  s_tx_tail = s_tx_head;
  s_control_tail = s_control_head;
  /* An endpoint transfer already submitted to the USB stack owns
   * s_tx_active until its completion callback. Do not reuse that buffer. */
  taskEXIT_CRITICAL();
}

bool UsbCdcTransport_ReadByte(uint8_t *byte)
{
  uint16_t tail;

  if ((!s_configured) || (byte == NULL))
  {
    return false;
  }
  tail = s_rx_tail;
  if (tail == s_rx_head)
  {
    return false;
  }

  *byte = s_rx_ring[tail];
  __DMB();
  s_rx_tail = (uint16_t)((tail + 1U) & USB_CDC_RX_RING_MASK);
  return true;
}

static uint8_t UsbCdcTransport_TxUsed(void)
{
  return (uint8_t)((s_tx_head + USB_CDC_TX_QUEUE_DEPTH - s_tx_tail) %
                   USB_CDC_TX_QUEUE_DEPTH);
}

static bool UsbCdcTransport_PortOpenLocked(void)
{
  return s_configured && ((!s_control_line_state_seen) || s_host_port_open) &&
    ((!s_sof_monitor_enabled) || (s_sof_seen &&
      ((uint32_t)(HAL_GetTick() - s_last_sof_ms) <= USB_CDC_SOF_TIMEOUT_MS)));
}

static void UsbCdcTransport_PushFifo(const uint8_t *data, uint16_t length)
{
  s_tx_queue[s_tx_head].length = length;
  (void)memcpy(s_tx_queue[s_tx_head].data, data, length);
  __DMB();
  s_tx_head = (uint8_t)((s_tx_head + 1U) % USB_CDC_TX_QUEUE_DEPTH);
  const uint8_t used = UsbCdcTransport_TxUsed();
  if (used > s_stats.tx_queue_peak) { s_stats.tx_queue_peak = used; }
}

bool UsbCdcTransport_CanQueueTelemetry(void)
{
  bool available;
  taskENTER_CRITICAL();
  available = UsbCdcTransport_PortOpenLocked() &&
    (s_control_head == s_control_tail) &&
    (UsbCdcTransport_TxUsed() <
      (USB_CDC_TX_QUEUE_DEPTH - 1U - USB_CDC_TX_CONTROL_RESERVE));
  taskEXIT_CRITICAL();
  return available;
}

void UsbCdcTransport_RecordTelemetryDeferred(void)
{
  taskENTER_CRITICAL();
  ++s_stats.tx_telemetry_deferred;
  taskEXIT_CRITICAL();
}

static bool UsbCdcTransport_Queue(const uint8_t *data, uint16_t length,
                                 bool control)
{
  bool accepted = false;

  if ((data == NULL) || (length == 0U) ||
      (length > USB_CDC_TRANSPORT_MAX_FRAME_SIZE))
  {
    return false;
  }

  /* Advance a completed transfer before admitting a new burst. USB callbacks
   * may flush queues; admission and the bounded copy share a critical section. */
  UsbCdcTransport_ProcessTransmit();
  taskENTER_CRITICAL();
  if (!UsbCdcTransport_PortOpenLocked())
  {
    taskEXIT_CRITICAL();
    return false;
  }
  const uint8_t used = UsbCdcTransport_TxUsed();
  if ((s_control_head == s_control_tail) &&
      (used < (control ? (USB_CDC_TX_QUEUE_DEPTH - 1U) :
        (USB_CDC_TX_QUEUE_DEPTH - 1U - USB_CDC_TX_CONTROL_RESERVE))))
  {
    UsbCdcTransport_PushFifo(data, length);
    accepted = true;
  }
  else if (control)
  {
    const uint8_t next = (uint8_t)((s_control_head + 1U) %
                                   USB_CDC_TX_CONTROL_BACKLOG);
    ++s_stats.tx_full_events;
    if (next != s_control_tail)
    {
      s_control_backlog[s_control_head].length = length;
      (void)memcpy(s_control_backlog[s_control_head].data, data, length);
      __DMB();
      s_control_head = next;
      const uint8_t pending = (uint8_t)((s_control_head +
        USB_CDC_TX_CONTROL_BACKLOG - s_control_tail) % USB_CDC_TX_CONTROL_BACKLOG);
      if (pending > s_stats.tx_control_backlog_peak)
      { s_stats.tx_control_backlog_peak = pending; }
      accepted = true; /* Retained for retry, not discarded. */
    }
  }
  if (!accepted)
  {
    ++s_stats.tx_dropped_frames;
    if (control) { ++s_stats.tx_control_drops; }
    else { ++s_stats.tx_full_events; }
    s_stats.tx_last_dropped_type = (length > 3U) ? data[3] : 0U;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

bool UsbCdcTransport_QueueTransmit(const uint8_t *data, uint16_t length)
{
  return UsbCdcTransport_Queue(data, length, false);
}

bool UsbCdcTransport_QueueControlTransmit(const uint8_t *data, uint16_t length)
{
  return UsbCdcTransport_Queue(data, length, true);
}

void UsbCdcTransport_ProcessTransmit(void)
{
  UsbCdcTxSlot *slot;
  uint8_t result;

  taskENTER_CRITICAL();
  if (!UsbCdcTransport_PortOpenLocked())
  {
    taskEXIT_CRITICAL();
    return;
  }
  /* Append staged control replies behind older FIFO data. New telemetry is
   * withheld until this backlog drains, so device sequence order is stable. */
  if ((s_control_tail != s_control_head) &&
      (UsbCdcTransport_TxUsed() < (USB_CDC_TX_QUEUE_DEPTH - 1U)))
  {
    UsbCdcTxSlot *pending = &s_control_backlog[s_control_tail];
    UsbCdcTransport_PushFifo(pending->data, pending->length);
    s_control_tail = (uint8_t)((s_control_tail + 1U) % USB_CDC_TX_CONTROL_BACKLOG);
  }
  if (s_tx_in_flight || (s_tx_tail == s_tx_head))
  {
    taskEXIT_CRITICAL();
    return;
  }

  slot = &s_tx_queue[s_tx_tail];
  (void)memcpy(s_tx_active, slot->data, slot->length);
  s_tx_in_flight = true;
  result = CDC_Transmit_FS(s_tx_active, slot->length);
  if (result == USBD_OK)
  {
    ++s_stats.tx_frames;
    s_stats.tx_bytes += slot->length;
    s_tx_tail = (uint8_t)((s_tx_tail + 1U) % USB_CDC_TX_QUEUE_DEPTH);
  }
  else
  {
    s_tx_in_flight = false;
    if (result == USBD_BUSY) { ++s_stats.tx_busy_returns; }
    else { ++s_stats.tx_submit_errors; }
  }
  /* CDC_Transmit_FS is a bounded, nonblocking <=64-byte USB submission.
   * Masking USB IRQ avoids a deconfiguration/callback changing ownership
   * between buffer selection and submission. Never put UART/waits here. */
  taskEXIT_CRITICAL();
}

bool UsbCdcTransport_GetStats(UsbCdcTransportStats *stats)
{
  if (stats == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *stats = *(const UsbCdcTransportStats *)&s_stats;
  taskEXIT_CRITICAL();
  return true;
}
