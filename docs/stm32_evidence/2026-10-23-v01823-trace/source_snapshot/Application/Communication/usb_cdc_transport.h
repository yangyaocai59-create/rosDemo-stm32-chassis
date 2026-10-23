/**
 * @file usb_cdc_transport.h
 * @brief Non-blocking byte transport between the Cube USB CDC callbacks and tasks.
 */

#ifndef APPLICATION_COMMUNICATION_USB_CDC_TRANSPORT_H_
#define APPLICATION_COMMUNICATION_USB_CDC_TRANSPORT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_CDC_TRANSPORT_MAX_FRAME_SIZE  64U

typedef struct
{
  uint32_t rx_packets;
  uint32_t rx_bytes;
  uint32_t rx_overflow_packets;
  uint32_t rx_dropped_bytes;
  uint32_t tx_frames;
  uint32_t tx_bytes;
  uint32_t tx_dropped_frames;
  uint32_t configurations;
  /* Local diagnostics only; USB v1 STATUS layout remains unchanged. */
  uint32_t tx_queue_peak;
  uint32_t tx_control_backlog_peak;
  uint32_t tx_full_events;
  uint32_t tx_control_drops;
  uint32_t tx_telemetry_deferred;
  uint32_t tx_busy_returns;
  uint32_t tx_submit_errors;
  uint32_t tx_last_dropped_type;
} UsbCdcTransportStats;

/* One task-context snapshot, sampled with USB and HAL tick IRQs masked.
 * Diagnostics only: this does not change any USB CDC v1 payload. */
typedef struct
{
  uint32_t sample_time_ms;
  uint32_t last_sof_ms;
  uint32_t sof_age_ms;
  uint32_t sof_count;
  uint32_t configurations;
  uint32_t deconfigurations;
  uint32_t control_line_events;
  bool configured;
  bool sof_monitor_enabled;
  bool sof_seen;
  bool control_line_state_seen;
  bool dtr_asserted;
  bool link_alive;
  bool host_port_open;
} UsbCdcLinkSnapshot;

/** Reset the transport before MX_USB_DEVICE_Init() can enable callbacks. */
void UsbCdcTransport_Init(void);

/** Enable the USB SOF interrupt after MX_USB_DEVICE_Init(). */
void UsbCdcTransport_EnableSofMonitor(void);

/** Cube CDC callback adapters. They copy/flag only and never parse commands. */
void UsbCdcTransport_OnConfigured(void);
void UsbCdcTransport_OnDeconfigured(void);
bool UsbCdcTransport_OnReceive(const uint8_t *data, uint32_t length);
void UsbCdcTransport_OnTransmitComplete(void);
void UsbCdcTransport_OnSof(uint32_t now_ms);
void UsbCdcTransport_OnControlLineState(uint16_t control_line_state);

/** Task-context API. */
bool UsbCdcTransport_IsConfigured(void);
/* Samples fresh time internally; never mix a caller's old time with an ISR's
 * newer SOF timestamp. The snapshot also keeps link and DTR decisions coherent. */
bool UsbCdcTransport_GetLinkSnapshot(UsbCdcLinkSnapshot *snapshot);
bool UsbCdcTransport_IsLinkAlive(void);
bool UsbCdcTransport_IsHostPortOpen(void);
void UsbCdcTransport_HandleLinkTimeout(void);
void UsbCdcTransport_HandleHostPortClosed(void);
bool UsbCdcTransport_ReadByte(uint8_t *byte);
bool UsbCdcTransport_QueueTransmit(const uint8_t *data, uint16_t length);
/* ACK/STATUS use reserved FIFO slots and a bounded retry backlog. Wire FIFO
 * ordering, including device sequence order, is preserved (no overtaking). */
bool UsbCdcTransport_QueueControlTransmit(const uint8_t *data, uint16_t length);
bool UsbCdcTransport_CanQueueTelemetry(void);
void UsbCdcTransport_RecordTelemetryDeferred(void);
void UsbCdcTransport_ProcessTransmit(void);
bool UsbCdcTransport_GetStats(UsbCdcTransportStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_COMMUNICATION_USB_CDC_TRANSPORT_H_ */
