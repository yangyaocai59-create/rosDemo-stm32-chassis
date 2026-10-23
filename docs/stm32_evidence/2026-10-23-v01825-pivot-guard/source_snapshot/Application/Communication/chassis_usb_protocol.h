/**
 * @file chassis_usb_protocol.h
 * @brief Versioned CRC-protected USB CDC protocol for chassis commands.
 */

#ifndef APPLICATION_COMMUNICATION_CHASSIS_USB_PROTOCOL_H_
#define APPLICATION_COMMUNICATION_CHASSIS_USB_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_USB_PROTOCOL_VERSION       1U
#define CHASSIS_USB_STATUS_PERIOD_MS     100U
#define CHASSIS_USB_WHEEL_STATE_PERIOD_MS  50U
#define CHASSIS_USB_BATTERY_STATE_PERIOD_MS 500U
/* Nominal producer period. Telemetry follows new sample_count/timestamp;
 * it is not a second independent 10 ms latest-snapshot resend timer. */
#define CHASSIS_USB_IMU_STATE_PERIOD_MS   10U

typedef enum
{
  CHASSIS_USB_MSG_CMD_VELOCITY = 0x01,
  CHASSIS_USB_MSG_CMD_STOP = 0x02,
  CHASSIS_USB_MSG_CMD_CLEAR_FAULT = 0x03,
  CHASSIS_USB_MSG_CMD_GET_STATUS = 0x04,
  CHASSIS_USB_MSG_CMD_CALIBRATE_IMU = 0x05,
  CHASSIS_USB_MSG_CMD_SET_IMU_STREAM = 0x06,
  CHASSIS_USB_MSG_ACK = 0x80,
  CHASSIS_USB_MSG_STATUS = 0x81,
  CHASSIS_USB_MSG_WHEEL_STATE = 0x82,
  CHASSIS_USB_MSG_BATTERY_STATE = 0x83,
  CHASSIS_USB_MSG_IMU_STATE = 0x84,
  CHASSIS_USB_MSG_IMU_RAW_V1 = 0x85,
  CHASSIS_USB_MSG_IMU_RAW_V2 = 0x86
} ChassisUsbMessageType;

typedef enum
{
  CHASSIS_USB_IMU_STREAM_LEGACY = 0,
  CHASSIS_USB_IMU_STREAM_RAW_V1 = 1,
  CHASSIS_USB_IMU_STREAM_RAW_V2 = 2
} ChassisUsbImuStream;

typedef enum
{
  CHASSIS_USB_RESULT_OK = 0,
  CHASSIS_USB_RESULT_BAD_LENGTH = 1,
  CHASSIS_USB_RESULT_BAD_VALUE = 2,
  CHASSIS_USB_RESULT_STATE_REJECTED = 3,
  CHASSIS_USB_RESULT_UNSUPPORTED = 4,
  CHASSIS_USB_RESULT_INTERNAL = 5
} ChassisUsbResult;

typedef struct
{
  uint32_t valid_frames;
  uint32_t crc_errors;
  uint32_t version_errors;
  uint32_t length_errors;
  uint32_t unsupported_messages;
  uint16_t last_rx_sequence;
} ChassisUsbProtocolStats;

void ChassisUsbProtocol_Init(uint32_t now_ms);
void ChassisUsbProtocol_Process(uint32_t now_ms);
bool ChassisUsbProtocol_GetStats(ChassisUsbProtocolStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_COMMUNICATION_CHASSIS_USB_PROTOCOL_H_ */
