/**
 * @file firmware_info.h
 * @brief Central firmware identity used by startup diagnostics.
 */

#ifndef APPLICATION_SYSTEM_FIRMWARE_INFO_H_
#define APPLICATION_SYSTEM_FIRMWARE_INFO_H_

#define FIRMWARE_NAME       "rosDemo Four-Wheel Differential Chassis"
#include "chassis_control_config.h"
#if CHASSIS_CONTROL_REVISION == 205
#define FIRMWARE_VERSION    "0.20.5-wheel-control-candidate"
#elif CHASSIS_CONTROL_REVISION == 204
#define FIRMWARE_VERSION    "0.20.4-wheel-control-candidate"
#elif CHASSIS_CONTROL_REVISION == 203
#define FIRMWARE_VERSION    "0.20.3-wheel-control-candidate"
#elif CHASSIS_CONTROL_REVISION == 202
#define FIRMWARE_VERSION    "0.20.2-wheel-control-candidate"
#elif CHASSIS_CONTROL_REVISION == 201
#define FIRMWARE_VERSION    "0.20.1-wheel-control-candidate"
#elif CHASSIS_CONTROL_V2
#define FIRMWARE_VERSION    "0.20.0-wheel-control-candidate"
#else
#define FIRMWARE_VERSION    "0.19.1-effective-track-candidate"
#endif
#define FIRMWARE_MCU        "STM32F407ZGT6"

#endif /* APPLICATION_SYSTEM_FIRMWARE_INFO_H_ */
