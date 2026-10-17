/**
 * @file encoder_service.h
 * @brief Quadrature encoder acquisition for the four-wheel chassis.
 *
 * raw_count remains the unmodified timer CNT value. delta, accumulated_count,
 * and direction apply the calibrated per-wheel polarity so positive always
 * means physical wheel-forward. No RPM, wheel geometry, gear ratio, or
 * closed-loop control is performed here.
 */

#ifndef APPLICATION_CHASSIS_ENCODER_SERVICE_H_
#define APPLICATION_CHASSIS_ENCODER_SERVICE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ENCODER_WHEEL_LF = 0,
  ENCODER_WHEEL_LR,
  ENCODER_WHEEL_RF,
  ENCODER_WHEEL_RR,
  ENCODER_WHEEL_COUNT
} EncoderWheel;

typedef enum
{
  ENCODER_DIRECTION_NEGATIVE = -1,
  ENCODER_DIRECTION_STOPPED = 0,
  ENCODER_DIRECTION_POSITIVE = 1
} EncoderDirection;

typedef struct
{
  /** Unmodified hardware timer CNT value. */
  uint32_t raw_count;
  /** Polarity-normalized sample delta; positive means wheel-forward. */
  int32_t delta;
  /** Polarity-normalized accumulated count; positive means wheel-forward. */
  int64_t accumulated_count;
  /** Polarity-normalized direction. */
  EncoderDirection direction;
} EncoderState;

/** Start all four encoder timers and capture their initial counters. */
bool EncoderService_Init(void);

/** Sample all counters and update delta, direction, and accumulated counts. */
void EncoderService_Update(void);

/** Copy a coherent snapshot of all four encoder states. */
bool EncoderService_GetSnapshot(EncoderState states[ENCODER_WHEEL_COUNT]);

/** Return the stable LF/LR/RF/RR name for a wheel index. */
const char *EncoderService_GetWheelName(EncoderWheel wheel);

/** Return STOP/POS/NEG, where POS means physical wheel-forward. */
const char *EncoderService_GetDirectionName(EncoderDirection direction);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_ENCODER_SERVICE_H_ */
