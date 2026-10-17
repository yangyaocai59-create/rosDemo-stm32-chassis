/**
 * @file chassis_config.h
 * @brief Calibrated mechanical parameters for the four-wheel chassis.
 *
 * These values describe the motor/encoder/wheel assembly rather than MCU pin
 * assignments. Keep electrical mappings in board_config.h. The nominal wheel
 * diameter and track width can later be replaced by effective values obtained
 * from loaded straight-line and rotation calibration.
 */

#ifndef APPLICATION_CHASSIS_CHASSIS_CONFIG_H_
#define APPLICATION_CHASSIS_CHASSIS_CONFIG_H_

/* WHEELTEC MG513P30-12V with the 13-PPR quadrature Hall encoder. */
#define CHASSIS_ENCODER_PPR                    13U
#define CHASSIS_ENCODER_QUADRATURE_MULTIPLIER  4U
#define CHASSIS_MOTOR_GEAR_RATIO               30U
#define CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV   \
  (CHASSIS_ENCODER_PPR * CHASSIS_ENCODER_QUADRATURE_MULTIPLIER * \
   CHASSIS_MOTOR_GEAR_RATIO)

/* Initial measured chassis geometry. SI units are used by runtime APIs. */
#define CHASSIS_WHEEL_DIAMETER_MM              65U
#define CHASSIS_WHEEL_TRACK_MM                 210U
#define CHASSIS_WHEELBASE_MM                   180U

#define CHASSIS_WHEEL_DIAMETER_M               0.065f
#define CHASSIS_WHEEL_RADIUS_M                 0.0325f
#define CHASSIS_PI_F                            3.14159265358979323846f
#define CHASSIS_WHEEL_CIRCUMFERENCE_M          \
  (CHASSIS_PI_F * CHASSIS_WHEEL_DIAMETER_M)
#define CHASSIS_WHEEL_TRACK_M                  0.210f
#define CHASSIS_WHEELBASE_M                    0.180f

/* Wheel-speed acquisition and filtering parameters. */
#define CHASSIS_SPEED_SAMPLE_PERIOD_MS         10U
#define CHASSIS_SPEED_FILTER_TIME_CONSTANT_MS  50U
#define CHASSIS_SPEED_STOP_TIMEOUT_MS          200U

#endif /* APPLICATION_CHASSIS_CHASSIS_CONFIG_H_ */
