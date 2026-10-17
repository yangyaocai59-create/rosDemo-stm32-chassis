/**
 * @file wheel_speed_control_config.h
 * @brief Initial wheel-speed PI and feedforward calibration.
 *
 * The feedforward values were fitted from the unloaded Stage 6A scans on
 * 2026-09-22 with VM recorded as an unverified 12 V assumption. They are
 * deliberately kept separate from board pin mappings and mechanical data.
 * Loaded-chassis calibration will replace these provisional values later.
 */

#ifndef APPLICATION_CHASSIS_WHEEL_SPEED_CONTROL_CONFIG_H_
#define APPLICATION_CHASSIS_WHEEL_SPEED_CONTROL_CONFIG_H_

/* Controller units: target/measurement in encoder counts/s, output in permille. */
#define WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS          0.050f
#define WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND   0.500f
#define WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE   80.0f
#define WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE        300
#define WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND  1500.0f
#define WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS           1600.0f
#define WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS               1.0f

/*
 * Extra signed feedforward used only for a zero-linear-speed skid-steer
 * pivot. Four fixed tyres require additional torque to overcome lateral scrub.
 * This is a provisional loaded-ground value and remains subject to calibration.
 */
#define WHEEL_SPEED_CONTROL_PIVOT_FF_BOOST_PERMILLE      80.0f

/* Forward feedforward: output = static + velocity_gain * abs(target_cps). */
#define WHEEL_SPEED_FF_LF_FWD_STATIC_PERMILLE            51.8f
#define WHEEL_SPEED_FF_LF_FWD_GAIN_PER_CPS                0.14600f
#define WHEEL_SPEED_FF_LR_FWD_STATIC_PERMILLE            45.0f
#define WHEEL_SPEED_FF_LR_FWD_GAIN_PER_CPS                0.14934f
#define WHEEL_SPEED_FF_RF_FWD_STATIC_PERMILLE            54.7f
#define WHEEL_SPEED_FF_RF_FWD_GAIN_PER_CPS                0.14703f
#define WHEEL_SPEED_FF_RR_FWD_STATIC_PERMILLE            58.9f
#define WHEEL_SPEED_FF_RR_FWD_GAIN_PER_CPS                0.15779f

/* Reverse feedforward magnitudes; the controller applies the negative sign. */
#define WHEEL_SPEED_FF_LF_REV_STATIC_PERMILLE            46.3f
#define WHEEL_SPEED_FF_LF_REV_GAIN_PER_CPS                0.14677f
#define WHEEL_SPEED_FF_LR_REV_STATIC_PERMILLE            50.4f
#define WHEEL_SPEED_FF_LR_REV_GAIN_PER_CPS                0.14667f
#define WHEEL_SPEED_FF_RF_REV_STATIC_PERMILLE            56.8f
#define WHEEL_SPEED_FF_RF_REV_GAIN_PER_CPS                0.14249f
#define WHEEL_SPEED_FF_RR_REV_STATIC_PERMILLE            56.9f
#define WHEEL_SPEED_FF_RR_REV_GAIN_PER_CPS                0.15108f

#endif /* APPLICATION_CHASSIS_WHEEL_SPEED_CONTROL_CONFIG_H_ */
