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

/* Four independent instances of the same continuous pivot PI law. */
#define WHEEL_SPEED_PIVOT_LF_KP 0.050f
#define WHEEL_SPEED_PIVOT_LF_KI 0.250f
#define WHEEL_SPEED_PIVOT_LR_KP 0.050f
#define WHEEL_SPEED_PIVOT_LR_KI 0.250f
#define WHEEL_SPEED_PIVOT_RF_KP 0.050f
#define WHEEL_SPEED_PIVOT_RF_KI 0.250f
#define WHEEL_SPEED_PIVOT_RR_KP 0.050f
#define WHEEL_SPEED_PIVOT_RR_KI 0.250f

/* Controller units: target/measurement in encoder counts/s, output in permille. */
#define WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS          0.050f
#define WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND   0.250f
#define WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE   50.0f
#define WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE        300
#define WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND  1000.0f
#define WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS           1600.0f
#define WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS               1.0f

/* Fixed pivot load offsets, ramped by target magnitude. These provisional
 * values are not a completed loaded calibration. No adaptive load withdrawal. */
#define WHEEL_SPEED_PIVOT_LOAD_FULL_CPS                    160.0f
#define WHEEL_SPEED_PIVOT_LOAD_LF_FWD_PERMILLE              60.0f
#define WHEEL_SPEED_PIVOT_LOAD_LF_REV_PERMILLE              65.0f
#define WHEEL_SPEED_PIVOT_LOAD_LR_FWD_PERMILLE               0.0f
#define WHEEL_SPEED_PIVOT_LOAD_LR_REV_PERMILLE               0.0f
#define WHEEL_SPEED_PIVOT_LOAD_RF_FWD_PERMILLE              60.0f
#define WHEEL_SPEED_PIVOT_LOAD_RF_REV_PERMILLE              45.0f
#define WHEEL_SPEED_PIVOT_LOAD_RR_FWD_PERMILLE               0.0f
#define WHEEL_SPEED_PIVOT_LOAD_RR_REV_PERMILLE               0.0f

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
