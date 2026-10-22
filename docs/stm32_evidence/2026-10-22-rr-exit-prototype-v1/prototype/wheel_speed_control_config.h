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
#define WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND   0.250f
#define WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE   50.0f
#define WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE        300
#define WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND  1000.0f
#define WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS           1600.0f
#define WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS               1.0f

/* Candidate loaded-pivot damping: only same-direction speed above target.
 * Never used for straight/rolling motion, startup or underspeed. */
#define WHEEL_SPEED_CONTROL_PIVOT_OVERSPEED_KP             0.150f

/* Candidate running-load CEILINGS for a pivot, NOT straight.
 * Front/rear load differs in same-wheel/same-direction ground captures.
 * Rear offsets remain zero to retain the tested 0.18.10 behavior.
 * Ramp from zero to full by 160 cps instead of a full step near zero. */
#define WHEEL_SPEED_PIVOT_LOAD_FULL_CPS                    160.0f
#define WHEEL_SPEED_PIVOT_LOAD_LF_FWD_PERMILLE              60.0f
#define WHEEL_SPEED_PIVOT_LOAD_LF_REV_PERMILLE              65.0f
#define WHEEL_SPEED_PIVOT_LOAD_LR_FWD_PERMILLE               0.0f
#define WHEEL_SPEED_PIVOT_LOAD_LR_REV_PERMILLE               0.0f
#define WHEEL_SPEED_PIVOT_LOAD_RF_FWD_PERMILLE              60.0f
#define WHEEL_SPEED_PIVOT_LOAD_RF_REV_PERMILLE              45.0f
#define WHEEL_SPEED_PIVOT_LOAD_RR_FWD_PERMILLE               0.0f
#define WHEEL_SPEED_PIVOT_LOAD_RR_REV_PERMILLE               0.0f

/* Front-only adaptive support. No GPIO, encoder or speed-command changes.
 * Load can be withdrawn on overspeed and restored within its old ceiling
 * during confirmed moving underspeed, without a duplicate positive I step.
 * Recovery BORROWS existing I headroom: signed I + transient <= 50 permille,
 * not an additional full kick on top of the old integral authority.
 * Confirmed moving underspeed/exact target transfers transient -> I at the
 * existing 40 permille/s rebase rate, conserving FF+I. Persistent overspeed
 * spends an error-scaled budget across load then positive I, sharing the
 * aggregate fall budget with transient decay. Gains/effort caps unchanged;
 * LF reverse first-start timing has a separate bounded exception below. */
#define WHEEL_SPEED_FRONT_LOAD_WITHDRAW_PER_SECOND        200.0f
#define WHEEL_SPEED_FRONT_LOAD_RESTORE_PER_SECOND         200.0f
#define WHEEL_SPEED_FRONT_LOAD_REBASE_PER_SECOND           40.0f
#define WHEEL_SPEED_FRONT_LOAD_ERROR_MIN_CPS               20.0f
#define WHEEL_SPEED_FRONT_LOAD_ERROR_RATIO                  0.10f
/* Qualify consecutive same-direction overspeed before withdrawing running
 * load support. P damping/I unwind and recovery termination remain immediate.
 * Valid controller dt is accumulated, not a count of three nominal ticks. */
#define WHEEL_SPEED_FRONT_LOAD_OVERSPEED_CONFIRM_MS          60U
#define WHEEL_SPEED_FRONT_STILL_TARGET_RATIO                0.25f
#define WHEEL_SPEED_FRONT_RECOVERY_CONFIRM_MS              60U
#define WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS               40U
#define WHEEL_SPEED_FRONT_RECOVERY_MAX_MS                 300U
#define WHEEL_SPEED_FRONT_RECOVERY_RISE_PER_SECOND        250.0f
#define WHEEL_SPEED_FRONT_RECOVERY_DECAY_PER_SECOND       400.0f
/* One continuously near-stationary episode has a finite outcome even below
 * the legacy fault-manager 200 cps gate. After the existing one-shot recovery
 * is exhausted, safetyTask latches failure and disables all four outputs.
 * Uses valid controller elapsed time; no force/timeout/speed-command inflation. */
#define WHEEL_SPEED_FRONT_NO_MOTION_LIMIT_MS             1500U

/* Front coordinated support; RF newly adopts LF's 0.18.16 behavior.
 * Individual legacy caps remain. Shared budgets apply to load + transient,
 * not P/error-I, classification transfer, target changes or opposing-I reset. */
/* 0.18.17: RF adopts LF's existing coordination budgets, with no change to
 * any numeric LF parameter. Direction-specific load ceilings stay above. */
#define WHEEL_SPEED_FRONT_COORD_SUPPORT_RISE_PER_SECOND   250.0f
#define WHEEL_SPEED_FRONT_COORD_SUPPORT_FALL_PER_SECOND   400.0f
#define WHEEL_SPEED_FRONT_COORD_LOAD_ERROR_GAIN             1.0f
#define WHEEL_SPEED_FRONT_COORD_LOAD_WITHDRAW_MIN_PER_SECOND 20.0f

/* LF reverse first start only, unchanged effort authority.
 * Valid elapsed time starts at policy entry, without target-change resets.
 * Exit occurs on the first control update reaching the 600 ms timer.
 * Existing 300 ms pulse is never shortened without motion/overspeed. */
#define WHEEL_SPEED_LF_FIRST_START_TOTAL_MS                600U
#define WHEEL_SPEED_LF_FIRST_START_FULL_HOLD_MS            100U

/* LR reverse running recovery, after legacy startup exits.
 * Existing 60-permille rear authority and 50-permille I limit are retained. */
#define WHEEL_SPEED_LR_RUN_MOVING_CONFIRM_MS               40U
#define WHEEL_SPEED_LR_RUN_UNDER_CONFIRM_MS                60U
#define WHEEL_SPEED_LR_RUN_REARM_MS                       100U
#define WHEEL_SPEED_LR_RUN_EPISODE_MS                     600U
#define WHEEL_SPEED_LR_RUN_RISE_PER_SECOND                250.0f
#define WHEEL_SPEED_LR_RUN_FALL_PER_SECOND                400.0f
#define WHEEL_SPEED_LR_RUN_HANDOFF_PER_SECOND              40.0f
#define WHEEL_SPEED_LR_RUN_ERROR_GAIN                       2.0f

/* RR forward running recovery: bounded running episode, not startup.
 * Candidate settings; validate under physical load before acceptance. */
#define WHEEL_SPEED_RR_RUN_MOVING_CONFIRM_MS               40U
#define WHEEL_SPEED_RR_RUN_UNDER_CONFIRM_MS                60U
#define WHEEL_SPEED_RR_RUN_REARM_MS                       100U
#define WHEEL_SPEED_RR_RUN_EPISODE_MS                     600U
#define WHEEL_SPEED_RR_RUN_RISE_PER_SECOND                250.0f
#define WHEEL_SPEED_RR_RUN_FALL_PER_SECOND                400.0f
#define WHEEL_SPEED_RR_RUN_HANDOFF_PER_SECOND              40.0f
#define WHEEL_SPEED_RR_RUN_ERROR_GAIN                       2.0f

/* OFFLINE prototype, supplement recovery EXIT only. Two fresh samples
 * spanning 30..60 ms; production estimator is nominally 20 ms/sample.
 * Faster/slower/invalid samples fall back to the unchanged filtered policy. */
#define WHEEL_SPEED_RR_EXIT_SAMPLE_MIN_MS                   15U
#define WHEEL_SPEED_RR_EXIT_SAMPLE_MAX_MS                   30U
#define WHEEL_SPEED_RR_EXIT_CONFIRM_MS                      40U

/*
 * Dynamic breakaway assistance used only for a zero-linear-speed skid-steer
 * pivot. Hysteresis and a minimum hold prevent threshold chatter; rear
 * RESTARTS use the shorter hold below, not the first-start hold.
 * Rear first-start release transfers load support into the bounded integral.
 * Rear RESTART-only effort instead decays separately after movement; only
 * load support already present before the restart retains the old handoff.
 * This does not increase the output/integral/assist limits below.
 */
#define WHEEL_SPEED_CONTROL_PIVOT_BREAKAWAY_PERMILLE    60.0f
#define WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS        20.0f
#define WHEEL_SPEED_CONTROL_PIVOT_RELEASE_CPS          50.0f
#define WHEEL_SPEED_CONTROL_PIVOT_HOLD_MS              300U

/* Rear wheels only, after motion has already been observed in this pivot:
 * top up existing integral support, then increase only if still stationary.
 * Restore missing force promptly at rest; once motion is seen, hand back
 * to PI without keeping the restart kick active for a fixed 300 ms.
 * Restart effort is withdrawn gradually, not copied into the steady integral.
 * Rear first startup retains the full original kick/hold. Front policy is
 * separate in wheel_speed_controller.c; these timings remain REAR-only. */
#define WHEEL_SPEED_CONTROL_REAR_RESTART_HOLD_MS            40U
#define WHEEL_SPEED_CONTROL_REAR_RESTART_RAISE_DELAY_MS     80U
#define WHEEL_SPEED_CONTROL_REAR_RESTART_RAISE_PERIOD_MS    40U
#define WHEEL_SPEED_CONTROL_REAR_RESTART_RAISE_STEP_PERMILLE 10.0f
#define WHEEL_SPEED_CONTROL_REAR_RESTART_DECAY_PERMILLE_PER_SECOND 200.0f

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
