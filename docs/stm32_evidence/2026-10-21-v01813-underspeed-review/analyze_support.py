"""Offline support balance from captured trace; no serial or hardware APIs."""
import hashlib
import json
import math
import pathlib

root = next(p for p in pathlib.Path(__file__).resolve().parents
            if (p / 'Application' / 'Chassis' / 'wheel_speed_controller.c').is_file())
evidence = pathlib.Path(__file__).resolve().parent
capture = evidence.parent / '2026-10-21-v01813-ground-failsafe' / 'repeat_134912'
trace = json.loads((capture / 'trace_parsed.json').read_text(encoding='utf-8'))
active = [r for r in trace if r['state'] == 2]
assert len(active) == 127
assert all(r['dt'] == 20 and r['req_w'] == -200 and r['applied_w'] == -200 for r in active)
target = 0.200 * 0.210 / 2 * 1560 / (math.pi * 0.065)
rf = [r['wheels']['RF'] for r in active]
# Printed cps truncates toward zero. Choose midpoint of first truncated bin,
# then reconstruct the 20/(50+20) filter from exact raw count/period inputs.
# Initial +/-0.5cps uncertainty decays geometrically; this is not extra telemetry.
filtered = [abs(rf[0]['cps']) + 0.5]
for w in rf[1:]:
    filtered.append(filtered[-1] + 20 / 70 * (-w['raw'] - filtered[-1]))
assert all(int(v) == abs(w['cps']) for v, w in zip(filtered, rf))
overspeed_times = [r['t'] for r, v in zip(active[1:], filtered[1:]) if v > target + 20]
assert len(overspeed_times) == 7
# No near-rest in this window. Actual private front state is not in wire trace;
# nominal withdrawal assumes load>=4 permille at each qualified event. Recorded
# combined FF+I changes and the following error integration check that inference.
withdrawal = len(overspeed_times) * 200 * 0.020
nominal_i = sum(target - v for v in filtered[1:]) * 0.250 * 0.020
combined = [-(w['ff_x10'] + w['i_x10']) / 10 for w in rf]
actual_delta = combined[-1] - combined[0]
assert abs(actual_delta - (nominal_i - withdrawal)) < 0.21
per_step_discrepancy = []
for index in range(1, len(active)):
    expected = (target - filtered[index]) * 0.250 * 0.020
    if filtered[index] > target + 20:
        expected -= 4
    per_step_discrepancy.append(abs(combined[index] - combined[index - 1] - expected))
# FF and I are each truncated to 0.1 permille. A difference of two summed
# samples can differ by <0.2; permit a further 0.01 for float/reconstruction.
assert max(per_step_discrepancy) < 0.21
controller = evidence / 'source_snapshot' / 'wheel_speed_controller.c'
config = evidence / 'source_snapshot' / 'wheel_speed_control_config.h'
assert hashlib.sha256(controller.read_bytes()).hexdigest() == 'e073fbc98778d724cd0be0f77a1b53cef943f73976c207ffbfc841f7cc34212e'
assert hashlib.sha256(config.read_bytes()).hexdigest() == '6b00169e2af0ece6bdaee9985968672072dde983d3499b4053c9df70ec788ef9'
qualified_counter = 0
qualified_times = []
for r, v in zip(active, filtered):
    qualified_counter = min(60, qualified_counter + 20) if v > target + 20 else 0
    if qualified_counter >= 60:
        qualified_times.append(r['t'])
result = {
    'kind': 'offline_diagnosis_not_a_motor_model_or_candidate_firmware',
    'capture': str(capture.relative_to(root)),
    'rf_window': {'first_ms': active[0]['t'], 'last_ms': active[-1]['t'], 'samples': len(active), 'dt_ms': 20},
    'actual_kinematic_target_magnitude_cps': target,
    'rf_displayed_cps_mean': sum(abs(w['cps']) for w in rf) / len(rf),
    'qualified_overspeed_times_ms': overspeed_times,
    'nominal_load_withdrawal_permille': withdrawal,
    'normal_I_net_addition_if_unheld_permille': nominal_i,
    'recorded_ff_plus_i_start_end_permille': [combined[0], combined[-1]],
    'recorded_ff_plus_i_net_change_permille': actual_delta,
    'predicted_net_change_permille': nominal_i - withdrawal,
    'max_per_step_balance_discrepancy_permille': max(per_step_discrepancy),
    'not_implemented_60ms_qualification_on_frozen_feedback_times': qualified_times,
    'limits': ['Initial filter state inferred within printed truncation bin; not full-precision MCU readback.',
               'Trace has no separate load/recovery/hold state. Balance is source-and-observation inference, not a direct private-state measurement.',
               'Prescribed feedback and arithmetic do not prove the physical cause of speed excursions or predict behavior after a control change.'],
    'sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in (capture / 'trace_parsed.json', controller, config)},
}
(evidence / 'support_balance.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2))
