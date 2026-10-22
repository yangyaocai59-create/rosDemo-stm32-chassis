import json
import math
import pathlib
import statistics
import sys


def metrics(folder):
    frames = json.loads((folder / "trace_parsed.json").read_text(encoding="utf-8"))
    decoded = json.loads((folder / "decoded.json").read_text(encoding="utf-8"))
    results = {}
    summary_path = folder / 'summary.json'
    directions = [r['direction'] for r in json.loads(summary_path.read_text(encoding='utf-8')).get('directions', [])] if summary_path.exists() else ['LEFT', 'RIGHT']
    for direction in directions:
        active = [f for f in frames if f["phase"] == direction + "_trace" and f["state"] == 2 and len(f["wheels"]) == 4]
        info = {"active_trace_samples": len(active), "wheel": {}}
        for wheel in ("LF", "LR", "RF", "RR"):
            values = [f["wheels"][wheel] for f in active]
            if not values:
                continue
            errors = [v["cps"] - v["target"] for v in values]
            raw_zero = [v["dc"] == 0 for v in values]
            longest = run = 0
            for zero in raw_zero:
                run = run + 1 if zero else 0
                longest = max(longest, run)
            ff_i = [(v["ff_x10"] + v["i_x10"]) / 10 for v in values]
            ff_release = []
            for previous, current in zip(values, values[1:]):
                if abs(current["ff_x10"]) < abs(previous["ff_x10"]):
                    ff_release.append(abs(current["ff_x10"] + current["i_x10"] - previous["ff_x10"] - previous["i_x10"]) / 10)
            info["wheel"][wheel] = {
                "mean_cps": round(statistics.mean(v["cps"] for v in values), 2),
                "rms_error_cps": round(math.sqrt(statistics.mean(e * e for e in errors)), 2),
                "raw_zero_percent": round(statistics.mean(raw_zero) * 100, 2),
                "max_consecutive_zero_interval_ms": longest * 20,
                "mean_abs_pwm_permille": round(statistics.mean(abs(v["pwm"]) for v in values), 2),
                "max_ff_plus_i_step_permille": round(max((abs(b-a) for a,b in zip(ff_i,ff_i[1:])),default=0), 2),
                "max_ff_plus_i_step_when_ff_reduces_permille": round(max(ff_release, default=0), 2),
            }
        records = [r for r in decoded["wheel"] if r["phase"] == direction + "_motion"]
        if records:
            info["net_counts"] = [records[-1]["values"][i] - records[0]["values"][i] for i in range(1,5)]
        results[direction] = info
    return results


baseline, candidate = map(pathlib.Path, sys.argv[1:3])
report = {"baseline_folder": str(baseline), "candidate_folder": str(candidate), "baseline": metrics(baseline), "candidate": metrics(candidate)}
(candidate / "baseline_comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))
