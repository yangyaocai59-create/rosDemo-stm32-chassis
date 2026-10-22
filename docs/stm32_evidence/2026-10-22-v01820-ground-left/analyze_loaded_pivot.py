import json
import pathlib
import re
import statistics
import sys


folder = pathlib.Path(sys.argv[1])
phase = ""
trace = []
current = None
for line in (folder / "ch340.log").read_text(encoding="utf-8").splitlines():
    marker = re.search(r"HOST_PHASE (\w+)", line)
    if marker:
        phase = marker[1]
    match = re.search(r"TRACE t=(\d+) dt=(\d+) req=(-?\d+)/(-?\d+) applied=(-?\d+)/(-?\d+) state=(\d+) stop=(\d+) fault=(\d+) stby=(\d+)", line)
    if match:
        values = list(map(int, match.groups()))
        current = dict(zip(("t", "dt", "req_v", "req_w", "applied_v", "applied_w", "state", "stop", "fault", "stby"), values))
        current.update(phase=phase, wheels={})
        trace.append(current)
    match = re.search(r"TRACE (LF|LR|RF|RR) dc=(-?\d+) raw=(-?\d+) target=(-?\d+) cps=(-?\d+) ff_x10=(-?\d+) p_x10=(-?\d+) i_x10=(-?\d+) pwm=(-?\d+)", line)
    if match and current is not None:
        current["wheels"][match[1]] = dict(zip(("dc", "raw", "target", "cps", "ff_x10", "p_x10", "i_x10", "pwm"), map(int, match.groups()[1:])))

decoded = json.loads((folder / "decoded.json").read_text(encoding="utf-8")) if (folder / "decoded.json").exists() else {}
analysis = {}
summary_path = folder / 'summary.json'
directions = [r['direction'] for r in json.loads(summary_path.read_text(encoding='utf-8')).get('directions', [])] if summary_path.exists() else ['LEFT', 'RIGHT']
for direction in directions:
    all_samples = [s for s in trace if s["phase"] == direction + "_trace"]
    active = [s for s in all_samples if s["state"] == 2 and len(s["wheels"]) == 4]
    info = {"trace_headers": len(all_samples), "complete_active_samples": len(active)}
    if active:
        info.update(time_span_ms=[active[0]["t"], active[-1]["t"]], dt_ms=sorted(set(s["dt"] for s in active)), req_v=sorted(set(s["req_v"] for s in active)), applied_v=sorted(set(s["applied_v"] for s in active)), req_w=sorted(set(s["req_w"] for s in active)), applied_w=sorted(set(s["applied_w"] for s in active)))
        info["wheels"] = {}
        for name in ("LF", "LR", "RF", "RR"):
            values = [s["wheels"][name] for s in active]
            result = {}
            for field in ("target", "cps", "raw", "pwm", "ff_x10", "p_x10", "i_x10"):
                numbers = [v[field] for v in values]
                result[field] = {"mean": round(statistics.mean(numbers), 2), "min": min(numbers), "max": max(numbers)}
            result["near_zero_filtered_samples"] = sum(abs(v["cps"]) <= 20 for v in values)
            result["integrator_limit_samples"] = sum(abs(v["i_x10"]) >= 499 for v in values)
            ff = [v["ff_x10"] for v in values]
            result["ff_changes"] = sum(a != b for a, b in zip(ff, ff[1:]))
            info["wheels"][name] = result
        gaps = [b["t"] - a["t"] for a,b in zip(active, active[1:])]
        info["source_gap_min_max_ms"] = [min(gaps), max(gaps)] if gaps else []
    for key in ("wheel", "status", "battery"):
        records = [r for r in decoded.get(key, []) if r["phase"] == direction + "_motion"]
        if key == "wheel" and records:
            info["usb_wheel_net_counts"] = [records[-1]["values"][i]-records[0]["values"][i] for i in range(1,5)]
            steady = [r for r in records if r["elapsed_s"] >= 2]
            if steady:
                info["usb_steady_cps_mean"] = [round(statistics.mean(r["values"][i] for r in steady), 2) for i in range(5,9)]
        elif key == "status" and records:
            info["status_states"] = sorted(set(r["values"][5] for r in records))
            info["status_flags"] = sorted(set(r["values"][9] for r in records))
            info["status_faults"] = sorted(set((r["values"][6],r["values"][8]) for r in records))
            info["status_command_accepted_rejected_end"] = list(records[-1]["values"][2:4])
        elif key == "battery" and records:
            info["supply_mV_min_max"] = [min(r["values"][6] for r in records), max(r["values"][6] for r in records)]
    analysis[direction] = info

(folder / "trace_parsed.json").write_text(json.dumps(trace, indent=2), encoding="utf-8")
(folder / "analysis.json").write_text(json.dumps(analysis, indent=2), encoding="utf-8")
print(json.dumps(analysis, indent=2))
