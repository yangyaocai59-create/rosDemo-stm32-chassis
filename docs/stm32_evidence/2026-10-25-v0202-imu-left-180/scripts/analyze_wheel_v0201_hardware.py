"""Analyze saved bounded hardware runs without accessing a device."""
import argparse
import json
import math
from pathlib import Path
import re
import statistics
import struct


def analyze(folder):
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    packets = json.loads((folder / "usb_frames.json").read_text(encoding="utf-8"))
    trace = json.loads((folder / "trace_v2.json").read_text(encoding="utf-8"))[-1]
    text = (folder / "ch340.log").read_text(encoding="utf-8")
    events = [dict(zip(("seq", "t", "kind", "state", "generation", "command"), map(int, m.groups())))
              for m in re.finditer(r"RUNTIME_EVENT seq=(\d+) t=(\d+) kind=(\d+) state=(\d+) generation=(\d+) command=(\d+)", text)]
    events = [e for e in events if e["t"] >= summary["initial_wheel"][0]]
    start = next(e["t"] for e in events if e["kind"] == 1)
    expected_stop_kind = {"explicit": 2, "timeout": 6, "dtr": 3}[summary["stop_mode"]]
    stop = next(e["t"] for e in events if e["kind"] == expected_stop_kind and e["t"] > start)
    active = [f for f in trace["frames"] if f["state"] == 2]
    settled = [f for f in active if f["t"] >= start + (800 if summary["planned_seconds"] >= 3 else 500)]
    if not settled:
        raise ValueError("no trace data after startup exclusion")
    statuses = [struct.unpack("<IIIIHBBBBHhhhhhhIIII", bytes.fromhex(p["payload_hex"])) for p in packets if p["type"] == 129]
    batteries = [struct.unpack("<IIIHHHHBBHHH", bytes.fromhex(p["payload_hex"])) for p in packets if p["type"] == 131]
    wheels = [struct.unpack("<Iqqqqiiii", bytes.fromhex(p["payload_hex"])) for p in packets if p["type"] == 130]
    imu = [struct.unpack("<IIiiiiiiiIIBB", bytes.fromhex(p["payload_hex"])) for p in packets if p["type"] == 132]
    baseline = [struct.unpack("<IIiiiiiiiIIBB", bytes.fromhex(p["payload_hex"]))[7]
                for p in packets if p["type"] == 132 and p["phase"] == "baseline"]
    final = trace["frames"][-1]
    counts = [b-a for a, b in zip(summary["initial_wheel"][1:5], summary["final_wheel"][1:5])]
    out = dict(folder=str(folder), result=summary["result"], setup=summary["setup"],
               firmware_user_report=summary["firmware_user_report"],
               case=summary["case"], stop_mode=summary["stop_mode"],
               boot_banner_verified=summary["boot_banner_verified"],
               mcu_active_span_s=(stop-start)/1000, events_this_run=events,
               host_velocity_span_s=summary.get("motion_command_seconds"),
               startup_exclusion_ms=800 if summary["planned_seconds"] >= 3 else 500,
               command_mm_s_mrad_s=summary["command_mm_s_mrad_s"],
               verified_stop_reason=summary.get("stop_checkpoint_status",summary["final_status"])[7],
               net_counts_including_settling=counts,
               wheel_distance_mm_including_settling=[n*math.pi*65/1560 for n in counts],
               active_trace_samples=len(active), settled_trace_samples=len(settled),
               settled_trace_source_span_ms=[settled[0]["t"], settled[-1]["t"]],
               snapshot_flags=sorted({f["snapshot"]["flags"] for f in trace["frames"]}),
               feedback_age_ms_range=[min((f["t"]-f["snapshot"]["speed_t"]) & 0xffffffff for f in active),
                                      max((f["t"]-f["snapshot"]["speed_t"]) & 0xffffffff for f in active)],
               control_limit_reasons=sorted({r for f in active for r in f["control"]["limits"]}),
               target_scale_x1000=sorted({f["control"]["scale_x1000"] for f in active}),
               pivot_masks=sorted({f["control"]["pivot"] for f in active}),
               vm_mV_range=[min(b[6] for b in batteries), max(b[6] for b in batteries)],
               accepted_delta=statuses[-1][2]-statuses[0][2], rejected_delta=statuses[-1][3]-statuses[0][3],
               device_transport_counter_deltas=[statuses[-1][i]-statuses[0][i] for i in (17,18,19)],
               host_crc_errors=summary["host_crc_errors"],
               fault_pairs=sorted({(s[6],s[8]) for s in statuses}),
               trace_final_stby=final["stby"],
               trace_final_ccr=[final["wheels"][name]["ccr"] for name in ("LF","LR","RF","RR")],
               last_three_wheel_feedback_zero=all(not any(w[5:]) for w in wheels[-3:]),
               final_stopped_verified=summary["final_stopped_verified"], wheels={})
    for name in ("LF", "LR", "RF", "RR"):
        values = [f["wheels"][name] for f in settled]
        sign = 1 if statistics.mean(v["target"] for v in values) >= 0 else -1
        out["wheels"][name] = dict(
            settled_mean_target_cps=statistics.mean(sign*v["target"] for v in values),
            settled_mean_feedback_cps=statistics.mean(sign*v["cps"] for v in values),
            settled_rms_error_cps=math.sqrt(statistics.mean((v["target"]-v["cps"])**2 for v in values)),
            settled_pwm_magnitude_range=[min(abs(v["pwm"]) for v in values),max(abs(v["pwm"]) for v in values)],
            settled_max_integral_magnitude=max(abs(v["i_x10"])/10 for v in values),
            settled_integral_at_50_samples=sum(abs(v["i_x10"])>=499 for v in values),
            settled_zero_net_count_samples=sum(v["dc"]==0 for v in values),
            trace_request_applied_mismatch_samples=sum(f["wheels"][name]["request"]!=f["wheels"][name]["pwm"] for f in active),
            encoder_bad_values=sorted({f["wheels"][name]["encoder"]["bad"] for f in trace["frames"]}))
    out["timing_lines"] = [l for l in text.splitlines() if any(k in l for k in ("CTRL TIMING:", "CTRL EXEC us", "CTRL V2"))]
    diagnostic_rows = [dict(time_ms=int(m[1]), generation=int(m[2]), samples=int(m[3]),
                            feedback_sequence=int(m[4]), age_ms=int(m[5]))
                       for m in re.finditer(r"\[(\d+)\].*?CTRL V2 gen=(\d+) n=(\d+) feedback=(\d+) age=(\d+)", text)]
    active_diagnostics = [r for r in diagnostic_rows if start <= r["time_ms"] < stop]
    out["active_diagnostics"] = active_diagnostics
    out["active_diagnostic_age_ms_range"] = ([min(r["age_ms"] for r in active_diagnostics),
                                               max(r["age_ms"] for r in active_diagnostics)]
                                              if active_diagnostics else None)
    out["diagnostic_age_unsigned_underflow_samples"] = sum(r["age_ms"] >= 0x80000000 for r in diagnostic_rows)
    if summary["case"] in ("left", "right") and baseline:
        bias = statistics.mean(baseline)
        rows = sorted({r[0]: r for r in imu if start <= r[0] <= stop+1000}.values())
        if any(r[11] != 2 or not r[12] & 1 for r in rows):
            raise ValueError("invalid IMU data within motion reference window")
        reference = 0.
        gaps = []
        for a,b in zip(rows,rows[1:]):
            dt = b[0]-a[0]
            if dt <= 0 or dt > 30:
                gaps.append([a[0], b[0]])
            reference += ((a[7]+b[7])/2-bias)*dt/1e6
        encoder = ((counts[2]+counts[3])-(counts[0]+counts[1]))/2*(math.pi*.065/1560)/.395
        out["yaw_reference"] = dict(external_truth_available=False, saved_bias_valid=bool(rows[0][12]&2),
            baseline_samples=len(baseline),baseline_z_mrad_s=bias,
            baseline_z_std_mrad_s=statistics.pstdev(baseline), imu_gaps_over_30ms=gaps,
            sample_window_ms=[rows[0][0],rows[-1][0]],
            imu_reference_deg=math.degrees(reference),encoder_with_track395mm_deg=math.degrees(encoder),
            command_source_active_integral_deg=(None if summary.get("target_imu_deg") else
                math.degrees(summary["command_mm_s_mrad_s"][1]*(stop-start)/1e6)))
    out["checks"] = dict(
        feedback_age_within_40ms=out["feedback_age_ms_range"][1]<=40,
        transport_clean=not any(out["device_transport_counter_deltas"]) and not out["host_crc_errors"] and not out["rejected_delta"],
        no_fault=out["fault_pairs"]==[(0,0)],
        trace_output_disabled=out["trace_final_stby"]==0 and not any(out["trace_final_ccr"]),
        encoder_config_clean=all(w["encoder_bad_values"]==[0] for w in out["wheels"].values()),
        final_stopped=out["final_stopped_verified"] and out["last_three_wheel_feedback_zero"])
    if summary["firmware_user_report"] == "0.20.2":
        out["checks"]["diagnostic_age_no_unsigned_underflow"] = out["diagnostic_age_unsigned_underflow_samples"] == 0
        out["checks"]["active_diagnostic_age_within_40ms"] = bool(active_diagnostics) and all(r["age_ms"] <= 40 for r in active_diagnostics)
    (folder/"analysis.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf-8")
    return out


if __name__ == "__main__":
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("folders",nargs="+",type=Path)
    args=ap.parse_args()
    for folder in args.folders:
        result=analyze(folder)
        print(json.dumps(result,indent=2))
