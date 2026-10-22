"""Finished-record analysis only. No serial, no control commands."""
import csv
import json
from pathlib import Path

here = Path(__file__).resolve().parent
frames = [f for f in json.loads((here / "capture/trace_parsed.json").read_text())
          if f["state"] == 2 and len(f["wheels"]) == 4]
decoded = json.loads((here / "capture/decoded.json").read_text())
assert len(frames) == 127
groups = {}
for wheel in ("LF", "LR", "RF", "RR"):
    indices = []
    for i, f in enumerate(frames):
        if f["wheels"][wheel]["dc"] == 0:
            if not indices or indices[-1][-1] != i-1:
                indices.append([])
            indices[-1].append(i)
    groups[wheel] = [dict(first_ms=frames[g[0]]["t"], last_ms=frames[g[-1]]["t"],
        covered_intervals_ms=sum(frames[i]["dt"] for i in g),
        pwm_min_max=[min(frames[i]["wheels"][wheel]["pwm"] for i in g),
                     max(frames[i]["wheels"][wheel]["pwm"] for i in g)])
        for g in indices]
rows = []
for f in frames:
    rr = f["wheels"]["RR"]
    # Nominal formula comparison only: private ownership/A is not in UART trace.
    nominal_ff = 58.9 + .15779 * abs(rr["target"])
    row = dict(t_ms=f["t"], RR_nominal_ff=round(nominal_ff, 4),
               RR_ff_excess_approx=round(rr["ff_x10"]/10-nominal_ff, 4))
    row.update({w+"_"+k: f["wheels"][w][k] for w in ("LF", "LR", "RF", "RR")
                for k in ("dc", "target", "cps", "ff_x10", "p_x10", "i_x10", "pwm")})
    rows.append(row)
with (here / "all_wheel_timeline.csv").open("w", newline="") as out:
    writer = csv.DictWriter(out, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
rr_events = []
for g in groups["RR"]:
    start, stop = g["first_ms"], g["last_ms"]
    window = [row for row in rows if start-140 <= row["t_ms"] <= stop+400]
    usb = [dict(t_ms=r["values"][0], RR_count=r["values"][4], RR_cps=r["values"][8])
           for r in decoded["wheel"] if r["phase"] == "LEFT_motion"
           and start-100 <= r["values"][0] <= stop+140]
    rr_events.append(dict(**g, window=window, usb=usb))
usb_plateaus = {}
motion = [r["values"] for r in decoded["wheel"] if r["phase"] == "LEFT_motion"]
for index, name in enumerate(("LF", "LR", "RF", "RR")):
    runs = []
    first = 0
    for i in range(1, len(motion)+1):
        if i == len(motion) or motion[i][1+index] != motion[first][1+index]:
            if i-first >= 2:
                runs.append(dict(first_ms=motion[first][0], last_ms=motion[i-1][0],
                    fixed_net_count=motion[first][1+index],
                    observed_same_count_span_ms=motion[i-1][0]-motion[first][0],
                    samples=i-first))
            first = i
    usb_plateaus[name] = runs
result = dict(kind="RECORDED_GAPS_NOT_PRIVATE_STATE_OR_UNIQUE_CAUSE",
    all_wheel_zero_groups=groups, RR_events=rr_events, USB_same_net_count_runs=usb_plateaus,
    RR_trace_gap_start_truncated=frames[0]["wheels"]["RR"]["dc"] == 0,
    RR_max_FF_I_combined_step_permille=max(abs(
        b["wheels"]["RR"]["ff_x10"]+b["wheels"]["RR"]["i_x10"]-
        a["wheels"]["RR"]["ff_x10"]-a["wheels"]["RR"]["i_x10"])/10
        for a, b in zip(frames, frames[1:])),
    limits=["FF excess is approximate; printed target/FF are quantized and ownership/A are not logged.",
            "Correlated wheel count gaps do not determine causal direction or exclude mechanical/electrical loading.",
            "Continuous nonzero PWM during no counts does not alone identify a controller or encoder fault.",
            "RR zero group starts at first retained trace frame: 140ms is visible coverage, not full gap length.",
            "USB same net count span is not a reconstructed high-frequency encoder waveform."])
(here / "event_review.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({k: v for k, v in result.items() if k != "RR_events"}, indent=2))
for event in rr_events:
    print("RR window: t dc/cps/FF/I/PWM; other raw increments LF/LR/RF")
    for r in event["window"]:
        print(r["t_ms"], *[r["RR_"+k] for k in ("dc", "cps", "ff_x10", "i_x10", "pwm")],
              "|", *[r[w+"_dc"] for w in ("LF", "LR", "RF")])
    print("USB", event["usb"])
