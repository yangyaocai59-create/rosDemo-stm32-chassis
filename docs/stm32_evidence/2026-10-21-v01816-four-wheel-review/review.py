"""Offline-only review. Reads archived captures; never imports serial or sends commands."""
import hashlib
import json
from pathlib import Path
import struct


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p / "Application").is_dir())
capture = here.parent / "2026-10-21-v01816-ground-pivot" / "capture"
trace = json.loads((capture / "trace_parsed.json").read_text(encoding="utf-8"))
decoded = json.loads((capture / "decoded.json").read_text(encoding="utf-8"))
active = [r for r in trace if r["state"] == 2]
assert len(active) == 127
assert all(b["t"] - a["t"] == 20 for a, b in zip(active, active[1:]))
assert all((r["req_v"], r["applied_v"], r["req_w"], r["applied_w"],
            r["fault"], r["stby"]) == (0, 0, -200, -200, 0, 1) for r in active)
assert all(r["wheels"][w]["target"] == (160 if w in ("LF", "LR") else -160)
           and r["wheels"][w]["pwm"] * r["wheels"][w]["target"] > 0
           for r in active for w in ("LF", "LR", "RF", "RR"))

# Check decoded arrays against archived packet payload bytes, not just printed text.
for key, fmt in (("wheel", "<I4q4i"), ("battery", "<IIIHHHHBBHHH")):
    for row in decoded[key]:
        assert list(struct.unpack(fmt, bytes.fromhex(row["payload_hex"]))) == row["values"]

events = {}
for wheel in ("LF", "LR", "RF", "RR"):
    runs, start = [], None
    for index in range(len(active) + 1):
        zero = index < len(active) and active[index]["wheels"][wheel]["dc"] == 0
        if zero and start is None:
            start = index
        if not zero and start is not None:
            rows = active[start:index]
            runs.append({"first_ms": rows[0]["t"], "last_ms": rows[-1]["t"],
                         "accounted_ms": 20 * len(rows),
                         "left_censored": start == 0, "right_censored": index == len(active),
                         "pwm_first": rows[0]["wheels"][wheel]["pwm"],
                         "pwm_last": rows[-1]["wheels"][wheel]["pwm"]})
            start = None
    events[wheel] = runs

manifest_path = root / "build/firmware/0.18.16/source_manifest.json"
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
mismatches = [name for name, expected in manifest["files"].items()
              if not (root / name).is_file() or sha(root / name) != expected.lower()]
assert not mismatches, mismatches
hex_path = root / "build/firmware/0.18.16/rosDemo.hex"
if not hex_path.exists():
    candidates = list(hex_path.parent.glob("*.hex"))
    assert len(candidates) == 1
    hex_path = candidates[0]
assert sha(hex_path) == "e67a36800caf447839c2a68a5bcdcf8fac18286085b37251844e97954cab2ede"

report = {
    "scope": "archived evidence only; no serial, motion, production edits or flashing",
    "inputs_sha256": {name: sha(capture / name) for name in
                      ("trace_parsed.json", "decoded.json", "ch340.log", "usb_frames.jsonl")},
    "source_manifest_sha256": sha(manifest_path),
    "source_files_checked": len(manifest["files"]), "source_mismatches": mismatches,
    "hex_sha256": sha(hex_path),
    "active_frames": len(active), "active_first_ms": active[0]["t"],
    "active_last_ms": active[-1]["t"], "dt_ms": 20,
    "targets_stby_sign_and_command_checks": "PASS",
    "payload_array_checks": {k: len(decoded[k]) for k in ("wheel", "battery")},
    "zero_count_runs": events,
    "left_overlap_usb": [r["values"] for r in decoded["wheel"]
                         if 271462 <= r["values"][0] <= 271862],
    "battery_tail": [r["values"] for r in decoded["battery"]
                     if 270322 <= r["values"][0] <= 272842],
    "trace_event_details": [r for r in active if
                            271342 <= r["t"] <= 271962 or 272102 <= r["t"] <= 272482],
    "caveats": ["Zero-count duration is frame accounting, not independent visual proof.",
                "USB and UART use the same hardware encoders, not independent sensors.",
                "PWM is software-reported command, not motor-terminal voltage or current.",
                "Battery telemetry is filtered and cannot exclude brief supply dips.",
                "Module identification LM2596 is user-reported; board ratings unknown."],
}
destination = here / "review.json"
destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"report": str(destination), "active_frames": len(active),
                  "source_files_checked": report["source_files_checked"],
                  "source_mismatches": len(mismatches), "hex_sha256": sha(hex_path),
                  "zero_count_runs": events}, ensure_ascii=False, indent=2))
