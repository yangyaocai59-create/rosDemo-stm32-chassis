"""RR production regression against frozen .19 and validated offline prototype.

Native executables only. No ports, GPIO, flash or physical fault clearing.
"""
import hashlib
import json
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[3]
out = root / "build/tests/rr_running"
out.mkdir(parents=True, exist_ok=True)
reference = root / "docs/stm32_evidence/2026-10-22-rr-running-prototype-v1"
frozen = reference / "baseline"
manifest = json.loads((root / "build/firmware/0.18.19/source_manifest.json").read_text())
proof = json.loads((reference / "verification.json").read_text())
def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
def tokens(p):
    return re.sub(r"\s+", "", re.sub(r"/\*.*?\*/|//[^\n]*", "", p.read_text(), flags=re.S))
for n, h in proof["files"].items():
    assert sha(reference / n) == h, n
for p in frozen.iterdir():
    matches = [h for n, h in manifest["files"].items() if Path(n).name == p.name]
    assert len(matches) == 1 and sha(p) == matches[0], p.name
for name in ("chassis_controller.c", "wheel_speed_controller.c", "wheel_speed_controller.h",
             "wheel_speed_control_config.h", "fault_manager.c"):
    folder = "Safety" if name == "fault_manager.c" else "Chassis"
    # .21 pure policy matches the approved exit prototype. Chassis transaction
    # guards are exercised by run_rr_exit_tests.py, not claimed .20 identity.
    if name == "chassis_controller.c":
        assert "s_control_generation" in (root / "Application" / folder / name).read_text()
        continue
    selected = reference / "prototype" if name == "fault_manager.c" else (
        root / "docs/stm32_evidence/2026-10-22-rr-exit-prototype-v1/prototype")
    assert tokens(root / "Application" / folder / name) == tokens(selected / name), name
logs = []
def run(label, command, emit=False):
    result = subprocess.run(list(map(str, command)), cwd=root, capture_output=True,
                            text=True, encoding="utf-8", errors="replace")
    output = result.stdout + result.stderr
    logs.append(label + "\n" + (output if len(output) < 16000 else
        f"{len(output.splitlines())} lines SHA256={hashlib.sha256(output.encode()).hexdigest()}\n"))
    (out / "test_run.log").write_text("\n".join(logs))
    assert result.returncode == 0, (label, output[-6000:])
    assert not re.search(r"\b(?:warning|error):", output, re.I), (label, output)
    if emit:
        print(output, end="", flush=True)
    return output
flags = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
         "-ITools/tests/imu_spi_probe/stubs", "-IApplication/Chassis",
         "-IApplication/Safety", "-IApplication/Services", "-IApplication/Diagnostics"]
runtime = ["Application/Chassis/chassis_runtime.c", "Application/Chassis/chassis_kinematics.c",
           "Application/Chassis/chassis_motion_profile.c", "Application/Diagnostics/motion_trace.c"]
for stem in ("rr_running", "rr_running_safety", "rr_running_fault_link", "rr_running_handoff"):
    manager = ["Application/Safety/fault_manager.c"] if stem.endswith("fault_link") else []
    deps = runtime[:3] if manager else runtime
    run(stem + " build", flags + ["Tools/tests/motion/test_" + stem + ".c"] + deps + manager +
        ["-lm", "-o", out / (stem + ".exe")])
    run(stem, [out / (stem + ".exe")], True)
baseline_flags = flags[:5] + ["-I" + str(frozen)] + flags[5:]
run("original .19 full runtime build", baseline_flags + [frozen / "test_runtime_motion.c",
    frozen / "chassis_controller.c", frozen / "wheel_speed_controller.c"] + runtime +
    ["-lm", "-o", out / "baseline_runtime.exe"])
original = run("original .19 full runtime", [out / "baseline_runtime.exe"])
assert len(re.findall(r"^PASS:", original, re.M)) == 15
print("PASS: frozen .19 complete original runtime 15 groups retained; only two owned-RR legacy waveforms replaced in current tests", flush=True)
for label, extra in (("baseline", ["-DBASELINE"]), ("candidate", [])):
    run(label + " scope build", flags + extra + ["Tools/tests/motion/test_rr_running_isolation.c"] +
        runtime + ["-lm", "-o", out / (label + "_scope.exe")])
for label, args, count in (("non-RR-forward scope", [], 80000), ("first start", ["startup"], 10000)):
    old = run(label + " baseline", [out / "baseline_scope.exe"] + args)
    new = run(label + " candidate", [out / "candidate_scope.exe"] + args)
    assert old == new and len(new.splitlines()) == 2 * count, label
    print(f"PASS: {count} {label} updates and LR private states bit-identical to frozen .19", flush=True)
identity = {}
for label, extra in (("prototype", ["-DREFERENCE"]), ("production", [])):
    run(label + " identity build", flags + extra + ["Tools/tests/motion/test_rr_running_identity.c"] +
        runtime + ["-lm", "-o", out / (label + "_identity.exe")])
    identity[label] = run(label + " identity", [out / (label + "_identity.exe")])
assert identity["prototype"] == identity["production"]
assert len(identity["production"].splitlines()) == 30000
print("PASS: 30000 RR owned running outputs/private states bit-identical to validated prototype", flush=True)
for label, selected_flags, extra in (("baseline", baseline_flags, ["-DBASELINE"]),
                                     ("candidate", flags, [])):
    manager = frozen / "fault_manager.c" if label == "baseline" else root / "Application/Safety/fault_manager.c"
    run(label + " fault build", selected_flags + extra + ["Tools/tests/motion/test_rr_running_fault_audit.c"] +
        runtime[:3] + [manager, "-lm", "-o", out / (label + "_fault.exe")])
grid = []
for base in (0, 0xFFFFFC00):
    for seed in list(range(-50, 51)) + [5.85]:
        pair = {}
        for label in ("baseline", "candidate"):
            output = run(f"{label} fault I={seed} base={base}", [out / (label + "_fault.exe"), seed, base])
            pair[label] = int(re.search(r"RESULT trip_from_stall_ms=(\d+)", output)[1])
        assert pair["candidate"] <= pair["baseline"], (base, seed, pair)
        grid.append(dict(base=base, seed_i=seed, **pair))
assert len(grid) == 204
(out / "fault_grid.json").write_text(json.dumps(grid, indent=2) + "\n")
print("PASS: 204 RR actual runtime/fault-manager audits: no added delay; latch/four-output shutdown/reject/explicit-clear/no-auto-resume, tick wrap", flush=True)
report = dict(result="PASS_OFFLINE_NOT_HARDWARE", new_pass_groups=17,
    unchanged_scope_updates=80000, first_start_updates=10000, running_identity_updates=30000,
    automatic_fault_audits=len(grid), max_added_fault_delay_ms=max(r["candidate"]-r["baseline"] for r in grid),
    candidate_trip_ms=sorted(set(r["candidate"] for r in grid)), baseline_full_runtime_groups=15,
    serial_opened=False, motion_sent=False, fault_cleared_on_hardware=False,
    frozen_baseline_sha256={p.name: sha(p) for p in frozen.iterdir()})
(out / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
