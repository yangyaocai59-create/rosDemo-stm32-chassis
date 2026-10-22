"""OFFLINE only: native mock tests and prescribed-input replay; no hardware."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess

here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p / "Application").is_dir())
proto, baseline = here / "prototype", here / "baseline"
out = root / "build/prototypes/rr_running_v1"
out.mkdir(parents=True, exist_ok=True)
manifest = json.loads((root / "build/firmware/0.18.19/source_manifest.json").read_text())
artifacts = json.loads((root / "build/firmware/0.18.19/verification.json").read_text())
def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
def unchanged():
    assert len(manifest["files"]) == 149
    assert all(sha(root / n) == h for n, h in manifest["files"].items())
    assert all(sha(root / f"build/firmware/0.18.19/rosDemo-0.18.19.{ext}") == h
               for ext, h in artifacts["sha256"].items())
unchanged()
frozen = {}
for p in baseline.iterdir():
    matches = [(n, h) for n, h in manifest["files"].items() if Path(n).name == p.name]
    assert len(matches) == 1 and sha(p) == matches[0][1], p.name
    frozen[matches[0][0]] = sha(p)
assert len(frozen) == 15
changed = sorted(p.name for p in proto.iterdir() if sha(p) != sha(baseline / p.name))
assert set(changed) == {"chassis_controller.c", "wheel_speed_controller.c",
                       "wheel_speed_controller.h", "wheel_speed_control_config.h", "fault_manager.c"}
# All pre-existing configuration defines, including PWM/I/slew caps, remain.
defines = lambda p: re.findall(r"^#define .*?$", p.read_text(), re.M)
assert [d for d in defines(proto / "wheel_speed_control_config.h") if "RR_RUN_" not in d] == defines(
    baseline / "wheel_speed_control_config.h")
compat_source = (here / "runtime_compat.c").read_text()
legacy_gate = ("w==MOTOR_DRIVER_WHEEL_LR &&\n"
               "          WheelSpeedController_IsLrRunningSupportOwned(targets.counts_per_second[w])")
new_gate = ("(w==MOTOR_DRIVER_WHEEL_LR &&\n"
            "          WheelSpeedController_IsLrRunningSupportOwned(targets.counts_per_second[w])) ||\n"
            "          (w==MOTOR_DRIVER_WHEEL_RR &&\n"
            "          WheelSpeedController_IsRrRunningSupportOwned(targets.counts_per_second[w]))")
assert compat_source.count(new_gate) == 2
assert compat_source[compat_source.index("/* Actual runtime/controller"):].replace(new_gate, legacy_gate) == (
    baseline / "test_runtime_motion.c").read_text()
old_manager, new_manager = [(p / "fault_manager.c").read_text() for p in (baseline, proto)]
for predicate in ("legacy_stall_condition", "lr_running_stall_condition"):
    pattern = r"const bool " + predicate + r" =\s*(.*?);"
    assert re.search(pattern, old_manager, re.S)[1] == re.search(pattern, new_manager, re.S)[1]
assert re.findall(r"^#define FAULT_MANAGER_STALL_.*$", old_manager, re.M) == re.findall(
    r"^#define FAULT_MANAGER_STALL_.*$", new_manager, re.M)
logs = []
def run(name, cmd, stdin=None, save=None, diagnostic=None):
    r = subprocess.run(list(map(str, cmd)), cwd=root, input=stdin, capture_output=True,
                       text=True, encoding="utf-8", errors="replace")
    output = r.stdout + r.stderr
    logs.append(name + "\n" + (output if len(output) < 16000 else
                f"{len(r.stdout.splitlines())} stdout lines SHA256={hashlib.sha256(r.stdout.encode()).hexdigest()}\n" + r.stderr[:4000]))
    (here / "test_run.log").write_text("\n".join(logs))
    assert r.returncode == 0, (name, output[-6000:])
    assert not re.search(r"\b(?:warning|error):", output, re.I), (name, output)
    if save:
        Path(save).write_text(r.stdout)
    if diagnostic:
        Path(diagnostic).write_text(r.stderr)
    if "grid" not in name:
        print(name + ": PASS", flush=True)
    return r.stdout
flags = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
         "-ITools/tests/imu_spi_probe/stubs", "-IApplication/Chassis",
         "-IApplication/Safety", "-IApplication/Services", "-IApplication/Diagnostics"]
runtime = ["Application/Chassis/chassis_runtime.c", "Application/Chassis/chassis_kinematics.c",
           "Application/Chassis/chassis_motion_profile.c", "Application/Diagnostics/motion_trace.c"]
def build(name, source, extra=(), manager=False, selected=proto, additional=(), dependencies=None):
    deps = runtime[:3] if manager else runtime
    if dependencies is not None:
        deps = dependencies
    cmd = flags[:5] + ["-I" + str(selected)] + flags[5:] + list(extra) + [source] + list(additional) + deps
    if manager:
        cmd += [selected / "fault_manager.c"]
    run(name + " build", cmd + ["-lm", "-o", out / (name + ".exe")])
def exe(name):
    return out / (name + ".exe")
for name in ("policy", "safety", "handoff"):
    build(name, here / f"test_{name}.c")
    run(name, [exe(name)])
build("fault_link", here / "test_fault_link.c", manager=True)
linkage = run("fault linkage", [exe("fault_link")])
assert len(re.findall(r"^PASS:", linkage, re.M)) == 4
build("front_safety", "Tools/tests/motion/test_front_recovery_safety.c", manager=True,
      additional=[proto / "chassis_controller.c", proto / "wheel_speed_controller.c"])
front_safety = run("unchanged front safety on prototype", [exe("front_safety")])
assert len(re.findall(r"^PASS:", front_safety, re.M)) == 2
for name, source in (("front_pivot", "test_front_pivot.c"),
                     ("front_handoff", "test_front_support_handoff.c"),
                     ("front_load", "test_front_load_confirmation.c")):
    build(name, "Tools/tests/motion/" + source, dependencies=[],
          additional=[proto / "wheel_speed_controller.c"])
    run(name, [exe(name)])
for label, selected, extra in (("baseline", baseline, ["-DBASELINE"]), ("candidate", proto, [])):
    build(label + "_scope", here / "isolation.c", extra, selected=selected)
for name, args, count in (("scope", [], 80000), ("startup", ["startup"], 10000)):
    streams = {label: run(name + " " + label, [exe(label + "_scope")] + args)
               for label in ("baseline", "candidate")}
    assert streams["baseline"] == streams["candidate"], name + " scope regression"
    assert len(streams["candidate"].splitlines()) == 2 * count
    print(f"{name}: {count} updates AND LR private state byte-identical", flush=True)
# Explicitly exercise the unchanged LR-owned running policy, not just startup.
for label, selected, extra in (("baseline", baseline, ["-DBASELINE"]), ("candidate", proto, [])):
    build(label + "_lr", here / "lr_identity.c", extra, selected=selected)
old_lr = run("LR running baseline", [exe("baseline_lr")])
new_lr = run("LR running prototype", [exe("candidate_lr")])
assert old_lr == new_lr and len(new_lr.splitlines()) == 30000
build("runtime_compat", here / "runtime_compat.c",
      additional=[proto / "chassis_controller.c", proto / "wheel_speed_controller.c"])
compat = run("prototype runtime scoped compatibility", [exe("runtime_compat")])
assert len(re.findall(r"^PASS:", compat, re.M)) == 15
# Keep the original harness as evidence: old immediate RR restart waveform
# is intentionally incompatible; do not silently claim it passed.
build("old_runtime", baseline / "test_runtime_motion.c",
      additional=[proto / "chassis_controller.c", proto / "wheel_speed_controller.c"])
r = subprocess.run([str(exe("old_runtime"))], cwd=root, capture_output=True, text=True)
(here / "old_runtime_compatibility.log").write_text(r.stdout + r.stderr)
assert r.returncode != 0
assert "fabsf(initial_bias[w]-expected)<0.001f" in r.stderr + r.stdout
print("Original runtime: expected owned-RR waveform incompatibility retained in log", flush=True)
grid = []
for label, selected, extra in (("baseline", baseline, ["-DBASELINE"]), ("candidate", proto, [])):
    build(label + "_fault", here / "audit_fault.c", extra, manager=True, selected=selected)
for base in (0, 0xFFFFFC00):
    for seed in list(range(-50, 51)) + [5.85]:
        pair = {}
        for label in ("baseline", "candidate"):
            output = run(f"{label} grid I={seed} base={base}", [exe(label + "_fault"), seed, base])
            pair[label] = int(re.search(r"RESULT trip_from_stall_ms=(\d+)", output)[1])
            if base == 0 and seed in (-50, 0, 5.85, 50):
                (here / f"{label}_fault_I{seed}.log").write_text(output)
        assert pair["candidate"] <= pair["baseline"], (base, seed, pair)
        grid.append(dict(base=base, seed_i=seed, **pair))
(here / "fault_grid.json").write_text(json.dumps(grid, indent=2) + "\n")
print(f"Automatic stall grid: {len(grid)} cases PASS; no added delay", flush=True)
capture = here.parent / "2026-10-22-v01819-ground-left/capture/trace_parsed.json"
frames = [f for f in json.loads(capture.read_text()) if f["state"] == 2]
assert len(frames) == 127 and all(f["dt"] == 20 for f in frames)
inputs = "".join(f'{f["t"]} {f["dt"]} ' + " ".join(str(f["wheels"][w][k])
                 for w in ("RR", "RF") for k in ("dc", "cps", "ff_x10", "i_x10", "pwm")) + "\n" for f in frames)
for label, selected, extra in (("baseline", baseline, ["-DBASELINE"]), ("candidate", proto, [])):
    build(label + "_replay", here / "replay.c", extra, selected=selected)
comparisons = {}
for prior in (0, 1):
    for label, shift in (("low", -.49), ("mid", 0), ("high", .49)):
        name = f"{label}_prior_restart_{prior}"
        a = run("baseline replay " + name, [exe("baseline_replay"), shift, prior], inputs,
                here / ("baseline_" + name + ".csv"))
        b = run("prototype replay " + name, [exe("candidate_replay"), shift, prior], inputs,
                here / ("candidate_" + name + ".csv"), here / ("support_" + name + ".csv"))
        assert a == (here.parent / "2026-10-22-v01819-rr-review" / (name + ".csv")).read_text()
        old, new = [[{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO(s))]
                    for s in (a, b)]
        support = [list(map(float, line.split(","))) for line in (here / ("support_" + name + ".csv")).read_text().splitlines()]
        assert len(support) == 127
        assert all(all(x[k] == y[k] for k in x if k.startswith("rf_")) for x, y in zip(old, new))
        first = next(int(t) for t, amount, active, age in support if amount > 0)
        old_first = next(int(row["t"]) for row in old if row["rr_restart_active"])
        first_over = next(i for i, r in enumerate(new) if i > 0 and r["rr_measured"] > r["rr_target"])
        # Check first overspeed following intervention, not a prior excursion.
        first_over = next(i for i, r in enumerate(new) if r["t"] > first and r["rr_measured"] > r["rr_target"])
        assert support[first_over][2] == 0
        assert support[first_over][1] <= max(0, support[first_over-1][1]-8) + .001
        comparisons[name] = dict(kind="PRESCRIBED_FEEDBACK_NOT_PLANT_PREDICTION",
            new_first_support_ms=first, old_first_restart_ms=old_first,
            earlier_ms=old_first-first,
            new_peak_temporary=max(s[1] for s in support),
            old_peak_pwm=max(abs(r["rr_pwm"]) for r in old),
            new_peak_pwm=max(abs(r["rr_pwm"]) for r in new),
            new_peak_positive_I_plus_temporary=max(max(0, r["rr_i"])+s[1] for r, s in zip(new, support)),
            first_overspeed_ms=int(new[first_over]["t"]), RF_all_fields_bit_identical=True)
(here / "recorded_comparison.json").write_text(json.dumps(comparisons, indent=2) + "\n")
baseline_motion = run("unchanged production .19 motion regression", [
    "powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "Tools/tests/run_motion_tests.ps1"])
usb = run("unchanged production .19 USB regression", [
    "powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "Tools/tests/run_usb_tx_tests.ps1"])
assert len(re.findall(r"^PASS:", baseline_motion, re.M)) == 62
assert len(re.findall(r"^PASS:", usb, re.M)) == 20
arm = Path("C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin")
arm_flags = [arm / "arm-none-eabi-gcc.exe", "-mcpu=cortex-m4", "-mthumb",
    "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-std=gnu11", "-O0", "-g3",
    "-Wall", "-Wextra", "-Werror", "-fstack-usage", "-ffunction-sections",
    "-fdata-sections", "-DUSE_HAL_DRIVER", "-DSTM32F407xx",
    "-ICore/Inc", "-IDrivers/STM32F4xx_HAL_Driver/Inc",
    "-IDrivers/CMSIS/Device/ST/STM32F4xx/Include", "-IDrivers/CMSIS/Include",
    "-IMiddlewares/Third_Party/FreeRTOS/Source/include",
    "-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
    "-IApplication/Chassis", "-IApplication/Safety", "-IApplication/Services"]
stack = {}
for label, selected in (("baseline", baseline), ("candidate", proto)):
    for stem in ("wheel_speed_controller", "chassis_controller", "fault_manager"):
        obj = out / (label + "_" + stem + ".o")
        run(label + " " + stem + " ARM OBJECT ONLY",
            arm_flags + ["-c", selected / (stem + ".c"), "-o", obj])
        su = obj.with_suffix(".su")
        (here / su.name).write_bytes(su.read_bytes())
        for line in su.read_text().splitlines():
            fields = line.split("\t")
            stack[label + ":" + fields[0].split(":")[-1]] = int(fields[1])
sizes = run("unlinked ARM objects size NOT firmware",
    [arm / "arm-none-eabi-size.exe"] + [out / (label + "_" + stem + ".o")
     for label in ("baseline", "candidate") for stem in (
        "wheel_speed_controller", "chassis_controller", "fault_manager")])
unchanged()
report = dict(result="OFFLINE_CHECKS_PASSED_NOT_HARDWARE_VALIDATED",
    production_motion_groups=62, production_USB_groups=20,
    candidate_policy_groups=5, candidate_immediate_shutdown_groups=1,
    candidate_fault_boundary_groups=4, candidate_front_safety_groups=2,
    candidate_owner_handoff_groups=2, candidate_runtime_scoped_compatibility_groups=15,
    original_runtime_full_suite_pass=False,
    unchanged_scope_updates=80000, startup_updates=10000, LR_owned_running_updates=30000,
    automatic_fault_grid_cases=len(grid),
    max_added_stall_delay_ms=max(r["candidate"]-r["baseline"] for r in grid),
    candidate_trip_from_stall_ms=sorted(set(r["candidate"] for r in grid)),
    baseline_trip_from_stall_ms=sorted(set(r["baseline"] for r in grid)),
    preserved_legacy_and_LR_stall_predicates_and_thresholds=True,
    recorded_comparisons=comparisons, frozen_baseline=frozen,
    prototype_changed_files=changed, production_source_test_hashes_unchanged=149,
    four_production_artifacts_unchanged=True, firmware_linked=False, hardware_tested=False,
    ARM_objects_only=True, ARM_object_sizes=sizes, ARM_function_static_stack_bytes=stack,
    serial_opened=False, motion_sent=False, flashed=False, production_modified=False,
    files={str(p.relative_to(here)): sha(p) for p in here.rglob("*")
           if p.is_file() and p.name not in ("verification.json", "test_run.log", "README.md")})
(here / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: v for k, v in report.items() if k not in ("files", "frozen_baseline")}, indent=2), flush=True)
