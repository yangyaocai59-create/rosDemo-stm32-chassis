"""Offline native stubs + prescribed-input replay. Never opens serial/hardware.
Only writes this evidence directory and build/prototypes/rr_exit_v1.
Does NOT link firmware or build HEX/BIN. Requires production .20 unchanged.
"""
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
out = root / "build/prototypes/rr_exit_v1"
out.mkdir(parents=True, exist_ok=True)
release = root / "build/firmware/0.18.20"
manifest = json.loads((release / "source_manifest.json").read_text())
artifacts = json.loads((release / "verification.json").read_text())["sha256"]
capture = here.parent / "2026-10-22-v01820-ground-left"
sealed = json.loads((capture / "capture_manifest.json").read_text())

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def unchanged():
    assert len(manifest["files"]) == 158
    assert all(sha(root / n) == h for n, h in manifest["files"].items())
    assert all(sha(release / f"rosDemo-0.18.20.{ext}") == h for ext, h in artifacts.items())
    assert all(sha(capture / n) == h for n, h in sealed.items())
    assert not (root / "build/firmware/0.18.21").exists()

unchanged()
for p in baseline.iterdir():
    matches = [(n, h) for n, h in manifest["files"].items() if Path(n).name == p.name]
    assert len(matches) == 1 and sha(p) == matches[0][1], p.name
assert len(list(baseline.iterdir())) == 15
changed = sorted(p.name for p in proto.iterdir() if sha(p) != sha(baseline / p.name))
assert changed == ["chassis_controller.c", "wheel_speed_control_config.h",
                   "wheel_speed_controller.c", "wheel_speed_controller.h"]
defines = lambda p: re.findall(r"^#define .*?$", p.read_text(), re.M)
assert [d for d in defines(proto / "wheel_speed_control_config.h") if "RR_EXIT_" not in d] == defines(
    baseline / "wheel_speed_control_config.h")
assert sha(proto / "fault_manager.c") == sha(baseline / "fault_manager.c")
logs = []

def run(name, cmd, stdin=None, save=None):
    r = subprocess.run(list(map(str, cmd)), cwd=root, input=stdin,
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    text = r.stdout + r.stderr
    logs.append(name + "\nCOMMAND " + repr(list(map(str, cmd))) + "\n" +
                (text if len(text) < 14000 else
                 f"stdout lines={len(r.stdout.splitlines())} sha256={hashlib.sha256(r.stdout.encode()).hexdigest()}\n" + r.stderr))
    (here / "test_run.log").write_text("\n".join(logs), encoding="utf-8")
    assert r.returncode == 0, (name, text[-5000:])
    assert not re.search(r"\b(?:warning|error):", text, re.I), (name, text)
    if save:
        (here / save).write_text(r.stdout, encoding="utf-8")
    if "grid" not in name:
        print(name + ": PASS", flush=True)
    return r.stdout

flags = ["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
         "-ITools/tests/imu_spi_probe/stubs", "-IApplication/Chassis",
         "-IApplication/Safety", "-IApplication/Services", "-IApplication/Diagnostics"]
runtime = ["Application/Chassis/chassis_runtime.c", "Application/Chassis/chassis_kinematics.c",
           "Application/Chassis/chassis_motion_profile.c", "Application/Diagnostics/motion_trace.c"]

def build(name, source, selected=proto, extra=(), deps=None):
    run(name + " build", flags[:5] + ["-I" + str(selected)] + flags[5:] + list(extra) +
        [source] + (runtime if deps is None else deps) + ["-lm", "-o", out / (name + ".exe")])

def exe(name):
    return out / (name + ".exe")

for name in ("fast", "fast_safety"):
    build(name, here / ("test_" + name + ".c"))
    run(name, [exe(name)])

# The original .20 regression is not rewritten to accommodate this change.
build("runtime_compat", here / "runtime_compat.c", deps=runtime +
      [proto / "chassis_controller.c", proto / "wheel_speed_controller.c"])
compat = run("candidate unchanged .20 runtime harness", [exe("runtime_compat")])
assert len(re.findall(r"^PASS:", compat, re.M)) == 15
front_groups = 0
for name in ("test_front_pivot", "test_front_support_handoff", "test_front_load_confirmation"):
    build(name, root / "Tools/tests/motion" / (name + ".c"),
          deps=[proto / "wheel_speed_controller.c"])
    front_groups += len(re.findall(r"^PASS:", run(name, [exe(name)]), re.M))
build("front_safety", root / "Tools/tests/motion/test_front_recovery_safety.c",
      deps=runtime[:3] + [proto / "chassis_controller.c", proto / "wheel_speed_controller.c",
                         proto / "fault_manager.c"])
assert len(re.findall(r"^PASS:", run("candidate unchanged front safety", [exe("front_safety")]), re.M)) == 2

# Negative controls: no fast observations must remain exactly .20 even in
# owned RR running. Observations injected into unrelated wheel updates must
# have no cross-wheel effect. Print exact hex floats, not rounded decimals.
for label, selected, extra in (("baseline", baseline, ["-DBASELINE"]), ("candidate", proto, [])):
    build(label + "_identity", here / "identity.c", selected, extra)
    build(label + "_scope", here / "isolation.c", selected, extra)
build("candidate_observed_scope", here / "isolation.c", extra=["-DINJECT_COUNTS"])
a = run("baseline RR missing-channel identity", [exe("baseline_identity")])
b = run("candidate RR missing-channel identity", [exe("candidate_identity")])
assert a == b and len(b.splitlines()) == 30000
for name, args, updates in (("scope", [], 80000), ("startup", ["startup"], 10000)):
    a = run("baseline " + name, [exe("baseline_scope")] + args)
    b = run("candidate " + name, [exe("candidate_scope")] + args)
    assert a == b and len(b.splitlines()) == 2 * updates
    if name == "scope":
        c = run("candidate observed unrelated scope", [exe("candidate_observed_scope")])
        assert a == c

# Actual unchanged fault manager + runtime. Sweep retained I, normal/tick-wrap
# time, and a distinct phase that proves fast exit actually ran before stall.
for label, selected, extra in (("baseline", baseline, ["-DBASELINE"]), ("candidate", proto, [])):
    build(label + "_fault", here / "audit_fault.c", selected, extra,
          deps=runtime[:3] + [selected / "fault_manager.c"])
grid = []
for phase in (0, 1):
    for base in (0, 0xFFFFFC00):
        for seed in list(range(-50, 51)) + [5.85]:
            pair = {}
            for label in ("baseline", "candidate"):
                text = run(f"{label} grid I={seed} base={base} exitphase={phase}",
                           [exe(label + "_fault"), seed, base, phase])
                pair[label] = int(re.search(r"RESULT trip_from_stall_ms=(\d+)", text)[1])
                if base == 0 and seed in (-50, 0, 5.85, 50):
                    (here / f"{label}_fault_I{seed}_phase{phase}.log").write_text(text)
            assert pair["candidate"] <= pair["baseline"], (phase, base, seed, pair)
            grid.append(dict(exit_phase=phase, base=base, seed_i=seed, **pair))
(here / "fault_grid.json").write_text(json.dumps(grid, indent=2) + "\n")
print(f"Actual automatic stall grid: {len(grid)} paired cases PASS; no added delay", flush=True)

frames = [f for f in json.loads((capture / "capture/trace_parsed.json").read_text()) if f["state"] == 2]
assert len(frames) == 127 and all(f["dt"] == 20 for f in frames)
inputs = "".join(f'{f["t"]} {f["dt"]} ' + " ".join(str(f["wheels"][w][k])
    for w in ("LF", "LR", "RF", "RR") for k in ("dc", "cps", "ff_x10", "i_x10", "pwm")) + "\n" for f in frames)
for label, selected, extra in (("baseline", baseline, ["-DBASELINE"]), ("candidate", proto, [])):
    build(label + "_replay", here / "replay.c", selected, extra)
comparisons = {}
for label, shift in (("low", -.49), ("mid", 0), ("high", .49)):
    streams = {}
    for flavor in ("baseline", "candidate"):
        streams[flavor] = run(flavor + " replay " + label,
            [exe(flavor + "_replay"), shift], inputs, flavor + "_" + label + ".csv")
    assert streams["baseline"] == (here.parent / "2026-10-22-v01820-running-review" /
                                   (label + "_conditional.csv")).read_text()
    old, new = [[{k: float(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO(s))]
                for s in (streams["baseline"], streams["candidate"])]
    assert len(old) == len(new) == 508
    assert all(x == y for x, y in zip(old, new) if x["wheel"] != 3)
    a, b = [[r for r in rows if r["wheel"] == 3] for rows in (old, new)]
    old_exit = next(r for r in a if not r["active"])
    new_exit = next(r for r in b if not r["active"])
    assert new_exit["t"] < old_exit["t"]
    assert all(x["raw"] == y["raw"] and x["measured"] == y["measured"] and
               x["target"] == y["target"] for x, y in zip(a, b))
    assert max(abs(r["pwm"]) for r in b) <= 300
    assert max(max(0, r["i"]) + r["temporary"] for r in b) <= 60.001
    comparisons[label] = dict(kind="PRESCRIBED_FEEDBACK_NOT_PLANT_PREDICTION",
        baseline_exit_ms=old_exit["t"], candidate_exit_ms=new_exit["t"],
        earlier_ms=old_exit["t"]-new_exit["t"],
        baseline_zero_temporary_ms=next(r["t"] for r in a if r["temporary"] == 0),
        candidate_zero_temporary_ms=next(r["t"] for r in b if r["temporary"] == 0),
        baseline_I_at_exit=old_exit["i"], candidate_I_at_exit=new_exit["i"],
        baseline_peak_pwm=max(r["pwm"] for r in a), candidate_peak_pwm=max(r["pwm"] for r in b),
        other_three_wheels_all_exported_fields_identical=True)
(here / "recorded_comparison.json").write_text(json.dumps(comparisons, indent=2) + "\n")

motion = run("UNCHANGED PRODUCTION .20 motion only", ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
             "-File", "Tools/tests/run_motion_tests.ps1"])
usb = run("UNCHANGED PRODUCTION .20 USB only", ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
          "-File", "Tools/tests/run_usb_tx_tests.ps1"])
assert len(re.findall(r"^PASS:", motion, re.M)) == 79
assert len(re.findall(r"^PASS:", usb, re.M)) == 20
# Target syntax/ABI/stack check only. No linker, image, programmer or port.
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
            arm_flags + ["-I" + str(selected), "-c", selected / (stem + ".c"), "-o", obj])
        su = obj.with_suffix(".su")
        (here / su.name).write_bytes(su.read_bytes())
        for line in su.read_text().splitlines():
            fields = line.split("\t")
            stack[label + ":" + fields[0].split(":")[-1]] = int(fields[1])
sizes = run("unlinked ARM objects size NOT FIRMWARE", [arm / "arm-none-eabi-size.exe"] +
    [out / (label + "_" + stem + ".o") for label in ("baseline", "candidate")
     for stem in ("wheel_speed_controller", "chassis_controller", "fault_manager")], save="ARM_object_sizes.txt")
unchanged()
report = dict(result="OFFLINE_ONLY_PASS_NOT_HARDWARE_VALIDATED",
    candidate_new_policy_groups=7, candidate_new_immediate_shutdown_groups=1,
    candidate_unchanged_runtime_groups=15, candidate_front_policy_groups=front_groups,
    candidate_front_safety_groups=2, adversarial_updates=30000,
    unchanged_scope_updates=80000, observed_unrelated_scope_updates=80000,
    unchanged_first_start_updates=10000, missing_channel_RR_identity_updates=30000,
    automatic_stall_paired_cases=len(grid),
    maximum_added_stall_delay_ms=max(r["candidate"]-r["baseline"] for r in grid),
    candidate_stall_delay_ms=sorted(set(r["candidate"] for r in grid)),
    production_only_motion_groups=79, production_only_USB_groups=20,
    prototype_changed_files=changed, fault_manager_byte_identical=True,
    production_158_source_test_hashes_unchanged=True, production_four_artifacts_unchanged=True,
    old_capture_sealed_files_unchanged=len(sealed), recorded_comparisons=comparisons,
    hardware_tested=False, serial_opened=False, motion_sent=False, flashed=False,
    firmware_built=False, production_modified=False,
    ARM_unlinked_objects_only=True, ARM_object_sizes=sizes,
    ARM_function_static_stack_bytes=stack,
    files={str(p.relative_to(here)): sha(p) for p in here.rglob("*") if p.is_file() and
           p.name not in ("verification.json", "README.md", "test_run.log")})
(here / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k:v for k,v in report.items() if k != "files"}, indent=2), flush=True)
