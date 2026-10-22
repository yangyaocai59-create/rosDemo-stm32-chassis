"""Frozen native C diagnosis of recorded input; never hardware or firmware."""
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/"Application").is_dir())
capture=here.parent/"2026-10-22-v01820-ground-left/capture"
manifest=json.loads((root/"build/firmware/0.18.20/source_manifest.json").read_text())
artifacts=json.loads((root/"build/firmware/0.18.20/verification.json").read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def unchanged():
    assert len(manifest["files"])==158
    assert all(sha(root/n)==h for n,h in manifest["files"].items())
    assert all(sha(root/f"build/firmware/0.18.20/rosDemo-0.18.20.{ext}")==h
               for ext,h in artifacts["sha256"].items())
unchanged()
frozen={}
for p in (here/"source_snapshot").iterdir():
    match=[(n,h) for n,h in manifest["files"].items() if Path(n).name==p.name]
    assert len(match)==1 and sha(p)==match[0][1],p.name
    frozen[match[0][0]]=sha(p)
assert len(frozen)==15
frames=[f for f in json.loads((capture/"trace_parsed.json").read_text()) if f["state"]==2]
assert len(frames)==127
assert all(f["dt"]==20 and f["req_w"]==f["applied_w"]==300 and not f["fault"] for f in frames)
assert all(f["stby"]==1 and f["wheels"][w]["target"]==(-240 if w in ("LF","LR") else 240)
           for f in frames for w in ("LF","LR","RF","RR"))
inputs="".join(f'{f["t"]} {f["dt"]} '+" ".join(str(f["wheels"][w][k])
    for w in ("LF","LR","RF","RR") for k in ("dc","cps","ff_x10","i_x10","pwm"))+"\n" for f in frames)
out=root/"build/analysis/v01820_running"
out.mkdir(parents=True,exist_ok=True)
cmd=["gcc","-std=c11","-Wall","-Wextra","-Werror","-I"+str(here/"source_snapshot"),
     "-ITools/tests/imu_spi_probe/stubs","-IApplication/Chassis","-IApplication/Safety",
     "-IApplication/Services","-IApplication/Diagnostics",here/"replay.c",
     "Application/Chassis/chassis_runtime.c","Application/Chassis/chassis_kinematics.c",
     "Application/Chassis/chassis_motion_profile.c","Application/Diagnostics/motion_trace.c",
     "-lm","-o",out/"replay.exe"]
build=subprocess.run(list(map(str,cmd)),cwd=root,capture_output=True,text=True)
(here/"build.log").write_text(build.stdout+build.stderr)
assert build.returncode==0,build.stderr
assert not re.search(r"\b(?:warning|error):",build.stdout+build.stderr,re.I)
def replay(shift=0,episode=200,stop_age=140,reverse=False,prescribed=None):
    r=subprocess.run([str(out/"replay.exe"),str(shift),str(episode),str(stop_age),str(int(reverse))],
        input=inputs if prescribed is None else prescribed,text=True,capture_output=True,cwd=root)
    assert r.returncode==0,r.stderr
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO(r.stdout))]
    assert len(rows)==508
    metrics={}
    for w,name in enumerate(("LF","LR","RF","RR")):
        wheel=[r for r in rows if r["wheel"]==w]
        errs={k:[] for k in ("cps","ff_x10","p_x10","i_x10","pwm")}
        for row,f in zip(wheel[1:],frames[1:]):
            for k in errs:
                alias={"cps":"measured","ff_x10":"ff","p_x10":"p","i_x10":"i","pwm":"pwm"}[k]
                printed=int(row[alias]*(10 if k.endswith("x10") else 1))
                errs[k].append(abs(printed-f["wheels"][name][k]))
        metrics[name]=dict(max_printed_difference={k:max(v) for k,v in errs.items()},
            exact_pwm_rows=sum(v==0 for v in errs["pwm"]),post_seed_rows=126)
    return r.stdout,rows,metrics
seed_metrics={}
for label,shift in (("low",-.49),("mid",0),("high",.49)):
    data,rows,metrics=replay(shift)
    (here/(label+"_conditional.csv")).write_text(data)
    seed_metrics[label]=metrics
    print(label,json.dumps(metrics),flush=True)
data,mid,metrics=replay()
rr=[r for r in mid if r["wheel"]==3]
estimator_cmd=cmd[:cmd.index(here/"replay.c")]+[here/"estimator_replay.c","-lm","-o",out/"estimator.exe"]
estimator_build=subprocess.run(list(map(str,estimator_cmd)),cwd=root,capture_output=True,text=True)
(here/"estimator_build.log").write_text(estimator_build.stdout+estimator_build.stderr)
assert estimator_build.returncode==0,estimator_build.stderr
assert not re.search(r"\b(?:warning|error):",estimator_build.stdout+estimator_build.stderr,re.I)
estimator=subprocess.run([str(out/"estimator.exe")],input=inputs,cwd=root,text=True,capture_output=True)
assert estimator.returncode==0,estimator.stderr
estimator_rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO(estimator.stdout))]
assert len(estimator_rows)==508
assert all(a["t"]==b["t"] and a["wheel"]==b["wheel"] and a["raw"]==b["raw"]
           and a["measured"]==b["measured"] for a,b in zip(estimator_rows,mid))
(here/"real_estimator_conditional.csv").write_text(estimator.stdout)
print("PASS: frozen actual estimator matches reconstructed filter in all 508 wheel samples",flush=True)
# Private initial timer and previous last-motion time are NOT logged. Screen
# sensitivity instead of silently assigning their values as measured facts.
screen=[]
for episode in range(120,601,20):
    for stop_age in range(0,201,20):
        _,rows,m=replay(0,episode,stop_age)
        maximum=max(m["RR"]["max_printed_difference"].values())
        screen.append(dict(episode_seed_ms=episode,stop_age_seed_ms=stop_age,
                           RR_max_printed_difference=maximum,RR_match_within_one_unit=maximum<=1))
matching=[r for r in screen if r["RR_match_within_one_unit"]]
(here/"hidden_seed_screen.json").write_text(json.dumps(screen,indent=2)+"\n")
print("Hidden seed screen",len(screen),"RR matched",len(matching),flush=True)
assert matching
assert all(max(v["max_printed_difference"].values())<=1
           for seed in seed_metrics.values() for v in seed.values())
_,reverse_rows,_=replay(reverse=True)
keyed=lambda rows:{(r["t"],r["wheel"]):r for r in rows}
assert keyed(mid)==keyed(reverse_rows)
# Prescribed RR perturbation only: do not infer a plant response. Test that
# controller state/output for other wheels are not directly overwritten.
perturbed="".join(f'{f["t"]} {f["dt"]} '+" ".join(str(
    f["wheels"][w][k]+(4 if i%4==0 else -1) if w=="RR" and k=="dc" and i>0
    else f["wheels"][w][k]) for w in ("LF","LR","RF","RR")
    for k in ("dc","cps","ff_x10","i_x10","pwm"))+"\n" for i,f in enumerate(frames))
_,perturbed_rows,_=replay(prescribed=perturbed)
assert all(keyed(mid)[k]==r for k,r in keyed(perturbed_rows).items() if k[1]!=3)
assert any(keyed(mid)[k]["pwm"]!=r["pwm"] for k,r in keyed(perturbed_rows).items() if k[1]==3)
print("PASS: update-order identity; RR-only input perturbation leaves LF/LR/RF outputs/states unchanged",flush=True)
first_moving=next(r for r in rr if r["raw"]>0)
margin=max(20,.1*rr[0]["target"])
first_raw_over=next(r for r in rr if r["t"]>=first_moving["t"] and r["raw"]>r["target"]+margin)
first_filtered_over=next(r for r in rr if r["t"]>=first_moving["t"] and r["measured"]>r["target"])
window40=[dict(t=b["t"],cps=(a["raw"]+b["raw"])/2)
          for a,b in zip(rr,rr[1:])]
first_window_over=next(r for r in window40 if r["t"]>=first_moving["t"] and r["cps"]>rr[0]["target"]+margin)
first_two_raw_over=next(b for a,b in zip(rr,rr[1:]) if a["t"]>=first_moving["t"]
    and a["raw"]>a["target"]+margin and b["raw"]>b["target"]+margin)
lag=dict(target_actual_cps=rr[0]["target"],nominal_filter_ms=50,sample_ms=20,
    raw_one_count_quantum_cps=50,window40_one_count_quantum_cps=25,
    first_recovered_count_ms=int(first_moving["t"]),
    first_raw_over_target_plus_margin_ms=int(first_raw_over["t"]),
    first_40ms_count_window_over_target_plus_margin_ms=int(first_window_over["t"]),
    first_two_consecutive_raw_over_margin_ms=int(first_two_raw_over["t"]),
    first_filtered_over_target_ms=int(first_filtered_over["t"]),
    raw_margin_to_filtered_delay_ms=int(first_filtered_over["t"]-first_raw_over["t"]),
    window40_margin_to_filtered_delay_ms=int(first_filtered_over["t"]-first_window_over["t"]),
    raw_two_sample_margin_to_filtered_delay_ms=int(first_filtered_over["t"]-first_two_raw_over["t"]),
    combined_FF_I_at_162642=sum(next(r for r in rr if r["t"]==162642)[k] for k in ("ff","i")),
    combined_FF_I_at_162782=sum(next(r for r in rr if r["t"]==162782)[k] for k in ("ff","i")),
    I_increase_during_handoff_permille=next(r for r in rr if r["t"]==162782)["i"]-
        next(r for r in rr if r["t"]==162722)["i"],
    note="Earlier raw/window detections are diagnostic comparisons, not proposed or validated new control thresholds.")
(here/"filter_recovery_lag.json").write_text(json.dumps(lag,indent=2)+"\n")
unchanged()
report=dict(kind="OFFLINE_DIAGNOSIS_NOT_A_CANDIDATE_OR_PLANT_MODEL",
    all_seed_metrics=seed_metrics,hidden_seed_screen_cases=len(screen),
    RR_matching_hidden_seeds=matching,
    update_order_outputs_private_diagnostic_fields_identical=True,
    actual_estimator_and_manual_reconstruction_identical_wheel_samples=508,
    RR_input_only_change_other_three_wheels_identical=True,
    filter_recovery_lag=lag,
    conditional_event_rows=[r for r in rr if int(r["t"]) in (
        162542,162622,162642,162682,162722,162742,162762,162782,162802,162902)],
    source_snapshot_sha256=frozen,
    input_capture_sha256={p.name:sha(p) for p in capture.iterdir() if p.is_file()},
    production_source_test_items_unchanged=158,four_firmware_artifacts_unchanged=True,
    serial_opened=False,motion_sent=False,production_modified=False,
    firmware_linked=False,flashed=False,
    limitations=["Initial sub-print-resolution values and policy ownership/history are hypotheses, not measured private state.",
                 "Front initial recovery=0 and adaptive load assigned all FF excess; matching is not unique proof.",
                 "RR seed stop age140ms inferred from later forced zero; ring begins inside an existing gap.",
                 "The 275 timer/stop-age screens use a midpoint quantization seed, not all continuous possibilities.",
                 "Prescribed feedback replay cannot predict the wheel response to changing control.",
                 "No physical cause, supply adequacy, or causal direction between wheels is established."])
(here/"analysis.json").write_text(json.dumps(report,indent=2)+"\n")
print("PASS: recorded-input diagnosis complete; production and firmware artifacts unchanged",flush=True)
