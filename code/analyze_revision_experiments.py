"""Recompute E3/E2 evidence from complete paired MATLAB seed checkpoints.

Run with Python, NumPy and SciPy. No simulation or estimator is called here.
All fault labels are reconstructed offline, after the online runs finish.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from collections import defaultdict

DEPENDENCIES = Path(os.environ.get("REVISION_PYTHON_DEPS",
    "D:/codex_agent/工具/2026-10-03_E3_E2_revision/python_packages"))
if DEPENDENCIES.is_dir():
    sys.path.insert(0, str(DEPENDENCIES))
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT = ROOT / "结果" / "2026-10-03_E3_E2_revision"
METHODS = ("ekf", "fgo", "cusum_ekf", "cusum_fgo")
METRICS = ("Maximum", "Mean", "CDF95", "RMSE3D")
E3_SCENARIOS = ("baseline_3f", "healthy_maneuver")
E2_SCENARIOS = ("baseline_3f", "size_2f", "size_5f", "nlos_weak", "nlos_strong")


def records(value):
    if isinstance(value, dict):
        return [value]
    return list(value) if np.size(value) else []


def pairs(value):
    return np.asarray(value, dtype=int).reshape(-1, 2)


def digest_array(value):
    value = np.asarray(value)
    h = hashlib.sha256()
    h.update(str(value.shape).encode())
    h.update(str(value.dtype).encode())
    h.update(value.tobytes(order="C"))
    return h.hexdigest()


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8-sig")
        return
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ("" if isinstance(v, (float, np.floating)) and not math.isfinite(v)
                                    else v) for k, v in row.items()})


def position_metrics(error):
    error = np.asarray(error, dtype=float)
    assert error.ndim == 3 and error.shape[1] == 3 and np.isfinite(error).all()
    norm = np.linalg.norm(error, axis=1)
    values = np.stack([np.max(norm, axis=0), np.mean(norm, axis=0),
        np.quantile(norm, .95, axis=0, method="hazen"), np.sqrt(np.mean(norm**2, axis=0))], axis=1)
    pooled = np.array([norm.max(), norm.mean(), np.quantile(norm, .95, method="hazen"),
                       np.sqrt(np.mean(norm**2))])
    rows = []
    for f, vector in enumerate(values, 1):
        rows.append(dict(Aggregation="follower", Follower=f, Samples=norm.shape[0],
                         **dict(zip(METRICS, map(float, vector)))))
    for aggregation, vector in (("pooled", pooled), ("macro", values.mean(axis=0))):
        rows.append(dict(Aggregation=aggregation, Follower=0, Samples=norm.size,
                         **dict(zip(METRICS, map(float, vector)))))
    return rows


def rising(state, observed=None):
    state = np.asarray(state, dtype=bool)
    previous = np.r_[False, state[:-1]]
    result = state & ~previous
    if observed is not None:
        observed = np.asarray(observed, dtype=bool)
        result &= observed & np.r_[True, observed[:-1]]
    return result


def event_metrics(times, start, end, threshold, alarm, isolated, observed,
                  next_start=math.inf, recovery_horizon=30, stable_epochs=3):
    """One edge/event; no zero-filling of missed, pre-existing or censored outcomes."""
    times = np.asarray(times, dtype=float)
    active = (times >= start) & (times <= end)
    previous = np.flatnonzero(times < start)
    out = {"FaultStart": float(start), "FaultEnd": float(end),
           "FaultObservedEpochs": int(np.sum(active & observed)),
           "FaultEpochs": int(np.sum(active))}
    for label, state in (("Detection", threshold), ("Alarm", alarm), ("Isolation", isolated)):
        pre = bool(len(previous) and state[previous[-1]] and observed[previous[-1]])
        candidates = np.flatnonzero(active & rising(state, observed))
        status = "preexisting" if pre else ("detected" if len(candidates) else "missed")
        if not np.any(active & observed):
            status = "unobservable"
        out[label + "Status"] = status
        out[label + "Time"] = float(times[candidates[0]]) if status == "detected" else math.nan
        out[label + "Delay"] = out[label + "Time"] - start
    out["IsolatedDuringFault"] = bool(np.any(active & observed & isolated))
    at_end = np.flatnonzero(times <= end)
    out["IsolatedAtFaultEnd"] = bool(len(at_end) and isolated[at_end[-1]])
    out.update(RecoveryStatus="not_applicable", RecoveryTime=math.nan,
               RecoveryDelay=math.nan, RecoveryConfirmTime=math.nan, CensorReason="",
               ReisolationAfterRecovery=False)
    if not out["IsolatedDuringFault"]:
        return out
    horizon_end = end + recovery_horizon
    indices = np.flatnonzero((times > end) & (times <= horizon_end) & (times < next_start))
    consecutive = 0
    missing = None
    recovered_index = None
    for k in indices:
        if not observed[k]:
            missing = k
            break
        if not isolated[k]:
            consecutive += 1
        else:
            consecutive = 0
        if consecutive == stable_epochs:
            recovered_index = k - stable_epochs + 1
            out.update(RecoveryStatus="recovered", RecoveryTime=float(times[recovered_index]),
                       RecoveryDelay=float(times[recovered_index] - end),
                       RecoveryConfirmTime=float(times[k]))
            rest = (times > times[k]) & (times <= horizon_end) & (times < next_start) & observed
            out["ReisolationAfterRecovery"] = bool(np.any(rest & isolated))
            break
    if recovered_index is None:
        if missing is not None:
            out.update(RecoveryStatus="censored", CensorReason="edge_unobservable")
        elif next_start <= horizon_end:
            out.update(RecoveryStatus="censored", CensorReason="next_same_edge_fault")
        elif times[-1] < horizon_end:
            out.update(RecoveryStatus="censored", CensorReason="run_ended")
        else:
            out["RecoveryStatus"] = "failed"
    return out


def build_trace(payload, method):
    history = records(payload["methods"][method]["history"])
    F = int(payload["cfg"]["uav_num"] - payload["cfg"]["high_num"])
    edge_pairs = sorted({tuple(map(int, pair)) for h in history for pair in pairs(h["pairs"])})
    edge_index = {pair: j for j, pair in enumerate(edge_pairs)}
    shape = (len(history), len(edge_pairs))
    trace = {"time": np.asarray(payload["range_time"], dtype=float),
             "pairs": np.asarray(edge_pairs, dtype=int), "followers": np.array(F)}
    fields = {"z": "signed_normalized_innovation", "cplus": "cusum_positive",
        "cminus": "cusum_negative", "cusum_used": "cusum_value", "weight": "final_weight",
        "prediction": "predicted_range", "measurement": "measurement",
        "innovation": "innovation", "confirm_count": "confirm_count", "release_count": "release_count",
        "rate_before": "predictor_rate_before", "rate_after": "predictor_rate_after",
        "predictor_after": "predictor_value_after"}
    bool_fields = {"alarm": "alarm_active", "ready": "baseline_ready",
                   "frozen": "predictor_frozen", "level_corrected": "predictor_level_corrected"}
    for key in [*fields, "true_range"]:
        trace[key] = np.full(shape, np.nan)
    for key in [*bool_fields, "observed", "admitted", "fault"]:
        trace[key] = np.zeros(shape, dtype=bool)
    trace["fault_bias"] = np.zeros(shape)
    for k, h in enumerate(history):
        local_pairs = pairs(h["pairs"])
        js = np.array([edge_index[tuple(p)] for p in local_pairs], dtype=int)
        trace["observed"][k, js] = True
        admitted = {tuple(p) for p in pairs(h["admitted_range_pairs"])}
        trace["admitted"][k, js] = [tuple(p) in admitted for p in local_pairs]
        detail = h["detail"]
        for key, original in fields.items():
            if original in detail:
                trace[key][k, js] = np.atleast_1d(detail[original])
        for key, original in bool_fields.items():
            if original in detail:
                trace[key][k, js] = np.atleast_1d(detail[original]).astype(bool)
        trace["true_range"][k, js] = [h["true_range"][i-1, j-1] for i, j in local_pairs]
    for segment in records(payload["fault_segments"]):
        j = edge_index[tuple(map(int, segment["edge"]))]
        active = (trace["time"] >= segment["start"]) & (trace["time"] <= segment["end"])
        assert not np.any(trace["fault"][active, j]), "Overlapping same-edge faults need explicit matching"
        trace["fault"][active, j] = True
        trace["fault_bias"][active, j] = float(segment["bias"])
    trace["isolated"] = trace["observed"] & ~trace["admitted"]
    trace["threshold"] = np.maximum(trace["cplus"], trace["cminus"]) >= 5
    trace["range_rate"] = np.gradient(trace["true_range"], trace["time"], axis=0)
    trace["range_acceleration"] = np.gradient(trace["range_rate"], trace["time"], axis=0)
    window = int(payload["cfg"]["sliding_window_length"]) if method == "cusum_fgo" else 1
    for key, values in (("retained_factors", trace["admitted"]),
                        ("retained_fault_factors", trace["admitted"] & trace["fault"])):
        cumulative = np.vstack([np.zeros((1, shape[1])), np.cumsum(values, axis=0)])
        trace[key] = cumulative[1:] - cumulative[np.maximum(0, np.arange(shape[0])+1-window)]
    return trace


def trace_statistics(payload, method, trace):
    seed = int(payload["seed"]); scenario = payload["scenario"]
    prefix = {"Scenario": scenario, "Seed": seed, "Method": method}
    time = trace["time"]; edge_pairs = trace["pairs"]; F = int(trace["followers"])
    segments = records(payload["fault_segments"])
    event_rows = []
    for index, seg in enumerate(segments, 1):
        pair = tuple(map(int, seg["edge"]))
        j = next(j for j, p in enumerate(edge_pairs) if tuple(p) == pair)
        later = [float(s["start"]) for s in segments
                 if tuple(map(int, s["edge"])) == pair and s["start"] > seg["end"]]
        row = event_metrics(time, seg["start"], seg["end"], trace["threshold"][:, j],
            trace["alarm"][:, j], trace["isolated"][:, j], trace["observed"][:, j],
            min(later, default=math.inf))
        row.update(prefix, Event=index, Follower=pair[0], Leader=pair[1]-F, Bias=float(seg["bias"]))
        event_rows.append(row)
    phases = {"all": time >= 16}
    if scenario == "healthy_maneuver":
        phases.update(pre=(time >= 16) & (time < 260), maneuver=(time >= 260) & (time <= 460),
                      post=time > 460)
    health_rows = []
    for scope, edge_mask in (("leader", edge_pairs[:, 1] > F), ("follower", edge_pairs[:, 1] <= F)):
        for phase, time_mask in phases.items():
            eligible = trace["observed"] & ~trace["fault"] & time_mask[:, None] & edge_mask[None, :]
            row = dict(prefix, EdgeScope=scope, Phase=phase, HealthyEdgeEpochs=int(eligible.sum()),
                       HealthyEdgeSeconds=float(eligible.sum()),
                       UnreadyHealthyEdgeEpochs=int(np.sum(eligible & ~trace["ready"])))
            for label, key in (("Alarm", "alarm"), ("Isolation", "isolated")):
                count = 0; residual = 0
                for j in np.flatnonzero(edge_mask):
                    state = trace[key][:, j]; observed = trace["observed"][:, j]
                    starts = rising(state, observed)
                    count += int(np.sum(starts & eligible[:, j]))
                    origin_fault = False
                    for k in range(len(time)):
                        if starts[k]:
                            origin_fault = bool(trace["fault"][k, j])
                        if not state[k]:
                            origin_fault = False
                        if eligible[k, j] and state[k] and origin_fault:
                            residual += 1
                occupied = int(np.sum(eligible & trace[key]))
                denominator = int(eligible.sum())
                row.update({label+"NewHealthyEpisodes": count, label+"OccupiedHealthyEpochs": occupied,
                    label+"ResidualFromFaultEpochs": residual,
                    label+"HealthyOccupancy": occupied/denominator if denominator else math.nan,
                    label+"EpisodesPerEdgeHour": count/(denominator/3600) if denominator else math.nan})
            health_rows.append(row)
    return event_rows, health_rows


def geometry_rows(payload):
    F = int(payload["cfg"]["uav_num"]-payload["cfg"]["high_num"])
    indices = np.rint(np.asarray(payload["range_time"])/float(payload["cfg"]["dt"])).astype(int)
    truth = np.asarray(payload["truth_xyz"])[indices]
    leaders = np.asarray(payload["leader_truth_xyz"])[indices]
    reference = payload["methods"][str(np.atleast_1d(payload["method_names"])[0])]
    history = records(reference["history"])
    rows = []
    for k, time in enumerate(payload["range_time"]):
        adjacency = np.eye(F+3, dtype=bool)
        for i, j in pairs(history[k]["pairs"]):
            adjacency[i-1, j-1] = adjacency[j-1, i-1] = True
        visited = {0}; frontier = [0]
        while frontier:
            for node in np.flatnonzero(adjacency[frontier.pop()]):
                if int(node) not in visited:
                    visited.add(int(node)); frontier.append(int(node))
        for f in range(F):
            delta = (leaders[k]-truth[k, :, f, None]).T
            distance = np.linalg.norm(delta, axis=1)
            valid = distance <= float(payload["cfg"]["communication_range"])
            H = delta[valid]/distance[valid, None]
            rank = int(np.linalg.matrix_rank(H)) if len(H) else 0
            condition = float(np.linalg.cond(H)) if rank == 3 else math.inf
            rows.append(dict(Scenario=payload["scenario"], Time=float(time), Follower=f+1,
                SourceColumn=int(np.atleast_1d(payload["cfg"]["revision_follower_source_indices"])[f]),
                LeaderEdges=int(valid.sum()), AllFollowerEdges=int(adjacency[f].sum()-1),
                TotalRangeEdges=int(len(pairs(history[k]["pairs"]))),
                Rank=rank, Condition=condition, MinRange=float(distance.min()),
                MaxRange=float(distance.max()), Connected=len(visited)==F+3))
    return rows


def analyze_one(path, output):
    payload = loadmat(path, simplify_cells=True)["payload"]
    assert bool(payload["complete"])
    scenario = str(payload["scenario"]); seed = int(payload["seed"])
    cfg = payload["cfg"]
    assert cfg["ekf_cusum_alarm_on_threshold"] == 5 and cfg["ekf_cusum_kappa"] == .5
    assert not cfg["imu_preintegration_enable"]
    expected_methods = ("cusum_ekf", "cusum_fgo") if scenario == "healthy_maneuver" else METHODS
    assert set(payload["methods"]) == set(expected_methods)
    metric_rows = []; event_rows = []; health_rows = []
    reference = payload["methods"][expected_methods[0]]
    for name in expected_methods:
        result = payload["methods"][name]
        assert np.array_equal(result["range_error"], reference["range_error"], equal_nan=True)
        history = records(result["history"])
        for h, ref in zip(history, records(reference["history"])):
            assert np.array_equal(h["measured_range"], ref["measured_range"], equal_nan=True)
            assert np.array_equal(h["graph_prior_positions"], ref["graph_prior_positions"])
        computed = position_metrics(result["error_xyz"])
        assert np.allclose([r["RMSE3D"] for r in computed if r["Aggregation"]=="follower"],
            np.atleast_1d(result["metrics"]["full_rmse_3d"]),rtol=1e-12,atol=1e-12)
        for row in computed:
            metric_rows.append(dict(Scenario=scenario, Seed=seed, Method=name, **row))
        if name.startswith("cusum"):
            trace = build_trace(payload, name)
            events, health = trace_statistics(payload, name, trace)
            event_rows.extend(events); health_rows.extend(health)
            trace_path = output/"derived"/scenario/f"seed_{seed:04d}_{name}_trace.npz"
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(trace_path, **trace)
    cache = payload["input_cache"]
    signature = {key: digest_array(cache[key]) for key in ("imu_gyro_b", "imu_gyro_r", "imu_gyro_wg",
                 "imu_acc_r", "gps_low_standard", "gps_high_standard", "range_standard")}
    signature["truth_xyz"] = digest_array(payload["truth_xyz"])
    signature["leader_truth_xyz"] = digest_array(payload["leader_truth_xyz"])
    schedules = [dict(Scenario=scenario, Seed=seed, Event=j, Start=float(s["start"]),
        End=float(s["end"]), Follower=int(s["edge"][0]), Leader=int(s["edge"][1])-int(cfg["uav_num"]-3),
        Bias=float(np.atleast_1d(s["bias_samples"])[0]),
        DrawIndex=int(np.atleast_1d(cfg["revision_nlos_draw_indices"])[j-1]))
        for j, s in enumerate(records(payload["fault_segments"]), 1)]
    assert all(np.ptp(np.atleast_1d(s["bias_samples"])) == 0 for s in records(payload["fault_segments"]))
    geometry = geometry_rows(payload) if seed == 1 else []
    return dict(metrics=metric_rows, events=event_rows, health=health_rows, signatures=signature,
                schedules=schedules, geometry=geometry, scenario=scenario, seed=seed,
                elapsed_seconds=float(payload["elapsed_seconds"]))


def grouped_summary(rows, keys, values):
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[k] for k in keys)].append(row)
    result = []
    for key, group in sorted(groups.items()):
        row = dict(zip(keys, key))
        for value in values:
            numbers = np.array([x[value] for x in group], dtype=float)
            valid = numbers[np.isfinite(numbers)]
            row[value+"_N"] = int(len(valid))
            row[value+"_Mean"] = float(valid.mean()) if len(valid) else math.nan
            row[value+"_SD"] = float(valid.std(ddof=1)) if len(valid)>1 else math.nan
        result.append(row)
    return result


def event_seed_summary(events, health):
    # Include seeds with no fault events as n/a, not zero-latency successes.
    keys = sorted({(r["Scenario"], r["Seed"], r["Method"]) for r in health})
    output = []
    for scenario, seed, method in keys:
        group = [e for e in events if (e["Scenario"],e["Seed"],e["Method"]) == (scenario,seed,method)]
        row = dict(Scenario=scenario, Seed=seed, Method=method, Events=len(group))
        for label in ("Detection", "Alarm", "Isolation"):
            for status in ("detected", "missed", "preexisting", "unobservable"):
                row[label+"_"+status] = sum(e[label+"Status"]==status for e in group)
            delays = [e[label+"Delay"] for e in group if math.isfinite(e[label+"Delay"])]
            row[label+"Delay"] = float(np.mean(delays)) if delays else math.nan
        for status in ("recovered", "failed", "censored", "not_applicable"):
            row["Recovery_"+status] = sum(e["RecoveryStatus"]==status for e in group)
        denominator = row["Recovery_recovered"]+row["Recovery_failed"]
        row["RecoveryFailureRate"] = row["Recovery_failed"]/denominator if denominator else math.nan
        delays = [e["RecoveryDelay"] for e in group if math.isfinite(e["RecoveryDelay"])]
        row["RecoveryDelay"] = float(np.mean(delays)) if delays else math.nan
        row["ReisolatedAfterRecovery"] = sum(e["ReisolationAfterRecovery"] for e in group)
        output.append(row)
    return output


def paired_position_rows(metrics):
    lookup = {(r["Scenario"], r["Seed"], r["Aggregation"], r["Follower"], r["Method"]): r for r in metrics}
    rows = []
    for r in metrics:
        if r["Method"] != "cusum_fgo":
            continue
        for baseline in ("ekf", "fgo", "cusum_ekf"):
            key = (r["Scenario"],r["Seed"],r["Aggregation"],r["Follower"],baseline)
            if key in lookup:
                b = lookup[key]
                row = {k:r[k] for k in ("Scenario","Seed","Aggregation","Follower")}
                row["Comparison"] = "cusum_fgo_minus_"+baseline
                for metric in METRICS:
                    row[metric+"Difference"] = r[metric]-b[metric]
                rows.append(row)
    return rows


def run_analysis(output, stage, partial=False):
    scenarios = E3_SCENARIOS if stage == "E3" else E2_SCENARIOS
    fingerprint = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    results = []; missing = []
    for scenario in scenarios:
        for seed in range(1,51):
            path = output/"raw"/scenario/f"seed_{seed:04d}.mat"
            if not path.exists():
                missing.append(dict(Scenario=scenario, Seed=seed)); continue
            derived = output/"derived"/scenario/f"seed_{seed:04d}_statistics.json"
            if derived.exists():
                cached = json.loads(derived.read_text(encoding="utf-8"))
                if cached.get("analyzer_sha256")==fingerprint and cached.get("raw_size")==path.stat().st_size:
                    results.append(cached["data"]); continue
            value = analyze_one(path,output)
            derived.parent.mkdir(parents=True,exist_ok=True)
            derived.write_text(json.dumps(dict(analyzer_sha256=fingerprint,raw_size=path.stat().st_size,
                raw_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),data=value),ensure_ascii=False),encoding="utf-8")
            results.append(value)
            print(f"ANALYZED {scenario} seed={seed}",flush=True)
    tables = output/"tables"/stage
    merged = {key:[row for r in results for row in r[key]] for key in ("metrics","events","health","schedules","geometry")}
    for key, rows in merged.items():
        write_csv(tables/f"{key}_per_seed.csv",rows)
    summary = grouped_summary(merged["metrics"],["Scenario","Method","Aggregation","Follower"],METRICS)
    write_csv(tables/"position_summary.csv",summary)
    health_values = ["HealthyEdgeEpochs","AlarmNewHealthyEpisodes","AlarmOccupiedHealthyEpochs", "AlarmResidualFromFaultEpochs",
        "AlarmHealthyOccupancy","AlarmEpisodesPerEdgeHour","IsolationNewHealthyEpisodes", "IsolationOccupiedHealthyEpochs",
        "IsolationResidualFromFaultEpochs","IsolationHealthyOccupancy","IsolationEpisodesPerEdgeHour"]
    write_csv(tables/"health_summary.csv",grouped_summary(merged["health"],["Scenario","Method","EdgeScope","Phase"],health_values))
    event_seeds = event_seed_summary(merged["events"],merged["health"])
    write_csv(tables/"event_seed_summary.csv",event_seeds)
    if event_seeds:
        event_values = [k for k in event_seeds[0] if k not in ("Scenario","Seed","Method")]
        write_csv(tables/"event_summary.csv",grouped_summary(event_seeds,["Scenario","Method"],event_values))
    paired = paired_position_rows(merged["metrics"])
    write_csv(tables/"paired_differences_per_seed.csv",paired)
    write_csv(tables/"paired_summary.csv",grouped_summary(paired,["Scenario","Aggregation","Follower","Comparison"],
        [m+"Difference" for m in METRICS]))
    write_csv(tables/"missing_runs.csv",missing)
    validation = validate_results(results,stage,missing)
    (tables/"validation.json").write_text(json.dumps(validation,indent=2),encoding="utf-8")
    if missing and not partial:
        raise RuntimeError(f"{len(missing)} required checkpoints missing; {stage} not complete")
    if not missing:
        assert validation["passed"],validation
        if stage == "E3":
            (output/"E3_VALIDATED.json").write_text(json.dumps(validation,indent=2),encoding="utf-8")
    print(json.dumps(validation,indent=2),flush=True)
    return results


def validate_results(results, stage, missing):
    findings = []; lookup={(r["scenario"],r["seed"]):r for r in results}
    for r in results:
        for g in r["geometry"]:
            if not (g["Connected"] and g["Rank"]==3 and g["LeaderEdges"]==3 and g["MaxRange"]<=500):
                findings.append(f"Invalid nominal geometry: {r['scenario']} t={g['Time']} f={g['Follower']}")
        metrics=r["metrics"]
        for method in {m["Method"] for m in metrics}:
            mm={m["Aggregation"]:m for m in metrics if m["Method"]==method and m["Aggregation"]!="follower"}
            if abs(mm["pooled"]["Mean"]-mm["macro"]["Mean"])>1e-10:
                findings.append("Equal sample-count macro Mean mismatch")
    if stage == "E2":
        for seed in range(1,51):
            base=lookup.get(("baseline_3f",seed))
            if not base: continue
            for scenario,scale in (("nlos_weak",.5),("nlos_strong",1.5)):
                other=lookup.get((scenario,seed))
                if not other: continue
                if base["signatures"] != other["signatures"]:
                    findings.append(f"Input signature mismatch {scenario} {seed}")
                for a,b in zip(base["schedules"],other["schedules"]):
                    if any(a[key]!=b[key] for key in ("Start","End","Follower","Leader","DrawIndex")) or abs(scale*a["Bias"]-b["Bias"])>1e-12:
                        findings.append(f"Fault pairing mismatch {scenario} {seed}")
    return dict(stage=stage,expected_runs=100 if stage=="E3" else 250,
        completed_runs=len(results),missing_runs=len(missing),findings=findings,passed=not missing and not findings,
        statistics_unit="seed",formal_seeds=list(range(1,51)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument("--stage",choices=["E3","E2"],required=True)
    parser.add_argument("--allow-partial",action="store_true")
    args=parser.parse_args()
    run_analysis(args.output,args.stage,args.allow_partial)


if __name__=="__main__":
    main()
