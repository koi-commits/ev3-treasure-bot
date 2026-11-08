"""Parse probe_tap_shadow.py V2 logs; incomplete trials never enter metrics."""

import argparse


MODEL_MAX_INFER_MS = 20
MODEL_MAX_CYCLE_MS = 80


def _fields(line):
    values = {}
    for token in line.split()[1:]:
        if "=" in token:
            key, value = token.split("=", 1)
            values[key] = value
    return values


def _score(rows, prediction, truth):
    tp = sum(int(r[truth]) == 1 and int(r[prediction]) == 1 for r in rows)
    fp = sum(int(r[truth]) == 0 and int(r[prediction]) == 1 for r in rows)
    fn = sum(int(r[truth]) == 1 and int(r[prediction]) == 0 for r in rows)
    return {"tp": tp, "fp": fp, "fn": fn,
            "precision": tp / max(1, tp + fp),
            "recall": tp / max(1, tp + fn)}


def _model_timing_ok(row):
    try:
        return (int(row["max_infer_ms"]) <= MODEL_MAX_INFER_MS and
                int(row["max_cycle_ms"]) <= MODEL_MAX_CYCLE_MS)
    except (KeyError, TypeError, ValueError):
        return False


def summarize(text):
    aborts = [_fields(line) for line in text.splitlines()
              if line.startswith("SHADOW_ABORT ")]
    results = [_fields(line) for line in text.splitlines()
               if line.startswith("SHADOW_RESULT ")]
    complete = [r for r in results if r.get("complete", "1") == "1"]
    for row in complete:  # legacy V1 used one label for both decisions
        if "label" in row:
            row.setdefault("pre_label", row["label"])
            row.setdefault("post_label", row["label"])
    transitions = {name: 0 for name in ("0to0", "0to1", "1to0", "1to1")}
    for row in complete:
        transition = row.get("transition", row["pre_label"] + "to" + row["post_label"])
        if transition in transitions:
            transitions[transition] += 1
    inference = [int(r["max_infer_ms"]) for r in complete if "max_infer_ms" in r]
    cycles = [int(r["max_cycle_ms"]) for r in complete if "max_cycle_ms" in r]
    model_eligible = [r for r in complete if _model_timing_ok(r)]
    incomplete = [r for r in results if r.get("complete", "1") != "1"]
    reasons = [a.get("reason", "unknown") for a in aborts]
    reasons += [r.get("abort_reason", "unknown") for r in incomplete]
    conditions = {}
    for row in complete:
        condition = row.get("condition", "UNKNOWN")
        conditions[condition] = conditions.get(condition, 0) + 1
    return {
        "attempted_trials": len(results) + len(aborts),
        "abort_lines": len(aborts),
        "abort_reasons": {reason: reasons.count(reason) for reason in sorted(set(reasons))},
        "result_lines": len(results), "complete_trials": len(complete),
        "incomplete_trials": len(results) - len(complete),
        "rule_vs_pre_probe": _score(complete, "rule", "pre_label"),
        "model_vs_post_probe": _score(model_eligible, "model", "post_label"),
        "model_eligible_trials": len(model_eligible),
        "model_timing_excluded_trials": len(complete) - len(model_eligible),
        "model_timing_limits_ms": {"infer": MODEL_MAX_INFER_MS,
                                   "cycle": MODEL_MAX_CYCLE_MS},
        "transitions": transitions,
        "complete_by_condition": conditions,
        "max_infer_ms": max(inference) if inference else None,
        "max_cycle_ms": max(cycles) if cycles else None,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("log")
    args = p.parse_args()
    with open(args.log, "r", encoding="utf-8", errors="replace") as f:
        report = summarize(f.read())
    print(report)


if __name__ == "__main__":
    main()
