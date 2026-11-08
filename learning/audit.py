"""Generate a machine-readable split/isolation and feature-leakage audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import os

from .contact_protocol import FEATURES, TAP_FEATURES, TAP_PROBE_FEATURES
from .seeding import SPLITS, STREAMS, assert_disjoint, derive_seed

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def digest(values):
    raw = ",".join(str(v) for v in values).encode("ascii")
    return hashlib.sha256(raw).hexdigest()


def build(count=4096):
    assert_disjoint(count)
    seeds = {}
    all_values = []
    for split in SPLITS:
        seeds[split] = {}
        for stream in STREAMS:
            values = [derive_seed(split, i, stream) for i in range(count)]
            all_values.extend(values)
            seeds[split][stream] = {"count": count, "sha256": digest(values)}
    contracts = {
        "passive_contact": list(FEATURES),
        "tap_seat": list(TAP_FEATURES),
        "tap_hold_probe": list(TAP_PROBE_FEATURES),
    }
    forbidden = ("object", "pocket", "truth", "world", "sim_pose", "captured",
                 "friction", "radius", "mass")
    leaks = [name for names in contracts.values() for name in names
             if any(token in name for token in forbidden)]
    return {
        "seed_protocol": "sha256(ev3-ai-v2|split|index|stream), 31-bit",
        "indices_per_namespace": count,
        "namespaces": seeds,
        "total_seeds": len(all_values),
        "unique_seeds": len(set(all_values)),
        "all_namespaces_disjoint": len(all_values) == len(set(all_values)),
        "feature_contracts": contracts,
        "forbidden_feature_tokens": list(forbidden),
        "detected_forbidden_features": leaks,
        "truth_usage": ["training labels", "reward shaping", "offline evaluation"],
        "truth_available_to_inference": False,
        "pc_ev3_shared_feature_function": "src/contact_features.py:tap_probe_feature",
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default=os.path.join(ROOT, "docs", "ai_data_audit.json"))
    p.add_argument("--count", type=int, default=4096)
    args = p.parse_args()
    report = build(args.count)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
