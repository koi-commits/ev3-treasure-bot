"""Stable, disjoint random streams for train/validation/test experiments."""

from __future__ import annotations

import hashlib


SPLITS = ("train", "val", "test", "audit")
STREAMS = ("scene", "physics", "actions", "controller")


def derive_seed(split: str, index: int, stream: str) -> int:
    if split not in SPLITS:
        raise ValueError("unknown split " + repr(split))
    if stream not in STREAMS:
        raise ValueError("unknown stream " + repr(stream))
    key = "ev3-ai-v2|{}|{}|{}".format(split, int(index), stream).encode("ascii")
    # Stable across Python versions/processes; compatible with random.Random and Sim.
    return int.from_bytes(hashlib.sha256(key).digest()[:8], "big") & 0x7FFFFFFF


def split_manifest(count=256):
    return {split: {stream: [derive_seed(split, i, stream) for i in range(count)]
                    for stream in STREAMS}
            for split in SPLITS}


def assert_disjoint(count=4096):
    manifest = split_manifest(count)
    all_values = []
    for stream in STREAMS:
        sets = {split: set(manifest[split][stream]) for split in SPLITS}
        for split in SPLITS:
            all_values.extend(manifest[split][stream])
        for i, left in enumerate(SPLITS):
            for right in SPLITS[i + 1:]:
                overlap = sets[left] & sets[right]
                if overlap:
                    raise AssertionError("seed overlap {} {} {}".format(stream, left, right))
    if len(all_values) != len(set(all_values)):
        raise AssertionError("seed collision across split/stream namespaces")
    return True

