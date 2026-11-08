"""Parse SHADOW_RESULT lines from probe_tap_shadow.py and print event metrics."""

import argparse
import re

PATTERN = re.compile(r"SHADOW_RESULT trial=(\d+) label=(\d) rule=(\d) model=(\d) p=([0-9.]+)")


def summarize(text):
    rows = [tuple(map(float, m.groups())) for m in PATTERN.finditer(text)]
    out = {"trials": len(rows)}
    for name, column in (("rule", 2), ("model", 3)):
        tp = sum(int(r[1]) == 1 and int(r[column]) == 1 for r in rows)
        fp = sum(int(r[1]) == 0 and int(r[column]) == 1 for r in rows)
        fn = sum(int(r[1]) == 1 and int(r[column]) == 0 for r in rows)
        out[name] = {"tp": tp, "fp": fp, "fn": fn,
                     "precision": tp / max(1, tp + fp),
                     "recall": tp / max(1, tp + fn)}
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("log")
    args = p.parse_args()
    with open(args.log, "r", encoding="utf-8", errors="replace") as f:
        report = summarize(f.read())
    print(report)


if __name__ == "__main__":
    main()
