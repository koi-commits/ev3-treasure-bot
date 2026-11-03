"""parse_logs.py — 把 logs\\*.txt（pbrun 输出）汇总成一张 Markdown 表。PC 上运行（CPython）。

    D:\\python\\python.exe tools\\parse_logs.py logs > logs\\summary.md

识别：mission.py 的 SUMMARY、demo_edge.py 的每段统计、probe_timing 的 PHASE 行、
calibrate_marker 的 RESULT、probe_load 的 SUGGEST、calibrate_geometry 的读数。
"""
import os
import re
import sys


def kv_after(line, keys):
    out = {}
    toks = line.replace(",", " ").split()
    for i, t in enumerate(toks[:-1]):
        if t in keys:
            out[t] = toks[i + 1]
    return out


MISSION_KEYS = ("time_ms", "max_white_run_mm", "twists", "lost", "overruns", "stalls", "seek_fail",
                "outside_returns", "suspects", "captures", "lost_in_carry")


def parse(path):
    txt = open(path, encoding="utf-8", errors="replace").read()
    name = os.path.basename(path)
    rows = []
    if "==== SUMMARY ====" in txt:
        d = {"file": name, "kind": "mission"}
        for line in txt.splitlines():
            if line.startswith("result:"):
                d["result"] = line[8:].strip()
            if line.startswith("mode "):
                d["mode"] = line.split()[1]
                d["sense"] = line.split()[3] if len(line.split()) > 3 else ""
            m = re.search(r"white_frac ([0-9.]+)", line)
            if m:
                d["white_frac"] = m.group(1)
            d.update(kv_after(line, MISSION_KEYS))
        d["overrun_events"] = txt.count("] overrun")
        d["edge_lost_events"] = txt.count("] edge lost")
        rows.append(d)
    segs = re.findall(r"edge_twists (\d+)\s*\n\s*white_frac=([0-9.]+)", txt)
    if segs:
        rows.append({"file": name, "kind": "demo_edge",
                     "result": "segments={} twists={} white_frac={}".format(
                         len(segs), "/".join(s[0] for s in segs), "/".join(s[1] for s in segs))})
    for line in txt.splitlines():
        if line.startswith("PHASE "):
            rows.append({"file": name, "kind": "probe_timing", "result": line.strip()})
        elif "==== RESULT:" in line:
            rows.append({"file": name, "kind": "calibrate_marker", "result": line.strip("= ").strip()})
        elif line.startswith("SUGGEST:") or line.startswith("difference HOLD-EMPTY"):
            rows.append({"file": name, "kind": "probe_load", "result": line.strip()})
        elif line.startswith("PROBLEM:"):
            rows.append({"file": name, "kind": "calibrate_marker", "result": line.strip()})
    if not rows:
        last = [l for l in txt.splitlines() if l.strip()][-1:] or ["(empty)"]
        rows.append({"file": name, "kind": "?", "result": last[0][:100]})
    return rows


def main(folder):
    files = sorted(f for f in os.listdir(folder) if f.endswith(".txt") and f != "README.txt")
    rows = []
    for f in files:
        rows += parse(os.path.join(folder, f))
    mission = [r for r in rows if r["kind"] == "mission"]
    other = [r for r in rows if r["kind"] != "mission"]
    if mission:
        cols = ["file", "mode", "sense", "result", "time_ms", "white_frac", "max_white_run_mm", "twists",
                "overruns", "lost", "stalls", "outside_returns", "suspects", "captures", "lost_in_carry"]
        print("| " + " | ".join(cols) + " |")
        print("|" + "---|" * len(cols))
        for r in mission:
            print("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |")
        print()
    if other:
        print("| file | kind | result |")
        print("|---|---|---|")
        for r in other:
            print("| {} | {} | {} |".format(r["file"], r["kind"], r["result"]))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "logs")
