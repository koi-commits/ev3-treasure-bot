"""Read offline bring-up logs. Diagnostics go to stderr; stdout is a git patch."""
import argparse
import ast
import difflib
import re
import sys
from pathlib import Path


def read(path):
    data = path.read_bytes()
    enc = ("utf-16" if data.startswith((b"\xff\xfe", b"\xfe\xff")) else
           "utf-16-le" if b"\0" in data[:128] else "utf-8-sig")
    return data.decode(enc, errors="replace")


def suggestions(log):
    changes = {}
    def report(msg):
        print(msg, file=sys.stderr)
    for a,b in re.findall(r"type touch: (.*?) gyro: (.+)", log):
        for key,value in (("touch",a),("gyro",b)):
            state = ("yes" if re.search("await|coroutine|generator",value,re.I) else
                     "no" if re.search(r"\b(bool|int|float)\b",value) else "unknown")
            report(f"{key}: {value.strip()} (awaitable: {state})")
    for d in re.findall(r"drift 30s\s*=.*?=\s*([-\d.]+) deg/min",log):
        report(f"drift {d} deg/min: "+("PASS" if abs(float(d))<2 else "FAIL"))
    for sign,v in re.findall(r"sign: (SAME|OPPOSITE) -> GYRO_POSITIVE_DIRECTION = (-?1)",log):
        report("gyro sign: "+sign)
        changes["GYRO_POSITIVE_DIRECTION"]=int(v)
    for v in re.findall(r"suggested AXLE_TRACK_MM =[^\n]*?=\s*(-?[\d.]+)",log):
        changes["AXLE_TRACK_MM"]=abs(float(v))
    for k in ("BLACK_REFLECTION","WHITE_REFLECTION"):
        m=re.findall(r"^"+k+r" = ([\d.]+)",log,re.M)
        if m: changes[k]=float(m[-1])
    if "WHITE_REFLECTION" in changes and "BLACK_REFLECTION" in changes:
        report("reflection threshold: %.2f"%((changes["WHITE_REFLECTION"]+changes["BLACK_REFLECTION"])/2))
    for block in re.split(r"==== RESULT:",log)[1:]:
        if not block.lstrip().startswith("PASS"): continue
        m=re.search(r"RGB_EDGE_BRIGHT = ([\d.]+)\s+SIGNATURES = (\{.*?^\})",block,re.S|re.M)
        if not m: continue
        sig=ast.literal_eval(m[2])
        changes.update(SIGNATURES=sig,RGB_EDGE_BRIGHT=float(m[1]))
        if "finish" in sig:
            def features(rgb):
                total=sum(rgb) or 1
                return (100*rgb[0]/total,100*rgb[1]/total,total/3)
            r,g,b=features(sig["finish"]["mean"])
            neg=[features(v["mean"]) for k,v in sig.items() if k!="finish"]
            if r>max(x[0] for x in neg) and g<min(x[1] for x in neg):
                changes["RED_RULE"]=tuple(round(x,1) for x in
                    ((r+max(x[0] for x in neg))/2,(g+min(x[1] for x in neg))/2,
                     (b+features(sig["floor"]["mean"])[2])/2))
            else: report("RED_RULE: samples do not separate red; review manually")
    for key,val in changes.items(): report(f"{key} = {val!r}")
    return changes


def patch(source, changes):
    lines=source.splitlines(True)
    for n in reversed(ast.parse(source).body):
        if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name):
            key=n.targets[0].id
            if key in changes:
                lines[n.lineno-1:n.end_lineno]=[f"{key} = {changes[key]!r}\n"]
    return "".join(difflib.unified_diff(source.splitlines(True),lines,"a/src/config.py","b/src/config.py"))


if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("logs",type=Path,nargs="?",default=Path("logs"))
    args=ap.parse_args()
    changes={}
    for path in sorted(args.logs.glob("*.txt"),key=lambda p:(p.stat().st_mtime,p.name)):
        changes.update(suggestions(read(path)))
    print(patch((Path(__file__).resolve().parents[1]/"src/config.py").read_text("utf8"),changes),end="")
