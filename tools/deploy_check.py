"""deploy_check.py — 证明"实际发给 EV3 的程序包含哪些模块"（PC 侧，无需连接）。

pb run 的行为（见 pb_cli/commands/program.py `_run`）：
    roots = await project.module_roots(script)
    compiled = await pb_client.compile_program(script, roots=roots)
    program = compiled.pack(legacy_main=...)
    await service.download(program, ...)
即：**本地 import 的模块会被打包一起下载到 hub**，不是只传入口文件。
所以"PC 上能跑"和"EV3 上跑的是同一套代码"之间只有一个变量：编译时实际收进去的模块集合。

本脚本把那个集合打印出来，逐个列出包名与字节数，并对比磁盘上的源码 md5，
用来回答"上传到 EV3 的到底是哪版代码"。

    D:\\python\\python.exe tools\\deploy_check.py
"""

import asyncio
import hashlib
import sys
from pathlib import Path

import pb_client
from pb_core import project

SRC = Path(__file__).resolve().parent.parent / "src"
ENTRIES = ["mission.py", "probe_tap_shadow.py", "demo_edge.py", "probe_load.py"]


async def check(name):
    path = SRC / name
    roots = await project.module_roots(path)
    compiled = await pb_client.compile_program(path, roots=roots)
    packed = compiled.pack(legacy_main=False)
    print("=== {} ===".format(name))
    print("  module roots: {}".format([str(r) for r in roots]))
    total = 0
    for m in compiled.modules:
        total += len(m.mpy)
        print("    {:<28} {:>7} bytes".format(m.name, len(m.mpy)))
    print("  modules={} payload={} bytes packed={} bytes".format(
        len(compiled.modules), total, len(packed)))
    deps = [m.name for m in compiled.modules]
    missing = [e for e in ("config", "fsm", "sensing", "hardware", "common") if e not in deps]
    print("  missing core deps: {}".format(missing if missing else "none"))
    if name == "probe_tap_shadow.py":
        for need in ("contact_inference", "contact_weights", "tap_probe_weights", "contact_features"):
            print("    {}: {}".format(need, "INCLUDED" if need in deps else "!!! MISSING"))
    print()
    return deps


async def main():
    seen = {}
    for e in ENTRIES:
        if not (SRC / e).exists():
            print("skip missing entry:", e)
            continue
        deps = await check(e)
        for d in deps:
            p = SRC / (d + ".py")
            if p.exists():
                seen[d] = hashlib.md5(p.read_bytes()).hexdigest()[:12]
    print("=== 模块源码 md5（PC 磁盘当前版本）===")
    for k in sorted(seen):
        print("  {:<28} {}".format(k, seen[k]))


asyncio.run(main())
