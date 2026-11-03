"""python tests/report_scenarios.py [n] — 仿真场景矩阵（地面真值评判），输出 Markdown 表。

列含义见 tests/evaluate.py。**可接受成功** = 任务成功 且 没有轮子完全越过白带 且 物体没被推出场地。
仿真只验证逻辑与相对趋势；物理参数（摩擦、负载噪声、物体重量……）全部是假设值，
所以这里给的是"在一组合理参数范围内的敏感性"，不是实机成功率。
"""
import sys

from evaluate import (HEADER, object_interior, object_near_edge, run_one, start_corner, start_near_edge,
                      start_normal, summarize)
from sim import BLUE_MARK

N = int(sys.argv[1]) if len(sys.argv) > 1 else 12


def row(name, n, **kw):
    runs = [run_one(i, **kw) for i in range(n)]
    line = summarize(name, runs)
    print(line)
    sys.stdout.flush()
    return line


if __name__ == "__main__":
    print("### A. edge_to_finish（寻边 -> 巡线 -> 识别终点停车）\n")
    print(HEADER)
    row("正常随机起点（距边 ≥150 mm）", 2 * N, mode="edge_to_finish", start=start_normal)
    row("边界起点（距边 60–120 mm，光斑可能在白边上）", 2 * N, mode="edge_to_finish", start=start_near_edge)
    row("角落起点", N, mode="edge_to_finish", start=start_corner)
    row("深色色标（反射率低于边界阈值）", N, mode="edge_to_finish", mark=BLUE_MARK)
    row("每次读传感器 30 ms（模式切换代价）", N, mode="edge_to_finish", sim_kw=dict(read_cost_ms=30))
    row("白带只有 25 mm 宽", N, mode="edge_to_finish", sim_kw=dict(band=25))
    row("光斑前伸 100 mm", N, mode="edge_to_finish", sim_kw=dict(sensor_offset=100))
    row("滑行减速只有 400 mm/s²", N, mode="edge_to_finish", sim_kw=dict(coast_decel=400, brake_decel=800))
    row("白带 25 mm + steep 参数组", N, mode="edge_to_finish", sim_kw=dict(band=25), cfg=dict(FOLLOW_PROFILE="steep"))

    print("\n### B. full（找终点 -> 搜索 -> 确认 -> 推送 -> 交付），接触式物体模型\n")
    print(HEADER)
    row("基准：物体在场地内部", N, mode="full", obj=object_interior)
    row("物体靠边（距白边 40–100 mm）", N, mode="full", obj=object_near_edge)
    row("边界起点 + 内部物体", N, mode="full", start=start_near_edge)
    row("物体轻（推动负载 10 mNm）", N, mode="full", obj_kw=dict(load=10))
    row("物体重（推动负载 40 mNm）", N, mode="full", obj_kw=dict(load=40))
    row("负载噪声 ×2", N, mode="full", sim_kw=dict(load_noise=5.0))
    row("物体-推板摩擦低 μ=0.1", N, mode="full", sim_kw=dict(contact_mu=0.1))
    row("物体-推板摩擦高 μ=0.6", N, mode="full", sim_kw=dict(contact_mu=0.6))
    row("物体大（r=35）", N, mode="full", obj_kw=dict(r=35))
    row("里程计误差（距离 +3%，转角 −3%）", N, mode="full", sim_kw=dict(odo_dist_scale=1.03, odo_turn_scale=0.97))
    row("里程计误差（距离 +8%，转角 −5%，未标定）", N, mode="full", sim_kw=dict(odo_dist_scale=1.08, odo_turn_scale=0.95))
    row("里程计误差 + 关闭场上自标定（对照）", N, mode="full", sim_kw=dict(odo_dist_scale=1.03, odo_turn_scale=0.97),
        cfg=dict(SELF_CAL=False))
    row("推板无侧板（有'肩部'凹角）", N, mode="full", sim_kw=dict(geom=dict(side_plates=False)))
    row("巡线 steep 参数组", N, mode="full", cfg=dict(FOLLOW_PROFILE="steep"))
