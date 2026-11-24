"""config 的几何自检。

这些约束不是"风格问题"：几何一旦不自洽，离线仿真会照样跑完、只是每次交付都差几厘米，
实机上则表现为"推着物体停在色标前 20 mm"或者"光斑永远看不到色标"。所以把它们变成测试。
"""

import unittest

import harness
import sim


class TestConfigGeometry(unittest.TestCase):
    def setUp(self):
        self.cfg = harness.fresh_config()

    def test_lead_fits_inside_the_plow(self):
        """物体中心必须先到色标，推板前端才能越过色标：OBJ_LEAD + r <= PLOW_FRONT。

        如果 OBJ_LEAD + r > PLOW_FRONT，物体中心还差一截时推板已经压到色标（或顶到边界），
        deliver() 会因为"到不了目标点"而放弃本次交付（打印 DELIVERED 之外的 END）。
        """
        lead = self.cfg.OBJ_LEAD_MM
        r = self.cfg.TIRE_RADIUS_MM
        front = self.cfg.PLOW_FRONT_MM
        self.assertLess(float(lead) + float(r), float(front),
                        "OBJ_LEAD(%.1f) + TIRE_RADIUS(%.1f) must be < PLOW_FRONT(%.1f)"
                        % (lead, r, front))

    def test_spot_is_behind_the_plow(self):
        """光斑在推板后面：SENSOR_OFFSET < PLOW_FRONT。（反过来 = 光斑装在推板外面。）"""
        self.assertLess(float(self.cfg.SENSOR_OFFSET_MM), float(self.cfg.PLOW_FRONT_MM))

    def test_tire_radius_matches_diameter(self):
        self.assertAlmostEqual(float(self.cfg.TIRE_RADIUS_MM) * 2.0,
                               float(self.cfg.TIRE_DIAMETER_MM), places=6)

    def test_sim_geometry_matches_config(self):
        """仿真几何必须与 config 的实测值一致，否则 evaluate 的统计就不代表实机几何。"""
        g = sim.GEOM
        self.assertTrue(g.get("v3"), "sim GEOM must be the v3 (CB-based) plow")
        self.assertAlmostEqual(g["cb"] + g["wall_far"], float(self.cfg.PLOW_FRONT_MM), places=6)
        self.assertAlmostEqual(2.0 * g["wall_y"], float(self.cfg.PLOW_MOUTH_MM), places=6)
        # 平行加长梁从 V 臂尖开始，不比臂尖更宽（否则臂尖与梁之间漏缝）
        self.assertLessEqual(g["wall_y"], g["v_tip_y"])
        self.assertLess(g["cb"] + g["v_far"], float(self.cfg.PLOW_FRONT_MM))
        # 物体坐进推板时中心正好压在挡板前一点点 = OBJ_LEAD
        seat = g["cb"] + g["bumper_x"] + float(self.cfg.TIRE_RADIUS_MM)
        self.assertLess(abs(seat - float(self.cfg.OBJ_LEAD_MM)), 5.0,
                        "sim seat %.1f mm vs OBJ_LEAD %.1f mm" % (seat, self.cfg.OBJ_LEAD_MM))
        self.assertAlmostEqual(sim.Sim().sensor_offset, float(self.cfg.SENSOR_OFFSET_MM), places=6)

    def test_lane_spacing_fits_the_plow_opening(self):
        """LANE_SPACING <= PLOW_MOUTH - 20：两条车道正中间的轮胎，中心仍在某一条车道的
        梁内侧 10 mm 以内，会被梁尖拨进通口，而不是从两条车道之间漏掉。"""
        limit = float(self.cfg.PLOW_MOUTH_MM) - 20.0
        self.assertLessEqual(float(self.cfg.LANE_SPACING_MM), limit,
                             "LANE_SPACING(%.1f) must be <= PLOW_MOUTH - 20 (%.1f)"
                             % (self.cfg.LANE_SPACING_MM, limit))

    def test_plow_swing_radius_is_derived(self):
        """原地转扫圈半径 = sqrt(PLOW_FRONT^2 + (PLOW_MOUTH/2 + 4)^2)，约 209 mm。"""
        c = self.cfg
        r = (c.PLOW_FRONT_MM ** 2 + (c.PLOW_MOUTH_MM / 2.0 + 4.0) ** 2) ** 0.5
        self.assertAlmostEqual(c.PLOW_SWING_R_MM, r, places=6)
        self.assertAlmostEqual(c.PLOW_TIP_HALF_MM, c.PLOW_MOUTH_MM / 2.0 + 4.0, places=6)
        self.assertTrue(205.0 < c.PLOW_SWING_R_MM < 215.0)

    def test_v3_removals_are_off_by_default(self):
        """任务书要求：旧的捕获/自学/反弹搜索默认全部关闭（文件保留，开关打开才走）。"""
        for name in ("USE_LOAD_CAPTURE", "USE_LEARNED_CONTACT",
                     "USE_BOUNCE_SEARCH", "USE_SELF_CAL"):
            self.assertFalse(getattr(self.cfg, name), name + " must default False")

    def test_sense_mode_default_is_reflection(self):
        """换底盘后 SIGNATURES 失效，bring-up 默认必须是 reflection（rgb 要等重新标定）。"""
        self.assertEqual(self.cfg.SENSE_MODE, "reflection")

    def test_marker_min_hits_is_sane(self):
        self.assertGreaterEqual(int(self.cfg.MARKER_MIN_HITS), 2)
        self.assertLessEqual(int(self.cfg.MARKER_MIN_HITS), 12)


if __name__ == "__main__":
    unittest.main()
