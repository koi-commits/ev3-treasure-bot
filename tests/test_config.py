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
        self.assertAlmostEqual(g["v_tip_y"], float(self.cfg.PLOW_TIP_HALF_MM), places=6)
        self.assertAlmostEqual(self.cfg.PLOW_TIP_HALF_MM, self.cfg.PLOW_MOUTH_MM / 2.0 + 3.5, places=6)
        # 物体坐进推板时中心正好压在挡板前一点点 = OBJ_LEAD
        seat = g["cb"] + g["bumper_x"] + float(self.cfg.TIRE_RADIUS_MM)
        self.assertLess(abs(seat - float(self.cfg.OBJ_LEAD_MM)), 5.0,
                        "sim seat %.1f mm vs OBJ_LEAD %.1f mm" % (seat, self.cfg.OBJ_LEAD_MM))
        self.assertAlmostEqual(sim.Sim().sensor_offset, float(self.cfg.SENSOR_OFFSET_MM), places=6)

    def test_lane_spacing_fits_the_plow_opening(self):
        """车道间距必须小于推板通口（2 * v_tip_y），否则相邻车道之间会漏掉一条轮胎。"""
        opening = 2.0 * sim.GEOM["v_tip_y"]
        self.assertLess(float(self.cfg.LANE_SPACING_MM), opening,
                        "LANE_SPACING(%.1f) must be < plow opening (%.1f)"
                        % (self.cfg.LANE_SPACING_MM, opening))

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
