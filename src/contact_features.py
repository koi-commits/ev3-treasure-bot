"""Shared PC/EV3 feature math for tap+seat shadow inference."""

TAP_PROBE_FEATURES = ("abs_load_l", "abs_load_r", "odometry_step",
                      "tap_progress", "phase_free", "phase_early", "phase_late",
                      "probe_left_arc", "probe_right_arc")


def tap_probe_feature(load_l, load_r, distance_step, progress,
                      is_free=False, is_early=False, is_late=False,
                      probe_left=False, probe_right=False):
    """Normalize only quantities available from EV3 motors/commands."""
    return (abs(load_l) / 200.0, abs(load_r) / 200.0, distance_step / 5.0,
            min(1.2, max(0.0, progress)),
            float(bool(is_free)), float(bool(is_early)), float(bool(is_late)),
            float(bool(probe_left)), float(bool(probe_right)))

