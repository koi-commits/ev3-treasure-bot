"""Dataset isolation, determinism, and no-hidden-truth observation contract."""

import unittest
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from learning.contact_protocol import FEATURES, TAP_FEATURES, TAP_PROBE_FEATURES, make_trial, passive_trace
from learning.seeding import SPLITS, STREAMS, assert_disjoint, derive_seed


class TestLearningProtocol(unittest.TestCase):
    def test_random_streams_are_disjoint(self):
        self.assertTrue(assert_disjoint(512))
        for stream in STREAMS:
            seeds = [derive_seed(split, 17, stream) for split in SPLITS]
            self.assertEqual(len(set(seeds)), len(SPLITS))

    def test_trial_and_trace_are_reproducible(self):
        a = make_trial(23, "test")
        b = make_trial(23, "test")
        self.assertEqual(a, b)
        ta = passive_trace(a, steps=20)
        tb = passive_trace(b, steps=20)
        for left, right in zip(ta, tb):
            np.testing.assert_array_equal(left, right)

    def test_model_features_contain_no_simulator_truth(self):
        forbidden = ("object", "pocket", "truth", "world", "sim_pose", "captured")
        for name in FEATURES + TAP_FEATURES + TAP_PROBE_FEATURES:
            self.assertFalse(any(token in name for token in forbidden), name)


if __name__ == "__main__":
    unittest.main()
