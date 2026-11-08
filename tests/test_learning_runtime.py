"""Learning environment contract and dependency-free EV3 inference checks."""

import unittest

import contact_weights
import tap_probe_weights
from contact_features import TAP_PROBE_FEATURES, tap_probe_feature
from contact_inference import ContactInference


class TestContactRuntime(unittest.TestCase):
    def test_weight_shapes(self):
        self.assertEqual(contact_weights.INPUT_SIZE, 7)
        self.assertEqual(len(contact_weights.W_IH), 3 * contact_weights.HIDDEN_SIZE)
        self.assertEqual(len(contact_weights.W_HH), 3 * contact_weights.HIDDEN_SIZE)
        self.assertEqual(len(contact_weights.W_OUT), contact_weights.HIDDEN_SIZE)

    def test_inference_is_deterministic_after_reset(self):
        model = ContactInference(contact_weights)
        sequence = [[0.1, 0.12, 0.0, 0.3, 0.0, 0.4, 0.0]] * 12
        first = [model.update(x)[0] for x in sequence]
        model.reset()
        second = [model.update(x)[0] for x in sequence]
        self.assertEqual(first, second)
        self.assertTrue(all(0.0 <= p <= 1.0 for p in first))

    def test_tap_probe_export_contract(self):
        self.assertEqual(tap_probe_weights.INPUT_SIZE, len(TAP_PROBE_FEATURES))
        feature = tap_probe_feature(-20, 30, 5, 0.5, is_early=True)
        self.assertEqual(feature, (0.1, 0.15, 1.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0))
        model = ContactInference(tap_probe_weights)
        probability, _ = model.update(feature)
        self.assertTrue(0.0 <= probability <= 1.0)


if __name__ == "__main__":
    unittest.main()

