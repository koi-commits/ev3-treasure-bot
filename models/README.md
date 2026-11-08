# Trained artifacts

- `contact_gru.pt`: 769-parameter GRU capture detector, trained on 240 randomized simulator episodes. This is the recommended learned artifact.
- `contact_gru_metrics.json`: held-out validation/test metrics and selected threshold/debounce.
- `gru_policy.pt`: behavior-cloned 24-unit GRU low-level policy. Kept to reproduce the documented 0/24 negative end-to-end result; do **not** deploy it.
- `gru_policy_train.json`: training metadata for that negative experiment.
- `final_expert.npz`: 29 acceptable Final trajectories (25,007 control samples) used for behavior cloning.

`src/contact_weights.py` is the MicroPython export of `contact_gru.pt`.

