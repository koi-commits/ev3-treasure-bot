# Trained artifacts

- `contact_gru.pt`: legacy 769-parameter passive detector retained for reproducibility; do not use it to trigger delivery.
- `contact_gru_metrics.json`: legacy frame/whole-negative metrics. Its 2/50 number is not a first-trigger metric and must not justify control.
- `contact_gru_event_v2.pt`: direct passive detector retrained for first events; negative result (2/4 correct triggers on its test protocol).
- `tap_seat_gru_v1.pt`: learned classifier on ordinary tap+seat only; negative result because it could not separate side contacts reliably.
- `tap_probe_gru_v1.pt`: current recommended **shadow-only** artifact. It evaluates Final tap+seat plus a gentle hold probe; see its metrics JSON.
- `tap_probe_gru_v1_export.json`: numerical parity, size, and PC timing for the MicroPython export.
- `gru_policy.pt`: behavior-cloned 24-unit GRU low-level policy. Kept to reproduce the documented 0/24 negative end-to-end result; do **not** deploy it.
- `gru_policy_train.json`: training metadata for that negative experiment.
- `final_expert.npz`: 29 acceptable Final trajectories (25,007 control samples) used for behavior cloning.

`src/contact_weights.py` is the MicroPython export of `contact_gru.pt`.

