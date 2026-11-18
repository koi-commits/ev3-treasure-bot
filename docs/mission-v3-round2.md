# Offline review: 150 mm paper, seeds 0–19 per layout

|Layout|Success old → new|Acceptable old → new|Object out (phase)|Robot out (phase)|Mean/max s|Coverage|Successful error median/max mm|
|---|---|---|---|---|---|---|---|
|interior|9 → 9|7 → 9|carry: 1|lane: 3|380/901|77.7%|10.0/47.2|
|near_edge|3 → 5|1 → 5|0|lane: 2|332/859|78.6%|18.5/47.9|
|marker_corner|1 → 1|1 → 1|lane: 3|marker_probe: 4, lane: 2|569/901|86.9%|11.0/11.0|
|near_start|— → 0|— → 0|early_carry: 17|lane: 1|380/900|72.6%|—|

Zero recorded carry pivots in all 80 runs. Early capture no longer deadlocks, but the tyre is typically pushed over before the first visible boundary: initial forward carry is unsuitable as a hardware acceptance test. Marker probing still risks exiting at a corner. Stage hardware tests and stop at each failed gate.

At baseline 7fe6a66 interior seed 2: true target (1154.1,450.0), estimated target transformed using actual pose at first target computation (1153.1,464.8), final tyre (1157.7,509.6). Target estimate contributed 14.9 mm; subsequent motion/odometry and seating contributed 45.0 mm, mostly along the wall. Estimated extent 156.6 vs actual 150 mm. This rules out paper-width error as the dominant term, but does not uniquely separate later heading drift and tyre motion. Do not retune OBJ_LEAD from this run.
