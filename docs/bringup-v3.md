# v3 hardware bring-up

Planned commands only (PowerShell from the repository root). Use an attended flat arena, keep CENTER accessible, verify 1 mm plow clearance under load, and measure the seated tyre lead from the axle. Touch raw awaitability remains unverified until probe b.

**No gyro (2026-11-24):** the team has no gyro sensor. `HEADING_SOURCE = "odometry"` (default): `sweep` / `capture` / `full` take their heading from `drive.angle()` (wheel encoders); the gyro is never constructed or read, and `GYRO_POSITIVE_DIRECTION` is not needed. Heading accuracy therefore depends on `AXLE_TRACK_MM`: offline, a 1% track error gives ~3° heading error at delivery and a 3% error ~9° (up to 33°) with far fewer deliveries. The **geometry** run below is required, and must reach an axle-track error < 1% before any mission run. Log each run; do not advance past a failed criterion. These commands redirect output to per-run logs. Inspect each log after the run; for live readings open a second PowerShell and use Get-Content on its log with -Wait. Config changes are manual, between runs.

|Run|Command|Setup / observe|Pass|Stop the session here if failed; action|
|---|---|---|---|---|
|env|` .\pbrun.cmd src\check_env.py *> logs\env.txt `|Read hub, motors and sensor diagnostics|Expected ports present; no import or I/O exception|Correct runtime or wiring; stop before motion|
|motors|` .\pbrun.cmd src\motor_id.py *> logs\motors.txt `|Lift wheels; watch A then D|A drives left forward; D drives right forward|Correct ports/directions; stop if reversed|
|probe-b|` .\pbrun.cmd src\probe_new_hw.py *> logs\probe-b.txt `|Set SECTIONS='b' (sections a and c are gyro-only: skip); press/release, move spot|Touch raw type logged; touch toggles; white separates (gyro column shows `-`)|Check ports, height and awaitable handling; stop on stale readings|
|geometry (**required**)|` .\pbrun.cmd src\calibrate_geometry.py *> logs\geometry.txt `|≥ 1.2 m clear floor; STRAIGHT 1000 mm then TURN 720° (see script header); update `WHEEL_DIAMETER_MM` first, then `AXLE_TRACK_MM`; re-run after each change|Straight error < 1% **and** 720° turn error < 1% (< 7.2°) on a repeat run; then set `GEOMETRY_CONFIRMED = True`|Repeat on the same floor surface; stop if two repeats disagree by > 1% (slip, loose wheel)|
|reflection|` .\pbrun.cmd src\calibrate.py *> logs\reflection.txt `|Sample actual floor and white tape|Separated readings and midpoint threshold, no contrast warning|Reset spot height/lighting and repeat; stop if overlap|
|edge-old|` .\pbrun.cmd src\demo_edge.py *> logs\edge-old.txt `|FOLLOW_PROFILE='validated'; observe all segments|Repeated white hits, positive travel, no lost edge or arena exit|Check motor sign and threshold; stop before profile comparison|
|edge-sim|` .\pbrun.cmd src\demo_edge.py *> logs\edge-sim.txt `|FOLLOW_PROFILE='v3_sim'; same start and lighting|Keeps edge throughout; compare travel, twists and overshoot with old|Restore validated; stop if either wheel might fall|
|marker|` .\pbrun.cmd src\calibrate_marker.py *> logs\marker.txt `|Sample actual floor, white and red paper|RESULT PASS; live labels separate red from both other surfaces|Repeat under run lighting; stop if ambiguous|
|rgb|` .\pbrun.cmd src\calibrate_marker.py *> logs\rgb.txt `|Set SENSE_MODE='rgb' and reviewed signatures; use live classification|Floor, white and finish each confirmed repeatedly; no false finish|Correct signatures and rerun; stop before driving|
|sweep|` .\pbrun.cmd src\mission.py *> logs\sweep.txt `|Set MISSION_MODE='sweep'; no tyre; log must show `heading source: odometry (drive.angle)`|Plow ends lanes inside margin; observed backing distance >=82 mm before near-line pivot|Stop if parallel on tape, missed boundary or insufficient backoff|
|capture|` .\pbrun.cmd src\mission.py *> logs\capture.txt `|Set MISSION_MODE='capture'; tyre in interior|Touch stays pressed with tyre seated; measure OBJ_LEAD_MM|Stop if tyre escapes or plow scrapes; fix placement|
|full|` .\pbrun.cmd src\mission.py *> logs\full.txt `|Set MISSION_MODE='full'; start with clear path to boundary|Tyre centre on interior part of paper; robot and tyre remain in arena|Stop on unsafe approach, lost tyre, or false delivery; keep log|

`sweep`, `capture` and `full` use the odometry heading (wheel encoders). The optional arena-based heading correction (`HEADING_FIX`, two white hits on one known wall) is off by default: offline it gave no measurable benefit; turn it on only for a comparison run.

After calibration: `python tools/suggest_config.py logs`. Review stdout as a unified diff; diagnostic values go to stderr. Use separate session log directories to avoid mixing calibrations.

## Sim-only starting points

243 parameter sets, each on the same 20 random starts for 20 s, ideal differential drive on a straight 40 mm band. Ranking: keep edge, minimize worst overshoot, maximize progress, minimize twists/m. No slip, corners, read latency or braking delay modelled.

Speed / approach / turn / escape / clear: **65 / 18 / 120 / 300 / 350** (selected), or change clear to **400** or **450**. All three tied: 20/20 kept, 54.76 mm/s, 8.70 mm worst wheel overshoot beyond the band, 7.17 twists/m. The simulation does not distinguish these clear times.
