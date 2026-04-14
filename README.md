# poseTranslatorCyberRig

`poseTranslatorCyberRig` is a Python desktop engineering tool for custom 2DOF cable/string motion rigs. It is meant to be used **alongside FlyPT Mover**, not instead of it.

Signal chain:

`Game telemetry -> FlyPT Mover -> poseTranslatorCyberRig -> motor controller / Arduino`

FlyPT still handles telemetry, cueing, filtering, and pose generation. This app handles the extra math FlyPT does not natively solve for a custom two-motor, four-cable, coupled-spool rig.

## Why this exists

This rig is not a linear actuator platform and not a rigid crank linkage. Each side has one motor shaft angle but two cable targets:

- left motor drives `FL` and `BL`
- right motor drives `FR` and `BR`

That means a desired pose produces four ideal cable deltas, but only two motor commands. The app computes the best-fit shaft angles with weighted least squares, predicts the cable deltas that those coupled spools can actually produce, and reports the mismatch.

## Main features

- PySide6 desktop UI with tabs for `Geometry`, `Solve`, `Analyze`, and `Live FlyPT`
- `Pose to Cable` mode: pitch/roll to cable lengths, shaft angles, counts, and warnings
- `Manual Cable Solver` mode: measured cable lengths to estimated pitch/roll and residual error
- Live FlyPT companion workflow over serial or UDP
- Embedded matplotlib plots for top view, side view, workspace heatmap, and live scrolling graphs
- Live graphing of:
  - incoming pitch
  - incoming roll
  - solved `q_L` and `q_R`
  - target counts left/right
  - ideal vs predicted cable deltas
  - RMS coupling error
- CSV export of captured live graph history
- JSON config save/load with an example config
- Geometry and workspace analysis warnings for asymmetry, limits, slack risk, and coupling error

## Math model

Coordinate system:

- `x` = forward/back
- `y` = left/right
- `z` = up/down

Rigid-body rotation:

- roll = `phi`
- pitch = `theta`
- `R(phi, theta) = R_y(theta) @ R_x(phi)`

with:

- `R_x(phi) = [[1,0,0],[0,cos(phi),-sin(phi)],[0,sin(phi),cos(phi)]]`
- `R_y(theta) = [[cos(theta),0,sin(theta)],[0,1,0],[-sin(theta),0,cos(theta)]]`

Moved anchor:

- `A_i_moved = R(phi, theta) @ (A_i - C) + C`

Cable lengths:

- neutral cable length: `L0_i = ||F_i - A_i||`
- target cable length: `L_i = ||F_i - A_i_moved||`
- ideal cable delta: `dL_i = L_i - L0_i`

Coupled spool model:

- `dL_FL_pred = s_FL * r_FL * q_L`
- `dL_BL_pred = s_BL * r_BL * q_L`
- `dL_FR_pred = s_FR * r_FR * q_R`
- `dL_BR_pred = s_BR * r_BR * q_R`

Weighted least-squares solve per side:

- `q = sum(w_i * (s_i r_i) * dL_i) / sum(w_i * (s_i r_i)^2)`

Residual reporting:

- per-cable residual `res_i = dL_i - dL_i_pred`
- RMS error `sqrt(mean(res_i^2))`

Encoder model:

- `counts = q / (2*pi) * counts_per_output_rev`
- or `counts_per_output_rev = motor_encoder_cpr * gearbox_ratio`

## Engineering checks

The app warns about:

- impossible or poor coupling between the two cables on one motor
- cable min/max limit violations
- motor angle limit violations
- slack risk from excessive pay-out
- invalid or zero spool radii
- mirrored or asymmetric geometry
- RMS error above the configured threshold
- requested poses outside the usable workspace

## UI guide

### Geometry tab

Enter:

- moving anchors `FL`, `BL`, `FR`, `BR`
- fixed cable exit points
- rotation center
- spool radii
- winding signs
- cable weights
- counts per output revolution or motor CPR plus gearbox ratio
- cable limits, motor limits, RMS threshold, and slack pay-out threshold

### Solve tab

Two modes:

- `Pose to Cable`
- `Manual Cable Solver`

You can also:

- type current cable lengths
- enter current encoder counts
- zero the current pose to neutral
- save/load config
- run geometry analysis or a workspace sweep

Results show:

- neutral cable lengths
- target cable lengths
- ideal cable deltas
- solved `q_L` and `q_R`
- shaft angles in radians and degrees
- predicted cable deltas
- per-cable residuals
- RMS error
- target, delta, and clamped counts
- warning messages

### Analyze tab

Sweep pitch/roll over a configurable grid and view:

- max RMS error
- mean RMS error
- worst pose
- valid pose percentage
- cable length range per cable
- motor angle range
- heatmap of workspace RMS error
- table of pose, ideal deltas, motor angles, residuals, and flags

### Live FlyPT tab

Input modes:

- `manual`
- `serial`
- `udp`

Accepted input line formats:

- CSV: `pitch_deg,roll_deg`
- tagged: `P=1.5,R=-2.0`

Live tools:

- connection status
- latest received pose
- live scrolling plots
- pause graphs
- clear graphs
- export live graph data to CSV

Output modes:

- `disabled`
- `serial`
- `csv`

Serial template example:

```text
L={left_counts},R={right_counts}\n
```

Additional template fields:

- `{left_delta_counts}`
- `{right_delta_counts}`

## Manual mode

Use `Manual Cable Solver` when you have four measured cable lengths and want to estimate the current platform pose. The solver minimizes cable length error against the configured geometry and reports residual RMS so you can see whether the measurement set is physically consistent.

## Live FlyPT companion mode

Use FlyPT to generate pitch and roll, then send those values into this app over serial or UDP. The app updates the solve continuously, graphs the incoming and solved values over time, and can forward target motor commands to a controller.

Recommended FlyPT handoff:

- `Game telemetry -> FlyPT Mover -> poseTranslatorCyberRig -> controller`
- Use FlyPT UDP output to `127.0.0.1:9000`
- Use the output string `P=<PPosePitch>,R=<PPoseRoll><13><10>`
- Keep FlyPT as the telemetry and pose-generation tool; use this app only for the custom cable/spool solve

## Output to controller

The app can output:

- target counts
- delta counts
- formatted serial strings

For safety, serial output uses clamped counts when motor angle limits are active.

## Assumptions and limitations

- Strings only pull. They do not push.
- The model handles roll and pitch only.
- Slack detection is heuristic.
- Best results assume near single-layer spool winding.
- Multi-layer winding changes effective radius and can invalidate the fixed-radius model.
- The inverse solver is numerical and can report inconsistency when measured lengths do not fit the rigid-body model well.

## Why single-layer spools matter

The spool model assumes each cable sees a fixed effective radius. If the cable stacks into multiple layers, the radius changes with angle, so the simple linear relation `dL = r * q` stops being accurate. For a coupled two-cable spool, that error directly increases residual mismatch and can distort motion.

## Installation

```bash
python -m venv .venv
pip install -r requirements.txt
```

## Run

```bash
python -m app.main
```

On Windows you can also use the bundled launcher:

```bat
run_full_workflow.bat
```

The launcher creates a local virtual environment, installs requirements, runs the tests, optionally opens FlyPT Mover, opens `New.Mover` if it is found beside the repo or in the parent folder, and launches the translator app.

Or install the package and use:

```bash
pose-translator
```

## Example config

- `examples/example_config.json`

## Tests

```bash
pytest -q
```

Included tests cover:

- zero pose near-zero deltas
- symmetric geometry symmetry behavior
- counts conversion correctness
- least-squares known cases
- inverse pose recovery
- invalid config rejection
- config save/load round-trip
