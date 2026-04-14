# poseTranslatorCyberRig

A Python desktop engineering app for **custom 2DOF cable/string rigs** with two motors and coupled multi-cable spools.

## Why this exists
FlyPT Mover can output pose commands, but it does not natively solve your custom coupled spool cable geometry:

- platform pose (pitch/roll) → cable lengths
- 4 cable targets → 2 motor shaft angles (coupled least squares)
- shaft angles → encoder counts and controller commands

This app fills that gap for tuning and live operation.

## Features

- PySide6 desktop UI with tabs: **Geometry**, **Solve**, **Analyze**, **Live I/O**
- Pose-to-cable solver (forward model)
- Manual Cable Solver (inverse model: measured cable lengths → estimated pitch/roll)
- Two-spool coupling least-squares solve per side
- Predicted cable deltas, residuals, RMS mismatch reporting
- Encoder target counts and delta counts
- Engineering checks (limits, slack risk, coupling error threshold, asymmetry warnings)
- Workspace sweep + heatmap + table
- Config save/load JSON
- Live input parser from serial/UDP:
  - CSV: `pitch_deg,roll_deg`
  - Tagged: `P=1.5,R=-2.0`
- Live output formatting for controller commands

## Math model

Coordinate system:

- `x` = forward/back
- `y` = left/right
- `z` = up/down

Rotation:

- roll = `phi`
- pitch = `theta`
- `R = R_y(theta) @ R_x(phi)`

Moved anchor:

- `A_i_moved = R @ (A_i - C) + C`

Cable lengths:

- Neutral: `L0_i = ||F_i - A_i||`
- Target: `L_i = ||F_i - A_i_moved||`
- Ideal delta: `dL_i = L_i - L0_i`

Motor/spool coupling:

- left motor drives FL, BL with one shaft angle `q_L`
- right motor drives FR, BR with one shaft angle `q_R`

Predicted delta for each cable:

- `dL_i_pred = s_i * r_i * q_side`

Each side solved by weighted least squares:

- `q = sum(w_i * (s_i r_i) * dL_i) / sum(w_i * (s_i r_i)^2)`

Residual:

- `res_i = dL_i - dL_i_pred`
- RMS = `sqrt(mean(res_i^2))`

Encoder conversion:

- `counts = q/(2*pi) * counts_per_output_rev`
- if not provided directly: `counts_per_output_rev = motor_encoder_cpr * gearbox_ratio`

## Assumptions and limitations

- Strings/cables only pull, never push.
- Slack detection is heuristic from cable-length envelope.
- Solver assumes rigid-body tilt with only roll/pitch (no heave/yaw/sway/surge).
- Best behavior assumes near single-layer spool winding; multi-layer winding changes effective radius and coupling.
- Inverse manual cable solve uses iterative least squares and may fail for inconsistent/noisy measurements.

## UI field guide

### Geometry tab

- moving anchors FL/BL/FR/BR (platform points)
- fixed cable exits FL/BL/FR/BR
- rotation center
- spool radii and winding signs (+1/-1)
- encoder model fields
- cable/motor/RMS limits
- save/load config + geometry consistency check

### Solve tab

Modes:

- **Pose to Cable**: input pitch/roll, compute cable targets, motor angles, counts
- **Manual Cable Solver**: input measured cable lengths, estimate pitch/roll + residual consistency

Also supports:

- current encoder counts to compute delta counts
- zero current pose to neutral lengths
- top/side visualization with cable limit highlighting

### Analyze tab

- sweep pitch/roll grid
- summary metrics:
  - max RMS
  - mean RMS
  - worst pose
  - valid pose percentage
  - cable length ranges
  - motor angle ranges
- table of sampled poses and key outputs
- workspace RMS heatmap

### Live I/O tab

Input mode:

- manual / serial / UDP

Input line formats:

- `2.0,-1.0`
- `P=2.0,R=-1.0`

Output mode:

- disabled / serial / csv

Template example:

- `L={left_counts},R={right_counts}\n`

## Installation

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python -m app.main
```

or

```bash
pose-translator
```

## Example config

- `examples/example_config.json`

## Tests

```bash
pytest -q
```

Includes tests for:

- zero pose near-zero deltas
- symmetric geometry symmetry behavior
- encoder conversion correctness
- least-squares known case
- config invalid-data rejection
- config save/load round-trip
