# 2DOF String Motion Translator

This folder contains a complete FlyPT-to-Sabertooth translator for a 2DOF string-pulled sim racing motion rig.

Signal path:

```text
FlyPT Mover UDP -> translator.py -> USB serial -> Seeed Studio XIAO -> Sabertooth 2x32 -> brushed DC motors
```

The rig uses Option A: one front-pull string per motor. Positive displacement pulls the platform. Negative displacement is never driven by the motors; rider weight returns the frame. Reverse drive is blocked in both the Python app and the XIAO sketch.

## Files

- `translator.py` - PyQt5 GUI, UDP receiver, kinematics, PID control loop, calibration, safety clamps, and serial output.
- `xiao_sabertooth.ino` - Arduino sketch for the Seeed Studio XIAO.
- `requirements.txt` - pinned Python dependencies.
- `config.json` - default runtime configuration.
- `README.md` - setup, wiring, flashing, and FlyPT instructions.
- `launch.bat` - Windows launcher that starts the GUI with `pythonw.exe` and triggers delayed auto-enable.
- `test_suite.py` - standalone math, safety, and phantom UDP pipeline tests.
- `pid_test_suite.py` - closed-loop PID, soft stop, hard stop, and reset validation tests.
- `deep_workflow_tests.py` - deeper app workflow tests for FlyPT/BeamNG profile checks, UDP bursts, serial output capture, and visualizer rendering.
- `test_custom_stress_suite.py` - 20 custom stress tests runnable with pytest.
- `mock_beamng_integration_test.py` - deterministic 120-second mock BeamNG lap through translator math.
- `generate_final_report.py` - final validation report generator.

## Wiring

Text wiring diagram:

```text
PC USB port
  |
  | USB cable
  v
Seeed Studio XIAO
  GND ------------------------ Sabertooth 0V / signal ground
  D6  ------------------------ Sabertooth S1
  A0  ------------------------ Left motor 10k pot wiper
  A1  ------------------------ Right motor 10k pot wiper
  3V3 ------------------------ Pot high side
  GND ------------------------ Pot low side
  5V or 3V3 is NOT connected -- Sabertooth is powered from motor battery/BEC side

Sabertooth 2x32
  B+  ------------------------ Motor battery positive
  B-  ------------------------ Motor battery negative
  M1A/M1B -------------------- Left motor
  M2A/M2B -------------------- Right motor
  0V  ------------------------ XIAO GND
  S1  ------------------------ XIAO D6
```

Sabertooth setup:

```text
Mode: simplified serial
Baud: 9600
Input: S1 receives serial bytes from XIAO D6
```

Keep the motor power wiring sized and fused for the actual motor current. Tie XIAO ground and Sabertooth signal ground together.
Wire each potentiometer mechanically to the matching motor shaft. The default firmware mapping is left pot on `A0` and right pot on `A1`.

## Python Setup

Use Python 3.9 through 3.12. PyQt5 wheels may not be available yet for newer Python releases.

```powershell
cd C:\Users\emmad\Downloads\CodeP\posetranslator
python -m venv venv
.\venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python translator.py
```

The app launches disabled. Select the XIAO serial port, confirm the baud rate is `9600`, then press `Apply and Save`. Press `ENABLE` only when the rig is mechanically safe to move. Press `DISABLE` to immediately send Sabertooth stop bytes `64` and `192`.

The app starts cleanly with no serial port connected and with FlyPT not running. The status bar shows UDP state, serial state, and the latest recoverable error.

Run the standalone test suite with:

```powershell
python test_suite.py
python pid_test_suite.py
python deep_workflow_tests.py
python -m pytest test_custom_stress_suite.py -v
python mock_beamng_integration_test.py
python generate_final_report.py
```

The test suites import the math directly from `translator.py`, check the no-reverse constraints, run phantom UDP loops with mocked serial output, verify the FlyPT BeamNG profile, and render-check the PyQt visualizer.

## Windows Launcher

Double-click:

```text
launch.bat
```

The launcher:

```text
1. Resolves the folder that contains launch.bat.
2. Uses venv\Scripts\activate.bat if that virtual environment exists.
3. Checks for pythonw.exe and shows a readable error if Python is missing.
4. Starts translator.py in the background with pythonw.exe.
5. Waits 2 seconds.
6. Writes autostart.enable beside translator.py.
```

`translator.py` watches for `autostart.enable` during startup, deletes it immediately when detected, waits 2 seconds after UI load, then clicks `ENABLE` exactly through the same path as the GUI button.

Because the batch file uses its own location instead of the current working directory, it works when double-clicked from Explorer, launched through a shortcut, or launched from a pinned shortcut.

## Pinning and Custom Icon

Windows stores custom icons on shortcuts, not inside `.bat` files. Use this flow:

```text
1. Right-click launch.bat.
2. Choose Show more options > Create shortcut.
3. Rename the shortcut to Motion Translator.
4. Right-click the shortcut > Properties.
5. On the Shortcut tab, choose Change Icon.
6. Browse to your custom .ico file and select it.
7. Click OK, then Apply.
8. Right-click the shortcut and choose Pin to Start.
9. For the taskbar, drag the shortcut to the taskbar or right-click it and choose Pin to taskbar if Windows shows that option.
```

Keep the shortcut target pointed at:

```text
C:\Users\emmad\Downloads\CodeP\posetranslator\launch.bat
```

If Windows will not pin a `.bat` shortcut directly to the taskbar, create a shortcut with this target instead and set the same custom icon:

```text
C:\Windows\System32\cmd.exe /c ""C:\Users\emmad\Downloads\CodeP\posetranslator\launch.bat""
```

## GUI Parameters

Stored in `config.json`:

```text
UDP Port: 9000
Serial Port: selected XIAO COM port
Baud Rate: 9600
Spool Diameter mm: 55
L pivot-to-attachment mm: 266.08023
Pitch Limit deg: 30
Roll Limit deg: 30
Max Velocity mm/s: 200
Deadband mm: 2
Kp: 2.0
Ki: 0.05
Kd: 0.1
Integral Clamp: 50
Alpha Smoothing: 0.3
Soft Stop Zone percent: 5
Left Pot ADC Min/Max: 0 / 4095
Right Pot ADC Min/Max: 0 / 4095
Left/Right Travel Range mm: computed during calibration
```

`Spool Diameter mm` is used during calibration to convert usable pot rotation into string travel. The target displacement still comes from the supplied pre-solved `L * sin(angle)` kinematics.

## Calibration Procedure

Calibration is disabled while the rig is enabled. Press `DISABLE` first, then press `Calibrate`.

```text
1. Confirm the XIAO is connected and sending live ADC values.
2. Move the rig to MINIMUM position: strings fully extended, frame at lowest travel.
3. Click Mark Minimum.
4. Move the rig to MAXIMUM position: strings pulled, frame at highest travel.
5. Click Mark Maximum.
6. Click Done.
```

The app validates that each side has more than `100` ADC counts of range. It then computes per-side travel:

```text
travel_range_rad = (adc_max - adc_min) / 4095.0 * (270 deg in radians)
travel_range_mm = travel_range_rad * (spool_diameter_mm / 2.0)
```

The PC saves calibration to `config.json`, sends `CAL=L_MIN:xxx,L_MAX:xxx,R_MIN:xxx,R_MAX:xxx` to the XIAO, and waits for `CAL_OK`. The XIAO stores the same calibration in EEPROM with magic byte `0xAB`.

## PID Tuning Guide

Recommended starting point:

```text
Kp = 2.0
Ki = 0.05
Kd = 0.1
Alpha = 0.3
```

Tuning symptoms:

```text
Kp too low: rig responds sluggishly, takes long to reach target
Kp too high: rig overshoots target, oscillates around it
Ki too low: rig never quite reaches target, steady state error remains
Ki too high: rig oscillates, integral winds up, unstable
Kd too low: rig overshoots on fast moves
Kd too high: rig is jerky, amplifies noise
Alpha too low: rig feels sluggish and damped, slow reaction
Alpha too high: rig is jerky, feels every noise spike
```

Tune `Kp` first until response is fast without oscillation. Add `Ki` until steady state error disappears. Add `Kd` to reduce overshoot. Finally tune `Alpha` to taste.

## FlyPT Mover Setup

1. Open FlyPT Mover.
2. Add or edit a UDP output.
3. Set the target IP to `127.0.0.1`.
4. Set the target port to match `UDP Port` in `config.json`, default `9000`.
5. Set the output string exactly to:

```text
P=<PPosePitch>,R=<PPoseRoll><13><10>
```

The Python app expects packets like:

```text
P=512,R=512
```

Both values must be integers from `0` to `1023`, with center at `511.5`.

The `Open FlyPT Mover` button looks in the same directory as `translator.py` for a file ending in `.fmover` and opens the first match. Put your FlyPT project file in this folder if you want that button to work.

## Flashing the XIAO

1. Open Arduino IDE.
2. Install the board support for your Seeed Studio XIAO variant.
3. Open `xiao_sabertooth.ino`.
4. Select the XIAO board and port.
5. Confirm the sketch uses `#define SABERTOOTH_TX_PIN D6`.
6. Confirm the sketch uses `#define POT_LEFT_PIN A0` and `#define POT_RIGHT_PIN A1`.
7. Upload the sketch.
8. After upload, reconnect the XIAO if the COM port changes.

The PC sends two raw bytes per packet over USB serial:

```text
motor1_byte, motor2_byte
```

The XIAO clamps every received byte before forwarding it:

```text
Motor 1: 64 to 127
Motor 2: 192 to 255
```

Stop bytes are:

```text
Motor 1 stop: 64
Motor 2 stop: 192
```

The XIAO sends pot feedback to the PC continuously:

```text
L=<left_adc>,R=<right_adc>
```

If either pot reads `0` or `4095` continuously for more than `200 ms`, the XIAO sends `ERR=POT_L` or `ERR=POT_R`, sends stop bytes, and holds stopped until the PC is manually re-enabled.

## XIAO Watchdog and LED

The XIAO sends stop bytes as its first Sabertooth action during setup.

Runtime behavior:

```text
Valid packet received: apply byte clamps and stop-zone scaling, forward to Sabertooth, reset watchdog
No packet for 500 ms: send stop bytes, enter watchdog state
Pot loss for 200 ms: send stop bytes, report pot error, hold stopped
```

LED behavior:

```text
Slow blink, 1000 ms period: idle or disabled
Solid on: receiving commands actively
Fast blink, 100 ms period: watchdog triggered
```

## Safety Behavior

The translator computes:

```text
pitch_deg = (pitch_raw - 511.5) * (PITCH_LIMIT_DEG / 511.5)
roll_deg  = (roll_raw  - 511.5) * (ROLL_LIMIT_DEG  / 511.5)

left_disp  = L * sin(roll_rad) + L * sin(pitch_rad)
right_disp = -L * sin(roll_rad) + L * sin(pitch_rad)
```

Positive displacement means the string must shorten, so the motor may pull. Negative displacement means the platform must return by rider weight, so command is forced to zero.

The PC PID loop runs independently for left and right:

```text
error = target_disp - measured_disp
raw_output = Kp * error + Ki * integral + Kd * derivative
```

After PID, Option A is applied before smoothing and stop-zone scaling:

```text
if target_disp > deadband:
  command = clamp(raw_output, 0, 127)
else:
  command = 0
  integral = 0
```

The PC also holds both commands at stop until fresh XIAO pot feedback has been received.

Sabertooth byte mapping:

```text
motor1_byte = int((left_command / 127.0) * 63 + 64)
motor2_byte = int((right_command / 127.0) * 63 + 192)
```

All output paths clamp the final bytes before writing:

```text
Motor 1 never below 64 and never above 127
Motor 2 never below 192 and never above 255
Disable and watchdog always send 64, 192
```

## Operating Checklist

1. Lift or mechanically unload the rig for the first test.
2. Start `translator.py`.
3. Select the XIAO COM port and press `Apply and Save`.
4. Start FlyPT Mover and confirm it is sending `P=<int>,R=<int>` packets.
5. Press `ENABLE`.
6. Watch live telemetry for pitch, roll, displacement, commands, serial bytes, and update rate.
7. Press `DISABLE` before changing wiring, flashing firmware, or closing the app.

On app close, the Python app sends stop bytes, stops both worker threads, joins them, and closes the serial port.
