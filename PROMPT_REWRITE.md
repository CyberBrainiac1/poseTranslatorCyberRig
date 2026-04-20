# Prompt Review and Clean Rewrite

## Quick review of your original prompt

Your prompt is very detailed and technically strong, but it has a few structural issues that can reduce model reliability:

1. **Large duplicated sections**: the entire physics/math + implementation brief appears twice.
2. **Potentially conflicting goals**: "implement exactly as written" and "keep iterating until everything works perfectly" can cause endless retries.
3. **Monolithic instructions**: all requirements are in one long block, which makes it easier for a model to miss constraints.
4. **No explicit output contract**: file-by-file acceptance criteria are present, but not grouped in a strict completion checklist.

## Cleaned master prompt (drop-in)

Use this version to improve execution consistency while preserving your exact engineering intent.

```text
You are implementing a complete 2DOF motion simulator translator with closed-loop potentiometer feedback.

Non-negotiable rules:
- Implement exactly the physics/math below.
- Production-quality code only.
- No placeholders, TODOs, or stubs.
- Motors must NEVER drive reverse (Option A enforced independently in PC app and firmware).
- Continue until all deliverables and quality gates pass.

Tech stack:
- Python: pyserial, PyQt5, numpy, socket, threading, queue
- Firmware: Arduino (Seeed XIAO), SoftwareSerial for Sabertooth TX

PROJECT OVERVIEW
- FlyPT Mover UDP packet format: "P=int,R=int" (0..1023, center 511.5)
- PC app parses UDP, runs kinematics + PID, sends motor bytes over serial
- XIAO reads two pots via ADC, applies safety layer + Sabertooth output

PHYSICS/MATH (IMPLEMENT EXACTLY)
1) Pose conversion
- pitch_deg = (pitch_raw - 511.5) * (PITCH_LIMIT_DEG / 511.5)
- roll_deg  = (roll_raw  - 511.5) * (ROLL_LIMIT_DEG  / 511.5)
- Convert deg→rad

2) Kinematics
- L default 266.08023 mm
- left_target_disp  = L*sin(roll_rad) + L*sin(pitch_rad)
- right_target_disp = -L*sin(roll_rad) + L*sin(pitch_rad)
- Positive = pull string, Negative = no drive (weight return)

3) Pot→position→displacement
- ADC: 0..4095, 12-bit
- shaft_angle_deg = (adc_raw / 4095.0) * 270.0
- pos = clamp((adc_raw - POT_MIN_ADC) / (POT_MAX_ADC - POT_MIN_ADC), 0..1)
- measured_disp = pos * TRAVEL_RANGE_MM
- TRAVEL_RANGE_MM = TRAVEL_RANGE_RAD * SPOOL_RADIUS_MM
- default spool radius = 27.5 mm

4) PID (per motor)
- error = target_disp - measured_disp
- output = Kp*error + Ki*integral + Kd*derivative
- derivative = (error - prev_error)/dt
- clamp integral to ±50
- defaults: Kp=2.0 Ki=0.05 Kd=0.1

5) Option A command gate
- command = clamp(pid_output, 0..127) if target_disp > DEADBAND else 0
- default deadband = 2.0 mm

6) Soft stop safety (firmware)
- SOFT_STOP_ZONE default 0.05
- if pos < zone: command *= pos/zone
- if pos > 1-zone: command *= (1-pos)/zone
- hard cutoff: if pos < 0.01 or pos > 0.99 => immediate stop bytes

7) Pot loss detection (firmware)
- if ADC is 0 or 4095 continuously >200 ms:
  - send ERR=POT_L or ERR=POT_R
  - send stop bytes
  - hold stopped until PC re-enable

8) Sabertooth mapping
- Motor1: stop 64, full fwd 127
- Motor2: stop 192, full fwd 255
- motor1_byte = int((left_command/127.0)*63 + 64)
- motor2_byte = int((right_command/127.0)*63 + 192)
- clamp on every write
- estop sends 64 and 192

9) Output smoothing
- EMA: smoothed = alpha*new + (1-alpha)*prev
- default alpha = 0.3

DELIVERABLE FILES (ALL REQUIRED)
1. translator.py
2. xiao_sabertooth.ino
3. requirements.txt (pinned)
4. config.json (defaults including pot calibration fields = 0)
5. README.md (wiring + setup + flashing + FlyPT packet format)
6. test_runner.py (all 14 tests)
7. launch.bat

translator.py requirements
- PyQt5 GUI with:
  - top row: ENABLE, DISABLE, CALIBRATE
  - left config panel (all listed params)
  - right telemetry panel at 60 Hz
  - collapsible visualization panel (side + front view)
- Launches DISABLED
- Calibration dialog with Mark Min/Max and manual jog (command=20), jog only when system DISABLED
- Open FlyPT Mover button (.fmover in app dir via os.startfile)
- Threading model:
  - UDP RX thread -> queue (drop if depth >5)
  - processing thread runs blocks 1..9
  - serial TX + serial RX threads
  - GUI QTimer at 60 Hz reads shared state with lock
- Shutdown: stop bytes, stop event, join threads, close serial
- Error handling:
  - malformed UDP skip
  - serial disconnect recoverable
  - pot loss error disables software
  - catch/log thread exceptions without GUI crash
- Autostart behavior:
  - if autostart.enable exists at launch: delete file, wait 2s, auto-enable

xiao_sabertooth.ino requirements
- Pins/constants:
  - SABERTOOTH_TX_PIN D6
  - POT_LEFT_PIN A0
  - POT_RIGHT_PIN A1
  - WATCHDOG_TIMEOUT_MS 500
  - SOFT_STOP_ZONE 0.05
  - POT_LOSS_TIMEOUT_MS 200
- setup:
  - send stop bytes first
  - analogReadResolution(12)
- loop:
  - read pots every cycle
  - pot loss detection + ERR serial message + hold stopped
  - send "L=adc_left,R=adc_right\n" each cycle
  - read incoming motor bytes
  - apply firmware soft stop using EEPROM calibration
  - clamp output bytes and write Sabertooth
  - watchdog timeout -> stop + fast LED blink
- LED states:
  - idle slow blink 1000 ms
  - receiving solid
  - watchdog fast blink 100 ms
- EEPROM:
  - store POT_MIN_L/POT_MAX_L/POT_MIN_R/POT_MAX_R
  - accept calibration command:
    CAL=L_MIN:val,L_MAX:val,R_MIN:val,R_MAX:val\n
TEST REQUIREMENTS (test_runner.py)
Implement and report PASS/FAIL for tests 1..14 exactly as specified:
1 neutral, 2 full pitch forward, 3 full pitch back, 4 full roll right, 5 full roll left,
6 forward+roll right, 7 forward+roll left, 8 rapid oscillation queue drop,
9 malformed packet, 10 empty bytes, 11 pot loss disable,
12 soft stop scaling at 97%, 13 PID windup clamp, 14 calibration persistence.

QUALITY GATES (must all pass)
- Robust error handling in every function
- No silent thread crashes
- Recoverable serial/UDP failures
- Clamp bytes at every write
- Option A enforced in both PC and firmware
- Pot loss disable in both PC and firmware
- PID windup clamp active
- Soft stop enforced in both layers
- Calibration persists across restart
- Clean startup with no serial device
- Clean startup with no FlyPT source

FINAL OUTPUT FORMAT
- First: concise implementation summary by file
- Second: test results table with commands and outcomes
- Third: explicit statement that all quality gates passed
```

## Optional tightening for even better model behavior

- Add: "If any requirement conflicts, prioritize safety constraints over performance and UI polish."
- Add: "If hardware is unavailable, use deterministic mocks and clearly mark mock-based tests."
- Add explicit timeout budget for test loops to prevent runaway retries.
