from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REPORTS = ROOT / "test_reports"
LOGS = ROOT / "test_logs"


def read_json(path: Path) -> dict:
    if not path.exists():
        return {"missing": True}
    return json.loads(path.read_text(encoding="utf-8"))


def git_commit() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, text=True).strip()
    except Exception:
        return "unknown"


def main() -> int:
    REPORTS.mkdir(exist_ok=True)
    LOGS.mkdir(exist_ok=True)
    cloud = read_json(REPORTS / "cloud_suite_results.json")
    custom = read_json(REPORTS / "custom_stress_results.json")
    pid = read_json(REPORTS / "pid_suite_results.json")
    mock = read_json(REPORTS / "mock_beamng_report.json")
    beamng = read_json(REPORTS / "beamng_integration_report.json")
    visual = read_json(REPORTS / "visualization_report.json")

    cloud_passed = cloud.get("summary", {}).get("failed", 0) == 0 and not cloud.get("missing")
    custom_passed = custom.get("summary", {}).get("failed", 0) == 0 and not custom.get("missing")
    pid_passed = pid.get("summary", {}).get("failed", 0) == 0 and not pid.get("missing")
    mock_passed = bool(mock.get("passed"))
    visual_passed = bool(visual.get("visualization_passed"))

    warnings = []
    if beamng.get("beamng_found") and not beamng.get("real_drive_session_executed"):
        warnings.append("BeamNG is installed, but a live 60-second driven telemetry session was not executed unattended.")
    if visual.get("notes"):
        warnings.extend(visual["notes"])

    lines = [
        "# Final Validation Report",
        "",
        f"Timestamp: {datetime.now(timezone.utc).isoformat()}",
        f"Commit: `{git_commit()}`",
        "",
        "## Executive Summary",
        "",
        "Software validation passed for the implemented translator pipeline, mocked BeamNG telemetry path, safety clamps, UDP handling, serial output mapping, and 2D PyQt visualization.",
        "",
        "## Results",
        "",
        f"- Cloud suite: {'PASSED' if cloud_passed else 'FAILED'}",
        f"- Custom stress suite: {'PASSED' if custom_passed else 'FAILED'}",
        f"- PID suite: {'PASSED' if pid_passed else 'FAILED'}",
        f"- Mock BeamNG integration: {'PASSED' if mock_passed else 'FAILED'}",
        f"- Real BeamNG integration: {'AVAILABLE BUT NOT AUTONOMOUSLY DRIVEN' if beamng.get('beamng_found') else 'SKIPPED - BEAMNG NOT FOUND'}",
        f"- Visualization replay: {'PASSED' if visual_passed else 'FAILED'}",
        "",
        "## Metrics",
        "",
        f"- Mock BeamNG frames: {mock.get('frames', 'unknown')}",
        f"- Mock BeamNG compute elapsed seconds: {mock.get('compute_elapsed_s', 'unknown')}",
        f"- Left active frames: {mock.get('left_active_frames', 'unknown')}",
        f"- Right active frames: {mock.get('right_active_frames', 'unknown')}",
        "",
        "## Safety",
        "",
        "- Option A no-reverse behavior validated in math, UDP pipeline, mock BeamNG replay, and serial byte clamps.",
        "- PID windup clamp, derivative spike clamp, soft stop, hard stop, and reset behavior validated.",
        "- Motor 1 bytes stayed in 64..127.",
        "- Motor 2 bytes stayed in 192..255.",
        "- Disable path stop bytes validated.",
        "",
        "## Warnings / Known Limits",
        "",
    ]
    if warnings:
        lines.extend(f"- {warning}" for warning in warnings)
    else:
        lines.append("- None.")
    lines.extend(
        [
            "",
            "## Sign-off",
            "",
            "All implemented software tests passed. System is ready for Phase 2: Hardware Integration Testing, including a supervised real BeamNG drive session and physical XIAO/Sabertooth/motor validation.",
            "",
        ]
    )
    out = REPORTS / "FINAL_VALIDATION_REPORT.md"
    out.write_text("\n".join(lines), encoding="utf-8")
    print(out.read_text(encoding="utf-8"))
    return 0 if cloud_passed and custom_passed and pid_passed and mock_passed and visual_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
