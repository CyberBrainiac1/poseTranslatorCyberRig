from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
OUTER_ROOT = ROOT.parent
REPORT_PATH = ROOT / "test_reports" / "beamng_integration_report.json"
BEAMNG_EXES = [
    Path(r"C:\Program Files (x86)\Steam\steamapps\common\BeamNG.drive\BeamNG.drive.exe"),
    Path(r"C:\Program Files (x86)\Steam\steamapps\common\BeamNG.drive\Bin64\BeamNG.drive.x64.exe"),
]


def main() -> int:
    installed = [str(path) for path in BEAMNG_EXES if path.exists()]
    flypt_profile = OUTER_ROOT / "New.Mover"
    profile_text = flypt_profile.read_text(encoding="utf-8") if flypt_profile.exists() else ""
    profile_ok = "Mover.Sources.UDP.BeamNG" in profile_text and "P=&lt;PPosePitch&gt;,R=&lt;PPoseRoll&gt;&lt;13&gt;&lt;10&gt;" in profile_text

    report = {
        "beamng_found": bool(installed),
        "beamng_paths": installed,
        "flypt_profile": str(flypt_profile),
        "flypt_profile_beamng_udp_configured": profile_ok,
        "real_drive_session_executed": False,
        "status": "REAL_BEAMNG_AVAILABLE_BUT_UNATTENDED_DRIVE_NOT_EXECUTED" if installed else "BEAMNG_NOT_FOUND",
        "reason": (
            "BeamNG is installed and the FlyPT profile is configured for BeamNG UDP, but this environment cannot "
            "autonomously drive a live BeamNG scenario and verify real sim telemetry without interactive control. "
            "The mock BeamNG integration test exercises the translator pipeline deterministically."
        ),
    }
    REPORT_PATH.parent.mkdir(exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if installed:
        print("BEAMNG FOUND - real unattended driving test not executed; see report.")
    else:
        print("BEAMNG NOT FOUND - Skipping real integration. Using mock only.")
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
