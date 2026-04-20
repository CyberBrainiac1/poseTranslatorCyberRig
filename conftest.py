from __future__ import annotations

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
REPORTS = ROOT / "test_reports"
SUMMARY_PATH = REPORTS / "test_results_summary.json"
DETAILS_PATH = REPORTS / "test_details.json"
HTML_PATH = REPORTS / "report.html"


def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    REPORTS.mkdir(exist_ok=True)
    tr = session.config.pluginmanager.get_plugin("terminalreporter")
    summary: dict[str, dict[str, int]] = {}
    details: list[dict[str, object]] = []

    for outcome in ("passed", "failed", "skipped"):
        for rep in tr.stats.get(outcome, []):
            if getattr(rep, "when", "") != "call":
                continue
            name = rep.nodeid.split("::")[-1]
            section = name.split("_")[1][0].upper() if "_" in name and len(name.split("_")) > 1 else "A"
            summary.setdefault(section, {"PASSED": 0, "FAILED": 0, "SKIPPED": 0})
            summary[section][outcome.upper()] += 1
            details.append({"test": rep.nodeid, "outcome": outcome.upper(), "duration_s": rep.duration})

    SUMMARY_PATH.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    DETAILS_PATH.write_text(json.dumps(details, indent=2) + "\n", encoding="utf-8")

    rows = "\n".join(
        f"<tr><td>{s}</td><td>{v['PASSED']}</td><td>{v['FAILED']}</td><td>{v['SKIPPED']}</td></tr>"
        for s, v in sorted(summary.items())
    )
    html = (
        "<html><body><h1>Cloud Test Summary</h1><table border='1'>"
        "<tr><th>Section</th><th>Passed</th><th>Failed</th><th>Skipped</th></tr>"
        f"{rows}</table></body></html>"
    )
    HTML_PATH.write_text(html, encoding="utf-8")
