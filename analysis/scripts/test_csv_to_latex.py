import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from csv_to_latex import csv_to_latex_table  # noqa: E402


def test_renders_header_and_body_rows():
    rows = [["Mode", "Recovery"], ["No Recovery", "0/1"], ["Deterministic", "1/1"]]
    tex = csv_to_latex_table(rows, caption="Test", label="tab:test")
    assert r"\begin{table}" in tex
    assert r"\end{table}" in tex
    assert "No Recovery" in tex
    assert "Deterministic" in tex
    assert tex.count(r"\\") == 3  # header + 2 body rows


def test_escapes_latex_special_characters():
    rows = [["Mode", "Energy/Task"], ["Deterministic", "5.2%SOC/task"]]
    tex = csv_to_latex_table(rows, caption="A_B & C", label="tab:x")
    assert r"5.2\%SOC/task" in tex
    assert r"A\_B \& C" in tex


def test_column_spec_matches_header_width():
    rows = [["Mode", "A", "B", "C"], ["x", "1", "2", "3"]]
    tex = csv_to_latex_table(rows, caption="c", label="l")
    assert r"\begin{tabular}{lrrr}" in tex


def test_raises_on_empty_rows():
    import pytest

    with pytest.raises(ValueError):
        csv_to_latex_table([], caption="c", label="l")


def test_main_reads_utf8_csv_without_mojibake(tmp_path):
    # Regression: reading the CSV without an explicit encoding defaults to the
    # platform locale (cp1252 on Windows), which mis-decodes a UTF-8 "±" byte
    # sequence into mojibake instead of the correct character - caught live when
    # a WSL2-written resilience_summary.csv was converted on Windows.
    import subprocess
    import sys as _sys

    csv_path = tmp_path / "in.csv"
    csv_path.write_bytes("Mode,Recovery Time\nDeterministic,28.04s (±1.98)\n".encode("utf-8"))
    out_path = tmp_path / "out.tex"

    subprocess.run(
        [
            _sys.executable, str(Path(__file__).resolve().parent / "csv_to_latex.py"),
            str(csv_path), "--caption", "c", "--label", "l", "--out", str(out_path),
        ],
        check=True,
    )
    tex = out_path.read_text(encoding="utf-8")
    assert r"$\pm$" in tex
    assert "�" not in tex  # the mojibake replacement character
