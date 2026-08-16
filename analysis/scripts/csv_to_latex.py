#!/usr/bin/env python3
"""Turn a generated results CSV (e.g. analysis/results/resilience_summary.csv) into a
LaTeX booktabs table for paper/tables/ - a pure, mechanical formatting step so numbers
in the paper are always a direct, reproducible copy of what generate_report.py actually
measured, never hand-retyped (and therefore never silently drifting from the source of
truth in experiments/).

    python3 analysis/scripts/csv_to_latex.py analysis/results/resilience_summary.csv \
        --caption "Mode B/C/D/E resilience comparison" \
        --label tab:resilience \
        --out paper/tables/resilience_summary.tex
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List


def _escape(cell: str) -> str:
    # Only the LaTeX-special characters this project's own generated CSVs can
    # plausibly contain (underscores in mode names, '%' in "%SOC/task", '±').
    return (
        cell.replace("\\", r"\textbackslash{}")
        .replace("&", r"\&")
        .replace("%", r"\%")
        .replace("_", r"\_")
        .replace("±", r"$\pm$")
    )


def csv_to_latex_table(rows: List[List[str]], *, caption: str, label: str) -> str:
    if not rows:
        raise ValueError("no rows to render")
    header, *body = rows
    col_spec = "l" + "r" * (len(header) - 1)
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        f"\\caption{{{_escape(caption)}}}",
        f"\\label{{{label}}}",
        # \width here refers to the tabular's own natural width (a graphicx
        # \resizebox feature) - this only shrinks the table when it's wider than
        # the text block (e.g. a wide, many-column results table), and leaves a
        # naturally-narrow table untouched rather than stretching it to fill the
        # page. Required package: graphicx (already in paper/main.tex).
        r"\resizebox{\ifdim\width>\linewidth\linewidth\else\width\fi}{!}{%",
        f"\\begin{{tabular}}{{{col_spec}}}",
        r"\toprule",
        " & ".join(_escape(c) for c in header) + r" \\",
        r"\midrule",
    ]
    for row in body:
        lines.append(" & ".join(_escape(c) for c in row) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}%", r"}", r"\end{table}"]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--caption", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    # Explicit UTF-8 on both ends: the source CSV may contain "±" written on a
    # different platform (WSL2 default UTF-8 vs. Windows default cp1252) - without
    # this, the byte sequence mis-decodes into mojibake instead of raising, so the
    # bad table would only be caught by eye, not by an error.
    with open(args.csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))

    tex = csv_to_latex_table(rows, caption=args.caption, label=args.label)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(tex, encoding="utf-8")
    print(f"[csv_to_latex] wrote {out_path}")


if __name__ == "__main__":
    main()
