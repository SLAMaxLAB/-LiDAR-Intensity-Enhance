#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path
from statistics import mean


DEFAULT_CSV = Path("/home/lb/Piont_cloudToImage/lidar_output/logs/joint_tguess_deskew_log.csv")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze joint Tguess-deskew optimization improvements."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_CSV,
        help=f"Path to joint_tguess_deskew_log.csv (default: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--skip-edge",
        type=int,
        default=300,
        help="Skip this many frames at both the beginning and the end after sorting by frame.",
    )
    parser.add_argument(
        "--include-unsolved",
        action="store_true",
        help="Include rows with solution_usable != 1.",
    )
    return parser.parse_args()


def parse_int(value, field_name):
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid integer for {field_name}: {value!r}") from exc


def parse_float(value, field_name):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid float for {field_name}: {value!r}") from exc
    if not math.isfinite(result):
        raise ValueError(f"Non-finite float for {field_name}: {value!r}")
    return result


def load_rows(csv_path):
    rows = []
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {
            "frame",
            "used_matches",
            "alpha",
            "initial_rmse",
            "final_rmse",
            "iterations",
            "time_ms",
            "solution_usable",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing required columns: {sorted(missing)}")

        for raw in reader:
            row = {
                "frame": parse_int(raw["frame"], "frame"),
                "used_matches": parse_int(raw["used_matches"], "used_matches"),
                "alpha": parse_float(raw["alpha"], "alpha"),
                "initial_rmse": parse_float(raw["initial_rmse"], "initial_rmse"),
                "final_rmse": parse_float(raw["final_rmse"], "final_rmse"),
                "iterations": parse_int(raw["iterations"], "iterations"),
                "time_ms": parse_float(raw["time_ms"], "time_ms"),
                "solution_usable": parse_int(raw["solution_usable"], "solution_usable"),
            }
            rows.append(row)
    rows.sort(key=lambda item: item["frame"])
    return rows


def summarize(rows, include_unsolved):
    absolute_improvements = []
    percentage_improvements = []
    times_ms = []
    iterations = []

    selected = 0
    dropped_nonpositive_initial = 0
    dropped_unsolved = 0

    for row in rows:
        if not include_unsolved and row["solution_usable"] != 1:
            dropped_unsolved += 1
            continue

        improvement_abs = row["initial_rmse"] - row["final_rmse"]
        absolute_improvements.append(improvement_abs)
        times_ms.append(row["time_ms"])
        iterations.append(row["iterations"])
        selected += 1

        if row["initial_rmse"] > 0.0:
            improvement_pct = improvement_abs / row["initial_rmse"] * 100.0
            percentage_improvements.append(improvement_pct)
        else:
            dropped_nonpositive_initial += 1

    if selected == 0:
        raise ValueError("No rows left after filtering.")

    return {
        "selected_rows": selected,
        "dropped_unsolved": dropped_unsolved,
        "dropped_nonpositive_initial": dropped_nonpositive_initial,
        "avg_initial_rmse": mean(row["initial_rmse"] for row in rows if include_unsolved or row["solution_usable"] == 1),
        "avg_final_rmse": mean(row["final_rmse"] for row in rows if include_unsolved or row["solution_usable"] == 1),
        "avg_improvement_abs": mean(absolute_improvements),
        "avg_improvement_pct": mean(percentage_improvements) if percentage_improvements else float("nan"),
        "avg_time_ms": mean(times_ms),
        "avg_iterations": mean(iterations),
    }


def main():
    args = parse_args()
    csv_path = args.csv.expanduser().resolve()
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    all_rows = load_rows(csv_path)
    if len(all_rows) <= 2 * args.skip_edge:
        raise ValueError(
            f"Not enough rows ({len(all_rows)}) to skip first/last {args.skip_edge} rows."
        )

    trimmed_rows = all_rows[args.skip_edge: len(all_rows) - args.skip_edge]
    result = summarize(trimmed_rows, include_unsolved=args.include_unsolved)

    first_frame = trimmed_rows[0]["frame"]
    last_frame = trimmed_rows[-1]["frame"]

    print(f"CSV: {csv_path}")
    print(f"Total rows: {len(all_rows)}")
    print(f"Skipped first/last rows: {args.skip_edge} / {args.skip_edge}")
    print(f"Analyzed frame span: {first_frame} -> {last_frame}")
    print(f"Rows after edge skip: {len(trimmed_rows)}")
    print(f"Rows used for statistics: {result['selected_rows']}")
    if not args.include_unsolved:
        print(f"Rows excluded because solution_usable != 1: {result['dropped_unsolved']}")
    if result["dropped_nonpositive_initial"] > 0:
        print(
            "Rows excluded from percentage-improvement average because initial_rmse <= 0: "
            f"{result['dropped_nonpositive_initial']}"
        )

    print("")
    print(f"Average initial RMSE: {result['avg_initial_rmse']:.6f}")
    print(f"Average final RMSE:   {result['avg_final_rmse']:.6f}")
    print(f"Average RMSE absolute improvement:   {result['avg_improvement_abs']:.6f}")
    print(f"Average RMSE percentage improvement: {result['avg_improvement_pct']:.4f}%")
    print(f"Average optimization time:           {result['avg_time_ms']:.3f} ms")
    print(f"Average iteration count:             {result['avg_iterations']:.3f}")


if __name__ == "__main__":
    main()
