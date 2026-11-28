#!/usr/bin/env python3
"""Sweep one SimpleFOC PID setting and report simulated step-response metrics."""

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile


def step_metrics(path, signal, target):
    with path.open(newline="") as file:
        samples = [
            (float(row["time_s"]), float(row[signal]),
             max(abs(float(row["id_a"])), abs(float(row["iq_a"]))))
            for row in csv.DictReader(file)
        ]
    if len(samples) < 2 or not all(math.isfinite(x) for row in samples for x in row):
        raise ValueError("Simulation CSV needs at least two finite samples")
    times, values, currents = zip(*samples)
    progress = [value / target for value in values]
    outside = [i for i, fraction in enumerate(progress) if abs(fraction - 1) > 0.02]
    settle_index = outside[-1] + 1 if outside else 0
    tail_start = times[-1] - 0.1 * (times[-1] - times[0])
    tail_errors = [abs(target - value) for time, value in zip(times, values)
                   if time >= tail_start]
    return (
        times[settle_index] if settle_index < len(times) else "",
        100 * max(0.0, max(progress) - 1),
        sum(tail_errors) / len(tail_errors),
        max(currents),
    )


def main():
    here = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("loop", choices=("current", "velocity"))
    parser.add_argument("parameter", choices=("P", "I", "D", "output_ramp"))
    parser.add_argument("values", type=float, nargs="+")
    parser.add_argument("--target", required=True, type=float, help="Step in A or rad/s")
    parser.add_argument("--config", type=Path, default=here / "configs/simplefoc_config.json")
    parser.add_argument("--binary", type=Path, default=here / "bazel-bin/pmsm_sim")
    args = parser.parse_args()
    if not math.isfinite(args.target) or args.target == 0:
        parser.error("--target must be finite and nonzero")
    if any(not math.isfinite(value) for value in args.values):
        parser.error("sweep values must be finite")

    try:
        config = json.loads(args.config.read_text())
        current = next(b for b in config["blocks"] if b["type"] == "SimpleFocCurrent")
        velocity = next(b for b in config["blocks"] if b["type"] == "SimpleFocVelocity")
        if args.loop == "current":
            plant = next(b for b in config["blocks"] if b["type"] == "ContinuousPmsm")
            plant["parameters"]["lock_rotor"] = True
            # Replace the outer loop with a q-axis current step.
            velocity_name = velocity["name"]
            velocity.clear()
            velocity.update(name=velocity_name, type="ConstantCurrent",
                            parameters={"value_a": args.target})
            config["connections"] = [
                route for route in config["connections"]
                if not route[0].startswith(velocity_name + ".")
            ]
            for route in config["connections"]:
                if route[1] == velocity_name + ".iq_ref":
                    route[1] = velocity_name + ".value"
            config["log_columns"] = [
                column for column in config["log_columns"]
                if not column[1].startswith(velocity_name + ".")
            ]
            pids = [current["parameters"][key] for key in ("PID_current_d", "PID_current_q")]
            signal = "iq_a"
        else:
            velocity["parameters"]["target_rad_s"] = args.target
            pids = [velocity["parameters"]["PID_velocity"]]
            signal = "omega_rad_s"

        with tempfile.TemporaryDirectory(prefix="simplefoc-") as directory:
            trial_config = Path(directory) / "trial.json"
            trial_csv = Path(directory) / "trial.csv"
            results = []
            for value in args.values:
                for pid in pids:
                    pid[args.parameter] = value
                trial_config.write_text(json.dumps(config))
                subprocess.run([str(args.binary.resolve()), str(trial_config), str(trial_csv)],
                               check=True, stdout=subprocess.DEVNULL)
                results.append((value, *step_metrics(trial_csv, signal, args.target)))
    except (OSError, ValueError, KeyError, StopIteration, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Tuning sweep failed ({type(error).__name__})\n{error}\n")

    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow([args.parameter, "settling_time_2pct_s", "overshoot_pct",
                     "mean_abs_tail_error_a" if args.loop == "current"
                     else "mean_abs_tail_error_rad_s", "peak_abs_d_or_q_current_a"])
    writer.writerows(results)


if __name__ == "__main__":
    main()
