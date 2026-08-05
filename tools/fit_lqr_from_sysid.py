#!/usr/bin/env python3
"""Identify per-leg-length LQR gains from physical sysid logs.

Input CSV uses the legacy ``lqr_sysid_csv`` log format. After filtering out
the leading prefix, the script fits, for each leg-length point:

    x[k+1] = A x[k] + B u[k] + c

and then solves the discrete infinite-horizon LQR problem. Printed K gains use
the firmware convention u = -K x.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


STATE_COLUMNS = [
    "theta_mrad",
    "theta_rate_mradps",
    "x_mm",
    "x_rate_mms",
    "pitch_mrad",
    "pitch_rate_mradps",
]
INPUT_COLUMNS = ["u_wheel_mNm", "u_leg_mNm"]
RAW_COLUMNS = [
    "t_ms",
    "point_mm",
    "phase",
    "sample",
    *STATE_COLUMNS,
    *INPUT_COLUMNS,
    "len_l_mm",
    "len_r_mm",
    "roll_mrad",
    "yaw_rate_mradps",
    "valid",
]
STATE_SIZE = 6
INPUT_SIZE = 2
FEATURE_SIZE = STATE_SIZE + INPUT_SIZE + 1


def parse_diag(text: str, size: int, name: str) -> list[float]:
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if len(values) != size:
        raise ValueError(f"{name} must contain {size} comma-separated values")
    if any((not math.isfinite(value)) or value <= 0.0 for value in values):
        raise ValueError(f"{name} values must be positive and finite")
    return values


def zeros(rows: int, cols: int) -> list[list[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]


def identity(size: int) -> list[list[float]]:
    matrix = zeros(size, size)
    for i in range(size):
        matrix[i][i] = 1.0
    return matrix


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*matrix)]


def matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    rows = len(left)
    cols = len(right[0])
    inner = len(right)
    out = zeros(rows, cols)
    for row in range(rows):
        for mid in range(inner):
            value = left[row][mid]
            if value == 0.0:
                continue
            for col in range(cols):
                out[row][col] += value * right[mid][col]
    return out


def matadd(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [left[row][col] + right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def matsub(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [left[row][col] - right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def diag(values: list[float]) -> list[list[float]]:
    matrix = zeros(len(values), len(values))
    for i, value in enumerate(values):
        matrix[i][i] = value
    return matrix


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    a = [row[:] + [vector[i]] for i, row in enumerate(matrix)]

    for pivot in range(size):
        best = max(range(pivot, size), key=lambda row: abs(a[row][pivot]))
        if abs(a[best][pivot]) < 1e-12:
            raise ValueError("singular linear system; add more excitation data")
        if best != pivot:
            a[pivot], a[best] = a[best], a[pivot]

        scale = a[pivot][pivot]
        for col in range(pivot, size + 1):
            a[pivot][col] /= scale

        for row in range(size):
            if row == pivot:
                continue
            factor = a[row][pivot]
            if factor == 0.0:
                continue
            for col in range(pivot, size + 1):
                a[row][col] -= factor * a[pivot][col]

    return [a[row][size] for row in range(size)]


def invert(matrix: list[list[float]]) -> list[list[float]]:
    size = len(matrix)
    columns = []
    eye = identity(size)
    for col in range(size):
        rhs = [eye[row][col] for row in range(size)]
        columns.append(solve_linear_system(matrix, rhs))
    return transpose(columns)


def row_to_sample(row: dict[str, str],
                  max_leg_error_mm: float | None,
                  max_theta_rate_mradps: float | None,
                  drop_initial_samples: int) -> tuple[int, list[float], list[float]] | None:
    if row["phase"] != "sample" or int(row["valid"]) == 0:
        return None
    if int(row["sample"]) < drop_initial_samples:
        return None
    point_mm = int(row["point_mm"])
    if max_leg_error_mm is not None:
        left_error = abs(float(row["len_l_mm"]) - point_mm)
        right_error = abs(float(row["len_r_mm"]) - point_mm)
        if max(left_error, right_error) > max_leg_error_mm:
            return None
    if max_theta_rate_mradps is not None:
        if abs(float(row["theta_rate_mradps"])) > max_theta_rate_mradps:
            return None
    state = [float(row[name]) / 1000.0 for name in STATE_COLUMNS]
    control = [float(row[name]) / 1000.0 for name in INPUT_COLUMNS]
    if not all(math.isfinite(value) for value in [*state, *control]):
        return None
    return point_mm, state, control


def read_raw_rows(path: Path,
                  max_leg_error_mm: float | None,
                  max_theta_rate_mradps: float | None,
                  drop_initial_samples: int) -> dict[int, list[tuple[list[float], list[float]]]]:
    rows_by_point: dict[int, list[tuple[list[float], list[float]]]] = defaultdict(list)
    with path.open(newline="") as file:
        for line_number, line in enumerate(file, start=1):
            line = line.strip()
            if not line or not line.startswith("lqr_sysid_csv,"):
                continue
            values = line.split(",")[1:]
            if len(values) != len(RAW_COLUMNS):
                raise ValueError(
                    f"raw line {line_number} has {len(values)} columns, "
                    f"expected {len(RAW_COLUMNS)}")
            row = dict(zip(RAW_COLUMNS, values))
            sample = row_to_sample(
                row, max_leg_error_mm, max_theta_rate_mradps,
                drop_initial_samples)
            if sample is None:
                continue
            point_mm, state, control = sample
            rows_by_point[point_mm].append((state, control))
    return dict(rows_by_point)


def read_csv_rows(path: Path,
                  max_leg_error_mm: float | None,
                  max_theta_rate_mradps: float | None,
                  drop_initial_samples: int) -> dict[int, list[tuple[list[float], list[float]]]]:
    rows_by_point: dict[int, list[tuple[list[float], list[float]]]] = defaultdict(list)
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        missing = [name for name in ["point_mm", "phase", "valid", *STATE_COLUMNS, *INPUT_COLUMNS]
                   if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(missing)}")
        for row in reader:
            sample = row_to_sample(
                row, max_leg_error_mm, max_theta_rate_mradps,
                drop_initial_samples)
            if sample is None:
                continue
            point_mm, state, control = sample
            rows_by_point[point_mm].append((state, control))
    return dict(rows_by_point)


def read_rows(path: Path,
              max_leg_error_mm: float | None,
              max_theta_rate_mradps: float | None,
              drop_initial_samples: int) -> dict[int, list[tuple[list[float], list[float]]]]:
    with path.open(newline="") as file:
        for line in file:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("lqr_sysid_csv,"):
                return read_raw_rows(
                    path, max_leg_error_mm, max_theta_rate_mradps,
                    drop_initial_samples)
            return read_csv_rows(
                path, max_leg_error_mm, max_theta_rate_mradps,
                drop_initial_samples)
    return {}


def fit_discrete_model(samples: list[tuple[list[float], list[float]]],
                       ridge: float) -> tuple[list[list[float]], list[list[float]], float]:
    if len(samples) < FEATURE_SIZE + 2:
        raise ValueError(f"need at least {FEATURE_SIZE + 2} valid rows per point")

    normal = zeros(FEATURE_SIZE, FEATURE_SIZE)
    rhs = zeros(FEATURE_SIZE, STATE_SIZE)
    transitions = 0
    for (state, control), (next_state, _) in zip(samples, samples[1:]):
        feature = [*state, *control, 1.0]
        for i in range(FEATURE_SIZE):
            rhs[i] = [rhs[i][col] + feature[i] * next_state[col]
                      for col in range(STATE_SIZE)]
            for j in range(FEATURE_SIZE):
                normal[i][j] += feature[i] * feature[j]
        transitions += 1

    for i in range(FEATURE_SIZE):
        normal[i][i] += ridge

    coefficients = zeros(FEATURE_SIZE, STATE_SIZE)
    for col in range(STATE_SIZE):
        solution = solve_linear_system(normal, [rhs[row][col] for row in range(FEATURE_SIZE)])
        for row, value in enumerate(solution):
            coefficients[row][col] = value

    a = zeros(STATE_SIZE, STATE_SIZE)
    b = zeros(STATE_SIZE, INPUT_SIZE)
    for state_out in range(STATE_SIZE):
        for state_in in range(STATE_SIZE):
            a[state_out][state_in] = coefficients[state_in][state_out]
        for control_in in range(INPUT_SIZE):
            b[state_out][control_in] = coefficients[STATE_SIZE + control_in][state_out]

    squared_error = 0.0
    count = 0
    for (state, control), (next_state, _) in zip(samples, samples[1:]):
        predicted = [
            sum(a[row][col] * state[col] for col in range(STATE_SIZE)) +
            sum(b[row][col] * control[col] for col in range(INPUT_SIZE)) +
            coefficients[FEATURE_SIZE - 1][row]
            for row in range(STATE_SIZE)
        ]
        for actual, estimate in zip(next_state, predicted):
            squared_error += (actual - estimate) ** 2
            count += 1

    rms = math.sqrt(squared_error / max(1, count))
    return a, b, rms


def solve_lqr(a: list[list[float]], b: list[list[float]],
              q_diag: list[float], r_diag: list[float],
              iterations: int, tolerance: float) -> list[list[float]]:
    q = diag(q_diag)
    r = diag(r_diag)
    p = [row[:] for row in q]
    at = transpose(a)
    bt = transpose(b)
    gain = zeros(INPUT_SIZE, STATE_SIZE)

    for _ in range(iterations):
        pb = matmul(p, b)
        s = matadd(r, matmul(bt, pb))
        k = matmul(matmul(invert(s), bt), matmul(p, a))
        next_p = matadd(q, matsub(matmul(matmul(at, p), a),
                                  matmul(matmul(matmul(at, p), b), k)))
        delta = max(abs(next_p[row][col] - p[row][col])
                    for row in range(STATE_SIZE)
                    for col in range(STATE_SIZE))
        p = next_p
        gain = k
        if delta < tolerance:
            break
    return gain


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="Filtered lqr_sysid_csv file")
    parser.add_argument("--q", default="60,2,8,1,120,4",
                        help="State cost diagonal for theta,theta_rate,x,x_rate,pitch,pitch_rate")
    parser.add_argument("--r", default="0.3,0.3",
                        help="Input cost diagonal for wheel torque, leg torque")
    parser.add_argument("--ridge", type=float, default=1e-8,
                        help="Small least-squares regularization")
    parser.add_argument("--max-leg-error-mm", type=float, default=None,
                        help="Drop rows whose left/right leg length is farther than this from point_mm")
    parser.add_argument("--max-theta-rate-mradps", type=float, default=None,
                        help="Drop rows whose absolute theta_rate exceeds this value")
    parser.add_argument("--drop-initial-samples", type=int, default=0,
                        help="Drop this many initial sample rows after each settle phase")
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument("--tolerance", type=float, default=1e-9)
    args = parser.parse_args()

    q_diag = parse_diag(args.q, STATE_SIZE, "q")
    r_diag = parse_diag(args.r, INPUT_SIZE, "r")
    rows_by_point = read_rows(
        args.csv, args.max_leg_error_mm, args.max_theta_rate_mradps,
        args.drop_initial_samples)
    if not rows_by_point:
        raise ValueError("no valid sample rows found")

    print("leg_length_mm," + ",".join(
        f"k{input_index}{state}" for input_index in range(INPUT_SIZE)
        for state in range(STATE_SIZE)))
    for point_mm in sorted(rows_by_point):
        samples = rows_by_point[point_mm]
        a, b, rms = fit_discrete_model(samples, args.ridge)
        gain = solve_lqr(a, b, q_diag, r_diag, args.iterations, args.tolerance)
        print(",".join(
            [str(point_mm)] +
            [f"{gain[input_index][state]:.12g}"
             for input_index in range(INPUT_SIZE)
             for state in range(STATE_SIZE)]))
        print(f"point_mm={point_mm} rows={len(samples)} model_rms={rms:.6g}",
              file=sys.stderr)


if __name__ == "__main__":
    main()
