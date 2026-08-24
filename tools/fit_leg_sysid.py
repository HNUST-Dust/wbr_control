#!/usr/bin/env python3
## @file fit_leg_sysid.py
#  @brief 从底盘系统辨识日志拟合腿部角度通道模型。
#  @details 该工具在主机侧运行，用于构建、采集、辨识或参数生成；不会编译进目标固件。生成参数写回固件前应按对应文档完成单位和符号约定检查。

"""Fit the physical leg-angle channel from chassis sysid logs.

The chassis sysid log is intentionally leg-only while the robot is on the
current test bench: wheel torque is kept at zero and only u_leg_mNm is excited.
This script fits a per-leg-length model:

    [theta, theta_rate][k+1] = A [theta, theta_rate][k] + B u_leg[k] + c

and solves a single-input LQR for u_leg = -K [theta, theta_rate].
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


RAW_COLUMNS = [
    "t_ms",
    "point_mm",
    "phase",
    "sample",
    "theta_mrad",
    "theta_rate_mradps",
    "x_mm",
    "x_rate_mms",
    "pitch_mrad",
    "pitch_rate_mradps",
    "u_wheel_mNm",
    "u_leg_mNm",
    "len_l_mm",
    "len_r_mm",
    "roll_mrad",
    "yaw_rate_mradps",
    "valid",
]
REQUIRED_COLUMNS = [
    "point_mm",
    "phase",
    "sample",
    "theta_mrad",
    "theta_rate_mradps",
    "u_leg_mNm",
    "len_l_mm",
    "len_r_mm",
    "valid",
]
STATE_SIZE = 2
INPUT_SIZE = 1
FEATURE_SIZE = STATE_SIZE + INPUT_SIZE + 1


@dataclass
class Sample:
    index: int
    theta_mrad: float
    theta_rate_mradps: float
    u_leg_mNm: float
    state: list[float]
    control: float


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
    out = zeros(len(left), len(right[0]))
    for row in range(len(left)):
        for mid in range(len(right)):
            value = left[row][mid]
            if value == 0.0:
                continue
            for col in range(len(right[0])):
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
    for index, value in enumerate(values):
        matrix[index][index] = value
    return matrix


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    augmented = [row[:] + [vector[row_index]] for row_index, row in enumerate(matrix)]
    for pivot in range(size):
        best = max(range(pivot, size), key=lambda row: abs(augmented[row][pivot]))
        if abs(augmented[best][pivot]) < 1e-12:
            raise ValueError("singular fit; collect more excitation or increase --ridge")
        if best != pivot:
            augmented[pivot], augmented[best] = augmented[best], augmented[pivot]
        scale = augmented[pivot][pivot]
        for col in range(pivot, size + 1):
            augmented[pivot][col] /= scale
        for row in range(size):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            if factor == 0.0:
                continue
            for col in range(pivot, size + 1):
                augmented[row][col] -= factor * augmented[pivot][col]
    return [augmented[row][size] for row in range(size)]


def invert(matrix: list[list[float]]) -> list[list[float]]:
    eye = identity(len(matrix))
    return transpose([
        solve_linear_system(matrix, [eye[row][col] for row in range(len(matrix))])
        for col in range(len(matrix))
    ])


def row_to_sample(row: dict[str, str],
                  max_leg_error_mm: float,
                  max_theta_rate_mradps: float,
                  max_abs_theta_mrad: float,
                  drop_initial_samples: int) -> tuple[int, Sample] | None:
    if row["phase"] != "sample" or int(row["valid"]) == 0:
        return None
    sample_index = int(row["sample"])
    if sample_index < drop_initial_samples:
        return None
    point_mm = int(row["point_mm"])
    left_error = abs(float(row["len_l_mm"]) - point_mm)
    right_error = abs(float(row["len_r_mm"]) - point_mm)
    if max(left_error, right_error) > max_leg_error_mm:
        return None
    theta_mrad = float(row["theta_mrad"])
    theta_rate_mradps = float(row["theta_rate_mradps"])
    u_leg_mNm = float(row["u_leg_mNm"])
    if abs(theta_mrad) > max_abs_theta_mrad:
        return None
    if abs(theta_rate_mradps) > max_theta_rate_mradps:
        return None
    values = [theta_mrad, theta_rate_mradps, u_leg_mNm]
    if not all(math.isfinite(value) for value in values):
        return None
    sample = Sample(
        index=sample_index,
        theta_mrad=theta_mrad,
        theta_rate_mradps=theta_rate_mradps,
        u_leg_mNm=u_leg_mNm,
        state=[theta_mrad / 1000.0, theta_rate_mradps / 1000.0],
        control=u_leg_mNm / 1000.0,
    )
    return point_mm, sample


def read_raw_rows(path: Path,
                  max_leg_error_mm: float,
                  max_theta_rate_mradps: float,
                  max_abs_theta_mrad: float,
                  drop_initial_samples: int) -> dict[int, list[Sample]]:
    rows_by_point: dict[int, list[Sample]] = defaultdict(list)
    with path.open(newline="") as file:
        for line_number, line in enumerate(file, start=1):
            stripped = line.strip()
            if not stripped or "lqr_sysid_csv," not in stripped:
                continue
            for record in stripped.split("lqr_sysid_csv,"):
                if not record:
                    continue
                values = record.split(",")
                if len(values) != len(RAW_COLUMNS):
                    print(
                        f"warning: skip raw line {line_number} record with "
                        f"{len(values)} columns, expected {len(RAW_COLUMNS)}",
                        file=sys.stderr)
                    continue
                sample = row_to_sample(
                    dict(zip(RAW_COLUMNS, values)),
                    max_leg_error_mm,
                    max_theta_rate_mradps,
                    max_abs_theta_mrad,
                    drop_initial_samples)
                if sample is not None:
                    point_mm, item = sample
                    rows_by_point[point_mm].append(item)
    return dict(rows_by_point)


def read_csv_rows(path: Path,
                  max_leg_error_mm: float,
                  max_theta_rate_mradps: float,
                  max_abs_theta_mrad: float,
                  drop_initial_samples: int) -> dict[int, list[Sample]]:
    rows_by_point: dict[int, list[Sample]] = defaultdict(list)
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(missing)}")
        for row in reader:
            sample = row_to_sample(
                row,
                max_leg_error_mm,
                max_theta_rate_mradps,
                max_abs_theta_mrad,
                drop_initial_samples)
            if sample is not None:
                point_mm, item = sample
                rows_by_point[point_mm].append(item)
    return dict(rows_by_point)


def read_rows(path: Path,
              max_leg_error_mm: float,
              max_theta_rate_mradps: float,
              max_abs_theta_mrad: float,
              drop_initial_samples: int) -> dict[int, list[Sample]]:
    with path.open(newline="") as file:
        for line in file:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("lqr_sysid_csv,"):
                return read_raw_rows(
                    path, max_leg_error_mm, max_theta_rate_mradps,
                    max_abs_theta_mrad,
                    drop_initial_samples)
            return read_csv_rows(
                path, max_leg_error_mm, max_theta_rate_mradps,
                max_abs_theta_mrad,
                drop_initial_samples)
    return {}


def consecutive_pairs(samples: list[Sample]) -> list[tuple[Sample, Sample]]:
    return [
        (current, next_sample)
        for current, next_sample in zip(samples, samples[1:])
        if next_sample.index == current.index + 1
    ]


def fit_model(samples: list[Sample], ridge: float) -> tuple[list[list[float]], list[list[float]], float, int]:
    pairs = consecutive_pairs(samples)
    if len(pairs) < FEATURE_SIZE + 2:
        raise ValueError(f"need at least {FEATURE_SIZE + 2} consecutive rows per point")
    normal = zeros(FEATURE_SIZE, FEATURE_SIZE)
    rhs = zeros(FEATURE_SIZE, STATE_SIZE)
    for current, next_sample in pairs:
        feature = [*current.state, current.control, 1.0]
        for i in range(FEATURE_SIZE):
            for j in range(FEATURE_SIZE):
                normal[i][j] += feature[i] * feature[j]
            for state_out in range(STATE_SIZE):
                rhs[i][state_out] += feature[i] * next_sample.state[state_out]
    for index in range(FEATURE_SIZE):
        normal[index][index] += ridge
    coefficients = zeros(FEATURE_SIZE, STATE_SIZE)
    for state_out in range(STATE_SIZE):
        solution = solve_linear_system(
            normal, [rhs[row][state_out] for row in range(FEATURE_SIZE)])
        for row, value in enumerate(solution):
            coefficients[row][state_out] = value
    a = zeros(STATE_SIZE, STATE_SIZE)
    b = zeros(STATE_SIZE, INPUT_SIZE)
    for state_out in range(STATE_SIZE):
        for state_in in range(STATE_SIZE):
            a[state_out][state_in] = coefficients[state_in][state_out]
        b[state_out][0] = coefficients[STATE_SIZE][state_out]
    squared_error = 0.0
    count = 0
    for current, next_sample in pairs:
        predicted = [
            sum(a[row][col] * current.state[col] for col in range(STATE_SIZE)) +
            b[row][0] * current.control +
            coefficients[FEATURE_SIZE - 1][row]
            for row in range(STATE_SIZE)
        ]
        for actual, estimate in zip(next_sample.state, predicted):
            squared_error += (actual - estimate) ** 2
            count += 1
    return a, b, math.sqrt(squared_error / max(1, count)), len(pairs)


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


def warn_quality(point_mm: int, samples: list[Sample]) -> None:
    theta_values = [sample.theta_mrad for sample in samples]
    u_values = [sample.u_leg_mNm for sample in samples]
    theta_range = max(theta_values) - min(theta_values)
    if theta_range < 10.0:
        print(f"warning: point_mm={point_mm} theta range is only {theta_range:.1f} mrad",
              file=sys.stderr)
    if not any(value > 0.0 for value in u_values) or not any(value < 0.0 for value in u_values):
        print(f"warning: point_mm={point_mm} u_leg does not contain both signs",
              file=sys.stderr)


def quality_status(samples: list[Sample], min_theta_range_mrad: float) -> str:
    theta_values = [sample.theta_mrad for sample in samples]
    u_values = [sample.u_leg_mNm for sample in samples]
    if (max(theta_values) - min(theta_values)) < min_theta_range_mrad:
        return "reject_low_theta_range"
    if not any(value > 0.0 for value in u_values) or not any(value < 0.0 for value in u_values):
        return "warn_one_sided_input"
    return "ok"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path, help="raw or CSV lqr_sysid log")
    parser.add_argument("--q", default="60,2",
                        help="state cost diagonal for theta,theta_rate")
    parser.add_argument("--r", default="0.3",
                        help="input cost for u_leg")
    parser.add_argument("--ridge", type=float, default=1e-8)
    parser.add_argument("--max-leg-error-mm", type=float, default=25.0)
    parser.add_argument("--max-theta-rate-mradps", type=float, default=1000.0)
    parser.add_argument("--max-abs-theta-mrad", type=float, default=500.0)
    parser.add_argument("--min-theta-range-mrad", type=float, default=10.0)
    parser.add_argument("--drop-initial-samples", type=int, default=20)
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument("--tolerance", type=float, default=1e-9)
    args = parser.parse_args()

    q_diag = parse_diag(args.q, STATE_SIZE, "q")
    r_diag = parse_diag(args.r, INPUT_SIZE, "r")
    rows_by_point = read_rows(
        args.log,
        args.max_leg_error_mm,
        args.max_theta_rate_mradps,
        args.max_abs_theta_mrad,
        args.drop_initial_samples)
    if not rows_by_point:
        raise ValueError("no valid sample rows found")

    print("leg_length_mm,k_leg_theta,k_leg_theta_rate,rows,pairs,model_rms,"
          "theta_min_mrad,theta_max_mrad,theta_rate_min_mradps,theta_rate_max_mradps,"
          "u_leg_min_mNm,u_leg_max_mNm,status")
    for point_mm in sorted(rows_by_point):
        samples = rows_by_point[point_mm]
        warn_quality(point_mm, samples)
        a, b, rms, pairs = fit_model(samples, args.ridge)
        gain = solve_lqr(a, b, q_diag, r_diag, args.iterations, args.tolerance)
        theta_values = [sample.theta_mrad for sample in samples]
        theta_rate_values = [sample.theta_rate_mradps for sample in samples]
        u_values = [sample.u_leg_mNm for sample in samples]
        status = quality_status(samples, args.min_theta_range_mrad)
        print(",".join([
            str(point_mm),
            f"{gain[0][0]:.12g}",
            f"{gain[0][1]:.12g}",
            str(len(samples)),
            str(pairs),
            f"{rms:.6g}",
            f"{min(theta_values):.6g}",
            f"{max(theta_values):.6g}",
            f"{min(theta_rate_values):.6g}",
            f"{max(theta_rate_values):.6g}",
            f"{min(u_values):.6g}",
            f"{max(u_values):.6g}",
            status,
        ]))


if __name__ == "__main__":
    main()
