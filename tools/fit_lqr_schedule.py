#!/usr/bin/env python3
## @file fit_lqr_schedule.py
#  @brief 将离散腿长增益样本拟合为三次 LQR 调度曲线。
#  @details 该工具在主机侧运行，用于构建、采集、辨识或参数生成；不会编译进目标固件。生成参数写回固件前应按对应文档完成单位和符号约定检查。

"""Fit cubic LQR gain schedules from discrete leg-length samples.

Input CSV columns:
  leg_length_m or leg_length_mm
  k00..k05, k10..k15

The script also accepts firmware-sample columns named k00_u6..k15_u6, where
values are scaled by 1e6.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


LEG_LENGTH_CENTER = 0.25
LEG_LENGTH_HALF_RANGE = 0.15
GAIN_ROWS = 2
GAIN_COLS = 6


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    a = [row[:] + [vector[i]] for i, row in enumerate(matrix)]

    for pivot in range(size):
        best = max(range(pivot, size), key=lambda row: abs(a[row][pivot]))
        if abs(a[best][pivot]) < 1e-12:
            raise ValueError("singular normal equation; add more distinct samples")
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


def fit_cubic(x_values: list[float], y_values: list[float]) -> list[float]:
    if len(x_values) != len(y_values):
        raise ValueError("x/y length mismatch")
    if len(x_values) < 4:
        raise ValueError("at least four samples are required for a cubic fit")

    powers = [[x**3, x**2, x, 1.0] for x in x_values]
    normal = [[0.0 for _ in range(4)] for _ in range(4)]
    rhs = [0.0 for _ in range(4)]
    for row, y in zip(powers, y_values):
        for i in range(4):
            rhs[i] += row[i] * y
            for j in range(4):
                normal[i][j] += row[i] * row[j]
    return solve_linear_system(normal, rhs)


def read_samples(path: Path) -> tuple[list[float], list[list[list[float]]]]:
    leg_lengths: list[float] = []
    gains = [[[] for _ in range(GAIN_COLS)] for _ in range(GAIN_ROWS)]

    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        fields = set(reader.fieldnames)
        use_mm = "leg_length_mm" in fields
        use_m = "leg_length_m" in fields
        if not use_mm and not use_m:
            raise ValueError("CSV must contain leg_length_mm or leg_length_m")

        for row in reader:
            leg_length = float(row["leg_length_mm"]) / 1000.0 if use_mm else float(row["leg_length_m"])
            if not math.isfinite(leg_length):
                raise ValueError(f"non-finite leg length: {leg_length}")
            leg_lengths.append(leg_length)
            for input_index in range(GAIN_ROWS):
                for state in range(GAIN_COLS):
                    key = f"k{input_index}{state}"
                    scaled_key = f"{key}_u6"
                    if key in fields:
                        value = float(row[key])
                    elif scaled_key in fields:
                        value = float(row[scaled_key]) / 1_000_000.0
                    else:
                        raise ValueError(f"missing gain column {key} or {scaled_key}")
                    if not math.isfinite(value):
                        raise ValueError(f"non-finite gain {key}: {value}")
                    gains[input_index][state].append(value)

    return leg_lengths, gains


def print_cpp_initializer(coefficients: list[list[list[float]]]) -> None:
    print("constexpr double kGainPolynomial[2][6][4] = {")
    for input_index, input_coefficients in enumerate(coefficients):
        print("    {")
        for state_coefficients in input_coefficients:
            formatted = ", ".join(f"{value:.15g}" for value in state_coefficients)
            print(f"        {{{formatted}}},")
        suffix = "," if input_index + 1 < GAIN_ROWS else ""
        print(f"    }}{suffix}")
    print("};")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="Discrete gain CSV")
    args = parser.parse_args()

    leg_lengths, gains = read_samples(args.csv)
    normalized = [
        (length - LEG_LENGTH_CENTER) / LEG_LENGTH_HALF_RANGE
        for length in leg_lengths
    ]
    coefficients = [[[] for _ in range(GAIN_COLS)] for _ in range(GAIN_ROWS)]
    for input_index in range(GAIN_ROWS):
        for state in range(GAIN_COLS):
            coefficients[input_index][state] = fit_cubic(
                normalized, gains[input_index][state])

    print_cpp_initializer(coefficients)


if __name__ == "__main__":
    main()
