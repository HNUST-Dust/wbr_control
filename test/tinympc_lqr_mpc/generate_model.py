#!/usr/bin/env python3
"""Generate the nominal discrete wheel-legged chassis model for the host test."""

from __future__ import annotations

import csv
import math
from pathlib import Path


NX = 6
NU = 2
DT = 0.001
TARGET_LEG_LENGTH_M = 0.25
RHO = 10.0
Q_DIAGONAL = [1500.0, 100.0, 500.0, 300.0, 24000.0, 800.0]
R_DIAGONAL = [90.0, 1.0]


def zeros(rows: int, columns: int) -> list[list[float]]:
    return [[0.0 for _ in range(columns)] for _ in range(rows)]


def identity(size: int) -> list[list[float]]:
    result = zeros(size, size)
    for index in range(size):
        result[index][index] = 1.0
    return result


def add(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [[a + b for a, b in zip(left_row, right_row)]
            for left_row, right_row in zip(left, right)]


def scale(matrix: list[list[float]], factor: float) -> list[list[float]]:
    return [[factor * value for value in row] for row in matrix]


def multiply(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [[sum(left[row][inner] * right[inner][column]
                 for inner in range(len(right)))
             for column in range(len(right[0]))]
            for row in range(len(left))]


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(column) for column in zip(*matrix)]


def subtract(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [[a - b for a, b in zip(left_row, right_row)]
            for left_row, right_row in zip(left, right)]


def inverse(matrix: list[list[float]]) -> list[list[float]]:
    """Gauss-Jordan inverse with partial pivoting for the 2x2 cache solve."""
    size = len(matrix)
    augmented = [row[:] + identity(size)[index]
                 for index, row in enumerate(matrix)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1.0e-15:
            raise RuntimeError("singular cache matrix")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        pivot_value = augmented[column][column]
        augmented[column] = [value / pivot_value for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [value - factor * pivot_entry
                              for value, pivot_entry
                              in zip(augmented[row], augmented[column])]
    return [row[size:] for row in augmented]


def diagonal(values: list[float]) -> list[list[float]]:
    result = zeros(len(values), len(values))
    for index, value in enumerate(values):
        result[index][index] = value
    return result


def maximum_difference(left: list[list[float]], right: list[list[float]]) -> float:
    return max(abs(a - b) for left_row, right_row in zip(left, right)
               for a, b in zip(left_row, right_row))


def precompute_tinympc_cache(
    ad: list[list[float]], bd: list[list[float]]
) -> tuple[list[list[float]], list[list[float]], list[list[float]], list[list[float]]]:
    """Mirror TinyMPC's offline infinite-horizon Riccati cache calculation."""
    # tiny_setup() first stores Q+rho*I/R+rho*I in the workspace and passes
    # those augmented matrices into tiny_precompute_and_set_cache(), which
    # adds rho once more. Mirror the checked-out upstream implementation
    # exactly so the offline cache is bit-for-bit comparable in the test.
    q1 = diagonal([value + 2.0 * RHO for value in Q_DIAGONAL])
    r1 = diagonal([value + 2.0 * RHO for value in R_DIAGONAL])
    gain_previous = zeros(NU, NX)
    p_previous = diagonal([RHO] * NX)
    gain = zeros(NU, NX)
    p = zeros(NX, NX)
    at = transpose(ad)
    bt = transpose(bd)
    for _ in range(1000):
        quu = add(r1, multiply(multiply(bt, p_previous), bd))
        gain = multiply(multiply(inverse(quu), bt), multiply(p_previous, ad))
        a_minus_bk = subtract(ad, multiply(bd, gain))
        p = add(q1, multiply(multiply(at, p_previous), a_minus_bk))
        if maximum_difference(gain, gain_previous) < 1.0e-5:
            break
        gain_previous = gain
        p_previous = p
    quu_inverse = inverse(add(r1, multiply(multiply(bt, p), bd)))
    am_bk_transpose = transpose(subtract(ad, multiply(bd, gain)))
    return gain, p, quu_inverse, am_bk_transpose


def matrix_exponential(matrix: list[list[float]]) -> list[list[float]]:
    """Scaling-and-squaring Taylor exponential for this small fixed matrix."""
    size = len(matrix)
    norm_inf = max(sum(abs(value) for value in row) for row in matrix)
    squarings = max(0, math.ceil(math.log2(norm_inf))) if norm_inf > 1.0 else 0
    reduced = scale(matrix, 1.0 / (2 ** squarings))
    result = identity(size)
    term = identity(size)
    for order in range(1, 40):
        term = scale(multiply(term, reduced), 1.0 / order)
        result = add(result, term)
        if max(abs(value) for row in term for value in row) < 1.0e-18:
            break
    for _ in range(squarings):
        result = multiply(result, result)
    return result


def emit_array(name: str, matrix: list[list[float]]) -> str:
    flat = [value for row in matrix for value in row]
    values = ",\n    ".join(f"{value:.17g}" for value in flat)
    return f"inline constexpr double {name}[{len(flat)}] = {{\n    {values}\n}};\n"


def main() -> None:
    test_dir = Path(__file__).resolve().parent
    repository = test_dir.parents[1]
    samples_path = repository / "tools/generated/physical_lqr_samples.csv"
    output_path = test_dir / "src/generated_chassis_model.hpp"

    with samples_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    sample = min(rows, key=lambda row: abs(float(row["leg_length_m"]) - TARGET_LEG_LENGTH_M))

    a = [[float(sample[f"a{i}{j}"]) for j in range(NX)] for i in range(NX)]
    b = [[float(sample[f"b{i}{j}"]) for j in range(NU)] for i in range(NX)]
    k = [[float(sample[f"k{i}{j}"]) for j in range(NX)] for i in range(NU)]

    augmented = zeros(NX + NU, NX + NU)
    for row in range(NX):
        for column in range(NX):
            augmented[row][column] = a[row][column] * DT
        for column in range(NU):
            augmented[row][NX + column] = b[row][column] * DT
    discrete = matrix_exponential(augmented)
    ad = [row[:NX] for row in discrete[:NX]]
    bd = [row[NX:] for row in discrete[:NX]]
    kinf, pinf, quu_inverse, am_bk_transpose = precompute_tinympc_cache(ad, bd)

    content = """// Generated by generate_model.py; do not edit manually.\n#pragma once\n\nnamespace wbr_tinympc_test {\n"""
    content += f"inline constexpr double kModelLegLengthM = {float(sample['leg_length_m']):.17g};\n"
    content += f"inline constexpr double kSampleTimeS = {DT:.17g};\n"
    content += emit_array("kAdRowMajor", ad)
    content += emit_array("kBdRowMajor", bd)
    content += emit_array("kContinuousLqrGainRowMajor", k)
    content += f"inline constexpr double kTinyMpcRho = {RHO:.17g};\n"
    content += emit_array("kTinyMpcKinfRowMajor", kinf)
    content += emit_array("kTinyMpcPinfRowMajor", pinf)
    content += emit_array("kTinyMpcQuuInverseRowMajor", quu_inverse)
    content += emit_array("kTinyMpcAmBkTransposeRowMajor", am_bk_transpose)
    content += "}  // namespace wbr_tinympc_test\n"
    output_path.write_text(content, encoding="utf-8")
    print(f"generated {output_path} from leg length {sample['leg_length_m']} m")


if __name__ == "__main__":
    main()
