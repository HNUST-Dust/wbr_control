#!/usr/bin/env python3
"""Derive the wheel-leg model and cubic LQR schedule from physical parameters.

Recalculation procedure, parameter provenance, sign conventions, firmware
sync steps, and validation checklist:

    docs/LQR_GAIN_RECOMPUTATION.md

The equations are the linearization of controller.m at:

    theta = phi = theta_dot = x_dot = phi_dot = 0
    T = Tp = 0

State:
    [theta, theta_dot, x, x_dot, phi, phi_dot]

Input:
    [T, Tp]

T and Tp are left/right total generalized torques. Therefore all wheel and
leg masses/inertias in this script are left/right combined quantities.
The standard feedback convention is:

    u = -K x

which is equivalent to the article's u = K * (x_ref - x).
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


STATE_SIZE = 6
INPUT_SIZE = 2
GAIN_CENTER_M = 0.25
GAIN_HALF_RANGE_M = 0.15


@dataclass(frozen=True)
class RobotParameters:
    wheel_mass_total_kg: float = 1.2
    wheel_inertia_total_kg_m2: float = 0.00682
    wheel_radius_m: float = 0.058
    leg_mass_total_kg: float = 2.0
    body_mass_kg: float = 6.9
    body_pitch_inertia_kg_m2: float = 0.066012040
    body_com_offset_m: float = -0.0497
    gravity_m_s2: float = 9.80665


ARTICLE_Q_DIAG = np.array([1.0, 1.0, 500.0, 100.0, 5000.0, 1.0])
ARTICLE_R_DIAG = np.array([1.0, 0.25])
# Current chassis tuning: retain the article weights except for theta_dot,
# whose larger weight supplies damping for the observed common-theta mode.
# CURRENT_Q_DIAG = np.array([3000.0, 400.0, 1500.0, 50.0, 16000.0, 100.0])
# CURRENT_R_DIAG = np.array([150.0, 1.0])
CURRENT_Q_DIAG = np.array([4000.0, 1.0, 1500.0, 1.0, 20000.0, 1.0])
CURRENT_R_DIAG = np.array([90.0, 1.0])


def read_leg_samples(path: Path) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        required = {
            "leg_length_m",
            "leg_com_from_wheel_m",
            "single_leg_inertia_kg_m2",
        }
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{path} must contain {sorted(required)}")
        for row in reader:
            leg_length = float(row["leg_length_m"])
            com_length = float(row["leg_com_from_wheel_m"])
            single_inertia = float(row["single_leg_inertia_kg_m2"])
            if not (0.0 < com_length < leg_length):
                raise ValueError(
                    f"invalid leg geometry h={leg_length}, L={com_length}")
            samples.append({
                "leg_length_m": leg_length,
                "leg_com_from_wheel_m": com_length,
                "single_leg_inertia_kg_m2": single_inertia,
            })
    return sorted(samples, key=lambda sample: sample["leg_length_m"])


def derive_continuous_model(
    parameters: RobotParameters,
    leg_length_m: float,
    leg_com_from_wheel_m: float,
    single_leg_inertia_kg_m2: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return continuous A/B for the left/right combined sagittal model."""
    mw = parameters.wheel_mass_total_kg
    iw = parameters.wheel_inertia_total_kg_m2
    radius = parameters.wheel_radius_m
    mp = parameters.leg_mass_total_kg
    ip = 2.0 * single_leg_inertia_kg_m2
    body_mass = parameters.body_mass_kg
    body_inertia = parameters.body_pitch_inertia_kg_m2
    body_offset = parameters.body_com_offset_m
    gravity = parameters.gravity_m_s2

    h = leg_length_m
    leg_com = leg_com_from_wheel_m
    leg_com_to_pivot = h - leg_com
    if leg_com_to_pivot <= 0.0:
        raise ValueError("leg COM must lie between the wheel and upper pivot")

    # Generalized acceleration order: [x_ddot, theta_ddot, phi_ddot].
    mass_matrix = np.array([
        [
            iw / (radius * radius) + mw + mp + body_mass,
            mp * leg_com + body_mass * h,
            -body_mass * body_offset,
        ],
        [
            mp * leg_com + body_mass * h,
            ip + mp * leg_com * leg_com + body_mass * h * h,
            -body_mass * body_offset * h,
        ],
        [
            -body_mass * body_offset,
            -body_mass * body_offset * h,
            body_inertia + body_mass * body_offset * body_offset,
        ],
    ])

    # Generalized coordinate order: [theta, x, phi].
    gravity_matrix = np.array([
        [0.0, 0.0, 0.0],
        [
            gravity * (mp * leg_com + body_mass * h),
            0.0,
            0.0,
        ],
        [0.0, 0.0, body_mass * gravity * body_offset],
    ])

    # Input order: [total wheel torque T, total leg posture torque Tp].
    input_matrix = np.array([
        [1.0 / radius, 0.0],
        [-1.0, 1.0],
        [0.0, 1.0],
    ])

    acceleration_from_position = np.linalg.solve(
        mass_matrix, gravity_matrix)
    acceleration_from_input = np.linalg.solve(mass_matrix, input_matrix)

    a = np.zeros((STATE_SIZE, STATE_SIZE))
    b = np.zeros((STATE_SIZE, INPUT_SIZE))
    a[0, 1] = 1.0
    a[2, 3] = 1.0
    a[4, 5] = 1.0

    # Map [x_ddot, theta_ddot, phi_ddot] into state derivative rows.
    state_acceleration_rows = [3, 1, 5]
    for generalized_row, state_row in enumerate(state_acceleration_rows):
        a[state_row, 0] = acceleration_from_position[generalized_row, 0]
        a[state_row, 2] = acceleration_from_position[generalized_row, 1]
        a[state_row, 4] = acceleration_from_position[generalized_row, 2]
        b[state_row, :] = acceleration_from_input[generalized_row, :]

    return a, b


def controllability_rank(a: np.ndarray, b: np.ndarray) -> int:
    blocks = [b]
    power = np.eye(a.shape[0])
    for _ in range(1, a.shape[0]):
        power = power @ a
        blocks.append(power @ b)
    return int(np.linalg.matrix_rank(np.concatenate(blocks, axis=1)))


def continuous_lqr(
    a: np.ndarray,
    b: np.ndarray,
    q_diag: np.ndarray,
    r_diag: np.ndarray,
) -> np.ndarray:
    """Solve continuous CARE through the stable Hamiltonian subspace."""
    q = np.diag(q_diag)
    r = np.diag(r_diag)
    r_inverse = np.linalg.inv(r)
    hamiltonian = np.block([
        [a, -b @ r_inverse @ b.T],
        [-q, -a.T],
    ])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)
    stable = np.where(eigenvalues.real < -1e-9)[0]
    if stable.size != a.shape[0]:
        raise ValueError(
            f"Hamiltonian has {stable.size} stable eigenvalues, "
            f"expected {a.shape[0]}")
    stable_vectors = eigenvectors[:, stable]
    upper = stable_vectors[:a.shape[0], :]
    lower = stable_vectors[a.shape[0]:, :]
    p = np.real(lower @ np.linalg.inv(upper))
    p = 0.5 * (p + p.T)
    return r_inverse @ b.T @ p


def verify_article_regression() -> None:
    """Check the CARE solver against the numeric example in controller.m."""
    a = np.array([
        [0, 1, 0, 0, 0, 0],
        [265.9556, 0, 0, 0, 80.6327, 0],
        [0, 0, 0, 1, 0, 0],
        [-25.4562, 0, 0, 0, 1.8637, 0],
        [0, 0, 0, 0, 0, 1],
        [156.6952, 0, 0, 0, 183.0614, 0],
    ], dtype=float)
    b = np.array([
        [0, 0],
        [-15.1389, 13.8563],
        [0, 0],
        [2.1208, -0.7158],
        [0, 0],
        [-4.2238, 16.8001],
    ], dtype=float)
    expected = np.array([
        [-44.3788, -6.8496, -22.2828, -21.5569, 28.7706, 4.3751],
        [11.2006, 0.7339, 3.7300, 3.2058, 151.73, 4.6387],
    ])
    actual = continuous_lqr(a, b, ARTICLE_Q_DIAG, ARTICLE_R_DIAG)
    max_error = float(np.max(np.abs(actual - expected)))
    if max_error > 0.002:
        raise ValueError(f"article LQR regression failed: max error {max_error}")


def fit_gain_polynomial(
    leg_lengths: np.ndarray,
    gains: np.ndarray,
) -> np.ndarray:
    normalized = (leg_lengths - GAIN_CENTER_M) / GAIN_HALF_RANGE_M
    coefficients = np.zeros((INPUT_SIZE, STATE_SIZE, 4))
    for input_index in range(INPUT_SIZE):
        for state_index in range(STATE_SIZE):
            coefficients[input_index, state_index, :] = np.polyfit(
                normalized, gains[:, input_index, state_index], 3)
    return coefficients


def evaluate_gain_polynomial(
    coefficients: np.ndarray,
    leg_length_m: float,
) -> np.ndarray:
    normalized = (leg_length_m - GAIN_CENTER_M) / GAIN_HALF_RANGE_M
    gain = np.zeros((INPUT_SIZE, STATE_SIZE))
    for input_index in range(INPUT_SIZE):
        for state_index in range(STATE_SIZE):
            gain[input_index, state_index] = np.polyval(
                coefficients[input_index, state_index], normalized)
    return gain


def print_cpp_initializer(coefficients: np.ndarray) -> None:
    print("constexpr double kGainPolynomial[2][6][4] = {")
    for input_index in range(INPUT_SIZE):
        print("    {")
        for state_index in range(STATE_SIZE):
            values = ", ".join(
                f"{value:.15g}"
                for value in coefficients[input_index, state_index])
            print(f"        {{{values}}},")
        print("    }" + ("," if input_index + 1 < INPUT_SIZE else ""))
    print("};")


def write_samples(
    output_path: Path,
    rows: list[dict[str, object]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    gain_fields = [
        f"k{input_index}{state_index}"
        for input_index in range(INPUT_SIZE)
        for state_index in range(STATE_SIZE)
    ]
    a_fields = [
        f"a{row}{column}"
        for row in range(STATE_SIZE)
        for column in range(STATE_SIZE)
    ]
    b_fields = [
        f"b{row}{column}"
        for row in range(STATE_SIZE)
        for column in range(INPUT_SIZE)
    ]
    fields = [
        "leg_length_m",
        "leg_com_from_wheel_m",
        "leg_com_to_pivot_m",
        "single_leg_inertia_kg_m2",
        "total_leg_inertia_kg_m2",
        "controllability_rank",
        "max_closed_loop_real",
        *a_fields,
        *b_fields,
        *gain_fields,
    ]
    with output_path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_coefficients(output_path: Path, coefficients: np.ndarray) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "input_index",
            "state_index",
            "p3",
            "p2",
            "p1",
            "p0",
        ])
        for input_index in range(INPUT_SIZE):
            for state_index in range(STATE_SIZE):
                writer.writerow([
                    input_index,
                    state_index,
                    *coefficients[input_index, state_index, :],
                ])


def main() -> None:
    default_input = Path(__file__).with_name("data") / "leg_mass_properties.csv"
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=default_input)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--coeff-output", type=Path, default=None)
    parser.add_argument(
        "--q", default=",".join(str(value) for value in CURRENT_Q_DIAG))
    parser.add_argument(
        "--r", default=",".join(str(value) for value in CURRENT_R_DIAG))
    args = parser.parse_args()

    q_diag = np.array([float(value) for value in args.q.split(",")])
    r_diag = np.array([float(value) for value in args.r.split(",")])
    if q_diag.shape != (STATE_SIZE,) or r_diag.shape != (INPUT_SIZE,):
        raise ValueError("Q must have 6 diagonal values and R must have 2")

    verify_article_regression()
    parameters = RobotParameters()
    samples = read_leg_samples(args.input)
    rows: list[dict[str, object]] = []
    gains: list[np.ndarray] = []
    leg_lengths: list[float] = []

    for sample in samples:
        leg_length = sample["leg_length_m"]
        leg_com = sample["leg_com_from_wheel_m"]
        single_inertia = sample["single_leg_inertia_kg_m2"]
        a, b = derive_continuous_model(
            parameters, leg_length, leg_com, single_inertia)
        rank = controllability_rank(a, b)
        if rank != STATE_SIZE:
            raise ValueError(
                f"uncontrollable model at h={leg_length}: rank={rank}")
        gain = continuous_lqr(a, b, q_diag, r_diag)
        closed_loop = np.linalg.eigvals(a - b @ gain)
        row: dict[str, object] = {
            "leg_length_m": leg_length,
            "leg_com_from_wheel_m": leg_com,
            "leg_com_to_pivot_m": leg_length - leg_com,
            "single_leg_inertia_kg_m2": single_inertia,
            "total_leg_inertia_kg_m2": 2.0 * single_inertia,
            "controllability_rank": rank,
            "max_closed_loop_real": float(np.max(closed_loop.real)),
        }
        for input_index in range(INPUT_SIZE):
            for state_index in range(STATE_SIZE):
                row[f"k{input_index}{state_index}"] = float(
                    gain[input_index, state_index])
        for matrix_row in range(STATE_SIZE):
            for matrix_column in range(STATE_SIZE):
                row[f"a{matrix_row}{matrix_column}"] = float(
                    a[matrix_row, matrix_column])
        for matrix_row in range(STATE_SIZE):
            for input_index in range(INPUT_SIZE):
                row[f"b{matrix_row}{input_index}"] = float(
                    b[matrix_row, input_index])
        rows.append(row)
        gains.append(gain)
        leg_lengths.append(leg_length)

    leg_lengths_array = np.array(leg_lengths)
    gains_array = np.stack(gains)
    coefficients = fit_gain_polynomial(leg_lengths_array, gains_array)

    maximum_fit_error = 0.0
    maximum_fitted_closed_loop_real = -np.inf
    for row, gain in zip(rows, gains):
        leg_length = float(row["leg_length_m"])
        fitted = evaluate_gain_polynomial(coefficients, leg_length)
        maximum_fit_error = max(
            maximum_fit_error, float(np.max(np.abs(fitted - gain))))
        a, b = derive_continuous_model(
            parameters,
            leg_length,
            float(row["leg_com_from_wheel_m"]),
            float(row["single_leg_inertia_kg_m2"]),
        )
        fitted_eigenvalues = np.linalg.eigvals(a - b @ fitted)
        maximum_fitted_closed_loop_real = max(
            maximum_fitted_closed_loop_real,
            float(np.max(fitted_eigenvalues.real)),
        )

    # Check the fitted schedule between the measured points. L and Ip are
    # linearly interpolated only for validation; production K evaluation uses
    # the cubic schedule directly.
    dense_maximum_closed_loop_real = -np.inf
    dense_lengths = np.linspace(
        leg_lengths_array.min(), leg_lengths_array.max(), 401)
    com_values = np.array([
        float(row["leg_com_from_wheel_m"]) for row in rows])
    inertia_values = np.array([
        float(row["single_leg_inertia_kg_m2"]) for row in rows])
    for leg_length in dense_lengths:
        leg_com = float(np.interp(
            leg_length, leg_lengths_array, com_values))
        single_inertia = float(np.interp(
            leg_length, leg_lengths_array, inertia_values))
        a, b = derive_continuous_model(
            parameters, leg_length, leg_com, single_inertia)
        fitted = evaluate_gain_polynomial(coefficients, leg_length)
        eigenvalues = np.linalg.eigvals(a - b @ fitted)
        dense_maximum_closed_loop_real = max(
            dense_maximum_closed_loop_real,
            float(np.max(eigenvalues.real)),
        )

    if args.output is not None:
        write_samples(args.output, rows)
    if args.coeff_output is not None:
        write_coefficients(args.coeff_output, coefficients)

    print(f"points={len(rows)}")
    print(
        f"leg_length_range_m={leg_lengths_array.min():.5f},"
        f"{leg_lengths_array.max():.5f}")
    print(f"max_gain_fit_abs_error={maximum_fit_error:.9g}")
    print(
        "max_fitted_closed_loop_real="
        f"{maximum_fitted_closed_loop_real:.9g}")
    print(
        "dense_max_fitted_closed_loop_real="
        f"{dense_maximum_closed_loop_real:.9g}")
    print_cpp_initializer(coefficients)


if __name__ == "__main__":
    main()
