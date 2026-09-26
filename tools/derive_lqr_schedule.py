#!/usr/bin/env python3
"""Derive common, differential, or unified wheel-leg LQR schedules.

Select a derivation with --mode common, --mode differential, or --mode
unified. The default is the firmware's unified ten-state model.
"""
import sys
import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import numpy as np
common_STATE_SIZE = 6
common_INPUT_SIZE = 2
common_GAIN_CENTER_M = 0.25
common_GAIN_HALF_RANGE_M = 0.15

@dataclass(frozen=True)
class common_RobotParameters:
    wheel_mass_total_kg: float = 1.2
    wheel_inertia_total_kg_m2: float = 0.0005046
    wheel_radius_m: float = 0.058
    leg_mass_total_kg: float = 2.066
    body_mass_kg: float = 8.788
    body_pitch_inertia_kg_m2: float = 0.133703941
    body_com_offset_m: float = -0.05235
    gravity_m_s2: float = 9.80665
common_ARTICLE_Q_DIAG = np.array([1.0, 1.0, 500.0, 100.0, 5000.0, 1.0])
common_ARTICLE_R_DIAG = np.array([1.0, 0.25])
common_CURRENT_Q_DIAG = np.array([1000.0, 5.0, 1500.0, 1.0, 20000.0, 1.0])
common_CURRENT_R_DIAG = np.array([60.0, 1.0])

def common_read_leg_samples(path: Path) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    with path.open(newline='') as file:
        reader = csv.DictReader(file)
        required = {'leg_length_m', 'leg_com_from_wheel_m', 'single_leg_inertia_kg_m2'}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f'{path} must contain {sorted(required)}')
        for row in reader:
            leg_length = float(row['leg_length_m'])
            com_length = float(row['leg_com_from_wheel_m'])
            single_inertia = float(row['single_leg_inertia_kg_m2'])
            if not 0.0 < com_length < leg_length:
                raise ValueError(f'invalid leg geometry h={leg_length}, L={com_length}')
            samples.append({'leg_length_m': leg_length, 'leg_com_from_wheel_m': com_length, 'single_leg_inertia_kg_m2': single_inertia})
    return sorted(samples, key=lambda sample: sample['leg_length_m'])

def common_derive_wheel_coordinate_linear_model(parameters: common_RobotParameters, leg_length_m: float, leg_com_from_wheel_m: float, single_leg_inertia_kg_m2: float) -> tuple[np.ndarray, np.ndarray]:
    """Return the analytic linear model using wheel-axis position.

    This is retained as an independent regression oracle.  The production
    model below is linearized from the paper's classical-mechanics equations
    and uses body/hip position as the translational state.
    """
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
        raise ValueError('leg COM must lie between the wheel and upper pivot')
    mass_matrix = np.array([[iw / (radius * radius) + mw + mp + body_mass, mp * leg_com + body_mass * h, -body_mass * body_offset], [mp * leg_com + body_mass * h, ip + mp * leg_com * leg_com + body_mass * h * h, -body_mass * body_offset * h], [-body_mass * body_offset, -body_mass * body_offset * h, body_inertia + body_mass * body_offset * body_offset]])
    gravity_matrix = np.array([[0.0, 0.0, 0.0], [gravity * (mp * leg_com + body_mass * h), 0.0, 0.0], [0.0, 0.0, body_mass * gravity * body_offset]])
    input_matrix = np.array([[1.0 / radius, 0.0], [-1.0, 1.0], [0.0, 1.0]])
    acceleration_from_position = np.linalg.solve(mass_matrix, gravity_matrix)
    acceleration_from_input = np.linalg.solve(mass_matrix, input_matrix)
    a = np.zeros((common_STATE_SIZE, common_STATE_SIZE))
    b = np.zeros((common_STATE_SIZE, common_INPUT_SIZE))
    a[0, 1] = 1.0
    a[2, 3] = 1.0
    a[4, 5] = 1.0
    state_acceleration_rows = [3, 1, 5]
    for generalized_row, state_row in enumerate(state_acceleration_rows):
        a[state_row, 0] = acceleration_from_position[generalized_row, 0]
        a[state_row, 2] = acceleration_from_position[generalized_row, 1]
        a[state_row, 4] = acceleration_from_position[generalized_row, 2]
        b[state_row, :] = acceleration_from_input[generalized_row, :]
    return (a, b)

def common_wheel_to_body_state_transform(leg_length_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Return P and P^-1 for x_body = P x_wheel at fixed leg length."""
    transform = np.eye(common_STATE_SIZE)
    inverse_transform = np.eye(common_STATE_SIZE)
    transform[2, 0] = leg_length_m
    transform[3, 1] = leg_length_m
    inverse_transform[2, 0] = -leg_length_m
    inverse_transform[3, 1] = -leg_length_m
    return (transform, inverse_transform)

def common_transform_wheel_model_to_body_state(a_wheel: np.ndarray, b_wheel: np.ndarray, leg_length_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Transform a fixed-leg-length linear model from x_wheel to x_body."""
    transform, inverse_transform = common_wheel_to_body_state_transform(leg_length_m)
    return (transform @ a_wheel @ inverse_transform, transform @ b_wheel)

def common_paper_state_derivative(parameters: common_RobotParameters, leg_length_m: float, leg_com_from_wheel_m: float, single_leg_inertia_kg_m2: float, state: np.ndarray, control: np.ndarray) -> np.ndarray:
    """Evaluate the paper's nonlinear equations (3)--(9).

    The seven solved unknowns are:

        [x_wheel_ddot, theta_ddot, phi_ddot, N, P, N_M, P_M]

    The returned state derivative uses the paper's final state coordinate
    x_body rather than the intermediate wheel-axis coordinate x_wheel.
    """
    if state.shape != (common_STATE_SIZE,) or control.shape != (common_INPUT_SIZE,):
        raise ValueError('state must have 6 values and control must have 2')
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
    if not 0.0 < leg_com < h:
        raise ValueError('leg COM must lie between the wheel and upper pivot')
    theta = float(state[0])
    theta_rate = float(state[1])
    phi = float(state[4])
    phi_rate = float(state[5])
    wheel_torque = float(control[0])
    posture_torque = float(control[1])
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    coefficients = np.zeros((7, 7))
    right_hand_side = np.zeros(7)
    coefficients[0, 0] = iw / radius + mw * radius
    coefficients[0, 3] = radius
    right_hand_side[0] = wheel_torque
    coefficients[1, 0] = -mp
    coefficients[1, 1] = -mp * leg_com * cos_theta
    coefficients[1, 3] = 1.0
    coefficients[1, 5] = -1.0
    right_hand_side[1] = -mp * leg_com * theta_rate ** 2 * sin_theta
    coefficients[2, 1] = mp * leg_com * sin_theta
    coefficients[2, 4] = 1.0
    coefficients[2, 6] = -1.0
    right_hand_side[2] = mp * gravity - mp * leg_com * theta_rate ** 2 * cos_theta
    coefficients[3, 1] = ip
    coefficients[3, 3] = leg_com * cos_theta
    coefficients[3, 4] = -leg_com * sin_theta
    coefficients[3, 5] = leg_com_to_pivot * cos_theta
    coefficients[3, 6] = -leg_com_to_pivot * sin_theta
    right_hand_side[3] = -wheel_torque + posture_torque
    coefficients[4, 0] = -body_mass
    coefficients[4, 1] = -body_mass * h * cos_theta
    coefficients[4, 2] = body_mass * body_offset * cos_phi
    coefficients[4, 5] = 1.0
    right_hand_side[4] = body_mass * (-h * theta_rate ** 2 * sin_theta + body_offset * phi_rate ** 2 * sin_phi)
    coefficients[5, 1] = body_mass * h * sin_theta
    coefficients[5, 2] = body_mass * body_offset * sin_phi
    coefficients[5, 6] = 1.0
    right_hand_side[5] = body_mass * (gravity - h * theta_rate ** 2 * cos_theta - body_offset * phi_rate ** 2 * cos_phi)
    coefficients[6, 2] = body_inertia
    coefficients[6, 5] = -body_offset * cos_phi
    coefficients[6, 6] = -body_offset * sin_phi
    right_hand_side[6] = posture_torque
    solution = np.linalg.solve(coefficients, right_hand_side)
    wheel_acceleration = solution[0]
    theta_acceleration = solution[1]
    phi_acceleration = solution[2]
    body_acceleration = wheel_acceleration + h * theta_acceleration * cos_theta - h * theta_rate ** 2 * sin_theta
    return np.array([theta_rate, theta_acceleration, state[3], body_acceleration, phi_rate, phi_acceleration])

def common_derive_continuous_model(parameters: common_RobotParameters, leg_length_m: float, leg_com_from_wheel_m: float, single_leg_inertia_kg_m2: float) -> tuple[np.ndarray, np.ndarray]:
    """Linearize the paper model in [theta, theta_dot, x_body, ...]."""
    equilibrium_state = np.zeros(common_STATE_SIZE)
    equilibrium_control = np.zeros(common_INPUT_SIZE)
    difference_step = 1e-06
    a = np.zeros((common_STATE_SIZE, common_STATE_SIZE))
    b = np.zeros((common_STATE_SIZE, common_INPUT_SIZE))
    for state_index in range(common_STATE_SIZE):
        perturbation = np.zeros(common_STATE_SIZE)
        perturbation[state_index] = difference_step
        positive = common_paper_state_derivative(parameters, leg_length_m, leg_com_from_wheel_m, single_leg_inertia_kg_m2, equilibrium_state + perturbation, equilibrium_control)
        negative = common_paper_state_derivative(parameters, leg_length_m, leg_com_from_wheel_m, single_leg_inertia_kg_m2, equilibrium_state - perturbation, equilibrium_control)
        a[:, state_index] = (positive - negative) / (2.0 * difference_step)
    for input_index in range(common_INPUT_SIZE):
        perturbation = np.zeros(common_INPUT_SIZE)
        perturbation[input_index] = difference_step
        positive = common_paper_state_derivative(parameters, leg_length_m, leg_com_from_wheel_m, single_leg_inertia_kg_m2, equilibrium_state, equilibrium_control + perturbation)
        negative = common_paper_state_derivative(parameters, leg_length_m, leg_com_from_wheel_m, single_leg_inertia_kg_m2, equilibrium_state, equilibrium_control - perturbation)
        b[:, input_index] = (positive - negative) / (2.0 * difference_step)
    return (a, b)

def common_verify_paper_model_linearization(parameters: common_RobotParameters, samples: list[dict[str, float]]) -> float:
    """Cross-check equations (3)--(9) against the analytic linear model."""
    maximum_error = 0.0
    for sample in samples:
        leg_length = sample['leg_length_m']
        leg_com = sample['leg_com_from_wheel_m']
        single_inertia = sample['single_leg_inertia_kg_m2']
        a_paper, b_paper = common_derive_continuous_model(parameters, leg_length, leg_com, single_inertia)
        a_wheel, b_wheel = common_derive_wheel_coordinate_linear_model(parameters, leg_length, leg_com, single_inertia)
        a_expected, b_expected = common_transform_wheel_model_to_body_state(a_wheel, b_wheel, leg_length)
        maximum_error = max(maximum_error, float(np.max(np.abs(a_paper - a_expected))), float(np.max(np.abs(b_paper - b_expected))))
    if maximum_error > 1e-06:
        raise ValueError(f'paper classical-equation linearization disagrees with the analytic coordinate transform: max error {maximum_error}')
    return maximum_error

def common_controllability_rank(a: np.ndarray, b: np.ndarray) -> int:
    blocks = [b]
    power = np.eye(a.shape[0])
    for _ in range(1, a.shape[0]):
        power = power @ a
        blocks.append(power @ b)
    return int(np.linalg.matrix_rank(np.concatenate(blocks, axis=1)))

def common_continuous_lqr(a: np.ndarray, b: np.ndarray, q_diag: np.ndarray, r_diag: np.ndarray) -> np.ndarray:
    """Solve continuous CARE through the stable Hamiltonian subspace."""
    q = np.diag(q_diag)
    r = np.diag(r_diag)
    r_inverse = np.linalg.inv(r)
    hamiltonian = np.block([[a, -b @ r_inverse @ b.T], [-q, -a.T]])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)
    stable = np.where(eigenvalues.real < -1e-09)[0]
    if stable.size != a.shape[0]:
        raise ValueError(f'Hamiltonian has {stable.size} stable eigenvalues, expected {a.shape[0]}')
    stable_vectors = eigenvectors[:, stable]
    upper = stable_vectors[:a.shape[0], :]
    lower = stable_vectors[a.shape[0]:, :]
    p = np.real(lower @ np.linalg.inv(upper))
    p = 0.5 * (p + p.T)
    return r_inverse @ b.T @ p

def common_verify_article_regression() -> None:
    """Check the CARE solver against the numeric example in controller.m."""
    a = np.array([[0, 1, 0, 0, 0, 0], [265.9556, 0, 0, 0, 80.6327, 0], [0, 0, 0, 1, 0, 0], [-25.4562, 0, 0, 0, 1.8637, 0], [0, 0, 0, 0, 0, 1], [156.6952, 0, 0, 0, 183.0614, 0]], dtype=float)
    b = np.array([[0, 0], [-15.1389, 13.8563], [0, 0], [2.1208, -0.7158], [0, 0], [-4.2238, 16.8001]], dtype=float)
    expected = np.array([[-44.3788, -6.8496, -22.2828, -21.5569, 28.7706, 4.3751], [11.2006, 0.7339, 3.73, 3.2058, 151.73, 4.6387]])
    actual = common_continuous_lqr(a, b, common_ARTICLE_Q_DIAG, common_ARTICLE_R_DIAG)
    max_error = float(np.max(np.abs(actual - expected)))
    if max_error > 0.002:
        raise ValueError(f'article LQR regression failed: max error {max_error}')

def common_fit_gain_polynomial(leg_lengths: np.ndarray, gains: np.ndarray) -> np.ndarray:
    normalized = (leg_lengths - common_GAIN_CENTER_M) / common_GAIN_HALF_RANGE_M
    coefficients = np.zeros((common_INPUT_SIZE, common_STATE_SIZE, 4))
    for input_index in range(common_INPUT_SIZE):
        for state_index in range(common_STATE_SIZE):
            coefficients[input_index, state_index, :] = np.polyfit(normalized, gains[:, input_index, state_index], 3)
    return coefficients

def common_evaluate_gain_polynomial(coefficients: np.ndarray, leg_length_m: float) -> np.ndarray:
    normalized = (leg_length_m - common_GAIN_CENTER_M) / common_GAIN_HALF_RANGE_M
    gain = np.zeros((common_INPUT_SIZE, common_STATE_SIZE))
    for input_index in range(common_INPUT_SIZE):
        for state_index in range(common_STATE_SIZE):
            gain[input_index, state_index] = np.polyval(coefficients[input_index, state_index], normalized)
    return gain

def common_print_cpp_initializer(coefficients: np.ndarray) -> None:
    print('constexpr double kGainPolynomial[2][6][4] = {')
    for input_index in range(common_INPUT_SIZE):
        print('    {')
        for state_index in range(common_STATE_SIZE):
            values = ', '.join((f'{value:.15g}' for value in coefficients[input_index, state_index]))
            print(f'        {{{values}}},')
        print('    }' + (',' if input_index + 1 < common_INPUT_SIZE else ''))
    print('};')

def common_write_samples(output_path: Path, rows: list[dict[str, object]]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    gain_fields = [f'k{input_index}{state_index}' for input_index in range(common_INPUT_SIZE) for state_index in range(common_STATE_SIZE)]
    a_fields = [f'a{row}{column}' for row in range(common_STATE_SIZE) for column in range(common_STATE_SIZE)]
    b_fields = [f'b{row}{column}' for row in range(common_STATE_SIZE) for column in range(common_INPUT_SIZE)]
    fields = ['leg_length_m', 'leg_com_from_wheel_m', 'leg_com_to_pivot_m', 'single_leg_inertia_kg_m2', 'total_leg_inertia_kg_m2', 'controllability_rank', 'max_closed_loop_real', *a_fields, *b_fields, *gain_fields]
    with output_path.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fields, lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

def common_write_coefficients(output_path: Path, coefficients: np.ndarray) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='') as file:
        writer = csv.writer(file, lineterminator='\n')
        writer.writerow(['input_index', 'state_index', 'p3', 'p2', 'p1', 'p0'])
        for input_index in range(common_INPUT_SIZE):
            for state_index in range(common_STATE_SIZE):
                writer.writerow([input_index, state_index, *coefficients[input_index, state_index, :]])

def common_main() -> None:
    default_input = Path(__file__).with_name('data') / 'leg_mass_properties0820.csv'
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=default_input)
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--coeff-output', type=Path, default=None)
    parser.add_argument('--q', default=','.join((str(value) for value in common_CURRENT_Q_DIAG)))
    parser.add_argument('--r', default=','.join((str(value) for value in common_CURRENT_R_DIAG)))
    args = parser.parse_args()
    q_diag = np.array([float(value) for value in args.q.split(',')])
    r_diag = np.array([float(value) for value in args.r.split(',')])
    if q_diag.shape != (common_STATE_SIZE,) or r_diag.shape != (common_INPUT_SIZE,):
        raise ValueError('Q must have 6 diagonal values and R must have 2')
    common_verify_article_regression()
    parameters = common_RobotParameters()
    samples = common_read_leg_samples(args.input)
    model_linearization_max_error = common_verify_paper_model_linearization(parameters, samples)
    rows: list[dict[str, object]] = []
    gains: list[np.ndarray] = []
    leg_lengths: list[float] = []
    for sample in samples:
        leg_length = sample['leg_length_m']
        leg_com = sample['leg_com_from_wheel_m']
        single_inertia = sample['single_leg_inertia_kg_m2']
        a, b = common_derive_continuous_model(parameters, leg_length, leg_com, single_inertia)
        rank = common_controllability_rank(a, b)
        if rank != common_STATE_SIZE:
            raise ValueError(f'uncontrollable model at h={leg_length}: rank={rank}')
        gain = common_continuous_lqr(a, b, q_diag, r_diag)
        closed_loop = np.linalg.eigvals(a - b @ gain)
        row: dict[str, object] = {'leg_length_m': leg_length, 'leg_com_from_wheel_m': leg_com, 'leg_com_to_pivot_m': leg_length - leg_com, 'single_leg_inertia_kg_m2': single_inertia, 'total_leg_inertia_kg_m2': 2.0 * single_inertia, 'controllability_rank': rank, 'max_closed_loop_real': float(np.max(closed_loop.real))}
        for input_index in range(common_INPUT_SIZE):
            for state_index in range(common_STATE_SIZE):
                row[f'k{input_index}{state_index}'] = float(gain[input_index, state_index])
        for matrix_row in range(common_STATE_SIZE):
            for matrix_column in range(common_STATE_SIZE):
                row[f'a{matrix_row}{matrix_column}'] = float(a[matrix_row, matrix_column])
        for matrix_row in range(common_STATE_SIZE):
            for input_index in range(common_INPUT_SIZE):
                row[f'b{matrix_row}{input_index}'] = float(b[matrix_row, input_index])
        rows.append(row)
        gains.append(gain)
        leg_lengths.append(leg_length)
    leg_lengths_array = np.array(leg_lengths)
    gains_array = np.stack(gains)
    coefficients = common_fit_gain_polynomial(leg_lengths_array, gains_array)
    maximum_fit_error = 0.0
    maximum_fitted_closed_loop_real = -np.inf
    for row, gain in zip(rows, gains):
        leg_length = float(row['leg_length_m'])
        fitted = common_evaluate_gain_polynomial(coefficients, leg_length)
        maximum_fit_error = max(maximum_fit_error, float(np.max(np.abs(fitted - gain))))
        a, b = common_derive_continuous_model(parameters, leg_length, float(row['leg_com_from_wheel_m']), float(row['single_leg_inertia_kg_m2']))
        fitted_eigenvalues = np.linalg.eigvals(a - b @ fitted)
        maximum_fitted_closed_loop_real = max(maximum_fitted_closed_loop_real, float(np.max(fitted_eigenvalues.real)))
    dense_maximum_closed_loop_real = -np.inf
    dense_lengths = np.linspace(leg_lengths_array.min(), leg_lengths_array.max(), 401)
    com_values = np.array([float(row['leg_com_from_wheel_m']) for row in rows])
    inertia_values = np.array([float(row['single_leg_inertia_kg_m2']) for row in rows])
    for leg_length in dense_lengths:
        leg_com = float(np.interp(leg_length, leg_lengths_array, com_values))
        single_inertia = float(np.interp(leg_length, leg_lengths_array, inertia_values))
        a, b = common_derive_continuous_model(parameters, leg_length, leg_com, single_inertia)
        fitted = common_evaluate_gain_polynomial(coefficients, leg_length)
        eigenvalues = np.linalg.eigvals(a - b @ fitted)
        dense_maximum_closed_loop_real = max(dense_maximum_closed_loop_real, float(np.max(eigenvalues.real)))
    if maximum_fitted_closed_loop_real >= 0.0:
        raise ValueError(f'fitted schedule is unstable at a measured point: max real pole {maximum_fitted_closed_loop_real}')
    if dense_maximum_closed_loop_real >= 0.0:
        raise ValueError(f'fitted schedule is unstable between measured points: max real pole {dense_maximum_closed_loop_real}')
    if args.output is not None:
        common_write_samples(args.output, rows)
    if args.coeff_output is not None:
        common_write_coefficients(args.coeff_output, coefficients)
    print(f'points={len(rows)}')
    print(f'paper_model_linearization_max_abs_error={model_linearization_max_error:.9g}')
    print(f'leg_length_range_m={leg_lengths_array.min():.5f},{leg_lengths_array.max():.5f}')
    print(f'max_gain_fit_abs_error={maximum_fit_error:.9g}')
    print(f'max_fitted_closed_loop_real={maximum_fitted_closed_loop_real:.9g}')
    print(f'dense_max_fitted_closed_loop_real={dense_maximum_closed_loop_real:.9g}')
    common_print_cpp_initializer(coefficients)
differential_STATE_SIZE = 4
differential_INPUT_SIZE = 2
differential_GAIN_CENTER_M = 0.25
differential_GAIN_HALF_RANGE_M = 0.15

@dataclass(frozen=True)
class differential_RobotParameters:
    wheel_mass_each_kg: float = 0.6
    wheel_inertia_each_kg_m2: float = 0.0002523
    wheel_radius_m: float = 0.058
    half_track_m: float = 0.215
    leg_mass_each_kg: float = 1.033
    body_mass_kg: float = 8.788
    body_yaw_inertia_kg_m2: float = 0.219437247
    gravity_m_s2: float = 9.80665
differential_DEFAULT_Q_DIAG = np.array([30.0, 8.0, 1800.0, 45.0])
differential_DEFAULT_R_DIAG = np.array([8.0, 1.0])

def differential_read_leg_samples(path: Path) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'leg_length_m', 'leg_com_from_wheel_m', 'single_leg_inertia_kg_m2'}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f'{path} must contain {sorted(required)}')
        for row in reader:
            samples.append({name: float(row[name]) for name in required})
    return sorted(samples, key=lambda sample: sample['leg_length_m'])

def differential_continuous_lqr(a: np.ndarray, b: np.ndarray, q_diag: np.ndarray, r_diag: np.ndarray) -> np.ndarray:
    q = np.diag(q_diag)
    r_inverse = np.diag(1.0 / r_diag)
    hamiltonian = np.block([[a, -b @ r_inverse @ b.T], [-q, -a.T]])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)
    stable = np.where(eigenvalues.real < -1e-09)[0]
    if stable.size != a.shape[0]:
        raise ValueError(f'CARE stable subspace has {stable.size} vectors')
    vectors = eigenvectors[:, stable]
    upper = vectors[:a.shape[0], :]
    lower = vectors[a.shape[0]:, :]
    p = np.real(lower @ np.linalg.inv(upper))
    p = 0.5 * (p + p.T)
    return r_inverse @ b.T @ p

def differential_controllability_rank(a: np.ndarray, b: np.ndarray) -> int:
    blocks = [b]
    power = np.eye(a.shape[0])
    for _ in range(1, a.shape[0]):
        power = power @ a
        blocks.append(power @ b)
    return int(np.linalg.matrix_rank(np.concatenate(blocks, axis=1)))

def differential_derive_model(parameters: differential_RobotParameters, leg_length_m: float, leg_com_from_wheel_m: float, single_leg_inertia_kg_m2: float) -> tuple[np.ndarray, np.ndarray]:
    """Return the symmetric differential block of the article model."""
    radius = parameters.wheel_radius_m
    half_track = parameters.half_track_m
    wheel_mass = parameters.wheel_mass_each_kg
    wheel_inertia = parameters.wheel_inertia_each_kg_m2
    leg_mass = parameters.leg_mass_each_kg
    body_mass = parameters.body_mass_kg
    yaw_inertia = parameters.body_yaw_inertia_kg_m2
    gravity = parameters.gravity_m_s2
    leg_length = leg_length_m
    wheel_to_com = leg_com_from_wheel_m
    pivot_to_com = leg_length - wheel_to_com
    leg_inertia = single_leg_inertia_kg_m2
    mass_wheel = -(2.0 * wheel_inertia * half_track ** 2 + radius ** 2 * pivot_to_com ** 2 * leg_mass + radius ** 2 * leg_length ** 2 * leg_mass + 2.0 * radius ** 2 * leg_length ** 2 * wheel_mass - radius ** 2 * wheel_to_com ** 2 * leg_mass) / (2.0 * radius * half_track)
    mass_leg = (8.0 * leg_inertia * leg_length ** 2 + pivot_to_com ** 4 * leg_mass - 3.0 * leg_length ** 4 * leg_mass + wheel_to_com ** 4 * leg_mass + 2.0 * pivot_to_com ** 2 * leg_length ** 2 * leg_mass - 2.0 * pivot_to_com ** 2 * wheel_to_com ** 2 * leg_mass + 2.0 * leg_length ** 2 * wheel_to_com ** 2 * leg_mass) / (8.0 * leg_length ** 2)
    yaw_wheel = -(2.0 * wheel_inertia * half_track ** 2 + yaw_inertia * radius ** 2) / (half_track * radius)
    yaw_leg = -yaw_inertia * leg_length / half_track
    acceleration_mass = np.array([[mass_wheel, mass_leg], [yaw_wheel, yaw_leg]])
    input_map = np.array([[-(radius + leg_length) / radius, 1.0], [-2.0 * half_track / radius, 0.0]])
    gravity_stiffness = gravity * (leg_length ** 2 * body_mass - pivot_to_com ** 2 * leg_mass + leg_length ** 2 * leg_mass + wheel_to_com ** 2 * leg_mass) / (2.0 * leg_length)
    generalized_input = np.linalg.solve(acceleration_mass, input_map)
    generalized_gravity = np.linalg.solve(acceleration_mass, np.array([gravity_stiffness, 0.0]))
    yaw_projection = np.array([-radius / half_track, -leg_length / half_track])
    a = np.zeros((differential_STATE_SIZE, differential_STATE_SIZE))
    b = np.zeros((differential_STATE_SIZE, differential_INPUT_SIZE))
    a[0, 1] = 1.0
    a[1, 2] = yaw_projection @ generalized_gravity
    a[2, 3] = 1.0
    a[3, 2] = generalized_gravity[1]
    b[1, :] = yaw_projection @ generalized_input
    b[3, :] = generalized_input[1, :]
    return (a, b)

def differential_fit_cubic(lengths: np.ndarray, gains: np.ndarray) -> np.ndarray:
    normalized = (lengths - differential_GAIN_CENTER_M) / differential_GAIN_HALF_RANGE_M
    coefficients = np.empty((differential_INPUT_SIZE, differential_STATE_SIZE, 4))
    for input_index in range(differential_INPUT_SIZE):
        for state_index in range(differential_STATE_SIZE):
            coefficients[input_index, state_index] = np.polyfit(normalized, gains[:, input_index, state_index], 3)
    return coefficients

def differential_evaluate_cubic(coefficients: np.ndarray, leg_length_m: float) -> np.ndarray:
    x = (leg_length_m - differential_GAIN_CENTER_M) / differential_GAIN_HALF_RANGE_M
    return ((coefficients[:, :, 0] * x + coefficients[:, :, 1]) * x + coefficients[:, :, 2]) * x + coefficients[:, :, 3]

def differential_main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=root / 'tools/data/leg_mass_properties0820.csv')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--q', default=','.join(map(str, differential_DEFAULT_Q_DIAG)))
    parser.add_argument('--r', default=','.join(map(str, differential_DEFAULT_R_DIAG)))
    args = parser.parse_args()
    q_diag = np.array([float(value) for value in args.q.split(',')])
    r_diag = np.array([float(value) for value in args.r.split(',')])
    if q_diag.shape != (differential_STATE_SIZE,) or r_diag.shape != (differential_INPUT_SIZE,):
        raise ValueError('Q requires 4 values and R requires 2 values')
    parameters = differential_RobotParameters()
    samples = differential_read_leg_samples(args.input)
    lengths: list[float] = []
    gains: list[np.ndarray] = []
    rows: list[dict[str, float | int]] = []
    for sample in samples:
        length = sample['leg_length_m']
        a, b = differential_derive_model(parameters, length, sample['leg_com_from_wheel_m'], sample['single_leg_inertia_kg_m2'])
        rank = differential_controllability_rank(a, b)
        if rank != differential_STATE_SIZE:
            raise ValueError(f'uncontrollable differential model at {length}: {rank}')
        gain = differential_continuous_lqr(a, b, q_diag, r_diag)
        closed_loop = np.linalg.eigvals(a - b @ gain)
        row: dict[str, float | int] = {'leg_length_m': length, 'controllability_rank': rank, 'max_closed_loop_real': float(closed_loop.real.max())}
        for output in range(differential_INPUT_SIZE):
            for state in range(differential_STATE_SIZE):
                row[f'k{output}{state}'] = float(gain[output, state])
        rows.append(row)
        lengths.append(length)
        gains.append(gain)
    coefficients = differential_fit_cubic(np.array(lengths), np.stack(gains))
    dense_worst = -np.inf
    for length in np.linspace(min(lengths), max(lengths), 401):
        lower = max((index for index, value in enumerate(lengths) if value <= length))
        upper = min((index for index, value in enumerate(lengths) if value >= length))
        ratio = 0.0 if lower == upper else (length - lengths[lower]) / (lengths[upper] - lengths[lower])
        sample = {name: (1.0 - ratio) * samples[lower][name] + ratio * samples[upper][name] for name in samples[lower]}
        a, b = differential_derive_model(parameters, length, sample['leg_com_from_wheel_m'], sample['single_leg_inertia_kg_m2'])
        fitted = differential_evaluate_cubic(coefficients, length)
        dense_worst = max(dense_worst, float(np.linalg.eigvals(a - b @ fitted).real.max()))
    if dense_worst >= 0.0:
        raise ValueError(f'unstable fitted schedule: max real pole {dense_worst}')
    print('differential model parameters:', parameters)
    print('Q:', q_diag, 'R:', r_diag)
    print('dense maximum closed-loop real pole:', dense_worst)
    print('C++ coefficients [output][state][descending cubic]:')
    print(np.array2string(coefficients, precision=16, separator=', '))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)
STATE_SIZE = 10
INPUT_SIZE = 4
GAIN_CENTER_M = 0.25
GAIN_HALF_RANGE_M = 0.15

def coordinate_transforms() -> tuple[np.ndarray, np.ndarray]:
    """Return z=T*x and v=S*u for modal and physical coordinates."""
    state = np.zeros((STATE_SIZE, STATE_SIZE))
    state[0, 4] = 0.5
    state[0, 6] = 0.5
    state[1, 5] = 0.5
    state[1, 7] = 0.5
    state[2, 0] = 1.0
    state[3, 1] = 1.0
    state[4, 8] = 1.0
    state[5, 9] = 1.0
    state[6, 2] = 1.0
    state[7, 3] = 1.0
    state[8, 4] = 0.5
    state[8, 6] = -0.5
    state[9, 5] = 0.5
    state[9, 7] = -0.5
    actuator = np.array([[1.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 1.0], [0.5, -0.5, 0.0, 0.0], [0.0, 0.0, 0.5, -0.5]])
    return (state, actuator)

def continuous_lqr(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    state_size = a.shape[0]
    r_inverse = np.linalg.inv(r)
    hamiltonian = np.block([[a, -b @ r_inverse @ b.T], [-q, -a.T]])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)
    stable = np.where(eigenvalues.real < -1e-09)[0]
    if stable.size != state_size:
        raise ValueError(f'CARE stable subspace has {stable.size} vectors')
    vectors = eigenvectors[:, stable]
    upper = vectors[:state_size, :]
    lower = vectors[state_size:, :]
    p = np.real(lower @ np.linalg.inv(upper))
    p = 0.5 * (p + p.T)
    return r_inverse @ b.T @ p

def controllability_rank(a: np.ndarray, b: np.ndarray) -> int:
    blocks = [b]
    power = np.eye(STATE_SIZE)
    for _ in range(1, STATE_SIZE):
        power = power @ a
        blocks.append(power @ b)
    return int(np.linalg.matrix_rank(np.concatenate(blocks, axis=1)))

def derive_full_model(sample: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    common_a, common_b = common_derive_continuous_model(common_RobotParameters(), sample['leg_length_m'], sample['leg_com_from_wheel_m'], sample['single_leg_inertia_kg_m2'])
    differential_a, differential_b = differential_derive_model(differential_RobotParameters(), sample['leg_length_m'], sample['leg_com_from_wheel_m'], sample['single_leg_inertia_kg_m2'])
    modal_a = np.zeros((STATE_SIZE, STATE_SIZE))
    modal_b = np.zeros((STATE_SIZE, INPUT_SIZE))
    modal_a[:6, :6] = common_a
    modal_a[6:, 6:] = differential_a
    modal_b[:6, :2] = common_b
    modal_b[6:, 2:] = differential_b
    state, actuator = coordinate_transforms()
    state_inverse = np.linalg.inv(state)
    return (state_inverse @ modal_a @ state, state_inverse @ modal_b @ actuator)

def physical_costs() -> tuple[np.ndarray, np.ndarray]:
    state, actuator = coordinate_transforms()
    modal_q = np.diag(np.concatenate((common_CURRENT_Q_DIAG, differential_DEFAULT_Q_DIAG)))
    modal_r = np.diag(np.concatenate((common_CURRENT_R_DIAG, differential_DEFAULT_R_DIAG)))
    return (state.T @ modal_q @ state, actuator.T @ modal_r @ actuator)

def fit_cubic(lengths: np.ndarray, gains: np.ndarray) -> np.ndarray:
    normalized = (lengths - GAIN_CENTER_M) / GAIN_HALF_RANGE_M
    coefficients = np.empty((INPUT_SIZE, STATE_SIZE, 4))
    for output in range(INPUT_SIZE):
        for state in range(STATE_SIZE):
            coefficients[output, state] = np.polyfit(normalized, gains[:, output, state], 3)
    return coefficients

def evaluate_cubic(coefficients: np.ndarray, length: float) -> np.ndarray:
    x = (length - GAIN_CENTER_M) / GAIN_HALF_RANGE_M
    return ((coefficients[:, :, 0] * x + coefficients[:, :, 1]) * x + coefficients[:, :, 2]) * x + coefficients[:, :, 3]

def write_cpp(path: Path, coefficients: np.ndarray) -> None:
    lines = ['// Generated by tools/derive_lqr_schedule.py --mode unified.', 'constexpr double kGainPolynomial[4][10][4] = {']
    for output in range(INPUT_SIZE):
        lines.append('\t{')
        for state in range(STATE_SIZE):
            values = ', '.join((f'{value:.17g}' for value in coefficients[output, state]))
            lines.append(f'\t\t{{{values}}},')
        lines.append('\t},')
    lines.append('};')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n')

def unified_main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=root / 'tools/data/leg_mass_properties0820.csv')
    parser.add_argument('--output', type=Path, default=root / 'tools/generated/unified_lqr_samples.csv')
    parser.add_argument('--cpp-output', type=Path, default=root / 'src/chassis_controller/chassis/unified_lqr_coefficients.inc')
    args = parser.parse_args()
    samples = common_read_leg_samples(args.input)
    q, r = physical_costs()
    lengths: list[float] = []
    gains: list[np.ndarray] = []
    rows: list[dict[str, float | int]] = []
    state_transform, actuator_transform = coordinate_transforms()
    modal_q = np.diag(np.concatenate((common_CURRENT_Q_DIAG, differential_DEFAULT_Q_DIAG)))
    modal_r = np.diag(np.concatenate((common_CURRENT_R_DIAG, differential_DEFAULT_R_DIAG)))
    maximum_equivalence_error = 0.0
    for sample in samples:
        a, b = derive_full_model(sample)
        rank = controllability_rank(a, b)
        if rank != STATE_SIZE:
            raise ValueError(f"uncontrollable full model at {sample['leg_length_m']}: {rank}")
        gain = continuous_lqr(a, b, q, r)
        common_a, common_b = common_derive_continuous_model(common_RobotParameters(), sample['leg_length_m'], sample['leg_com_from_wheel_m'], sample['single_leg_inertia_kg_m2'])
        differential_a, differential_b = differential_derive_model(differential_RobotParameters(), sample['leg_length_m'], sample['leg_com_from_wheel_m'], sample['single_leg_inertia_kg_m2'])
        modal_gain = np.zeros((INPUT_SIZE, STATE_SIZE))
        modal_gain[:2, :6] = continuous_lqr(common_a, common_b, modal_q[:6, :6], modal_r[:2, :2])
        modal_gain[2:, 6:] = continuous_lqr(differential_a, differential_b, modal_q[6:, 6:], modal_r[2:, 2:])
        expected = np.linalg.inv(actuator_transform) @ modal_gain @ state_transform
        maximum_equivalence_error = max(maximum_equivalence_error, float(np.max(np.abs(gain - expected))))
        poles = np.linalg.eigvals(a - b @ gain)
        rows.append({'leg_length_m': sample['leg_length_m'], 'controllability_rank': rank, 'max_closed_loop_real': float(poles.real.max())})
        lengths.append(sample['leg_length_m'])
        gains.append(gain)
    if maximum_equivalence_error > 1e-05:
        raise ValueError(f'full/modal CARE mismatch: {maximum_equivalence_error}')
    coefficients = fit_cubic(np.array(lengths), np.stack(gains))
    dense_worst = -np.inf
    for length in np.linspace(min(lengths), max(lengths), 401):
        lower = max((index for index, value in enumerate(lengths) if value <= length))
        upper = min((index for index, value in enumerate(lengths) if value >= length))
        ratio = 0.0 if lower == upper else (length - lengths[lower]) / (lengths[upper] - lengths[lower])
        sample = {name: (1.0 - ratio) * samples[lower][name] + ratio * samples[upper][name] for name in samples[lower]}
        a, b = derive_full_model(sample)
        dense_worst = max(dense_worst, float(np.linalg.eigvals(a - b @ evaluate_cubic(coefficients, length)).real.max()))
    if dense_worst >= 0.0:
        raise ValueError(f'unstable fitted full schedule: {dense_worst}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    write_cpp(args.cpp_output, coefficients)
    print('full model dimensions: A=10x10 B=10x4 K=4x10')
    print('maximum direct/modal CARE equivalence error:', maximum_equivalence_error)
    print('dense maximum closed-loop real pole:', dense_worst)

def main() -> None:
    mode = 'unified'
    if '--mode' in sys.argv:
        index = sys.argv.index('--mode')
        if index + 1 >= len(sys.argv):
            raise SystemExit('--mode requires common, differential, or unified')
        mode = sys.argv[index + 1]
        del sys.argv[index:index + 2]
    handlers = {'common': common_main, 'differential': differential_main, 'unified': unified_main}
    try:
        handler = handlers[mode]
    except KeyError:
        raise SystemExit(f'unknown LQR derivation mode: {mode}') from None
    handler()
if __name__ == '__main__':
    main()
