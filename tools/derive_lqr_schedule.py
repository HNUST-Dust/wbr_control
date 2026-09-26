#!/usr/bin/env python3
"""Derive the firmware's unified ten-state, four-input wheel-leg LQR schedule."""

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

STATE_SIZE = 10
INPUT_SIZE = 4
GAIN_CENTER_M = 0.25
GAIN_HALF_RANGE_M = 0.15

# State: position, speed, yaw, yaw_rate, left angle/rate, right angle/rate, pitch, pitch_rate.
# Input: left/right wheel torque, left/right leg torque.
Q = np.array([
    [1500., 0.  , 0.    , 0.    , 0.   , 0.  , 0.   , 0.  , 0.    , 0.  ],
    [0.   , 1.  , 0.    , 0.    , 0.   , 0.  , 0.   , 0.  , 0.    , 0.  ],
    [0.   , 0.  , 30.   , 0.    , 0.   , 0.  , 0.   , 0.  , 0.    , 0.  ],
    [0.   , 0.  , 0.    , 8.    , 0.   , 0.  , 0.   , 0.  , 0.    , 0.  ],
    [0.   , 0.  , 0.    , 0.    , 700. , 0.  , -200., 0.  , 0.    , 0.  ],
    [0.   , 0.  , 0.    , 0.    , 0.   , 12.5, 0.   , -10., 0.    , 0.  ],
    [0.   , 0.  , 0.    , 0.    , -200., 0.  , 700. , 0.  , 0.    , 0.  ],
    [0.   , 0.  , 0.    , 0.    , 0.   , -10., 0.   , 12.5, 0.    , 0.  ],
    [0.   , 0.  , 0.    , 0.    , 0.   , 0.  , 0.   , 0.  , 20000., 0.  ],
    [0.   , 0.  , 0.    , 0.    , 0.   , 0.  , 0.   , 0.  , 0.    , 1.  ],
])
R = np.array([
    [62.    , 58.    , 0.   , 0.    ],
    [58.    , 62.    , 0.   , 0.    ],
    [0.     , 0.     , 1.25 , 0.75  ],
    [0.     , 0.     , 0.75 , 1.25  ],
])


@dataclass(frozen=True)
class RobotParameters:
    wheel_mass_each_kg: float = 0.6
    wheel_inertia_each_kg_m2: float = 0.0002523
    wheel_radius_m: float = 0.058
    half_track_m: float = 0.215
    leg_mass_each_kg: float = 1.033
    body_mass_kg: float = 8.788
    body_pitch_inertia_kg_m2: float = 0.133703941
    body_yaw_inertia_kg_m2: float = 0.219437247
    body_com_offset_m: float = -0.05235
    gravity_m_s2: float = 9.80665


def read_leg_samples(path: Path) -> list[dict[str, float]]:
    samples = []
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'leg_length_m', 'leg_com_from_wheel_m', 'single_leg_inertia_kg_m2'}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f'{path} must contain {sorted(required)}')
        for row in reader:
            sample = {name: float(row[name]) for name in required}
            if not 0. < sample['leg_com_from_wheel_m'] < sample['leg_length_m']:
                raise ValueError(f'invalid leg geometry: {sample}')
            samples.append(sample)
    return sorted(samples, key=lambda sample: sample['leg_length_m'])


def planar_state_derivative(parameters, sample, state, control):
    """Evaluate the pitch/translation equations used by the ten-state model."""
    mw = 2. * parameters.wheel_mass_each_kg
    iw = 2. * parameters.wheel_inertia_each_kg_m2
    radius = parameters.wheel_radius_m
    mp = 2. * parameters.leg_mass_each_kg
    ip = 2. * sample['single_leg_inertia_kg_m2']
    body_mass = parameters.body_mass_kg
    body_inertia = parameters.body_pitch_inertia_kg_m2
    body_offset = parameters.body_com_offset_m
    gravity = parameters.gravity_m_s2
    leg_length = sample['leg_length_m']
    leg_com = sample['leg_com_from_wheel_m']
    leg_com_to_pivot = leg_length - leg_com
    theta, theta_rate, _, body_speed, phi, phi_rate = state
    wheel_torque, posture_torque = control
    sin_theta, cos_theta = np.sin(theta), np.cos(theta)
    sin_phi, cos_phi = np.sin(phi), np.cos(phi)
    coefficients = np.zeros((7, 7))
    rhs = np.zeros(7)
    coefficients[0, 0], coefficients[0, 3] = iw / radius + mw * radius, radius
    rhs[0] = wheel_torque
    coefficients[1, 0] = -mp
    coefficients[1, 1] = -mp * leg_com * cos_theta
    coefficients[1, 3], coefficients[1, 5] = 1., -1.
    rhs[1] = -mp * leg_com * theta_rate**2 * sin_theta
    coefficients[2, 1] = mp * leg_com * sin_theta
    coefficients[2, 4], coefficients[2, 6] = 1., -1.
    rhs[2] = mp * gravity - mp * leg_com * theta_rate**2 * cos_theta
    coefficients[3, 1] = ip
    coefficients[3, 3] = leg_com * cos_theta
    coefficients[3, 4] = -leg_com * sin_theta
    coefficients[3, 5] = leg_com_to_pivot * cos_theta
    coefficients[3, 6] = -leg_com_to_pivot * sin_theta
    rhs[3] = -wheel_torque + posture_torque
    coefficients[4, 0] = -body_mass
    coefficients[4, 1] = -body_mass * leg_length * cos_theta
    coefficients[4, 2] = body_mass * body_offset * cos_phi
    coefficients[4, 5] = 1.
    rhs[4] = body_mass * (-leg_length * theta_rate**2 * sin_theta
                          + body_offset * phi_rate**2 * sin_phi)
    coefficients[5, 1] = body_mass * leg_length * sin_theta
    coefficients[5, 2] = body_mass * body_offset * sin_phi
    coefficients[5, 6] = 1.
    rhs[5] = body_mass * (gravity - leg_length * theta_rate**2 * cos_theta
                          - body_offset * phi_rate**2 * cos_phi)
    coefficients[6, 2] = body_inertia
    coefficients[6, 5] = -body_offset * cos_phi
    coefficients[6, 6] = -body_offset * sin_phi
    rhs[6] = posture_torque
    wheel_accel, theta_accel, phi_accel = np.linalg.solve(coefficients, rhs)[:3]
    body_accel = (wheel_accel + leg_length * theta_accel * cos_theta
                  - leg_length * theta_rate**2 * sin_theta)
    return np.array([theta_rate, theta_accel, body_speed, body_accel,
                     phi_rate, phi_accel])


def planar_matrices(parameters, sample):
    state_size, input_size, step = 6, 2, 1.e-6
    state, control = np.zeros(state_size), np.zeros(input_size)
    a, b = np.zeros((state_size, state_size)), np.zeros((state_size, input_size))
    for index in range(state_size):
        delta = np.zeros(state_size)
        delta[index] = step
        a[:, index] = (planar_state_derivative(parameters, sample, state + delta, control)
                       - planar_state_derivative(parameters, sample, state - delta, control)) / (2. * step)
    for index in range(input_size):
        delta = np.zeros(input_size)
        delta[index] = step
        b[:, index] = (planar_state_derivative(parameters, sample, state, control + delta)
                       - planar_state_derivative(parameters, sample, state, control - delta)) / (2. * step)
    return a, b


def yaw_matrices(parameters, sample):
    radius, half_track = parameters.wheel_radius_m, parameters.half_track_m
    wheel_mass = parameters.wheel_mass_each_kg
    wheel_inertia = parameters.wheel_inertia_each_kg_m2
    leg_mass = parameters.leg_mass_each_kg
    body_mass, yaw_inertia = parameters.body_mass_kg, parameters.body_yaw_inertia_kg_m2
    leg_length = sample['leg_length_m']
    wheel_to_com = sample['leg_com_from_wheel_m']
    pivot_to_com = leg_length - wheel_to_com
    leg_inertia = sample['single_leg_inertia_kg_m2']
    mass_wheel = -(2. * wheel_inertia * half_track**2
                   + radius**2 * pivot_to_com**2 * leg_mass
                   + radius**2 * leg_length**2 * leg_mass
                   + 2. * radius**2 * leg_length**2 * wheel_mass
                   - radius**2 * wheel_to_com**2 * leg_mass) / (2. * radius * half_track)
    mass_leg = (8. * leg_inertia * leg_length**2
                + pivot_to_com**4 * leg_mass - 3. * leg_length**4 * leg_mass
                + wheel_to_com**4 * leg_mass
                + 2. * pivot_to_com**2 * leg_length**2 * leg_mass
                - 2. * pivot_to_com**2 * wheel_to_com**2 * leg_mass
                + 2. * leg_length**2 * wheel_to_com**2 * leg_mass) / (8. * leg_length**2)
    yaw_wheel = -(2. * wheel_inertia * half_track**2
                  + yaw_inertia * radius**2) / (half_track * radius)
    yaw_leg = -yaw_inertia * leg_length / half_track
    mass = np.array([[mass_wheel, mass_leg], [yaw_wheel, yaw_leg]])
    input_map = np.array([[-(radius + leg_length) / radius, 1.],
                          [-2. * half_track / radius, 0.]])
    stiffness = parameters.gravity_m_s2 * (leg_length**2 * body_mass
                - pivot_to_com**2 * leg_mass + leg_length**2 * leg_mass
                + wheel_to_com**2 * leg_mass) / (2. * leg_length)
    input_accel = np.linalg.solve(mass, input_map)
    gravity_accel = np.linalg.solve(mass, np.array([stiffness, 0.]))
    projection = np.array([-radius / half_track, -leg_length / half_track])
    a, b = np.zeros((4, 4)), np.zeros((4, 2))
    a[0, 1], a[2, 3] = 1., 1.
    a[1, 2], a[3, 2] = projection @ gravity_accel, gravity_accel[1]
    b[1, :], b[3, :] = projection @ input_accel, input_accel[1, :]
    return a, b


def coordinate_transforms():
    state = np.zeros((STATE_SIZE, STATE_SIZE))
    state[0, 4], state[0, 6] = .5, .5
    state[1, 5], state[1, 7] = .5, .5
    state[2, 0], state[3, 1] = 1., 1.
    state[4, 8], state[5, 9] = 1., 1.
    state[6, 2], state[7, 3] = 1., 1.
    state[8, 4], state[8, 6] = .5, -.5
    state[9, 5], state[9, 7] = .5, -.5
    actuator = np.array([[1., 1., 0., 0.], [0., 0., 1., 1.],
                         [.5, -.5, 0., 0.], [0., 0., .5, -.5]])
    return state, actuator


def derive_model(sample):
    """Return one 10-state, four-input model in firmware physical coordinates."""
    parameters = RobotParameters()
    planar_a, planar_b = planar_matrices(parameters, sample)
    yaw_a, yaw_b = yaw_matrices(parameters, sample)
    reduced_a, reduced_b = np.zeros((STATE_SIZE, STATE_SIZE)), np.zeros((STATE_SIZE, INPUT_SIZE))
    reduced_a[:6, :6], reduced_a[6:, 6:] = planar_a, yaw_a
    reduced_b[:6, :2], reduced_b[6:, 2:] = planar_b, yaw_b
    state_transform, actuator_transform = coordinate_transforms()
    inverse_state = np.linalg.inv(state_transform)
    return inverse_state @ reduced_a @ state_transform, inverse_state @ reduced_b @ actuator_transform


def continuous_lqr(a, b):
    r_inverse = np.linalg.inv(R)
    hamiltonian = np.block([[a, -b @ r_inverse @ b.T], [-Q, -a.T]])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)
    stable = np.where(eigenvalues.real < -1.e-9)[0]
    if stable.size != STATE_SIZE:
        raise ValueError(f'CARE stable subspace has {stable.size} vectors')
    vectors = eigenvectors[:, stable]
    p = np.real(vectors[STATE_SIZE:, :] @ np.linalg.inv(vectors[:STATE_SIZE, :]))
    p = .5 * (p + p.T)
    return r_inverse @ b.T @ p


def controllability_rank(a, b):
    blocks, power = [b], np.eye(STATE_SIZE)
    for _ in range(1, STATE_SIZE):
        power = power @ a
        blocks.append(power @ b)
    return int(np.linalg.matrix_rank(np.concatenate(blocks, axis=1)))


def fit_cubic(lengths, gains):
    x = (lengths - GAIN_CENTER_M) / GAIN_HALF_RANGE_M
    coefficients = np.empty((INPUT_SIZE, STATE_SIZE, 4))
    for output in range(INPUT_SIZE):
        for state in range(STATE_SIZE):
            coefficients[output, state] = np.polyfit(x, gains[:, output, state], 3)
    return coefficients


def evaluate_cubic(coefficients, length):
    x = (length - GAIN_CENTER_M) / GAIN_HALF_RANGE_M
    return ((coefficients[:, :, 0] * x + coefficients[:, :, 1]) * x
            + coefficients[:, :, 2]) * x + coefficients[:, :, 3]


def write_cpp(path, coefficients):
    lines = ['// Generated by tools/derive_lqr_schedule.py --mode unified.',
             'constexpr double kGainPolynomial[4][10][4] = {']
    for output in range(INPUT_SIZE):
        lines.append('\t{')
        for state in range(STATE_SIZE):
            values = ', '.join(f'{value:.17g}' for value in coefficients[output, state])
            lines.append(f'\t\t{{{values}}},')
        lines.append('\t},')
    lines.append('};')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n')


def interpolate_sample(samples, lengths, length):
    lower = max(i for i, value in enumerate(lengths) if value <= length)
    upper = min(i for i, value in enumerate(lengths) if value >= length)
    ratio = 0. if lower == upper else (length - lengths[lower]) / (lengths[upper] - lengths[lower])
    return {name: (1. - ratio) * samples[lower][name] + ratio * samples[upper][name]
            for name in samples[lower]}


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('unified',), default='unified', help=argparse.SUPPRESS)
    parser.add_argument('--input', type=Path, default=root / 'tools/data/leg_mass_properties0820.csv')
    parser.add_argument('--output', type=Path, default=root / 'tools/generated/unified_lqr_samples.csv')
    parser.add_argument('--cpp-output', type=Path, default=root / 'src/chassis_controller/chassis/unified_lqr_coefficients.inc')
    args = parser.parse_args()
    samples = read_leg_samples(args.input)
    lengths, gains, rows = [], [], []
    for sample in samples:
        a, b = derive_model(sample)
        rank = controllability_rank(a, b)
        if rank != STATE_SIZE:
            raise ValueError(f"uncontrollable model at {sample['leg_length_m']}: rank={rank}")
        gain = continuous_lqr(a, b)
        poles = np.linalg.eigvals(a - b @ gain)
        rows.append({'leg_length_m': sample['leg_length_m'], 'controllability_rank': rank,
                     'max_closed_loop_real': float(poles.real.max())})
        lengths.append(sample['leg_length_m'])
        gains.append(gain)
    coefficients = fit_cubic(np.array(lengths), np.stack(gains))
    dense_worst = -np.inf
    for length in np.linspace(min(lengths), max(lengths), 401):
        a, b = derive_model(interpolate_sample(samples, lengths, length))
        dense_worst = max(dense_worst, float(np.linalg.eigvals(
            a - b @ evaluate_cubic(coefficients, length)).real.max()))
    if dense_worst >= 0.:
        raise ValueError(f'unstable fitted schedule: max real pole {dense_worst}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    write_cpp(args.cpp_output, coefficients)
    print('model dimensions: A=10x10 B=10x4 K=4x10')
    print('dense maximum closed-loop real pole:', dense_worst)


if __name__ == '__main__':
    main()
