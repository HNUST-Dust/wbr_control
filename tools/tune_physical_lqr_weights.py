#!/usr/bin/env python3
## @file tune_physical_lqr_weights.py
#  @brief 通过受约束时域仿真搜索物理模型的 LQR 权重。
#  @details 该工具在主机侧运行，用于构建、采集、辨识或参数生成；不会编译进目标固件。生成参数写回固件前应按对应文档完成单位和符号约定检查。

"""Search diagonal LQR weights against constrained time-domain simulations.

This is a candidate generator, not an automatic firmware writer.  It uses the
same physical model and state/input conventions as
``derive_physical_lqr_schedule.py``:

    x = [theta, theta_rate, body_position, body_speed, pitch, pitch_rate]
    u = [left/right total wheel torque, left/right total leg posture torque]

Every candidate is evaluated at the short, middle, and long CAD leg lengths.
The simulation includes the control period, configurable command delay,
first-order actuator lag, and total generalized-torque saturation.  Search is
performed in log space while R_leg is fixed, because multiplying all Q and R
entries by one common factor does not change the LQR gain.

The physical model is only the first stage of the workflow.  Before trusting
the ranking on hardware, identify/validate A and B with CSV data containing:

    timestamp, leg length,
    theta, theta_rate, body_position, body_speed, pitch, pitch_rate,
    total wheel torque actually sent, total leg posture torque actually sent.

Legacy ``lqr_sysid_csv`` logs consumed by ``tools/fit_lqr_from_sysid.py`` use
this state/input convention.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from derive_physical_lqr_schedule import (
    RobotParameters,
    continuous_lqr,
    derive_continuous_model,
    read_leg_samples,
)


STATE_SIZE = 6
INPUT_SIZE = 2
STATE_NAMES = (
    "theta", "theta_rate", "x", "x_rate", "pitch", "pitch_rate")
SEARCH_NAMES = (*("q_" + name for name in STATE_NAMES), "r_wheel")


@dataclass(frozen=True)
class SimulationSettings:
    dt_s: float
    duration_s: float
    delay_steps: int
    wheel_time_constant_s: float
    leg_time_constant_s: float
    state_scales: np.ndarray
    input_limits: np.ndarray


@dataclass(frozen=True)
class CandidateResult:
    score: float
    q_diag: np.ndarray
    r_diag: np.ndarray
    worst_leg_length_m: float
    worst_scenario: str
    peak_normalized_state: float
    terminal_normalized_state: float
    wheel_saturation_fraction: float
    leg_saturation_fraction: float
    max_closed_loop_real: float


def parse_positive_vector(text: str, size: int, name: str) -> np.ndarray:
    values = np.array(
        [float(item.strip()) for item in text.split(",") if item.strip()],
        dtype=float,
    )
    if values.shape != (size,):
        raise ValueError(f"{name} must contain {size} comma-separated values")
    if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
        raise ValueError(f"{name} values must be positive and finite")
    return values


def select_model_samples(samples: list[dict[str, float]]) -> list[dict[str, float]]:
    """Use the extrema and the point nearest the schedule center."""
    if len(samples) <= 3:
        return samples
    center = min(samples, key=lambda sample: abs(sample["leg_length_m"] - 0.25))
    selected = [samples[0], center, samples[-1]]
    return sorted(
        {sample["leg_length_m"]: sample for sample in selected}.values(),
        key=lambda sample: sample["leg_length_m"],
    )


def initial_conditions(state_scales: np.ndarray) -> list[tuple[str, np.ndarray]]:
    """Small-signal disturbances used consistently for every candidate."""
    scenarios: list[tuple[str, np.ndarray]] = []
    fractions = (0.60, 0.45, 0.35, 0.45, 0.60, 0.45)
    for index, name in enumerate(STATE_NAMES):
        state = np.zeros(STATE_SIZE)
        state[index] = fractions[index] * state_scales[index]
        scenarios.append((name, state))

    coupled = np.zeros(STATE_SIZE)
    coupled[0] = 0.45 * state_scales[0]
    coupled[4] = -0.45 * state_scales[4]
    scenarios.append(("theta_pitch_opposed", coupled))
    return scenarios


def rk4_step(
    state: np.ndarray,
    control: np.ndarray,
    a: np.ndarray,
    b: np.ndarray,
    dt_s: float,
) -> np.ndarray:
    def derivative(value: np.ndarray) -> np.ndarray:
        return a @ value + b @ control

    k1 = derivative(state)
    k2 = derivative(state + 0.5 * dt_s * k1)
    k3 = derivative(state + 0.5 * dt_s * k2)
    k4 = derivative(state + dt_s * k3)
    return state + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_case(
    a: np.ndarray,
    b: np.ndarray,
    gain: np.ndarray,
    initial_state: np.ndarray,
    settings: SimulationSettings,
) -> tuple[float, float, float, float, float]:
    state = initial_state.copy()
    applied = np.zeros(INPUT_SIZE)
    delay_line = [np.zeros(INPUT_SIZE) for _ in range(settings.delay_steps + 1)]
    actuator_tau = np.array([
        settings.wheel_time_constant_s,
        settings.leg_time_constant_s,
    ])
    actuator_alpha = np.where(
        actuator_tau > 0.0,
        np.minimum(1.0, settings.dt_s / np.maximum(actuator_tau, settings.dt_s)),
        1.0,
    )

    steps = max(1, int(round(settings.duration_s / settings.dt_s)))
    normalized_energy = 0.0
    peak_normalized_state = 0.0
    wheel_saturated_steps = 0
    leg_saturated_steps = 0

    for _ in range(steps):
        requested = -gain @ state
        wheel_saturated_steps += int(abs(requested[0]) > settings.input_limits[0])
        leg_saturated_steps += int(abs(requested[1]) > settings.input_limits[1])
        limited = np.clip(requested, -settings.input_limits, settings.input_limits)
        delay_line.append(limited)
        delayed = delay_line.pop(0)
        applied += actuator_alpha * (delayed - applied)

        state = rk4_step(state, applied, a, b, settings.dt_s)
        normalized = state / settings.state_scales
        normalized_squared = float(normalized @ normalized)
        normalized_energy += normalized_squared * settings.dt_s
        peak_normalized_state = max(
            peak_normalized_state, float(np.max(np.abs(normalized))))

        if not np.all(np.isfinite(state)) or peak_normalized_state > 25.0:
            return 1.0e12, peak_normalized_state, peak_normalized_state, 1.0, 1.0

    terminal_normalized_state = float(
        np.linalg.norm(state / settings.state_scales))
    wheel_fraction = wheel_saturated_steps / steps
    leg_fraction = leg_saturated_steps / steps

    # The hard peak term rejects candidates that trade a short dangerous
    # excursion for a low average.  Saturation duty is deliberately expensive
    # because the linear LQR assumptions no longer hold while saturated.
    score = (
        normalized_energy / settings.duration_s
        + 2.0 * peak_normalized_state * peak_normalized_state
        + 8.0 * terminal_normalized_state * terminal_normalized_state
        + 30.0 * wheel_fraction
        + 20.0 * leg_fraction
    )
    return (
        score,
        peak_normalized_state,
        terminal_normalized_state,
        wheel_fraction,
        leg_fraction,
    )


def evaluate_candidate(
    q_diag: np.ndarray,
    r_diag: np.ndarray,
    models: list[tuple[float, np.ndarray, np.ndarray]],
    settings: SimulationSettings,
) -> CandidateResult:
    worst_score = -math.inf
    worst_leg_length = 0.0
    worst_scenario = ""
    worst_metrics = (0.0, 0.0, 0.0, 0.0)
    maximum_closed_loop_real = -math.inf

    for leg_length, a, b in models:
        try:
            gain = continuous_lqr(a, b, q_diag, r_diag)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            return CandidateResult(
                1.0e12, q_diag, r_diag, leg_length, "CARE failure",
                math.inf, math.inf, 1.0, 1.0, math.inf)

        eigenvalues = np.linalg.eigvals(a - b @ gain)
        maximum_closed_loop_real = max(
            maximum_closed_loop_real, float(np.max(eigenvalues.real)))
        if maximum_closed_loop_real >= 0.0:
            return CandidateResult(
                1.0e12, q_diag, r_diag, leg_length, "unstable ideal model",
                math.inf, math.inf, 1.0, 1.0, maximum_closed_loop_real)

        for scenario_name, initial_state in initial_conditions(settings.state_scales):
            metrics = simulate_case(a, b, gain, initial_state, settings)
            if metrics[0] > worst_score:
                worst_score = metrics[0]
                worst_leg_length = leg_length
                worst_scenario = scenario_name
                worst_metrics = metrics[1:]

    return CandidateResult(
        worst_score,
        q_diag,
        r_diag,
        worst_leg_length,
        worst_scenario,
        worst_metrics[0],
        worst_metrics[1],
        worst_metrics[2],
        worst_metrics[3],
        maximum_closed_loop_real,
    )


def vector_to_weights(vector: np.ndarray, fixed_r_leg: float) -> tuple[np.ndarray, np.ndarray]:
    values = np.power(10.0, vector)
    return values[:STATE_SIZE], np.array([values[STATE_SIZE], fixed_r_leg])


def result_row(rank: int, result: CandidateResult) -> dict[str, object]:
    row: dict[str, object] = {
        "rank": rank,
        "score": result.score,
        "worst_leg_length_m": result.worst_leg_length_m,
        "worst_scenario": result.worst_scenario,
        "peak_normalized_state": result.peak_normalized_state,
        "terminal_normalized_state": result.terminal_normalized_state,
        "wheel_saturation_fraction": result.wheel_saturation_fraction,
        "leg_saturation_fraction": result.leg_saturation_fraction,
        "max_closed_loop_real": result.max_closed_loop_real,
        "r_wheel": result.r_diag[0],
        "r_leg": result.r_diag[1],
    }
    for name, value in zip(STATE_NAMES, result.q_diag):
        row[f"q_{name}"] = value
    return row


def write_results(path: Path, results: list[CandidateResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [result_row(rank, result) for rank, result in enumerate(results, 1)]
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    tool_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Search diagonal physical-model LQR weights in log space")
    parser.add_argument(
        "--input", type=Path,
        default=tool_dir / "data" / "leg_mass_properties.csv")
    parser.add_argument(
        "--q-center", default="4000,1,1500,1,20000,1",
        help="Center Q diagonal: theta,theta_rate,x,x_rate,pitch,pitch_rate")
    parser.add_argument(
        "--r-center", default="90,1",
        help="Center R diagonal; R_leg is fixed during search")
    parser.add_argument(
        "--state-scales", default="0.0872665,1,0.15,0.5,0.0872665,1",
        help="Allowable state magnitudes in SI units")
    parser.add_argument(
        "--input-limits", default="9.5,20",
        help="Total generalized wheel,leg torque limits in N.m")
    parser.add_argument("--dt-ms", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=1.5)
    parser.add_argument("--delay-ms", type=float, default=4.0)
    parser.add_argument("--wheel-time-constant-ms", type=float, default=12.0)
    parser.add_argument("--leg-time-constant-ms", type=float, default=8.0)
    parser.add_argument("--candidates", type=int, default=96)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--spread-decades", type=float, default=0.7)
    parser.add_argument("--seed", type=int, default=20260722)
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument(
        "--all-leg-lengths", action="store_true",
        help="Evaluate all CAD leg-length samples instead of min/center/max")
    parser.add_argument(
        "--output", type=Path,
        default=tool_dir / "generated" / "lqr_weight_search.csv")
    args = parser.parse_args()

    if args.dt_ms <= 0.0 or args.duration_s <= 0.0:
        raise ValueError("dt and duration must be positive")
    if args.candidates < 1 or args.rounds < 1 or args.top < 1:
        raise ValueError("candidates, rounds, and top must be positive")

    q_center = parse_positive_vector(args.q_center, STATE_SIZE, "q-center")
    r_center = parse_positive_vector(args.r_center, INPUT_SIZE, "r-center")
    state_scales = parse_positive_vector(
        args.state_scales, STATE_SIZE, "state-scales")
    input_limits = parse_positive_vector(
        args.input_limits, INPUT_SIZE, "input-limits")
    dt_s = args.dt_ms / 1000.0
    settings = SimulationSettings(
        dt_s=dt_s,
        duration_s=args.duration_s,
        delay_steps=max(0, int(round(args.delay_ms / args.dt_ms))),
        wheel_time_constant_s=args.wheel_time_constant_ms / 1000.0,
        leg_time_constant_s=args.leg_time_constant_ms / 1000.0,
        state_scales=state_scales,
        input_limits=input_limits,
    )

    parameters = RobotParameters()
    all_samples = read_leg_samples(args.input)
    selected_samples = (
        all_samples if args.all_leg_lengths else select_model_samples(all_samples))
    models: list[tuple[float, np.ndarray, np.ndarray]] = []
    for sample in selected_samples:
        a, b = derive_continuous_model(
            parameters,
            sample["leg_length_m"],
            sample["leg_com_from_wheel_m"],
            sample["single_leg_inertia_kg_m2"],
        )
        models.append((sample["leg_length_m"], a, b))

    center = np.log10(np.concatenate((q_center, [r_center[0]])))
    fixed_r_leg = float(r_center[1])
    rng = np.random.default_rng(args.seed)
    results: list[CandidateResult] = []
    best_vector = center.copy()
    spread = args.spread_decades

    for round_index in range(args.rounds):
        vectors = [best_vector.copy()]
        for _ in range(args.candidates - 1):
            vectors.append(best_vector + rng.normal(0.0, spread, best_vector.shape))

        round_results: list[tuple[CandidateResult, np.ndarray]] = []
        for vector in vectors:
            q_diag, r_diag = vector_to_weights(vector, fixed_r_leg)
            result = evaluate_candidate(q_diag, r_diag, models, settings)
            round_results.append((result, vector))
        round_results.sort(key=lambda item: item[0].score)
        results.extend(result for result, _ in round_results)
        best_vector = round_results[0][1].copy()
        spread *= 0.45
        print(
            f"round={round_index + 1} best_score={round_results[0][0].score:.8g} "
            f"q={round_results[0][0].q_diag.tolist()} "
            f"r={round_results[0][0].r_diag.tolist()}")

    results.sort(key=lambda result: result.score)
    unique_results: list[CandidateResult] = []
    seen: set[tuple[float, ...]] = set()
    for result in results:
        key = tuple(np.round(np.log10(np.concatenate((result.q_diag, result.r_diag))), 10))
        if key in seen:
            continue
        seen.add(key)
        unique_results.append(result)
    write_results(args.output, unique_results)

    best = unique_results[0]
    print("\nbest candidate")
    print("Q = np.array([" + ", ".join(f"{value:.9g}" for value in best.q_diag) + "])")
    print("R = np.array([" + ", ".join(f"{value:.9g}" for value in best.r_diag) + "])")
    print(
        f"score={best.score:.9g} worst_leg_length_m={best.worst_leg_length_m:.5f} "
        f"worst_scenario={best.worst_scenario}")
    print(
        f"peak_normalized_state={best.peak_normalized_state:.6g} "
        f"terminal_normalized_state={best.terminal_normalized_state:.6g} "
        f"wheel_saturation_fraction={best.wheel_saturation_fraction:.6g} "
        f"leg_saturation_fraction={best.leg_saturation_fraction:.6g}")
    print(f"results={args.output}")
    print(f"reported_top_candidates={min(args.top, len(unique_results))}")
    for rank, result in enumerate(unique_results[:args.top], 1):
        print(
            f"{rank}: score={result.score:.6g} "
            f"Q={np.round(result.q_diag, 6).tolist()} "
            f"R={np.round(result.r_diag, 6).tolist()}")


if __name__ == "__main__":
    main()
