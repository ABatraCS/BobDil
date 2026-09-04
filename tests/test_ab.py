"""Paired and blind A/B: the arithmetic, and the refusals that make it honest.

Paired A/B's whole value is that the driver's lap-to-lap variance has been
removed from the comparison. That is only true if the two runs really did see
identical inputs, so most of what is tested here is the machinery that refuses
to report a difference it cannot attribute.
"""

from __future__ import annotations

import pytest

from bobdil import ab, recording
from bobdil.generated.frames import VehicleState


def run(
    states: list[VehicleState], *, step_dt=1e-3, kernel_id=1, vehicle_hash=0
) -> recording.Recording:
    return recording.Recording(
        meta=recording.RecordingMeta(
            step_dt=step_dt,
            kernel_id=kernel_id,
            evaluations_per_step=4,
            vehicle_hash=vehicle_hash,
            build_hash=0,
        ),
        frames=states,
    )


def ramp(count: int, *, steer=0.1, acc_y=0.0, yaw_vel=0.0) -> list[VehicleState]:
    return [
        VehicleState(
            step_index=step,
            sim_time=step * 1e-3,
            steering_angle_command=steer,
            acc_y=acc_y * step,
            yaw_vel=yaw_vel * step,
        )
        for step in range(1, count + 1)
    ]


def test_two_identical_runs_report_no_difference():
    result = ab.compare(run(ramp(100)), run(ramp(100)), label_a="A", label_b="B")
    assert result.identical
    assert result.worst is None


def test_a_difference_is_attributed_to_the_signal_and_the_moment():
    a = run(ramp(100, acc_y=0.0))
    b = run(ramp(100, acc_y=0.01))
    result = ab.compare(a, b, label_a="A", label_b="B")
    assert not result.identical
    assert result.worst.signal == "acc_y"
    # The ramp diverges monotonically, so the worst moment is the last frame.
    assert result.worst.at_time_s == pytest.approx(0.100)
    assert result.worst.max_abs == pytest.approx(1.0)


def test_signals_are_ranked_so_the_biggest_effect_is_read_first():
    a = run(ramp(50))
    b = run(ramp(50, acc_y=0.001, yaw_vel=0.1))
    ranked = [difference.signal for difference in ab.compare(a, b, label_a="A", label_b="B").ranked]
    assert ranked[0] == "yaw_vel"
    assert "acc_y" in ranked


def test_runs_with_different_driver_inputs_are_refused():
    # Not a small caveat in the report: a comparison of two runs the driver drove
    # differently is measuring the driver, which is exactly the noise paired A/B
    # exists to remove.
    a = run(ramp(50, steer=0.1))
    b = run(ramp(50, steer=0.2))
    with pytest.raises(ab.NotComparable, match="driver input"):
        ab.compare(a, b, label_a="A", label_b="B")


def test_runs_of_different_vehicles_are_refused():
    with pytest.raises(ab.NotComparable, match="vehicle"):
        ab.compare(
            run(ramp(10), vehicle_hash=1),
            run(ramp(10), vehicle_hash=2),
            label_a="A",
            label_b="B",
        )


def test_runs_from_different_kernels_are_refused():
    with pytest.raises(ab.NotComparable, match="kernel"):
        ab.compare(run(ramp(10), kernel_id=1), run(ramp(10), kernel_id=3), label_a="A", label_b="B")


def test_runs_of_different_lengths_are_refused():
    with pytest.raises(ab.NotComparable, match="length"):
        ab.compare(run(ramp(10)), run(ramp(11)), label_a="A", label_b="B")


def test_machine_dependent_fields_are_never_compared():
    # step_time_us differs between any two runs on any machine. Comparing it
    # would make every A/B report a difference, and the real one would be lost.
    a = run(ramp(20))
    b = run(ramp(20))
    for index, state in enumerate(b.frames):
        state.step_time_us = float(index)
        state.rtf = 0.9
        state.host_time_ns = 12345
    assert ab.compare(a, b, label_a="A", label_b="B").identical


# --- blind A/B ------------------------------------------------------------


def test_a_blind_trial_sequence_is_balanced_and_reproducible():
    first = ab.blind_sequence(8, seed=7)
    assert first == ab.blind_sequence(8, seed=7)
    # Balanced: an unbalanced sequence lets a driver score above chance by
    # guessing the more frequent setup every time.
    assert first.count("A") == first.count("B") == 4


def test_an_odd_trial_count_is_refused_because_it_cannot_be_balanced():
    with pytest.raises(ValueError, match="even"):
        ab.blind_sequence(7, seed=1)


def test_scoring_reports_hits_and_whether_they_beat_chance():
    truth = ["A", "B", "A", "B", "A", "B", "A", "B"]
    score = ab.score_blind(truth, truth)
    assert score.hits == 8
    assert score.trials == 8
    assert score.p_value < 0.01
    assert score.above_chance


def test_chance_level_guessing_is_reported_as_not_a_result():
    truth = ["A", "B"] * 8
    guesses = ["A"] * 16
    score = ab.score_blind(truth, guesses)
    assert score.hits == 8
    assert not score.above_chance
    assert "cannot" in score.verdict().lower()


def test_a_guess_count_that_does_not_match_the_trials_is_an_error():
    with pytest.raises(ValueError, match="8 trials"):
        ab.score_blind(["A"] * 8, ["A"] * 7)
