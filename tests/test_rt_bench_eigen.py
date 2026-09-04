"""The eigenvalue sweep: parsing omc's linear model, and the step bound from it.

The step bound is the number Phase 0 exists to produce, so it is tested against
cases whose answer is known in closed form rather than against a golden file.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from rt_bench import eigen

# Captured verbatim from `linearize(...)` on the fixture plant at a skidpad
# operating point. Trimmed to what is parsed.
LINEARIZED_MODEL = """model linearized_model "BobDil_Experiments_OpSkidpad"
  parameter Integer n = 5 "number of states";
  parameter Integer m = 0 "number of inputs";
  parameter Integer p = 0 "number of outputs";

  parameter Real x0[n] = {0.0249, 0.0008, 24.98, -1.397, 0.746};
  parameter Real u0[m] = zeros(0);

  parameter Real A[n, n] =
\t[0, 1, 0, 0, 0;
\t-1771.428571499298, -102.8571428569721, 0.2309319759896158, -6.672895614238093, 3.133276255423929;
\t0, 0, -0.1220970414831377, 0.7462464736450517, -1.397647991936023;
\t0, 0, -0.6500248352960011, -2.780373158309418, -23.67595951828965;
\t0, 0, 0.5493306270655086, 3.48141848004567, -4.418231165926513];

  parameter Real B[n, m] = zeros(n, m);

  parameter Real C[p, n] = zeros(p, n);

  parameter Real D[p, m] = zeros(p, m);

  Real x[n](start=x0);
  input Real u[m];
  output Real y[p];

  Real 'x_plant.rollAngle' = x[1];
  Real 'x_plant.rollRate' = x[2];
  Real 'x_plant.vx' = x[3];
  Real 'x_plant.vy' = x[4];
  Real 'x_plant.yawRate' = x[5];
equation
  der(x) = A * x + B * u;
  y = C * x + D * u;
end linearized_model;
"""


def test_parsing_recovers_the_state_matrix_and_its_state_names():
    model = eigen.parse_linearized_model(LINEARIZED_MODEL)
    assert model.order == 5
    assert model.a.shape == (5, 5)
    assert model.a[0][1] == pytest.approx(1.0)
    assert model.a[1][0] == pytest.approx(-1771.428571499298)
    # The 'x_plant.' prefix is omc's, not the model's, and keeping it makes
    # every state name in the report unreadable.
    assert model.state_names == ["rollAngle", "rollRate", "vx", "vy", "yawRate"]


def test_a_truncated_linearization_is_an_error_not_a_partial_matrix():
    with pytest.raises(eigen.LinearizationError):
        eigen.parse_linearized_model("model linearized_model\nend linearized_model;\n")


@pytest.mark.parametrize(
    ("method", "limit"),
    [("euler", 2.0), ("midpoint", 2.0), ("rk4", 2.7852935634)],
)
def test_real_axis_stability_limit_matches_the_known_value(method, limit):
    # A single real eigenvalue at -1 puts the limit directly in units of h.
    bound = eigen.max_stable_step(np.array([-1.0 + 0j]), method)
    assert bound == pytest.approx(limit, rel=1e-4)


def test_the_bound_scales_inversely_with_the_fastest_eigenvalue():
    slow = eigen.max_stable_step(np.array([-1.0 + 0j]), "rk4")
    fast = eigen.max_stable_step(np.array([-1.0 + 0j, -100.0 + 0j]), "rk4")
    assert fast == pytest.approx(slow / 100.0, rel=1e-4)


def test_rk4_admits_a_purely_imaginary_pair_that_euler_does_not():
    pair = np.array([2.0j, -2.0j])
    assert eigen.max_stable_step(pair, "rk4") == pytest.approx(2.8284271 / 2.0, rel=1e-3)
    # Forward Euler is unstable on the imaginary axis for any positive step, so
    # there is no step that works, not merely a small one.
    assert eigen.max_stable_step(pair, "euler") == 0.0


def test_a_purely_algebraic_model_has_no_step_bound():
    assert eigen.max_stable_step(np.array([]), "rk4") == math.inf


def test_the_binding_mode_names_the_state_that_sets_the_step():
    model = eigen.parse_linearized_model(LINEARIZED_MODEL)
    sweep = eigen.analyse(model, "rk4")
    # The roll mode is two orders of magnitude faster than the body modes here,
    # so it is what sets the step, and saying so is the useful half of the
    # answer -- "0.03 s" alone does not tell anyone what to change.
    assert sweep.binding_states[0] in {"rollAngle", "rollRate"}
    assert sweep.max_step_s == pytest.approx(2.7852935634 / abs(sweep.binding_eigenvalue), rel=0.2)
    assert sweep.stiffness_ratio > 10


def test_an_unstable_plant_is_reported_rather_than_bounded():
    # A positive real part is a divergent model. No step size fixes that, and
    # silently returning a small one would hide a modelling error behind a
    # timing number.
    model = eigen.LinearModel(
        name="unstable", a=np.array([[1.0]]), state_names=["x"], operating_point={}
    )
    sweep = eigen.analyse(model, "rk4")
    assert sweep.unstable_states == ["x"]
