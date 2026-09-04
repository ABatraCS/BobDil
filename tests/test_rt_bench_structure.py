"""What the structural report claims about a model, checked against fixed text.

These are *tests*, not validations: they run omc not at all. Every input here is
a literal captured from a real omc 1.27 run, so the parsers are held to the
format the compiler actually emits rather than to the one we remember it
emitting.
"""

from __future__ import annotations

import pytest

from rt_bench import operating_points, structure

# Captured verbatim from `checkModel(BobDil.Experiments.DilSmokePlant)`.
CHECK_MODEL_OUTPUT = """Check of BobDil.Experiments.DilSmokePlant completed successfully.
Class BobDil.Experiments.DilSmokePlant has 38 equation(s) and 38 variable(s).
8 of these are trivial equation(s)."""

# The shape of `<model>_init.xml`, trimmed to the attributes read here.
INIT_XML = """<?xml version = "1.0" encoding="UTF-8"?>
<fmiModelDescription
  fmiVersion = "1.0"
  modelName = "BobDil.Experiments.DilSmokePlant"
  numberOfEventIndicators = "6"
  numberOfTimeEvents = "2"
>
</fmiModelDescription>
"""

#: A model with one linear system, one torn non-linear system, and a state set.
INFO_JSON = {
    "format": "Transformational debugger info",
    "info": {"name": "Some.Vehicle"},
    "variables": {
        "vx": {"kind": "state"},
        "vy": {"kind": "state"},
        "$STATESET1.x[1]": {"kind": "state"},
        "alphaF": {"kind": "variable"},
    },
    "equations": [
        {"eqIndex": 0, "tag": "dummy"},
        {"eqIndex": 1, "section": "regular", "tag": "assign", "defines": ["vx"]},
        {
            "eqIndex": 2,
            "section": "regular",
            "tag": "system",
            "display": "linear",
            "equation": [11, 12, 13],
        },
        {
            "eqIndex": 3,
            "section": "regular",
            "tag": "tornsystem",
            "display": "torn non-linear",
            "equation": [21, 22, 23, 24, 25],
        },
        {
            "eqIndex": 4,
            "section": "initial",
            "tag": "system",
            "display": "non-linear",
            "equation": [31, 32],
        },
    ],
}


def test_check_model_output_gives_equation_counts():
    counts = structure.parse_check_model(CHECK_MODEL_OUTPUT)
    assert counts.equations == 38
    assert counts.variables == 38
    assert counts.trivial == 8


def test_a_failed_check_model_is_not_read_as_zero_equations():
    counts = structure.parse_check_model("Error: nothing here parses.")
    assert counts.equations is None


def test_init_xml_gives_event_counts():
    events = structure.parse_init_xml(INIT_XML)
    assert events.state_events == 6
    assert events.time_events == 2


def test_info_json_counts_states_excluding_state_set_members():
    report = structure.parse_info_json(INFO_JSON)
    # $STATESET members are states omc invented for dynamic state selection.
    # Counting them as model states overstates the model and hides the fact
    # that state selection is on at all.
    assert report.continuous_states == 2
    assert report.dynamic_state_selection is True
    assert report.state_sets == ["$STATESET1"]


def test_info_json_classifies_algebraic_systems_by_kind_and_size():
    report = structure.parse_info_json(INFO_JSON)
    # Only the two regular-section systems are real-time relevant; the initial
    # section runs once, before the first step, and cannot miss a deadline.
    assert [s.size for s in report.runtime_systems] == [3, 5]
    assert report.max_nonlinear_size == 5
    assert report.nonlinear_systems == 1
    assert report.linear_systems == 1
    assert report.initial_systems == 1


def test_torn_systems_are_reported_as_torn():
    report = structure.parse_info_json(INFO_JSON)
    torn = [s for s in report.systems if s.torn]
    assert len(torn) == 1
    assert torn[0].linear is False


def test_a_model_with_no_algebraic_loops_reports_none():
    report = structure.parse_info_json(
        {"info": {"name": "Explicit"}, "variables": {}, "equations": []}
    )
    assert report.systems == []
    assert report.max_nonlinear_size == 0
    assert report.dynamic_state_selection is False


def test_verdict_flags_the_three_things_that_break_a_deadline():
    report = structure.parse_info_json(INFO_JSON)
    report.events = structure.EventCounts(state_events=6, time_events=0)
    text = " ".join(report.concerns())
    assert "non-linear" in text
    assert "state event" in text
    assert "state selection" in text


def test_verdict_on_a_clean_model_is_empty():
    report = structure.parse_info_json(
        {"info": {"name": "Explicit"}, "variables": {"vx": {"kind": "state"}}, "equations": []}
    )
    report.events = structure.EventCounts(state_events=0, time_events=0)
    assert report.concerns() == []


class TestOperatingPointBindings:
    """The sweep must not linearize a model at a point it failed to set.

    Every one of these is cheap and would have caught a defect that instead cost
    four twenty-minute omc runs: the operating points carried the fixture's own
    state name, so `rt_bench vehicle` translated the real car correctly and then
    failed all four linearizations with `Modified element vx not found`.
    """

    def test_a_state_binding_uses_a_start_modifier(self):
        fragments = operating_points.bind_starts(
            "BobDil.Experiments.DilSmokePlant", {operating_points.SPEED: 18.0}
        )
        assert fragments == ["vx(start = 18.0)"]

    def test_a_parameter_binding_is_bound_directly(self):
        # initialVel is a parameter; `initialVel(start = ...)` is not the same
        # thing and does not set the operating point.
        fragments = operating_points.bind_starts(
            "BobLib.Experiments.Standards.VehicleFMI", {operating_points.SPEED: 18.0}
        )
        assert fragments == ["initialVel = 18.0"]

    def test_an_unknown_model_refuses_rather_than_guessing(self):
        with pytest.raises(operating_points.UnboundModel) as caught:
            operating_points.bind_starts("Some.Other.Car", {operating_points.SPEED: 12.0})
        assert "START_BINDINGS" in str(caught.value)

    def test_a_model_missing_the_role_refuses(self):
        with pytest.raises(operating_points.UnboundModel):
            operating_points.bind_starts(
                "BobDil.Experiments.DilSmokePlant", {"tyre_temperature": 80.0}
            )

    def test_a_point_with_no_starts_needs_no_binding(self):
        assert operating_points.bind_starts("Anything.At.All", {}) == []

    def test_every_default_point_binds_for_every_known_model(self):
        for model in operating_points.START_BINDINGS:
            for point in operating_points.DEFAULT:
                assert operating_points.bind_starts(model, point.starts)
