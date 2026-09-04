"""Phase 0 -- the viability gate (architecture.md 6).

Nothing in BobLib or BobSim measures whether the vehicle model can be stepped
in real time, and every schedule estimate below Phase 0 is unfounded until it
is measured. This package answers the three questions that decide it, for any
Modelica model, before a single line of the loop is trusted:

* **Structure** (:mod:`rt_bench.structure`) -- how much work is *inside* a step
  that a fixed-step integrator cannot bound: non-linear algebraic systems, state
  events, and runtime state selection.
* **Stability** (:mod:`rt_bench.eigen`) -- the largest explicit step that is
  numerically stable at representative operating points, and which state sets
  it. A model can be far too fast to time out and still be unusable at 1 kHz.
* **Timing** -- measured by the kernel itself (``bobdil-kernel bench``), because
  the only honest measurement of the step cost is the code that will run it.
  :mod:`rt_bench.report` joins that measurement to the two above.

The three are separate on purpose: they fail for different reasons and are
fixed by different changes. A large non-linear system is a modelling problem, a
small stable step is a physics problem, and a slow step is an implementation
problem.
"""

from . import eigen, operating_points, structure

#: ``report`` is deliberately not imported here. It is the only module that
#: shells out to omc, and it imports the session layer to find one; keeping it
#: off the package import path is what lets the parsers above be exercised on a
#: machine that has neither.
__all__ = ["eigen", "operating_points", "structure"]
