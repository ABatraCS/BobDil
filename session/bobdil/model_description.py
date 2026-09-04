"""Parse ``modelDescription.xml`` so the kernel never has to.

The real-time process must not contain an XML parser. Not because parsing is
hard, but because it happens once and it allocates, and anything that allocates
in the process holding a 1 ms deadline is a liability that has to be reasoned
about forever. So the session -- which is allowed to be slow -- resolves every
value reference here and writes a flat ``key=value`` manifest the kernel reads
in a few hundred bytes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree


class ModelDescriptionError(ValueError):
    """The FMU is not something BobDil can drive. Always fatal, never guessed past."""


@dataclass
class Variable:
    name: str
    value_reference: int
    causality: str
    variability: str
    description: str = ""

    @property
    def is_tunable(self) -> bool:
        return self.variability == "tunable"


@dataclass
class ModelDescription:
    model_name: str
    model_identifier: str
    guid: str
    generation_tool: str
    continuous_states: int
    event_indicators: int
    supports_model_exchange: bool
    supports_co_simulation: bool
    variables: dict[str, Variable] = field(default_factory=dict)

    def reference(self, name: str) -> int | None:
        variable = self.variables.get(name)
        return None if variable is None else variable.value_reference

    def require(self, names: tuple[str, ...]) -> list[str]:
        """Names the FMU does not expose. Reported, not worked around."""
        return [name for name in names if name not in self.variables]

    def tunables(self) -> dict[str, Variable]:
        return {name: v for name, v in self.variables.items() if v.is_tunable}

    def describe(self) -> str:
        return (
            f"{self.model_name}\n"
            f"  identifier         {self.model_identifier}\n"
            f"  built by           {self.generation_tool}\n"
            f"  continuous states  {self.continuous_states}\n"
            f"  event indicators   {self.event_indicators}\n"
            f"  interfaces         "
            f"{'ModelExchange ' if self.supports_model_exchange else ''}"
            f"{'CoSimulation' if self.supports_co_simulation else ''}\n"
            f"  variables          {len(self.variables)}"
        )


def parse(xml_text: str) -> ModelDescription:
    root = ElementTree.fromstring(xml_text)
    if root.tag != "fmiModelDescription":
        raise ModelDescriptionError(f"root element is <{root.tag}>, not <fmiModelDescription>")

    version = root.get("fmiVersion", "")
    if not version.startswith("2."):
        raise ModelDescriptionError(
            f"FMI {version} is not supported. BobDil binds FMI 2.0 Model Exchange only."
        )

    model_exchange = root.find("ModelExchange")
    co_simulation = root.find("CoSimulation")
    if model_exchange is None:
        # Worth an explicit message: this is the single most likely way to end
        # up with an FMU BobDil cannot use, and the reason is not obvious.
        raise ModelDescriptionError(
            "this FMU exports no ModelExchange interface.\n"
            "BobDil cannot use a Co-Simulation FMU: fmi2DoStep runs the FMU's own "
            "variable-step solver, which is unbounded work per call with no way to impose "
            'a deadline. Re-export with fmuType="me".'
        )

    variables: dict[str, Variable] = {}
    for element in root.iterfind(".//ScalarVariable"):
        name = element.get("name")
        reference = element.get("valueReference")
        if name is None or reference is None:
            continue
        variables[name] = Variable(
            name=name,
            value_reference=int(reference),
            causality=element.get("causality", "local"),
            variability=element.get("variability", "continuous"),
            description=element.get("description", ""),
        )

    derivatives = root.find("./ModelStructure/Derivatives")
    continuous_states = 0 if derivatives is None else len(list(derivatives.iterfind("Unknown")))

    return ModelDescription(
        model_name=root.get("modelName", ""),
        model_identifier=model_exchange.get("modelIdentifier", ""),
        guid=root.get("guid", ""),
        generation_tool=root.get("generationTool", ""),
        continuous_states=continuous_states,
        event_indicators=int(root.get("numberOfEventIndicators", "0")),
        supports_model_exchange=True,
        supports_co_simulation=co_simulation is not None,
        variables=variables,
    )


def parse_file(path: Path) -> ModelDescription:
    return parse(path.read_text(encoding="utf-8"))
