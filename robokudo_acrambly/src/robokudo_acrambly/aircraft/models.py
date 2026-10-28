# %% Imports

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum, StrEnum
from pathlib import Path

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.geometry import Mesh, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from robokudo_acrambly.aircraft.exceptions import MissingAircraftResource
from robokudo_acrambly.aircraft.types import (
    AircraftPart,
    BottomWing,
    Chassis,
    FrontWheel,
    MotorGrill,
    Propeller,
    RearWheelLeft,
    RearWheelRight,
    RearWing,
    TopWing,
    UnderBody,
    UpperBody,
    Window,
)

# %% Classification labels


class AircraftLabel(StrEnum):
    """
    Classification labels identifying the configured aircraft kinds.
    """

    BOTTOM_WING = "BottomWing"
    CHASSIS = "Chassis"
    FRONT_WHEEL = "FrontWheel"
    MOTOR_GRILL = "MotorGrill"
    PROPELLER = "Propeller"
    REAR_WHEEL_LEFT = "RearWheelLeft"
    REAR_WHEEL_RIGHT = "RearWheelRight"
    REAR_WING = "RearWing"
    TOP_WING = "TopWing"
    UNDER_BODY = "UnderBody"
    UPPER_BODY = "UpperBody"
    WINDOW = "Window"


# %% Model definitions


@dataclass(frozen=True)
class AircraftModelDefinition:
    """
    A semantic kind and its replaceable metric mesh resource.
    """

    label: AircraftLabel
    """
    Accepted classification label, independent of the resource filename.
    """

    semantic_type: type[AircraftPart]
    """
    Concrete annotation type for this model.
    """

    resource: Path
    """
    Mesh path relative to the configured mesh directory.
    """

    metres_per_mesh_unit: float = 1.0
    """
    Metres represented by one source coordinate unit.
    """


class AircraftModel(Enum):
    """
    The twelve configured aircraft models.
    """

    BOTTOM_WING = AircraftModelDefinition(
        AircraftLabel.BOTTOM_WING, BottomWing, Path("BottomWing.stl")
    )
    CHASSIS = AircraftModelDefinition(
        AircraftLabel.CHASSIS, Chassis, Path("Chassis.stl")
    )
    FRONT_WHEEL = AircraftModelDefinition(
        AircraftLabel.FRONT_WHEEL, FrontWheel, Path("FrontWheel.stl")
    )
    MOTOR_GRILL = AircraftModelDefinition(
        AircraftLabel.MOTOR_GRILL, MotorGrill, Path("MotorGrill.stl")
    )
    PROPELLER = AircraftModelDefinition(
        AircraftLabel.PROPELLER, Propeller, Path("Propeller.stl")
    )
    REAR_WHEEL_LEFT = AircraftModelDefinition(
        AircraftLabel.REAR_WHEEL_LEFT, RearWheelLeft, Path("RearWheelLeft.stl")
    )
    REAR_WHEEL_RIGHT = AircraftModelDefinition(
        AircraftLabel.REAR_WHEEL_RIGHT, RearWheelRight, Path("RearWheelRight.stl")
    )
    REAR_WING = AircraftModelDefinition(
        AircraftLabel.REAR_WING, RearWing, Path("RearWing.stl")
    )
    TOP_WING = AircraftModelDefinition(
        AircraftLabel.TOP_WING, TopWing, Path("TopWing.stl")
    )
    UNDER_BODY = AircraftModelDefinition(
        AircraftLabel.UNDER_BODY, UnderBody, Path("UnderBody.stl")
    )
    UPPER_BODY = AircraftModelDefinition(
        AircraftLabel.UPPER_BODY, UpperBody, Path("UpperBody.stl")
    )
    WINDOW = AircraftModelDefinition(AircraftLabel.WINDOW, Window, Path("Window.stl"))


# %% Loaded geometry


@dataclass
class AircraftGeometry:
    """
    Loaded geometry for one semantic model, reusable across independent bodies.
    """

    definition: AircraftModelDefinition
    """
    Semantic and resource configuration for the loaded mesh.
    """

    geometry: Mesh
    """
    Unattached template with its metric mesh already loaded.
    """

    def create_body(self, body_name: PrefixedName) -> Body:
        """
        Create independent visual and collision shapes at the source mesh origin.
        """
        return Body(
            name=body_name,
            visual=ShapeCollection([deepcopy(self.geometry)]),
            collision=ShapeCollection([deepcopy(self.geometry)]),
        )


@dataclass
class AircraftModelCatalog:
    """
    Load the configured aircraft resources once during construction.
    """

    mesh_directory: Path
    """
    Directory against which configured resources are resolved.
    """

    definitions: tuple[AircraftModelDefinition, ...] = field(
        default_factory=lambda: tuple(model.value for model in AircraftModel)
    )
    """
    Models to load, including any resource replacements.
    """

    models: dict[AircraftLabel, AircraftGeometry] = field(init=False)
    """
    Loaded model geometry indexed by accepted classification label.
    """

    def __post_init__(self) -> None:
        """
        Prepare all geometry before the catalog becomes available to callers.
        """
        for definition in self.definitions:
            mesh_path = self.mesh_directory / definition.resource
            if not mesh_path.is_file():
                raise MissingAircraftResource(mesh_path)
        self.models = {}
        for definition in self.definitions:
            mesh_path = self.mesh_directory / definition.resource
            geometry = Mesh(
                filename=str(mesh_path),
                scale=Scale(*([definition.metres_per_mesh_unit] * 3)),
            )
            geometry.mesh
            self.models[definition.label] = AircraftGeometry(definition, geometry)
