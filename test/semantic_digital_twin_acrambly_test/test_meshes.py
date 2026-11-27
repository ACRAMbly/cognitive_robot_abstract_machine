"""
Tests that the meshes of the ACRAMbly objects ship in the resources of the distribution.
"""

# %% Imports

from __future__ import annotations

from enum import StrEnum
from pathlib import Path

import pytest

# %% Expected meshes


class AircraftMeshFile(StrEnum):
    """
    The file of each aircraft part's surface mesh.
    """

    BOTTOM_WING = "BottomWing.stl"
    CHASSIS = "Chassis.stl"
    FRONT_WHEEL = "FrontWheel.stl"
    MOTOR_GRILL = "MotorGrill.stl"
    PROPELLER = "Propeller.stl"
    REAR_WHEEL_LEFT = "RearWheelLeft.stl"
    REAR_WHEEL_RIGHT = "RearWheelRight.stl"
    REAR_WING = "RearWing.stl"
    TOP_WING = "TopWing.stl"
    UNDER_BODY = "UnderBody.stl"
    UPPER_BODY = "UpperBody.stl"
    WINDOW = "Window.stl"


class TetrisBlockMeshFile(StrEnum):
    """
    The file of each tetris block's surface mesh.
    """

    STRAIGHT = "child_cube_0_colored.stl"
    CORNER = "child_cube_2_colored.stl"


# %% Mesh locations


@pytest.fixture
def meshes_directory() -> Path:
    """
    :return: The directory in the resources of the distribution that holds the meshes.
    """
    return (
        Path(__file__).resolve().parents[2]
        / "semantic_digital_twin_acrambly"
        / "resources"
        / "meshes"
    )


def test_aircraft_mesh_directory_holds_a_surface_mesh_per_aircraft_part(
    meshes_directory: Path,
) -> None:
    """
    Every aircraft part has its surface mesh in the aircraft mesh directory.
    """
    surface_meshes = {
        path.name for path in (meshes_directory / "aircraft").glob("*.stl")
    }

    assert surface_meshes == set(AircraftMeshFile)


def test_tetris_block_mesh_directory_holds_a_surface_mesh_per_tetris_block(
    meshes_directory: Path,
) -> None:
    """
    Every tetris block has its surface mesh in the tetris block mesh directory.
    """
    surface_meshes = {
        path.name for path in (meshes_directory / "tetris_block").glob("*.stl")
    }

    assert surface_meshes == set(TetrisBlockMeshFile)
