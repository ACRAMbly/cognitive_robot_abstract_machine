from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.minimal_robot import MinimalRobot
from semantic_digital_twin.robots.pr2 import PR2, PR2MobileBase
from semantic_digital_twin.robots.stretch import Stretch, StretchMobileBase
from semantic_digital_twin.robots.tiago import Tiago, TiagoMujoco
from semantic_digital_twin.world_description.connections import (
    DifferentialDrive,
    OmniDrive,
)
from semantic_digital_twin.world_description.world_entity import Body


# %% drive type resolved from the robot's mobile base


def test_omni_drive_robot_reports_omni_drive():
    assert PR2.get_drive_connection_type() is OmniDrive


def test_differential_drive_robots_report_differential_drive():
    assert Tiago.get_drive_connection_type() is DifferentialDrive
    assert TiagoMujoco.get_drive_connection_type() is DifferentialDrive
    assert Stretch.get_drive_connection_type() is DifferentialDrive


# %% robot without a mobile base


def test_robot_without_mobile_base_has_no_drive():
    assert MinimalRobot.get_drive_connection_type() is None


# %% mobile base exposes its bound drive as a property


def test_mobile_base_drive_connection_type_property_resolves_generic():
    omni_base = PR2MobileBase(root=Body(name=PrefixedName("base")))
    differential_base = StretchMobileBase(root=Body(name=PrefixedName("base")))

    assert omni_base.drive_connection_type is OmniDrive
    assert differential_base.drive_connection_type is DifferentialDrive
