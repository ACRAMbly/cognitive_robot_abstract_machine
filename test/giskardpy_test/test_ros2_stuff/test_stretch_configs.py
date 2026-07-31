from giskardpy.middleware.ros2.scripts.iai_robots.stretch.configs import (
    StretchStandaloneInterface,
)
from semantic_digital_twin.world_description.connections import DifferentialDrive

# %% controlled joints


def test_controlled_joints_resolve_the_drive_from_the_world(stretch_world_copy):
    """
    The base drive is looked up in the world instead of being named by a literal, so the
    interface controls the base whatever the drive connection ended up being called.
    """
    drive = stretch_world_copy.get_connections_by_type(DifferentialDrive)[0]
    interface = StretchStandaloneInterface()

    controlled_joint_names = interface.controlled_joint_names(stretch_world_copy)

    assert controlled_joint_names[0] == drive.name
    assert controlled_joint_names[1:] == interface.joint_names


def test_every_controlled_joint_exists_in_the_world(stretch_world_copy):
    """
    Registering a joint the world does not know about fails at setup time, so every
    declared name must resolve to a connection.
    """
    interface = StretchStandaloneInterface()

    for joint_name in interface.controlled_joint_names(stretch_world_copy):
        assert stretch_world_copy.get_connection_by_name(joint_name) is not None
