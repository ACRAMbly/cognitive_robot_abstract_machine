import math
import os
from pathlib import Path
from typing import Literal

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.specifications.connections import (
    ScrewConnectionSpecification,
)
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
    Vector3,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.geometry import Box, Color, Mesh, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../assets")


def select_arm(graspable: HasGraspCandidates, robot: Tracy) -> Arm:
    """
    The arm that reaches a body from the side it stands on.

    :param graspable: The annotation of the body to reach.
    :param robot: The robot whose arms are chosen from.
    :return: The left arm for a body on the left, the right arm otherwise.
    """
    body_y = float(graspable.root.global_pose.position.to_np().reshape(-1)[1])
    return robot.left_arm if body_y > 0 else robot.right_arm


def spawn_body(
    spawn_world: World,
    position: tuple[float, float, float] = (0.0, 0.0, 1.5),
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    shape_type: Literal["mesh", "box"] = "box",
    *,
    mesh_filename: str | None = None,
    name: str = "box",
    scale: Scale = Scale(0.05, 0.05, 0.05),
    color: Color = Color(1.0, 1.0, 0.0, 1.0),
) -> HasGraspCandidates:
    """Spawn a graspable free-floating body via the Semantic Digital Twin API.

    Args:
        spawn_world: The world to spawn the body into.
        position: (x, y, z) position of the body.
        rotation: (yaw, pitch, roll) rotation of the body.
        shape_type: ``"box"`` for a coloured box shape, ``"mesh"`` for an STL mesh file.
        mesh_filename: Required when *shape_type* is ``"mesh"``. Name of the STL file
            inside the assets directory.
        name: Required when *shape_type* is ``"box"``. Base name for the body.
        scale: Box dimensions (``"box"`` only).
        color: Box colour (``"box"`` only).

    Returns:
        The annotation that makes the spawned body graspable.
    """
    with spawn_world.modify_world():
        if shape_type == "mesh":
            if mesh_filename is None:
                raise ValueError("mesh_filename is required for shape_type='mesh'")
            mesh_path = Path(_ASSETS_DIR, mesh_filename)
            if not mesh_path.exists():
                raise FileNotFoundError(f"Mesh file not found: {mesh_path}")
            mesh_shape = Mesh(origin=HomogeneousTransformationMatrix(), filename=str(mesh_path))
            collision_shapes = ShapeCollection([mesh_shape])
            visual_shapes = collision_shapes
            body_name = PrefixedName(prefix="objects", name=mesh_path.stem)
        else:
            collision_shapes = ShapeCollection(shapes=[Box(scale=scale)])
            visual_shapes = ShapeCollection(shapes=[Box(scale=scale, color=color)])
            body_name = PrefixedName(name)

        body = Body(
            name=body_name,
            collision=collision_shapes,
            visual=visual_shapes,
        )
        spawn_world.add_kinematic_structure_entity(body)
        spawn_world.add_connection(
            Connection6DoF.create_with_dofs(
                parent=spawn_world.root,
                child=body,
                world=spawn_world,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=position[0], y=position[1], z=position[2],
                    yaw=rotation[0], pitch=rotation[1], roll=rotation[2],
                ),
            )
        )
        graspable = HasGraspCandidates(root=body)
        spawn_world.add_semantic_annotations([graspable])

    return graspable


def spawn_screw_body(
    spawn_world: World,
    position: tuple[float, float, float] = (0.0, 0.0, 1.0),
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    shape_type: Literal["mesh", "box"] = "mesh",
    *,
    parent: Body | None = None,
    mesh_filename: str | None = None,
    name: str | None = None,
    scale: Scale | None = None,
    color: Color = Color(1.0, 1.0, 0.0, 1.0),
    screw_pitch: float = 0.002,
    turns: float = 1.0,
    axis: Vector3 | None = None,
    maximum_speed: float = 1.0,
    dof_limits: DegreeOfFreedomLimits | None = None,
) -> HasGraspCandidates:
    """Spawn a graspable body that is threaded onto a single-DoF screw joint.

    The counterpart of :func:`spawn_body`: the body is attached to *parent* -- a plate,
    a nut, a table -- through a
    :class:`~semantic_digital_twin.world_description.connections.ScrewConnection`
    instead of a free-floating ``Connection6DoF``. Driving the joint's only degree of
    freedom then rotates and translates the body at once, so a planner can screw it in by
    turning a joint rather than by following a precomputed helix.

    Sign convention
    ---------------
    A :class:`~semantic_digital_twin.world_description.connections.ScrewConnection`
    rotates the child counterclockwise about *axis* (right-hand rule) and translates it
    by ``axis * screw_pitch * angle / (2 * pi)`` as the degree of freedom grows. This
    function therefore expects *axis* to point **from the parent towards the screw**, so
    that driving the body *in* -- towards the plate -- is a **negative** rotation: the
    limits are ``[-2*pi*turns, 0]`` and ``Close`` (which drives towards the lower limit)
    tightens the screw while ``Open`` backs it out again. Only right-handed threads are
    supported: the derived limits and the ``Close`` direction assume a positive
    *screw_pitch*, which this function enforces.

    :param spawn_world: The world to spawn the body into.
    :param position: (x, y, z) position of the joint frame in *parent*'s frame, in
        metres.
    :param rotation: (yaw, pitch, roll) rotation of the joint frame in *parent*'s frame,
        in radians.
    :param shape_type: ``"mesh"`` for an STL mesh file, ``"box"`` for a coloured box.
    :param parent: The body the screw is threaded into, normally the plate or nut. The
        world root if ``None``.
    :param mesh_filename: Required when *shape_type* is ``"mesh"``. Name of the STL file
        inside the assets directory. Its units are taken as metres; pass *scale* to
        resize a mesh authored in other units.
    :param name: Name of the spawned body. Defaults to the mesh file's stem for meshes
        and to ``"screw"`` for boxes.
    :param scale: ``Mesh`` scale (``"mesh"``) or box dimensions (``"box"``). ``None``
        keeps a mesh at its file's size, i.e. the STL must be metric.
    :param color: Box colour (``"box"`` only).
    :param screw_pitch: Metres the body advances along *axis* per full revolution.
    :param turns: Revolutions the joint must be able to turn *inwards*. Used to derive
        *dof_limits* when those are not given.
    :param axis: Unit vector of the thread axis, pointing from the parent towards the
        screw. Defaults to ``Vector3(0, 0, 1)``.
    :param maximum_speed: Velocity limit of the joint in rad/s, used to derive
        *dof_limits* when those are not given.
    :param dof_limits: Explicit limits, overriding the ones derived from *turns*. A
        :class:`~giskardpy.motion_statechart.goals.open_close.Close` goal clamps against
        ``dof.limits.lower.position`` and crashes without limits, so this function never
        leaves them unset.
    :return: The annotation that makes the spawned body graspable. The joint is
        ``annotation.root.parent_connection``, i.e. the body's single-DoF connection.
    :raises ValueError: If *screw_pitch* is not positive, or *turns* is negative while
        *dof_limits* is ``None``.
    :raises FileNotFoundError: If *mesh_filename* does not exist in the assets directory.
    """
    if screw_pitch <= 0.0:
        raise ValueError(f"screw_pitch must be positive, got {screw_pitch}")
    if axis is None:
        axis = Vector3(0.0, 0.0, 1.0)

    if dof_limits is None:
        if turns < 0.0:
            raise ValueError(f"turns must not be negative, got {turns}")
        # Inwards is a negative rotation about an axis that points from the parent to
        # the screw, see the sign convention above.
        dof_limits = DegreeOfFreedomLimits.from_position_range_and_speed(
            lower_position=-2.0 * math.pi * turns,
            upper_position=0.0,
            maximum_speed=maximum_speed,
        )

    with spawn_world.modify_world():
        if shape_type == "mesh":
            if mesh_filename is None:
                raise ValueError("mesh_filename is required for shape_type='mesh'")
            mesh_path = Path(_ASSETS_DIR, mesh_filename)
            if not mesh_path.exists():
                raise FileNotFoundError(f"Mesh file not found: {mesh_path}")
            mesh_shape = Mesh(
                origin=HomogeneousTransformationMatrix(),
                filename=str(mesh_path),
                # ``Mesh`` carries its own scale; ``None`` keeps the file's units, which
                # are metres for the assets used here.
                **({} if scale is None else {"scale": scale}),
            )
            collision_shapes = ShapeCollection([mesh_shape])
            visual_shapes = collision_shapes
            body_name = PrefixedName(prefix="objects", name=name or mesh_path.stem)
        else:
            box_scale = scale or Scale(0.02, 0.02, 0.05)
            collision_shapes = ShapeCollection(shapes=[Box(scale=box_scale)])
            visual_shapes = ShapeCollection(shapes=[Box(scale=box_scale, color=color)])
            body_name = PrefixedName(name or "screw")

        body = Body(
            name=body_name,
            collision=collision_shapes,
            visual=visual_shapes,
        )
        spawn_world.add_kinematic_structure_entity(body)

        # The body and its connection have to enter the world in the same modification
        # block: a body without a parent connection would briefly be a second root, and
        # the world asserts that it has exactly one. ``connect`` opens its own block,
        # which nests into this one instead of publishing in between.
        parent_body = spawn_world.root if parent is None else parent
        ScrewConnectionSpecification(
            axis=axis,
            screw_pitch=screw_pitch,
            dof_limits=dof_limits,
        ).connect(
            spawn_world,
            child=body,
            parent=parent_body,
            parent_T_connection=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=position[0],
                y=position[1],
                z=position[2],
                yaw=rotation[0],
                pitch=rotation[1],
                roll=rotation[2],
            ),
        )

        graspable = HasGraspCandidates(root=body)
        spawn_world.add_semantic_annotations([graspable])

    return graspable
