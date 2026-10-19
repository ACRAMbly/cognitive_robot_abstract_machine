import math

import numpy as np
import pytest

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import ReferenceFrameMismatchError
from semantic_digital_twin.spatial_types import (
    Line3,
    OrthogonalProjection,
    Point2,
    Point3,
    Pose,
    Vector3,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

# %% fixtures


@pytest.fixture
def rotated_mini_world(mini_world: World) -> World:
    """
    :func:`mini_world` with its tip turned a quarter turn about the root's z axis.
    """
    mini_world.connections[0].position = math.pi / 2
    return mini_world


# %% project


class TestProject:
    def test_drops_height_in_the_plane_frame(self):
        frame = Body(name=PrefixedName("world"))
        point = Point3(1.5, -2.5, 3.0, reference_frame=frame)

        projected = OrthogonalProjection(frame).project(point)

        assert isinstance(projected, Point2)
        assert projected.reference_frame is frame
        np.testing.assert_allclose(projected.to_np(), [1.5, -2.5])

    def test_transforms_a_point_of_another_frame_into_the_plane_frame(
        self, rotated_mini_world: World
    ):
        root, tip = rotated_mini_world.root, rotated_mini_world.bodies[1]
        tip_P_point = Point3(1, 0, 2, reference_frame=tip)
        root_P_point = rotated_mini_world.transform(tip_P_point, root)

        projected = OrthogonalProjection(root).project(tip_P_point)

        assert projected.reference_frame is root
        np.testing.assert_allclose(
            projected.to_np(), root_P_point.to_np()[:2], atol=1e-12
        )

    def test_pose_position_projects_like_a_point(self):
        frame = Body(name=PrefixedName("world"))
        pose = Pose.from_xyz_rpy(
            x=1.5, y=-2.5, z=3.0, roll=0.1, pitch=0.2, yaw=0.3, reference_frame=frame
        )

        projected = OrthogonalProjection(frame).project(pose.position)

        np.testing.assert_allclose(projected.to_np(), [1.5, -2.5])


# %% embed


class TestEmbed:
    def test_default_height_is_the_plane(self):
        frame = Body(name=PrefixedName("world"))
        point = Point2(1.0, 2.0, reference_frame=frame)

        embedded = OrthogonalProjection(frame).embed(point)

        assert isinstance(embedded, Point3)
        assert embedded.reference_frame is frame
        np.testing.assert_allclose(embedded.to_np()[:3], [1.0, 2.0, 0.0])

    def test_explicit_height(self):
        point = Point2(1.0, 2.0)

        embedded = OrthogonalProjection(None).embed(point, height=5.0)

        np.testing.assert_allclose(embedded.to_np()[:3], [1.0, 2.0, 5.0])

    def test_is_a_right_inverse_of_project(self):
        frame = Body(name=PrefixedName("world"))
        point = Point2(1.0, 2.0, reference_frame=frame)
        projection = OrthogonalProjection(frame)

        round_trip = projection.project(projection.embed(point, height=4.0))

        np.testing.assert_allclose(round_trip.to_np(), point.to_np())

    def test_point_on_another_plane_is_rejected(self):
        frame = Body(name=PrefixedName("world"))
        other_frame = Body(name=PrefixedName("other"))
        point = Point2(1.0, 2.0, reference_frame=other_frame)

        with pytest.raises(ReferenceFrameMismatchError):
            OrthogonalProjection(frame).embed(point)


# %% preimage


class TestPreimage:
    def test_is_the_vertical_line_through_the_embedded_point(self):
        frame = Body(name=PrefixedName("world"))
        point = Point2(1.0, 2.0, reference_frame=frame)
        projection = OrthogonalProjection(frame)

        preimage = projection.preimage(point)

        assert isinstance(preimage, Line3)
        np.testing.assert_allclose(
            preimage.point.to_np(), projection.embed(point).to_np()
        )
        np.testing.assert_allclose(
            preimage.direction.to_np(), Vector3.Z(reference_frame=frame).to_np()
        )
        assert preimage.direction.reference_frame is frame

    def test_every_point_of_the_preimage_projects_onto_the_point(self):
        frame = Body(name=PrefixedName("world"))
        point = Point2(1.0, 2.0, reference_frame=frame)
        projection = OrthogonalProjection(frame)
        preimage = projection.preimage(point)

        point_on_line = preimage.point + preimage.direction * 7.0

        np.testing.assert_allclose(
            projection.project(point_on_line).to_np(), point.to_np()
        )
