"""Stub for legacy grasp module — full GraspDescription API was removed."""
from __future__ import annotations


class GraspDescription:
    def grasp_pose_sequence(self, *args, **kwargs):
        return None, None, None

    def pose_sequence(self, *args, **kwargs):
        return None, None, None

    def grasp_target_pose(self, *args, **kwargs):
        return None

    @property
    def gripper_position(self):
        return 0.0


class PreferredGraspAlignment:
    pass


class GraspPose:
    pass
