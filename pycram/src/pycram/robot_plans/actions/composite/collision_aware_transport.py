from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from scipy.spatial.transform import Rotation as R

import numpy as np
import yaml

from semantic_digital_twin.world import World as SemanticWorld
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix as tm,
)
from typing_extensions import Union, Optional, Any, Iterable, List, Dict

from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction
from ....datastructures.enums import (
    Arms,
    VerticalAlignment,
    ApproachDirection,
    AxisIdentifier,
)
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....view_manager import ViewManager

import logging

logger = logging.getLogger(__name__)


def loginfo(msg):
    logger.info(msg)
    print(f"[INFO] {msg}")


def logwarn(msg):
    logger.warning(msg)
    print(f"[WARN] {msg}")


# GRASP CLASSIFIER

class GraspClassifier:
    """Classifies and manages grasps from YAML file with grasp scoring."""

    def __init__(self, grasp_data: Dict, robot=None):
        self.grasp_data = grasp_data
        self.robot = robot
        self.classified_grasps = self._classify_grasps()

    def _classify_grasps(self) -> Dict[ApproachDirection, List[Dict]]:
        classified = {direction: [] for direction in ApproachDirection}
        for grasp in self.grasp_data["grasps"]:
            pose = self._create_pose(grasp)
            approach_dir = self._determine_approach_direction(pose)
            vertical_align = self._determine_vertical_alignment(pose)
            grasp_info = {
                "id": grasp["id"],
                "pose": pose,
                "approach_direction": approach_dir,
                "vertical_alignment": vertical_align,
                "score": 1.0,
            }
            classified[approach_dir].append(grasp_info)
        return classified

    def _create_pose(self, grasp_data: Dict) -> PoseStamped:
        position = [
            grasp_data["position"]["x"],
            grasp_data["position"]["y"],
            grasp_data["position"]["z"],
        ]
        orientation = [
            grasp_data["orientation"]["x"],
            grasp_data["orientation"]["y"],
            grasp_data["orientation"]["z"],
            grasp_data["orientation"]["w"],
        ]
        return PoseStamped.from_list(position, orientation)

    def _determine_approach_direction(self, pose: PoseStamped) -> ApproachDirection:
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        r = R.from_quat(quat)
        approach_vector = r.apply([0, 0, -1])
        abs_approach_xy = np.abs(approach_vector[:2])
        if abs_approach_xy[0] > abs_approach_xy[1]:
            return (
                ApproachDirection.LEFT
                if approach_vector[0] < 0
                else ApproachDirection.RIGHT
            )
        else:
            return (
                ApproachDirection.BACK
                if approach_vector[1] < 0
                else ApproachDirection.FRONT
            )

    def _determine_vertical_alignment(
        self, pose: PoseStamped
    ) -> Optional[VerticalAlignment]:
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        r = R.from_quat(quat)
        up_vector = r.apply([0, 0, 1])
        if up_vector[2] > 0.5:
            return VerticalAlignment.TOP
        elif up_vector[2] <= -0.5:
            return VerticalAlignment.BOTTOM
        return None

    def _calculate_distance_score(
        self, grasp: Dict, robot_pos: np.ndarray, obj_pos: np.ndarray
    ) -> float:
        grasp_pose = grasp["pose"]
        grasp_world_pos = np.array(
            [
                obj_pos[0] + grasp_pose.position.x,
                obj_pos[1] + grasp_pose.position.y,
                obj_pos[2] + grasp_pose.position.z,
            ]
        )
        distance = np.linalg.norm(robot_pos - grasp_world_pos)
        return 1.0 / (1.0 + distance)

    def _calculate_orientation_score(self, grasp: Dict) -> float:
        grasp_pose = grasp["pose"]
        quat = [
            grasp_pose.orientation.x,
            grasp_pose.orientation.y,
            grasp_pose.orientation.z,
            grasp_pose.orientation.w,
        ]
        tilt_penalty = abs(quat[0]) + abs(quat[1])
        return 1.0 / (1.0 + tilt_penalty)

    def _calculate_combined_score(
        self,
        grasp: Dict,
        robot_pos: np.ndarray,
        obj_pos: np.ndarray,
        distance_weight: float = 0.6,
        orientation_weight: float = 0.4,
    ) -> float:
        distance_score = self._calculate_distance_score(grasp, robot_pos, obj_pos)
        orientation_score = self._calculate_orientation_score(grasp)
        return (distance_weight * distance_score) + (
            orientation_weight * orientation_score
        )

    def update_scores(
        self,
        robot_pos: np.ndarray,
        obj_pos: np.ndarray,
        distance_weight: float = 0.6,
        orientation_weight: float = 0.4,
    ) -> None:
        robot_pos = np.array(robot_pos)
        obj_pos = np.array(obj_pos)
        loginfo(f"Updating grasp scores: robot_pos={robot_pos}, obj_pos={obj_pos}")
        for direction, grasps in self.classified_grasps.items():
            for grasp in grasps:
                grasp["score"] = self._calculate_combined_score(
                    grasp, robot_pos, obj_pos, distance_weight, orientation_weight
                )

    def get_best_grasp_for_direction(
        self,
        approach_direction: ApproachDirection,
        vertical_alignment: Optional[VerticalAlignment] = None,
    ) -> Optional[Dict]:
        grasps = self.classified_grasps.get(approach_direction, [])
        if vertical_alignment:
            grasps = [
                g for g in grasps if g["vertical_alignment"] == vertical_alignment
            ]
        if not grasps:
            return None
        return sorted(grasps, key=lambda x: x["score"], reverse=True)[0]

    def get_top_n_grasps_for_direction(
        self,
        approach_direction: ApproachDirection,
        n: int = 5,
        vertical_alignment: Optional[VerticalAlignment] = None,
    ) -> List[Dict]:
        grasps = self.classified_grasps.get(approach_direction, [])
        if vertical_alignment:
            grasps = [
                g for g in grasps if g["vertical_alignment"] == vertical_alignment
            ]
        return sorted(grasps, key=lambda x: x["score"], reverse=True)[:n]

    def get_all_grasps_sorted(self) -> List[Dict]:
        all_grasps = []
        for grasps in self.classified_grasps.values():
            all_grasps.extend(grasps)
        all_grasps.sort(key=lambda x: x["score"], reverse=True)
        return all_grasps

    def get_grasp_by_id(self, grasp_id: int) -> Optional[Dict]:
        for grasps in self.classified_grasps.values():
            for grasp in grasps:
                if grasp["id"] == grasp_id:
                    return grasp
        return None

    def get_grasp_count_by_direction(self) -> Dict[str, int]:
        return {d.name: len(g) for d, g in self.classified_grasps.items()}

    def print_score_summary(self) -> None:
        print("----GRASP SCORE SUMMARY----")
        for direction in ApproachDirection:
            grasps = self.classified_grasps.get(direction, [])
            if grasps:
                scores = [g["score"] for g in grasps]
                print(
                    f"\n{direction.name}: Count={len(grasps)}, "
                    f"Range={min(scores):.4f}-{max(scores):.4f}, "
                    f"Best ID={max(grasps, key=lambda x: x['score'])['id']}"
                )


# HELPER FUNCTIONS


def load_grasp_data(yaml_path: str) -> Dict:
    """Load grasp data from YAML file."""
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


def _get_body_position(body: Body, world: SemanticWorld, root) -> np.ndarray:
    """Get the world position of a Body in the semantic world."""
    for conn in world.connections:
        if conn.child == body:
            origin = conn.origin
            if hasattr(origin, "translation"):
                t = origin.translation
                return np.array([t[0], t[1], t[2]])
            elif hasattr(origin, "xyz"):
                return np.array(origin.xyz)
            else:
                try:
                    mat = np.array(origin)
                    return mat[:3, 3]
                except Exception:
                    pass
    try:
        transform = world.transform(tm(reference_frame=body), target_frame=root)
        if hasattr(transform, "translation"):
            return np.array(transform.translation[:3])
        return np.array(transform)[:3, 3]
    except Exception:
        pass
    logwarn(f"Could not get position of '{body.name}', returning origin")
    return np.array([0.0, 0.0, 0.0])


def _get_robot_position(robot_view, world: SemanticWorld, root) -> np.ndarray:
    """Get the robot base position from the semantic world."""
    try:
        transform = world.transform(
            tm(reference_frame=robot_view.root), target_frame=root
        )
        if hasattr(transform, "translation"):
            return np.array(transform.translation[:3])
        return np.array(transform)[:3, 3]
    except Exception:
        pass
    return np.array([0.0, 0.0, 0.0])


def check_position_occupied(
    target_pos_xyz: np.ndarray,
    world: SemanticWorld,
    root,
    exclude_bodies: List[Body] = None,
    tolerance: float = 0.08,
) -> Optional[Body]:
    """Check if a target position is occupied in the semantic world."""
    if exclude_bodies is None:
        exclude_bodies = []
    exclude_names = set(str(b.name) for b in exclude_bodies)
    exclude_names.add(str(root.name))
    for conn in world.connections:
        child = conn.child
        if str(child.name) in exclude_names:
            continue
        if not hasattr(child, "visual") or child.visual is None:
            continue
        body_pos = _get_body_position(child, world, root)
        if np.linalg.norm(target_pos_xyz - body_pos) < tolerance:
            return child
    return None


def find_free_position(
    original_target_xyz: np.ndarray,
    world: SemanticWorld,
    root,
    exclude_bodies: List[Body] = None,
    offset: float = 0.05,
    max_attempts: int = 8,
    tolerance: float = 0.08,
) -> np.ndarray:
    """Find a free position near the original target."""
    if (
        check_position_occupied(
            original_target_xyz, world, root, exclude_bodies, tolerance
        )
        is None
    ):
        loginfo("Target position is free.")
        return original_target_xyz

    logwarn("Target occupied. Finding alternative...")
    directions = [
        (1, 0, 0),
        (-1, 0, 0),
        (0, 1, 0),
        (0, -1, 0),
        (1, 1, 0),
        (-1, 1, 0),
        (1, -1, 0),
        (-1, -1, 0),
    ]
    directions = [(np.array(d) / np.linalg.norm(d)).tolist() for d in directions]

    for d in directions[:max_attempts]:
        new_pos = original_target_xyz + np.array(d) * offset
        if (
            check_position_occupied(new_pos, world, root, exclude_bodies, tolerance)
            is None
        ):
            loginfo(
                f"Found free position at offset "
                f"({d[0] * offset:.2f}, {d[1] * offset:.2f})"
            )
            return new_pos

    for mult in [2, 3, 4]:
        for d in directions[:4]:
            new_pos = original_target_xyz + np.array(d) * offset * mult
            if (
                check_position_occupied(
                    new_pos, world, root, exclude_bodies, tolerance
                )
                is None
            ):
                return new_pos

    logwarn("No free position found. Using original.")
    return original_target_xyz


# ARM SELECTION


def _choose_best_arm(robot_view, obj_body: Body, world: SemanticWorld, root) -> Arms:
    """Intelligently choose the closest available arm based on tip-to-object distance."""
    obj_pos = _get_body_position(obj_body, world, root)
    try:
        left_fk = world.compute_forward_kinematics(root, robot_view.left_arm.tip)
        if hasattr(left_fk, "translation"):
            left_pos = np.array(left_fk.translation[:3])
        else:
            left_pos = np.array(left_fk)[:3, 3]

        right_fk = world.compute_forward_kinematics(root, robot_view.right_arm.tip)
        if hasattr(right_fk, "translation"):
            right_pos = np.array(right_fk.translation[:3])
        else:
            right_pos = np.array(right_fk)[:3, 3]

        left_dist = np.linalg.norm(left_pos - obj_pos)
        right_dist = np.linalg.norm(right_pos - obj_pos)

        loginfo(f"Arm distances - Left: {left_dist:.3f}, Right: {right_dist:.3f}")

        if left_dist <= right_dist:
            return Arms.LEFT
        else:
            return Arms.RIGHT

    except Exception as e:
        loginfo(f"Arm distance calc failed ({e}), defaulting to LEFT")
        return Arms.LEFT


# GRASP SELECTION

# Mapping from axis + sign to approach/vertical
_AXIS_DIRECTION_MAP = {
    (AxisIdentifier.X, 1): ApproachDirection.RIGHT,
    (AxisIdentifier.X, -1): ApproachDirection.LEFT,
    (AxisIdentifier.Y, 1): ApproachDirection.FRONT,
    (AxisIdentifier.Y, -1): ApproachDirection.BACK,
    (AxisIdentifier.Z, 1): VerticalAlignment.TOP,
    (AxisIdentifier.Z, -1): VerticalAlignment.BOTTOM,
}


def _calculate_closest_faces(vec_obj_frame: List[float]) -> tuple:
    """
    Calculate the two closest faces from a
    direction vector in the object frame. Returns a tuple of
    (ApproachDirection or VerticalAlignment).
    """
    all_axes = [AxisIdentifier.X, AxisIdentifier.Y, AxisIdentifier.Z]
    vec_list = vec_obj_frame

    sorted_axes = sorted(
        all_axes,
        key=lambda axis: abs(vec_list[list(axis.value).index(1)]),
        reverse=True,
    )

    primary_axis = sorted_axes[0]
    primary_idx = list(primary_axis.value).index(1)
    primary_sign = int(np.sign(vec_list[primary_idx]))
    primary_face = _AXIS_DIRECTION_MAP.get(
        (primary_axis, primary_sign), ApproachDirection.FRONT
    )

    if len(sorted_axes) > 1:
        secondary_axis = sorted_axes[1]
        secondary_idx = list(secondary_axis.value).index(1)
        secondary_sign = int(np.sign(vec_list[secondary_idx]))
    else:
        secondary_axis = primary_axis
        secondary_sign = -primary_sign

    secondary_face = _AXIS_DIRECTION_MAP.get(
        (secondary_axis, secondary_sign), VerticalAlignment.TOP
    )

    return primary_face, secondary_face


def _choose_grasp_auto(
    robot_pos: np.ndarray, obj_pos: np.ndarray, manipulator=None
) -> GraspDescription:
    """Automatically calculate grasp based on robot-object geometry."""
    vec_world = robot_pos - obj_pos
    vec_local = vec_world.tolist()

    primary, secondary = _calculate_closest_faces(vec_local)

    final_approach = ApproachDirection.FRONT
    final_vertical = VerticalAlignment.TOP

    if isinstance(primary, VerticalAlignment):
        final_vertical = primary
        if isinstance(secondary, ApproachDirection):
            final_approach = secondary
    elif isinstance(primary, ApproachDirection):
        final_approach = primary
        if isinstance(secondary, VerticalAlignment):
            final_vertical = secondary

    loginfo(
        f"Auto-Grasp (geometry): Approach={final_approach.name}, "
        f"Vertical={final_vertical.name}"
    )
    return GraspDescription(
        approach_direction=final_approach,
        vertical_alignment=final_vertical,
        manipulator=manipulator,
        rotate_gripper=False,
    )


def _choose_grasp_from_classifier(
    grasp_classifier: GraspClassifier,
    robot_pos: np.ndarray,
    obj_pos: np.ndarray,
    manipulator=None,
    distance_weight: float = 0.6,
    orientation_weight: float = 0.4,
) -> GraspDescription:
    """Use GraspClassifier to select the best grasp based on scoring."""
    grasp_classifier.update_scores(
        robot_pos, obj_pos, distance_weight, orientation_weight
    )

    # Calculate ideal approach direction based on geometry
    vec_world = robot_pos - obj_pos
    vec_local = vec_world.tolist()

    primary, secondary = _calculate_closest_faces(vec_local)

    # Determine ideal approach direction
    if isinstance(primary, ApproachDirection):
        ideal_approach = primary
    elif isinstance(secondary, ApproachDirection):
        ideal_approach = secondary
    else:
        ideal_approach = ApproachDirection.FRONT

    # Determine ideal vertical alignment
    if isinstance(primary, VerticalAlignment):
        ideal_vertical = primary
    elif isinstance(secondary, VerticalAlignment):
        ideal_vertical = secondary
    else:
        ideal_vertical = VerticalAlignment.TOP

    loginfo(
        f"Ideal grasp direction: Approach={ideal_approach.name}, "
        f"Vertical={ideal_vertical.name if ideal_vertical else 'ANY'}"
    )

    # Log top 5 grasps for the ideal direction
    top_5 = grasp_classifier.get_top_n_grasps_for_direction(ideal_approach, n=5)
    if top_5:
        loginfo(f"Top 5 grasps for {ideal_approach.name} direction:")
        for i, g in enumerate(top_5):
            v_align = (
                g["vertical_alignment"].name if g["vertical_alignment"] else "NONE"
            )
            loginfo(
                f"  {i + 1}. ID={g['id']}, Score={g['score']:.4f}, "
                f"Vertical={v_align}"
            )
    else:
        loginfo(f"No grasps found for {ideal_approach.name} direction")

    # Try to get best grasp matching ideal direction + vertical
    grasp_info = grasp_classifier.get_best_grasp_for_direction(
        ideal_approach, ideal_vertical
    )

    if grasp_info is None:
        # Try without vertical alignment constraint
        grasp_info = grasp_classifier.get_best_grasp_for_direction(ideal_approach)

    if grasp_info is None:
        # Fallback: get best grasp from any direction (highest score)
        all_grasps = grasp_classifier.get_all_grasps_sorted()
        if all_grasps:
            grasp_info = all_grasps[0]

    if grasp_info:
        approach = grasp_info["approach_direction"]
        vertical = grasp_info["vertical_alignment"] or VerticalAlignment.TOP

        loginfo(
            f"GraspClassifier selected grasp ID={grasp_info['id']}: "
            f"Approach={approach.name}, Vertical={vertical.name}, "
            f"Score={grasp_info['score']:.4f}"
        )
        return GraspDescription(
            approach_direction=approach,
            vertical_alignment=vertical,
            manipulator=manipulator,
            rotate_gripper=False,
        )
    else:
        logwarn("GraspClassifier found no grasps, falling back to auto-grasp")
        return _choose_grasp_auto(robot_pos, obj_pos, manipulator)


# ACTION CLASS

@dataclass
class CollisionAwareTransportAction(ActionDescription):
    """
    Transport an object by automatically:
    1. Checking if target position is occupied
    2. Finding a free position if occupied (offset by 5cm)
    3. Choosing the closest arm (Left vs Right)
    4. Using GraspClassifier with distance & orientation scoring to select the best grasp
    5. Executing park -> pick -> park -> place -> park
    """

    object_designator: Body
    """
    Object designator describing the object that should be transported.
    """
    target_location: PoseStamped
    """
    Pose in the world at which the object should be placed.
    """
    arm: Arms
    """
    Arm to use for transport. Pass None for auto-selection.
    """
    grasp_classifier: Optional[GraspClassifier] = None
    """
    Optional GraspClassifier instance for selecting grasps from YAML data.
    """
    grasp_description: Optional[GraspDescription] = None
    """
    Manual grasp description (used if grasp_classifier is not provided).
    """
    placement_offset: float = 0.05
    """
    Offset distance (in meters) to use when target is occupied. Default: 5cm.
    """
    collision_tolerance: float = 0.08
    """
    Distance threshold to consider positions as 'occupied'. Default: 8cm.
    """
    distance_weight: float = 0.6
    """
    Weight for distance scoring (0-1). Default: 0.6.
    """
    orientation_weight: float = 0.4
    """
    Weight for orientation scoring (0-1). Default: 0.4.
    """
    _pre_perform_callbacks = []

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        # Access world and robot from context (set by SequentialPlan)
        world = self.world
        robot_view = self.robot_view
        root = world.root

        target_xyz = np.array(
            [
                self.target_location.position.x,
                self.target_location.position.y,
                self.target_location.position.z,
            ]
        )
        loginfo(
            f"=== CollisionAwareTransport: '{self.object_designator.name}' ==="
        )

        # Step 1: Collision detection and position adjustment
        actual_target_xyz = find_free_position(
            target_xyz,
            world,
            root,
            [self.object_designator],
            self.placement_offset,
            tolerance=self.collision_tolerance,
        )

        if not np.array_equal(actual_target_xyz, target_xyz):
            logwarn(
                f"Adjusted to ({actual_target_xyz[0]:.3f}, "
                f"{actual_target_xyz[1]:.3f}, {actual_target_xyz[2]:.3f})"
            )

        # Step 2: Auto-select arm based on proximity if not specified
        if self.arm:
            chosen_arm = self.arm
        else:
            chosen_arm = _choose_best_arm(
                robot_view, self.object_designator, world, root
            )

        loginfo(f"Arm: {chosen_arm.name}")

        # Get the manipulator for the chosen arm
        arm_view = ViewManager.get_arm_view(chosen_arm, robot_view)
        manipulator = arm_view.manipulator

        # Step 3: Grasp selection
        if self.grasp_description:
            chosen_grasp = self.grasp_description
        elif self.grasp_classifier:
            # Use arm tip position for grasp scoring (not robot root)
            arm_fk = world.compute_forward_kinematics(root, arm_view.tip)
            if hasattr(arm_fk, "translation"):
                rp = np.array(arm_fk.translation[:3])
            else:
                rp = np.array(arm_fk)[:3, 3]
            op = _get_body_position(self.object_designator, world, root)
            chosen_grasp = _choose_grasp_from_classifier(
                self.grasp_classifier,
                rp,
                op,
                manipulator,
                self.distance_weight,
                self.orientation_weight,
            )
        else:
            arm_fk = world.compute_forward_kinematics(root, arm_view.tip)
            if hasattr(arm_fk, "translation"):
                rp = np.array(arm_fk.translation[:3])
            else:
                rp = np.array(arm_fk)[:3, 3]
            op = _get_body_position(self.object_designator, world, root)
            chosen_grasp = _choose_grasp_auto(rp, op, manipulator)
        # Step 4: Build target pose
        actual_target_pose = PoseStamped.from_list(
            frame=root, position=actual_target_xyz.tolist()
        )

        # Step 5: Execute transport sequence
        SequentialPlan(
            self.context,
            ParkArmsAction.description([Arms.BOTH]),
            PickUpAction.description(
                object_designator=self.object_designator,
                grasp_description=chosen_grasp,
                arm=chosen_arm,
            ),
            ParkArmsAction.description([Arms.BOTH]),
            PlaceAction.description(
                object_designator=self.object_designator,
                target_location=actual_target_pose,
                arm=chosen_arm,
            ),
            ParkArmsAction.description([Arms.BOTH]),
        ).perform()

        loginfo(f"=== Done: '{self.object_designator.name}' ===")

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        pass

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms] = None,
        grasp_classifier: Optional[GraspClassifier] = None,
        grasp_description: Optional[GraspDescription] = None,
        placement_offset: float = 0.05,
        collision_tolerance: float = 0.08,
        distance_weight: float = 0.6,
        orientation_weight: float = 0.4,
    ) -> PartialDesignator[CollisionAwareTransportAction]:
        return PartialDesignator[CollisionAwareTransportAction](
            CollisionAwareTransportAction,
            object_designator=object_designator,
            target_location=target_location,
            arm=arm,
            grasp_classifier=grasp_classifier,
            grasp_description=grasp_description,
            placement_offset=placement_offset,
            collision_tolerance=collision_tolerance,
            distance_weight=distance_weight,
            orientation_weight=orientation_weight,
        )


CollisionAwareTransportActionDescription = CollisionAwareTransportAction.description