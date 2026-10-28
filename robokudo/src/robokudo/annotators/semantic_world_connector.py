# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from timeit import default_timer

import numpy as np
from py_trees.common import Status
from scipy.optimize import linear_sum_assignment
from rclpy.time import Time as ROSTime
from typing_extensions import TYPE_CHECKING

from robokudo import world
from robokudo.annotators.core import BaseAnnotator, ThreadedAnnotator
from robokudo.annotators.semantic_observations import (
    SemanticObservationAcceptance,
    ObservationPose,
)
from robokudo.cas import CASViews
from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    PoseAnnotation,
    StampedPoseAnnotation,
)
from robokudo.types.cv import ImageROI
from robokudo.types.scene import ObjectHypothesis
from robokudo.types.belief_creation import BeliefCreations
from robokudo.utils.annotator_helper import (
    draw_bounding_boxes_from_object_hypotheses,
    transform_pose_from_camera_to_world,
)
from robokudo.utils.hypothesis_comparators import ObjectHypothesisComparator
from robokudo.world import world_instance

if TYPE_CHECKING:
    from typing_extensions import Mapping

    from robokudo.cas import CAS
    from robokudo.types.belief_state import ObjectBeliefState
    from robokudo.types.belief_creation import ModelGeometry
    from semantic_digital_twin.semantic_annotations.mixins import HasRootBody

# %% Belief association


@dataclass(init=False, eq=False)
class SemanticDigitalTwinConnector(ThreadedAnnotator):
    """
    An annotator that synchronizes the current state of the world with the semdt.
    """

    @dataclass(init=False)
    class Descriptor(BaseAnnotator.Descriptor):
        """
        Association thresholds and optional semantic acceptance configuration.
        """

        @dataclass
        class Parameters(BaseAnnotator.Descriptor.Parameters):
            """
            Control association confidence and configured semantic kinds.
            """

            confidence_threshold: float = 0.05
            """
            Minimum similarity required to update an existing belief.
            """

            semantic_types: dict[str, type[HasRootBody]] | None = None
            """
            Accepted model labels and annotation types, or generic acceptance.
            """

            model_geometry: Mapping[str, ModelGeometry] | None = None
            """
            Application model geometry indexed by accepted classification label.
            """

        parameters: Parameters = field(default_factory=Parameters)
        """
        Parameters owned by this connector descriptor.
        """

    descriptor: Descriptor = field(init=False)
    """
    Semantic acceptance and association configuration for this connector.
    """

    def __init__(
        self,
        name: str = "SemanticDigitalTwinSynchronization",
        descriptor: SemanticDigitalTwinConnector.Descriptor | None = None,
    ) -> None:
        """
        Configure observation acceptance and association for the local belief world.
        """
        super().__init__(name, descriptor)
        self.rk_logger.debug("%s.__init__()" % self.__class__.__name__)

        self.object_comparator = (
            ObjectHypothesisComparator(self.get_cas)
            .with_comparator_for(ImageROI, weight=0.2)
            .with_comparator_for(StampedPoseAnnotation, weight=0.4)
            .with_comparator_for(BoundingBox3DAnnotation, weight=0.4)
        )

    def compute(self) -> Status:
        """
        Synchronise the current RoboKudo state with the current semdt state.
        """
        start_timer = default_timer()

        cas = self.get_cas()
        object_hypotheses: list[ObjectHypothesis] = cas.filter_annotations_by_type(
            ObjectHypothesis
        )
        object_beliefs = list(world.get_object_belief_states().values())

        associated_hypotheses = self.associate_hypotheses_with_beliefs(
            object_hypotheses, object_beliefs, cas
        )
        self.create_association_visualization(associated_hypotheses)
        self.log_world_state()

        end_timer = default_timer()
        self.feedback_message = f"Processing took {(end_timer - start_timer):.4f}s"
        return Status.SUCCESS

    def associate_hypotheses_with_beliefs(
        self,
        object_hypotheses: list[ObjectHypothesis],
        object_beliefs: list[ObjectBeliefState],
        cas: CAS,
    ) -> list[tuple[ObjectHypothesis, ObjectBeliefState]]:
        """
        Associate current object hypotheses with existing or new object beliefs.
        """
        if len(object_hypotheses) == 0:
            return []

        # Association uses the newest pose annotation from each hypothesis.
        self.add_world_pose_annotations(object_hypotheses, cas)
        belief_creations = BeliefCreations.prepare(
            object_hypotheses,
            cas,
            self.descriptor.parameters.semantic_types,
            self.descriptor.parameters.model_geometry,
            self.get_class_name(),
        )

        if len(object_beliefs) == 0:
            return self.create_new_beliefs_for_hypotheses(
                object_hypotheses, cas, belief_creations
            )

        cost_matrix = self.create_association_cost_matrix(
            object_hypotheses, object_beliefs
        )
        hypothesis_indices, belief_indices = linear_sum_assignment(cost_matrix)

        associated_hypotheses = self.apply_assignment(
            object_hypotheses,
            object_beliefs,
            cost_matrix,
            hypothesis_indices,
            belief_indices,
            cas,
            belief_creations,
        )
        associated_hypotheses.extend(
            self.create_new_beliefs_for_unmatched_hypotheses(
                object_hypotheses, set(hypothesis_indices), cas, belief_creations
            )
        )
        return associated_hypotheses

    def add_world_pose_annotations(
        self, object_hypotheses: list[ObjectHypothesis], cas: CAS
    ) -> None:
        """
        Add world-frame stamped poses to hypotheses when a camera pose is available.
        """
        if self.descriptor.parameters.semantic_types is not None:
            acceptance = SemanticObservationAcceptance(
                self.descriptor.parameters.semantic_types
            )
            prepared_observations = [
                acceptance.prepare(object_hypothesis, cas, self.get_class_name())
                for object_hypothesis in object_hypotheses
            ]
            for observation in prepared_observations:
                if not any(
                    annotation is observation.world_pose
                    for annotation in observation.hypothesis.annotations
                ):
                    observation.hypothesis.annotations.append(observation.world_pose)
            return
        if cas.camera_to_world_transform is None:
            return

        for object_hypothesis in object_hypotheses:
            pose_annotation = self.get_latest_pose_annotation(object_hypothesis)
            if pose_annotation is None:
                continue
            if (
                isinstance(pose_annotation, StampedPoseAnnotation)
                and pose_annotation.frame == cas.world_frame
            ):
                continue

            pose_in_world = transform_pose_from_camera_to_world(
                cas,
                PoseAnnotation(
                    translation=pose_annotation.translation,
                    rotation=pose_annotation.rotation,
                ),
            )
            stamped_pose = StampedPoseAnnotation()
            stamped_pose.source = self.get_class_name()
            stamped_pose.translation = pose_in_world.translation
            stamped_pose.rotation = pose_in_world.rotation
            if cas.world_frame is not None:
                stamped_pose.frame = cas.world_frame
            if isinstance(pose_annotation, StampedPoseAnnotation):
                stamped_pose.timestamp = pose_annotation.timestamp
            elif cas.data_timestamp is not None:
                stamped_pose.timestamp = ROSTime(
                    nanoseconds=cas.data_timestamp
                ).to_msg()
            elif cas.contains(CASViews.CAMERA_INFO):
                stamped_pose.timestamp = cas.get(CASViews.CAMERA_INFO).header.stamp

            object_hypothesis.annotations.append(stamped_pose)

    @staticmethod
    def get_latest_pose_annotation(
        object_hypothesis: ObjectHypothesis,
    ) -> PoseAnnotation | StampedPoseAnnotation | None:
        """
        Return the newest supported plain or stamped pose annotation.
        """
        return ObservationPose.get_latest_pose_annotation(object_hypothesis)

    def create_association_cost_matrix(
        self,
        object_hypotheses: list[ObjectHypothesis],
        object_beliefs: list[ObjectBeliefState],
    ) -> np.ndarray:
        """
        Create a negative-similarity cost matrix for Hungarian assignment.
        """
        cost_matrix = np.full((len(object_hypotheses), len(object_beliefs)), 1e9)
        for hypothesis_idx, object_hypothesis in enumerate(object_hypotheses):
            for belief_idx, object_belief in enumerate(object_beliefs):
                similarity = self.object_comparator.compute_similarity(
                    object_hypothesis, object_belief.latest_hypothesis
                )
                cost_matrix[hypothesis_idx][belief_idx] = -similarity
        return cost_matrix

    def apply_assignment(
        self,
        object_hypotheses: list[ObjectHypothesis],
        object_beliefs: list[ObjectBeliefState],
        cost_matrix: np.ndarray,
        hypothesis_indices: np.ndarray,
        belief_indices: np.ndarray,
        cas: CAS,
        belief_creations: BeliefCreations | None = None,
    ) -> list[tuple[ObjectHypothesis, ObjectBeliefState]]:
        """
        Update matched beliefs or create new beliefs for low-confidence matches.
        """
        associated_hypotheses: list[tuple[ObjectHypothesis, ObjectBeliefState]] = []
        threshold = self.descriptor.parameters.confidence_threshold
        belief_creations = belief_creations or BeliefCreations()

        for hypothesis_index, belief_index in zip(hypothesis_indices, belief_indices):
            object_hypothesis = object_hypotheses[hypothesis_index]
            object_belief = object_beliefs[belief_index]
            similarity = -cost_matrix[hypothesis_index, belief_index]

            if similarity > threshold:
                world.update_belief_state_with_object_hypothesis(
                    object_belief, object_hypothesis, cas
                )
            else:
                object_belief = belief_creations.create_belief(object_hypothesis, cas)
            associated_hypotheses.append((object_hypothesis, object_belief))

        return associated_hypotheses

    def create_new_beliefs_for_hypotheses(
        self,
        object_hypotheses: list[ObjectHypothesis],
        cas: CAS,
        belief_creations: BeliefCreations | None = None,
    ) -> list[tuple[ObjectHypothesis, ObjectBeliefState]]:
        """
        Create object beliefs for all given hypotheses.
        """
        belief_creations = belief_creations or BeliefCreations()
        return [
            (
                object_hypothesis,
                belief_creations.create_belief(object_hypothesis, cas),
            )
            for object_hypothesis in object_hypotheses
        ]

    def create_new_beliefs_for_unmatched_hypotheses(
        self,
        object_hypotheses: list[ObjectHypothesis],
        matched_hypothesis_indices: set[int],
        cas: CAS,
        belief_creations: BeliefCreations | None = None,
    ) -> list[tuple[ObjectHypothesis, ObjectBeliefState]]:
        """
        Create object beliefs for hypotheses not returned by Hungarian assignment.
        """
        unmatched_hypotheses = [
            object_hypothesis
            for hypothesis_index, object_hypothesis in enumerate(object_hypotheses)
            if hypothesis_index not in matched_hypothesis_indices
        ]
        # Hungarian assignment only pairs min(len(hypotheses), len(beliefs)); extras start new beliefs.
        return self.create_new_beliefs_for_hypotheses(
            unmatched_hypotheses, cas, belief_creations
        )

    def log_world_state(self) -> None:
        """
        Log a compact summary of the current SemDT world contents.
        """
        rk_world = world_instance()
        self.rk_logger.debug(
            f"SemDT \nKS Entities: {len(rk_world.kinematic_structure_entities)}\nViews: {len(rk_world.semantic_annotations)}\nConnections: {len(rk_world.connections)}"
        )

    def create_association_visualization(
        self,
        associated_hypotheses: list[tuple[ObjectHypothesis, ObjectBeliefState]],
    ) -> None:
        """
        Publish a 2D image showing which SemDT UUID each hypothesis maps to.
        """
        cas = self.get_cas()
        if not cas.contains(CASViews.COLOR_IMAGE):
            return

        visualization_img = cas.get_copy(CASViews.COLOR_IMAGE)
        object_hypotheses = [oh for oh, _ in associated_hypotheses]
        labels_by_hypothesis_id = {
            id(oh): self.get_object_belief_label(object_belief)
            for oh, object_belief in associated_hypotheses
        }

        draw_bounding_boxes_from_object_hypotheses(
            visualization_img,
            object_hypotheses,
            lambda oh: labels_by_hypothesis_id.get(id(oh), "unassociated"),
        )
        self.get_annotator_output_struct().set_image(visualization_img)

    @staticmethod
    def get_object_belief_label(object_belief: ObjectBeliefState) -> str:
        """
        Return a compact display label for an object belief UUID.
        """
        return f"{str(object_belief.uuid)[:10]}..."
