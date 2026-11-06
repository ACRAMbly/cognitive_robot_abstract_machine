# %% Imports

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from typing_extensions import TYPE_CHECKING

from robokudo import world
from robokudo.annotators.semantic_observations import SemanticObservationAcceptance
from robokudo.types.annotation import Classification
from robokudo.types.exceptions import MissingModelGeometry
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.mixins import IsPerceivable
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from typing_extensions import Mapping

    from robokudo.cas import CAS
    from robokudo.annotators.semantic_observations import PreparedObservation
    from robokudo.types.annotation import StampedPoseAnnotation
    from robokudo.types.belief_state import ObjectBeliefState
    from robokudo.types.scene import ObjectHypothesis
    from semantic_digital_twin.semantic_annotations.mixins import HasRootBody


# %% Model geometry


@dataclass
class ModelGeometry(ABC):
    """
    Construct independent bodies from application-owned model geometry.
    """

    @abstractmethod
    def create_body(self, body_name: PrefixedName) -> Body:
        """
        Prepare a detached body with loaded visual and collision geometry.
        """


# %% Prepared creation


@dataclass
class BeliefCreation:
    """
    Detached model body and accepted pose ready for world insertion.
    """

    body: Body
    """Body whose geometry is complete before persistent registration."""

    world_pose: StampedPoseAnnotation
    """
    Accepted mesh-origin pose in the declared world frame.
    """

    semantic_annotation: HasRootBody
    """Typed annotation rooted at the detached body."""

    has_model_geometry: bool
    """
    Whether model geometry replaces the generic bounding-box creation path.
    """

    @classmethod
    def from_observation(
        cls,
        observation: PreparedObservation,
        model_geometry: Mapping[str, ModelGeometry] | None,
    ) -> BeliefCreation:
        """
        Prepare one model body and its semantic annotation without registration.
        """
        label = next(
            annotation.classname
            for annotation in observation.hypothesis.annotations
            if isinstance(annotation, Classification)
        )
        if model_geometry is not None and label not in model_geometry:
            raise MissingModelGeometry(label)
        body = (
            Body()
            if model_geometry is None
            else model_geometry[label].create_body(PrefixedName(label))
        )
        for collection in (body.visual, body.collision):
            for shape in collection.shapes:
                shape.mesh
        annotation = observation.semantic_type(root=body)
        if isinstance(annotation, IsPerceivable):
            annotation.class_label = label
        return cls(body, observation.world_pose, annotation, model_geometry is not None)


@dataclass
class BeliefCreations:
    """
    Geometry prepared for an observation before any beliefs change.
    """

    by_hypothesis: dict[int, BeliefCreation] = field(default_factory=dict)
    """
    Detached creations indexed by observation instance identity.
    """

    @classmethod
    def prepare(
        cls,
        hypotheses: list[ObjectHypothesis],
        cas: CAS,
        semantic_types: dict[str, type[HasRootBody]] | None,
        model_geometry: Mapping[str, ModelGeometry] | None,
        source: str,
    ) -> BeliefCreations:
        """
        Load required geometry for every configured hypothesis before insertion.
        """
        if semantic_types is None:
            return cls()
        acceptance = SemanticObservationAcceptance(semantic_types)
        return cls(
            {
                id(hypothesis): BeliefCreation.from_observation(
                    acceptance.prepare(hypothesis, cas, source), model_geometry
                )
                for hypothesis in hypotheses
            }
        )

    def create_belief(
        self, hypothesis: ObjectHypothesis, cas: CAS
    ) -> ObjectBeliefState:
        """
        Register a prepared belief or create a generic belief for the hypothesis.
        """
        creation = self.by_hypothesis.get(id(hypothesis))
        return world.add_object_hypothesis_as_belief_state(hypothesis, cas, creation)
