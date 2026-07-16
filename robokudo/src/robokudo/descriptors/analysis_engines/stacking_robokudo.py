"""Query-driven colored-cube perception for the Tracy robot."""

from __future__ import annotations

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.cluster_pose_bb import ClusterPoseBBAnnotator
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.image_cluster_extractor import ImageClusterExtractor
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.query import GenerateQueryResult, QueryAnnotator
from robokudo.descriptors import CrDescriptorFactory
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline


def create_color_cluster_descriptor() -> ImageClusterExtractor.Descriptor:
    """Configure single-contour segmentation for supported block colors."""
    descriptor = ImageClusterExtractor.Descriptor()
    descriptor.parameters.color_name_to_hsv_range["yellow"] = {
        "hsv_min": (22, 130, 85),
        "hsv_max": (65, 255, 255),
    }
    descriptor.parameters.num_of_objects = 1
    return descriptor


class AnalysisEngine(AnalysisEngineInterface):
    """Detect one queried colored cube and estimate its bounding-box pose."""

    def name(self) -> str:
        """Return the analysis engine identifier."""
        return "demo"

    def implementation(self) -> Pipeline:
        """Create the query-driven color segmentation pipeline."""
        camera_descriptor = CrDescriptorFactory.create_descriptor("orbbec")
        color_cluster_descriptor = create_color_cluster_descriptor()

        pipeline = Pipeline("ColoredCubePipeline")
        pipeline.add_children(
            [
                pipeline_init(),
                QueryAnnotator(),
                CollectionReaderAnnotator(descriptor=camera_descriptor),
                ImagePreprocessorAnnotator("ImagePreprocessor"),
                ImageClusterExtractor(descriptor=color_cluster_descriptor),
                ClusterPoseBBAnnotator(),
                GenerateQueryResult(),
            ]
        )
        return pipeline
