"""
Analysis engine answering perception queries for the Stretch robot.

Localizes objects standing on the dominant plane in view of the Stretch's RealSense and
reports their poses in response to a
:class:`~robokudo_msgs.action.Query`, so a plan can correct an object's pose before
grasping it.

.. note::
    The pipeline localizes but does not recognize: it attaches no
    :class:`~robokudo.types.annotation.Classification`, so every reported object designator
    has an empty ``type`` and the caller decides what it asked for. Adding a classifying
    annotator (for example
    :class:`~robokudo.annotators.clip_annotator.ClipAnnotator` or
    :class:`~robokudo.annotators.simple_yolo_annotator.SimpleYoloAnnotator`) before
    :class:`~robokudo.annotators.query.GenerateQueryResult` fills that field in without any
    other change.
"""

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.cluster_pose_bb import ClusterPoseBBAnnotator
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.plane import PlaneAnnotator
from robokudo.annotators.pointcloud_cluster_extractor import PointCloudClusterExtractor
from robokudo.annotators.pointcloud_crop import PointcloudCropAnnotator
from robokudo.annotators.query import QueryAnnotator, GenerateQueryResult, QueryReply
from robokudo.descriptors.factories.cr_descriptor_factory import (
    CollectionReaderDescriptorFactory,
)
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline

CAMERA_CONFIG_NAME = "realsense"
"""
Camera configuration this engine reads from.

The Stretch carries a RealSense D435i publishing on the stock ``realsense2_camera`` topics,
and its colour frame is named ``camera_color_optical_frame``, so the shared RealSense config
applies unchanged. Pass overrides to
:meth:`~robokudo.descriptors.factories.cr_descriptor_factory.CollectionReaderDescriptorFactory.create_descriptor`
if a particular robot publishes elsewhere.
"""


class AnalysisEngine(AnalysisEngineInterface):
    """
    Query-driven tabletop localization for the Stretch.
    """

    def name(self) -> str:
        """
        Get the name of the analysis engine.

        :return: The name identifier of this analysis engine
        """
        return "stretch_demo"

    def implementation(self) -> Pipeline:
        """
        Build the pipeline that answers a query with the poses of the objects in view.

        The pipeline waits for a query, reads a frame, isolates the dominant plane,
        treats what stands on it as objects, estimates a pose per object from its
        bounding box, and replies.

        :return: The configured pipeline for Stretch perception
        """
        camera_descriptor = CollectionReaderDescriptorFactory.create_descriptor(
            CAMERA_CONFIG_NAME
        )

        pipeline = Pipeline("StretchPipeline")
        pipeline.add_children(
            [
                pipeline_init(),
                QueryAnnotator(),
                CollectionReaderAnnotator(descriptor=camera_descriptor),
                ImagePreprocessorAnnotator("ImagePreprocessor"),
                PointcloudCropAnnotator(),
                PlaneAnnotator(),
                PointCloudClusterExtractor(),
                ClusterPoseBBAnnotator(),
                # Left unconfigured so that filtering by query stays off: it compares the
                # requested type against Classification annotations, which this pipeline
                # does not produce, so filtering a typed query would discard every object.
                # ..warning:: `GenerateQueryResult.Descriptor.parameters` is a class
                #     attribute shared by every instance, so overriding it here would
                #     change the setting for other pipelines in the same process too.
                GenerateQueryResult(),
                QueryReply(),
            ]
        )
        return pipeline
