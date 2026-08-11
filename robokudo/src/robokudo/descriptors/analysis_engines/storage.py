"""
Analysis engine that records Kinect camera data to storage.
"""

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.storage import StorageWriter

from robokudo.descriptors.factories.cr_descriptor_factory import (
    CollectionReaderDescriptorFactory,
)
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline


class AnalysisEngine(AnalysisEngineInterface):
    """
    Records preprocessed Kinect camera data with StorageWriter.
    """

    def name(self) -> str:
        return "storage"

    def implementation(self) -> Pipeline:
        """Create a pipeline for recording sensor data.

        This method constructs a processing pipeline that captures and stores
        sensor data from a Kinect camera. The pipeline preprocesses the data
        before storing it for later use.

        Pipeline configuration options:

        * Standard Kinect config (with transform lookup)
        * Kinect config without transform lookup (default)

        :return: The configured pipeline for data recording
        """

        camera_descriptor = CrDescriptorFactory.create_descriptor("orbbec")

        seq = Pipeline()
        seq.add_children(
            [
                pipeline_init(),
                CollectionReaderAnnotator(descriptor=camera_descriptor),
                ImagePreprocessorAnnotator("ImagePreprocessor"),
                StorageWriter(),
            ]
        )
        return seq
