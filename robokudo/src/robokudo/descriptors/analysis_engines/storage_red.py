"""Analysis engine for recording Orbbec sensor topics to MCAP.

The recorder subscribes to source ROS topics independently of CAS processing,
so the bag can be replayed through the same Orbbec camera interface.

The pipeline implements the following functionality:

* Reading data from an Orbbec camera
* Image preprocessing
* MCAP recording of RGB-D, calibration, and transform topics
"""

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.mcap_recorder import (
    McapRecorderAnnotator,
    McapRecorderConfiguration,
)
from robokudo.descriptors import CrDescriptorFactory
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline


class AnalysisEngine(AnalysisEngineInterface):
    """Record replayable Orbbec input while running image preprocessing.

    The MCAP bag contains the three configured Orbbec topics and both transform
    topics required by the camera-to-world lookup.
    """

    def name(self) -> str:
        """Get the name of the analysis engine.

        :return: The name identifier of this analysis engine
        """
        return 'storage'

    def implementation(self) -> Pipeline:
        """Create the Orbbec processing and MCAP recording pipeline.

        :return: Configured pipeline for sensor processing and recording.
        """
        camera_descriptor = CrDescriptorFactory.create_descriptor('orbbec')
        camera_config = camera_descriptor.parameters.camera_config
        recording_configuration = McapRecorderConfiguration(
            topics=(
                camera_config.topic_depth,
                camera_config.topic_color,
                camera_config.topic_cam_info,
                '/tf',
                '/tf_static',
            )
        )

        seq = Pipeline()
        seq.add_children(
            [
                pipeline_init(),
                CollectionReaderAnnotator(descriptor=camera_descriptor),
                ImagePreprocessorAnnotator('ImagePreprocessor'),
                McapRecorderAnnotator(configuration=recording_configuration),
            ]
        )
        return seq
