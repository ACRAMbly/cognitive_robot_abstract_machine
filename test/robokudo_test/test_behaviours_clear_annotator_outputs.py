import py_trees
import pytest

from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
from robokudo.behaviours.clear_annotator_outputs import ClearAnnotatorOutputs
from robokudo.exceptions import ParentPipelineMissing


class TestClearAnnotatorOutputs:
    def test_raises_without_parent_pipeline(
        self, annotator_output_buffer: AnnotatorOutputPerPipelineMap
    ):
        root = py_trees.composites.Sequence("Sequence", memory=True)
        clear_outputs = ClearAnnotatorOutputs()
        root.add_child(clear_outputs)

        with pytest.raises(ParentPipelineMissing):
            clear_outputs.update()
