# Finding the parts of an object

An object often comes as a single mesh, while a robot needs its parts: the handle of a
mug to grasp it, the legs of a table to avoid them. {py:class}`~semantic_digital_twin.pipeline.annotation_part_finding.AnnotationPartFinder`
gives an annotation the parts its part-whole relationships call for, by finding them in
the shape of its root body and splitting each one off into a body and annotation of its
own.

```python
from semantic_digital_twin.pipeline.annotation_part_finding import AnnotationPartFinder

parts = AnnotationPartFinder().find_parts(mug)
mug.handle  # the handle that was found, or still None
```

Only relationships the annotation has no part in yet are looked at. An optional part,
such as a mug's handle, and each of any number of parts, such as a table's legs, is
added only when the finder is at least `minimum_confidence` sure of it.

## How the parts are found

The shape is rendered from eleven directions around it. Qwen3-VL is asked for boxes
around the part in each view ("the handle of the mug"), SAM 2.1 masks the part inside
the boxes, and each face of the shape belongs to the part when most views that show it
mask it. The confidence of a part is the share of the views showing it that agree. In a
benchmark on 93 kitchen objects this found 78 of 87 handles and marked nothing on any of
six cups without one, at about 5 seconds per object on an A40.

Every number of this pipeline is a field of
{py:class}`~semantic_digital_twin.pipeline.vision_language_part_locating.VisionLanguagePartLocator`,
{py:class}`~semantic_digital_twin.pipeline.vision_language_part_locating.CameraRing`,
{py:class}`~semantic_digital_twin.pipeline.vision_language_models.QwenBoxDetector` and
{py:class}`~semantic_digital_twin.pipeline.vision_language_models.Sam2BoxSegmenter`.

## Where the models run

By default `AnnotationPartFinder` asks the part finding server on neem-4, which is
reachable from within the university's network and needs nothing beyond this package:

```python
from semantic_digital_twin.adapters.part_finding.client import RemotePartLocator

AnnotationPartFinder(locator=RemotePartLocator(base_url="http://localhost:8790"))
```

The server keeps every answer, so the same question about the same shape is answered
from its cache.

To run the models on your own GPU instead, install the `vision` extra, after installing
the torch build that matches your driver's CUDA version (`nvidia-smi` shows it):

```bash
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install -e "semantic_digital_twin[vision]"
```

```python
from semantic_digital_twin.pipeline.vision_language_models import (
    QwenBoxDetector, QwenModel, Quantization, Sam2BoxSegmenter,
)
from semantic_digital_twin.pipeline.vision_language_part_locating import (
    VisionLanguagePartLocator,
)

locator = VisionLanguagePartLocator(
    detector=QwenBoxDetector(model=QwenModel.QWEN3_VL_4B, quantization=Quantization.FOUR_BIT),
    segmenter=Sam2BoxSegmenter(),
)
AnnotationPartFinder(locator=locator)
```

The 8B model needs about 19 GB of GPU memory; the 4B model in 4 bits fits into 8 GB
and found 73 of the 87 handles in the benchmark. On a CPU (`device=Device.CPU`) a
single object takes minutes.

## Running the server

The server is the same locator behind http. Start it with
`semantic-digital-twin-part-finding-server` (see `--help` for the model, quantization,
device, port and cache), or build the image in
`semantic_digital_twin/docker/part_finding`. It checks no credentials, so run it only
inside a trusted network.
