# Keep NOCTIS in a separate local runtime

The recorded-Orbbec analysis engine runs Robokudo and FoundationPose in the existing
Python 3.12 environment and NOCTIS in a persistent local worker using its existing
Python 3.8 environment. NOCTIS's installed xformers requires PyTorch 2.2.1, while the
FoundationPose environment uses PyTorch 2.7.1; isolation reuses both installations and
avoids making an unverified dependency migration a prerequisite. The NOCTIS annotator
communicates with the worker, preserving the requested engine structure of one
annotator per model. Both stages run sequentially on each matched recorded observation.
The target GPU is an RTX 3080 with 10 GiB memory; combined model memory use must be
verified during local validation.
