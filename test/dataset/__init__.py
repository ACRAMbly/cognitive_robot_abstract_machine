"""
Runnable stand-ins that tests launch as their own processes.

The paths are exposed here rather than the modules being imported: each script imports
the message packages it serves at module level, so importing one would break collection
wherever that package is absent instead of letting the tests skip.
"""

from pathlib import Path

PERCEPTION_PIPELINE_STAND_IN_PATH = (
    Path(__file__).parent / "perception_pipeline_stand_in.py"
)
"""
Script serving a perception query action with a canned detection.
"""
