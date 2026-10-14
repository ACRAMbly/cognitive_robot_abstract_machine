"""
Imports the module named by the first argument and prints, as a JSON list, the top-level
packages the interpreter loaded for it that it had not loaded at startup.
"""

import importlib
import json
import sys

loaded_at_startup = set(sys.modules)
importlib.import_module(sys.argv[1])

print(
    json.dumps(
        sorted(
            {name.partition(".")[0] for name in set(sys.modules) - loaded_at_startup}
        )
    )
)
