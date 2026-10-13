"""
Import the module named by the first command line argument.

Run in a fresh interpreter, this checks that the module can be the first one of its
package to be imported.
"""

import importlib
import sys

importlib.import_module(sys.argv[1])
