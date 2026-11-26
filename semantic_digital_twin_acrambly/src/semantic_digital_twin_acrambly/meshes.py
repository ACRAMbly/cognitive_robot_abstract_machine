"""
The directories of the meshes of the ACRAMbly objects.
"""

from __future__ import annotations

from pathlib import Path

from krrood.patterns.path_enumeration import PathEnumeration

# %% resource directories


class ResourceDirectory(PathEnumeration):
    """
    The directories of the resources of this distribution, which lie outside its source.
    """

    MESHES = Path(__file__).resolve().parents[2] / "resources" / "meshes"
    """
    The meshes of the ACRAMbly objects, in one directory per kind of object.
    """


# %% mesh directories


class MeshDirectory(PathEnumeration):
    """
    The directory of the meshes of each kind of ACRAMbly object.
    """

    AIRCRAFT = ResourceDirectory.MESHES / "aircraft"
    """
    The meshes of the aircraft parts, each named after its part.
    """

    TETRIS_BLOCK = ResourceDirectory.MESHES / "tetris_block"
    """
    The meshes of the tetris blocks, in millimetres.
    """
