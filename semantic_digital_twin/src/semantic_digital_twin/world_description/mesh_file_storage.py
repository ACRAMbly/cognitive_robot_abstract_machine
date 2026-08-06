from __future__ import annotations

import atexit
import os
import shutil
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

from krrood.singleton import SingletonMeta
from typing_extensions import ClassVar


@dataclass
class MeshFileStorage(metaclass=SingletonMeta):
    """
    The place this process writes exported mesh files to.

    Each export gets a directory of its own beneath a single root, so a material or
    texture written beside a mesh belongs to that mesh alone. The root is removed when the
    process exits, which makes an exported path valid for exactly as long as the process
    that wrote it.

    ..note:: The root is created when this class is first instantiated, not on import, so
        a process that exports no mesh writes nothing.
    """

    root_prefix: ClassVar[str] = "semantic_digital_twin_meshes_"
    """
    Marks a temporary directory as a mesh session root of this package.
    """

    owner_process_id: int = field(init=False, default_factory=os.getpid)
    """
    The process that created the root, and the only one permitted to remove it.
    """

    root: Path = field(init=False)
    """
    The directory every mesh this process exports lives beneath.
    """

    def __post_init__(self) -> None:
        self.root = Path(
            tempfile.mkdtemp(
                prefix=f"{self.root_prefix}{self.owner_process_id}_",
                dir=tempfile.gettempdir(),
            )
        )
        atexit.register(self.remove)

    @staticmethod
    def create_mesh_directory(parent: Path) -> Path:
        """
        Create a directory holding a single mesh export.

        The name carries the creating process, so a mesh file named after its directory
        stays unique even against exports placed under a different parent.

        :param parent: The directory to create the mesh's directory in.
        :return: The path of the created directory.
        """
        return Path(tempfile.mkdtemp(prefix=f"{os.getpid()}_", dir=parent))

    def allocate_directory(self) -> Path:
        """
        Create a directory holding a single mesh export inside this process's root.

        :return: The path of the created directory.
        """
        return self.create_mesh_directory(self.root)

    def remove(self) -> None:
        """
        Delete the root and every mesh exported into it.

        Does nothing in a process that inherited the root by forking, so a child exiting
        cannot take the files away from the parent that owns them.
        """
        if os.getpid() != self.owner_process_id:
            return
        atexit.unregister(self.remove)
        shutil.rmtree(self.root, ignore_errors=True)
