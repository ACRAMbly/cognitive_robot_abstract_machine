import subprocess
import sys
import tempfile
from pathlib import Path

import trimesh

from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.mesh_file_storage import MeshFileStorage

from .dataset import export_mesh_and_print_session_root

# %% where exported mesh files live


def test_exported_meshes_share_one_session_root(mesh_file_storage):
    """
    Meshes exported without an explicit directory collect under a single root owned by
    this process, rather than being scattered directly across the system temporary
    directory where nothing can ever find them again.
    """
    first = Mesh.from_trimesh(mesh=trimesh.creation.box(extents=(1.0, 1.0, 1.0)))
    second = Mesh.from_trimesh(mesh=trimesh.creation.box(extents=(2.0, 2.0, 2.0)))

    assert Path(first.filename).parent.parent == mesh_file_storage.root
    assert Path(second.filename).parent.parent == mesh_file_storage.root
    assert mesh_file_storage.root.parent == Path(tempfile.gettempdir())


# %% the lifetime of exported mesh files


def test_session_root_is_removed_on_process_exit():
    """
    A process that exports meshes leaves nothing behind once it exits normally.

    The export is run in a subprocess because the cleanup runs at interpreter shutdown,
    which the test process itself never reaches while the test is running.
    """
    script_path = Path(export_mesh_and_print_session_root.__file__)

    result = subprocess.run(
        [sys.executable, str(script_path)],
        capture_output=True,
        text=True,
        timeout=300,
    )

    assert result.returncode == 0, result.stderr
    session_root = Path(result.stdout.strip())
    assert not session_root.exists()


def test_remove_deletes_the_session_root():
    storage = MeshFileStorage()
    Mesh.from_trimesh(mesh=trimesh.creation.box(extents=(1.0, 1.0, 1.0)))

    storage.remove()
    MeshFileStorage.clear_instance()

    assert not storage.root.exists()


def test_file_supplied_mesh_is_not_removed(mesh_file_storage, tmp_path):
    """
    Cleanup reclaims only what the export wrote; a mesh whose path the caller supplied
    stays where the caller put it.
    """
    caller_owned_path = tmp_path / "caller_owned.stl"
    trimesh.creation.box(extents=(1.0, 1.0, 1.0)).export(caller_owned_path)
    mesh = Mesh.from_file(file_path=str(caller_owned_path))

    mesh_file_storage.remove()

    assert Path(mesh.filename).exists()
