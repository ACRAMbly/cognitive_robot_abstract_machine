"""
Generates a committed :class:`VerbalizationSurface` snapshot module from a
:class:`SymbolicSurfaceSnapshot`.

Uses :class:`~krrood.code_generation.generator.CodeGenerator` so the module is produced
rather than hand-transcribed, the same way :mod:`krrood.ormatic.sqlalchemy_generator`
produces ``ormatic_interface.py``: run the generator, review the diff, commit it.
"""

from __future__ import annotations

import importlib.resources
from dataclasses import dataclass, field
from pathlib import Path

from typing_extensions import Any, Dict, Tuple, Type, Union

from krrood.code_generation.formatting import run_black_on_file
from krrood.code_generation.generator import CodeGenerator
from krrood.code_generation.imports import get_imports_from_types
from krrood.code_generation.type_hints import value_to_source
from krrood.entity_query_language.predicate import SymbolicCallable
from krrood.entity_query_language.testing.surface_verification import (
    SymbolicSurfaceSnapshot,
    VerbalizationSurface,
)


@dataclass
class VerbalizationSurfaceGenerator:
    """
    Renders the Python source of a ``SURFACES`` snapshot module for a
    :class:`SymbolicSurfaceSnapshot`.
    """

    snapshot: SymbolicSurfaceSnapshot
    """
    The snapshot whose covered callables and renderings this generator emits.
    """

    code_generator: CodeGenerator = field(init=False)
    """
    Renderer bound to this package's templates directory.
    """

    def __post_init__(self):
        templates = importlib.resources.files(__package__) / "templates"
        self.code_generator = CodeGenerator(template_directory=str(templates))

    def covered_callables(self) -> Tuple[Type[SymbolicCallable], ...]:
        """:return: the discovered callables that implement their own verbalization
        fragment, in the order they appear in the generated module."""
        return tuple(
            cls
            for cls in self.snapshot.discovered_callables()
            if self.snapshot.has_fragment(cls)
        )

    def _entry(self, cls: Type[SymbolicCallable]) -> Dict[str, Any]:
        return {
            "class_name": cls.__qualname__,
            "sentence": value_to_source(self.snapshot.rendered_surface(cls)),
        }

    def generate(self) -> str:
        """:return: the Python source of a module declaring ``SURFACES``, one
        :class:`VerbalizationSurface` per covered callable."""
        covered = self.covered_callables()
        imports = get_imports_from_types([VerbalizationSurface, Tuple, *covered])
        entries = [self._entry(cls) for cls in covered]
        return self.code_generator.render(
            "verbalization_surfaces.py.jinja", imports=imports, entries=entries
        )

    def write(self, path: Union[str, Path]) -> None:
        """
        Render :meth:`generate` to *path* and format it with Black.

        :param path: The file to write the generated module to.
        """
        Path(path).write_text(self.generate())
        run_black_on_file(str(path))
