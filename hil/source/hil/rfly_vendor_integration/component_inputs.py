"""Explicit input paths for optional, offline board component checks."""
import os
from pathlib import Path


def component_input(variable, *, directory=True):
    """Resolve a configured directory or file for a component check."""
    value = os.environ.get(variable, "")
    if not value or not Path(value).is_absolute():
        raise ValueError(f"Set {variable} to an explicit absolute input path")
    path = Path(value).resolve()
    valid = path.is_dir() if directory else path.is_file()
    if not valid:
        kind = "directory" if directory else "file"
        raise FileNotFoundError(f"{variable} requires an existing {kind}: {path}")
    return path
