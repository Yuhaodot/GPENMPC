"""Read explicitly configured external dependencies."""
import json
import os
from pathlib import Path


def external_path(key):
    config = os.environ.get("GPENMPC_EXTERNAL_PATHS", "")
    if not config or not Path(config).is_file():
        raise ValueError("Set GPENMPC_EXTERNAL_PATHS to the dependency configuration JSON.")
    value = json.loads(Path(config).read_text(encoding="utf-8-sig")).get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"Configure dependency {key}.")
    return value
