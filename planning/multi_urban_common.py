"""Deterministic helpers for multi-city planning artifacts."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any


def canonical_bytes(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        + "\n"
    ).encode("utf-8")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest().upper()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def file_reference(path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    return {
        "path": str(resolved),
        "bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def write_content_addressed_json(
    directory: Path, prefix: str, value: Any
) -> dict[str, Any]:
    payload = canonical_bytes(value)
    digest = sha256_bytes(payload)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / f"{prefix}_{digest[:16]}.json"
    if target.exists():
        if target.read_bytes() != payload:
            raise RuntimeError(f"Content-address collision at {target}")
    else:
        temporary = target.with_suffix(target.suffix + ".tmp")
        if temporary.exists():
            raise FileExistsError(temporary)
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    return file_reference(target)
