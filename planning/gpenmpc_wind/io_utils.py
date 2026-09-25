"""Canonical, content-addressed I/O for the planning executables."""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Iterable


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest().upper()


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def file_reference(path: Path, root: Path | None = None) -> dict[str, Any]:
    resolved = path.resolve()
    display = str(resolved if root is None else resolved.relative_to(root.resolve()))
    return {"path": display, "bytes": resolved.stat().st_size, "sha256": sha256_path(resolved)}


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError(f"Refusing to overwrite a different existing artifact: {path}")
        return
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def write_content_addressed_json(root: Path, prefix: str, value: Any) -> Path:
    payload = canonical_bytes(value)
    digest = sha256_bytes(payload)
    target = root / f"{prefix}_{digest[:16]}.json"
    _atomic_write(target, payload)
    return target


def write_csv(path: Path, rows: Iterable[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise RuntimeError(f"Refusing to overwrite existing CSV: {path}")
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("x", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
