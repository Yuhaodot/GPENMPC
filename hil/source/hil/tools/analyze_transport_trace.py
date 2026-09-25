"""Read the native receive batch associated with a transport failure."""
from pathlib import Path
from gpenmpc_external_path import external_path

import h5py
import numpy as np


MAT = Path(external_path('fragment_continuity_fixture')) / 'SHORT_HIL' / 'RAW_BOARD_LOCAL_SHORT_HIL.mat'


def main() -> None:
    with h5py.File(MAT, "r") as handle:
        def scalar(path: str):
            return handle[path][()].reshape(-1)[0].item()

        def string(path: str) -> str:
            return "".join(chr(int(value)) for value in handle[path][()].reshape(-1))

        def dereference(path: str, index: int) -> np.ndarray:
            dataset = handle[path]
            reference = dataset[0, index] if dataset.ndim == 2 else dataset[index]
            return handle[reference][()] if reference else np.array([])

        error_root = "rawIo/raw_transport/first_error/"
        print("first_error", {
            key: string(error_root + key) if key in {"identifier", "message", "operation"}
            else scalar(error_root + key)
            for key in handle[error_root.rstrip("/")].keys()
        })

        receive_root = "rawIo/raw_transport/last_receive/records/"
        for index in range(handle[receive_root + "bytes"].shape[1]):
            wire = dereference(receive_root + "bytes", index).reshape(-1).astype(np.uint8)

            def record_scalar(name: str):
                value = dereference(receive_root + name, index).reshape(-1)
                return value[0].item() if len(value) else None

            print(
                "datagram", index + 1,
                "bytes", len(wire),
                "hex", wire.tobytes().hex().upper(),
                "attempted", record_scalar("attempted"),
                "ok", record_scalar("ok"),
                "complete", record_scalar("bytes_complete"),
                "source_matched", record_scalar("source_matched"),
                "source_port", record_scalar("source_port"),
                "received_bytes", record_scalar("received_bytes"),
            )

        frame_root = "rawIo/raw_transport/last_frames/"
        for index in range(handle[frame_root + "raw_frame"].shape[1]):
            wire = dereference(frame_root + "raw_frame", index).reshape(-1).astype(np.uint8)
            validation_error = dereference(frame_root + "validation_error", index).reshape(-1)
            validation_error_text = "".join(chr(int(value)) for value in validation_error)
            print(
                "frame", index + 1,
                "bytes", len(wire),
                "hex", wire.tobytes().hex().upper(),
                "validated", dereference(frame_root + "validated", index).reshape(-1)[0].item(),
                "error", validation_error_text,
            )

        fragment_root = "rawIo/raw_transport/last_forwarding_fragments/"
        print("fragment_reason", string(fragment_root + "reason"))
        fragment = handle[fragment_root + "raw_fragment"][()].reshape(-1).astype(np.uint8)
        print("fragment_hex", fragment.tobytes().hex().upper())


if __name__ == "__main__":
    main()
