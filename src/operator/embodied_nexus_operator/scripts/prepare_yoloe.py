#!/usr/bin/env python3
"""Download the pretrained YOLOE model and cache the requested text embedding."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np


def prepare_model(weights_path: Path, embedding_path: Path, target_class: str) -> None:
    """Download YOLOE weights and cache the text embedding for a class prompt.

    Args:
        weights_path: Destination path for the pretrained YOLOE segmentation weights.
        embedding_path: Destination path for the cached prompt embedding.
        target_class: Open vocabulary text prompt to cache.

    Returns:
        None.
    """
    import shutil
    import ultralytics
    import torch
    from ultralytics import YOLOE

    weights_path.parent.mkdir(parents=True, exist_ok=True)
    embedding_path.parent.mkdir(parents=True, exist_ok=True)
    weights_path = weights_path.resolve()
    embedding_path = embedding_path.resolve()
    previous_directory = Path.cwd()
    try:
        os.chdir(weights_path.parent)
        model = YOLOE(str(weights_path) if weights_path.is_file() else "yoloe-11s-seg.pt")
        if not weights_path.is_file():
            downloaded_value = str(getattr(model, "ckpt_path", "") or "yoloe-11s-seg.pt")
            downloaded = Path(downloaded_value).expanduser().resolve()
            if not downloaded.is_file():
                downloaded = (weights_path.parent / "yoloe-11s-seg.pt").resolve()
            if downloaded.is_file() and downloaded != weights_path:
                shutil.copy2(downloaded, weights_path)
        model.set_classes([target_class])
        prompt_embeddings = model.model.pe.detach().cpu().numpy()
        np.savez_compressed(
            embedding_path,
            classes=np.asarray([target_class]),
            embeddings=prompt_embeddings,
        )
    finally:
        os.chdir(previous_directory)
    if not weights_path.is_file() or not embedding_path.is_file():
        raise RuntimeError("YOLOE preparation did not produce both local model assets")
    manifest = {
        "model": "YOLOE-11s-seg",
        "checkpoint": weights_path.name,
        "checkpoint_source": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yoloe-11s-seg.pt",
        "checkpoint_sha256": hashlib.sha256(weights_path.read_bytes()).hexdigest(),
        "ultralytics_version": str(ultralytics.__version__),
        "torch_version": str(torch.__version__),
        "cuda_available": bool(torch.cuda.is_available()),
        "cuda_device": (str(torch.cuda.get_device_name(0)) if torch.cuda.is_available() else ""),
        "text_prompt": str(target_class),
        "text_embeddings": embedding_path.name,
        "text_embeddings_sha256": hashlib.sha256(embedding_path.read_bytes()).hexdigest(),
    }
    manifest_path = embedding_path.with_suffix(".manifest.json")
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"YOLOE checkpoint: {weights_path}")
    print(f"Text embedding: {embedding_path}")
    print(f"Asset manifest: {manifest_path}")


def main() -> None:
    """Parse options and prepare local model assets.

    Args:
        None.

    Returns:
        None.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    default_dir = Path("~/.cache/embodied-nexus/models").expanduser()
    parser.add_argument("--weights", type=Path, default=default_dir / "yoloe-11s-seg.pt")
    parser.add_argument("--embedding", type=Path,
                        default=default_dir / "yoloe-red-rectangular-box-text.npz")
    parser.add_argument("--class-prompt", default="red rectangular box")
    args = parser.parse_args()
    prepare_model(args.weights.expanduser(), args.embedding.expanduser(), args.class_prompt)


if __name__ == "__main__":
    main()
