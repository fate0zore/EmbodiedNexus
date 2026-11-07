"""Pretrained YOLOE open-vocabulary instance segmentation adapter."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


class YOLOEBoxDetector:
    """Load local YOLOE segmentation weights and a cached text prompt."""

    def __init__(self, weights_path: str, embeddings_path: str, confidence: float = 0.25,
                 class_prompt: str = "box") -> None:
        """Load local model assets and bind the configured text prompt.

        Args:
            weights_path: Local pretrained YOLOE segmentation checkpoint.
            embeddings_path: Local YOLOE prompt embedding file prepared offline.
            confidence: Minimum confidence accepted for a box detection.
            class_prompt: Text prompt represented by the cached embedding.

        Returns:
            None.
        """
        weights = Path(weights_path).expanduser().resolve()
        embeddings = Path(embeddings_path).expanduser().resolve()
        prompt = str(class_prompt).strip()
        if not prompt:
            raise ValueError("YOLOE text prompt must not be empty")
        if not weights.is_file() or not embeddings.is_file():
            raise FileNotFoundError(
                f"YOLOE assets are missing; prepare local weights and prompt embeddings: {weights}, {embeddings}")
        manifest_path = embeddings.with_suffix(".manifest.json")
        if not manifest_path.is_file():
            raise FileNotFoundError(f"YOLOE asset manifest is missing: {manifest_path}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if str(manifest.get("text_prompt", "")).strip().lower() != prompt.lower():
            raise ValueError("YOLOE prompt embedding manifest does not match the configured text prompt")
        weights_hash = hashlib.sha256(weights.read_bytes()).hexdigest()
        embedding_hash = hashlib.sha256(embeddings.read_bytes()).hexdigest()
        if manifest.get("checkpoint_sha256") != weights_hash:
            raise ValueError("YOLOE checkpoint hash differs from its local manifest")
        if manifest.get("text_embeddings_sha256") != embedding_hash:
            raise ValueError("YOLOE prompt embedding hash differs from its local manifest")
        with np.load(embeddings, allow_pickle=False) as archive:
            classes = archive["classes"].astype(str).tolist()
            prompt_embeddings = np.asarray(archive["embeddings"], dtype=np.float32)
        if classes != [prompt]:
            raise ValueError("YOLOE prompt embedding archive does not contain the configured class")
        if prompt_embeddings.ndim != 3 or not np.isfinite(prompt_embeddings).all():
            raise ValueError("YOLOE prompt embeddings must be a finite three-dimensional tensor")
        from ultralytics import YOLOE
        import torch

        self.model = YOLOE(str(weights))
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.model.model.to(device)
        self.model.set_classes(classes, torch.as_tensor(prompt_embeddings, device=device))
        self.confidence = float(confidence)
        self.class_prompt = prompt

    def segment(self, rgb_image: np.ndarray) -> tuple[np.ndarray, float]:
        """Return the highest-confidence prompted instance mask.

        Args:
            rgb_image: HxWx3 RGB image as uint8 values.

        Returns:
            Binary target mask and detection confidence.
        """
        image = np.asarray(rgb_image)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("YOLOE expects a three-channel RGB image")
        result = self.model.predict(image[:, :, ::-1].copy(), verbose=False,
                                    conf=self.confidence, max_det=20)[0]
        if result.boxes is None or result.masks is None or len(result.boxes) == 0:
            raise RuntimeError("YOLOE did not detect a segmented target for the requested class")
        confidences = result.boxes.conf.detach().cpu().numpy()
        selected = int(np.argmax(confidences))
        mask = result.masks.data[selected].detach().cpu().numpy()
        if mask.shape != image.shape[:2]:
            mask = cv2.resize(mask.astype(np.float32), (image.shape[1], image.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
        return mask > 0.5, float(confidences[selected])
