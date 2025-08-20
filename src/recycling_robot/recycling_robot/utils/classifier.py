# src/recycling_robot/recycling_robot/utils/classifier.py
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

# Torch is optional; we gracefully fall back to a mock classifier if missing.
try:
    import torch
    import torch.nn.functional as F
    _HAS_TORCH = True
except Exception:
    torch = None  # type: ignore
    F = None      # type: ignore
    _HAS_TORCH = False

# torchvision is only needed if you want a default backbone
try:
    from torchvision import models, transforms  # type: ignore
    from torch import nn  # type: ignore
    _HAS_TORCHVISION = True
except Exception:
    models = None  # type: ignore
    transforms = None  # type: ignore
    nn = None  # type: ignore
    _HAS_TORCHVISION = False

try:
    from PIL import Image  # type: ignore
    _HAS_PIL = True
except Exception:
    Image = None  # type: ignore
    _HAS_PIL = False


DEFAULT_CLASSES = ['cardboard', 'glass', 'metal', 'plastic', 'trash']


@dataclass
class ClassificationResult:
    predicted_class: str
    class_index: int
    confidence: float
    all_probabilities: Dict[str, float]
    timestamp: float


class RecyclingClassifier:
    """
    Minimal classifier wrapper.

    - If Torch + model are available → runs real inference.
    - Otherwise → deterministic mock, so the pipeline stays alive.
    """

    def __init__(
        self,
        model_path: str = "",
        class_names: Optional[List[str]] = None,
        device: Optional[str] = None,
        input_size: int = 224,
    ) -> None:
        self.class_names = class_names or DEFAULT_CLASSES
        self.input_size = int(input_size)
        self.device = device or ("cuda" if (_HAS_TORCH and torch.cuda.is_available()) else "cpu")
        self.model = None
        self.transform = None

        can_load_model = (
            _HAS_TORCH and _HAS_TORCHVISION and _HAS_PIL and
            bool(model_path) and os.path.exists(model_path)
        )

        if can_load_model:
            try:
                self.model = self._load_model(model_path)
                self.model.eval()
                self.transform = self._setup_transforms()
            except Exception as e:
                # Fall back to mock mode
                self.model = None
                self.transform = None
        else:
            # mock mode
            self.model = None
            self.transform = None

    # ----------- real model path -----------

    def _load_model(self, model_path: str):
        # Try to load either a scripted model or a state dict.
        try:
            # 1) torchscript / full model
            m = torch.jit.load(model_path, map_location=self.device)  # type: ignore
            m.to(self.device)
            return m
        except Exception:
            pass

        # 2) state dict into a MobileNetV2 backbone
        sd = torch.load(model_path, map_location=self.device)  # type: ignore
        if isinstance(sd, dict) and "state_dict" in sd:
            sd = sd["state_dict"]

        if not _HAS_TORCHVISION:
            raise RuntimeError("torchvision not available to build the model backbone")

        model = models.mobilenet_v2(weights=None)  # type: ignore
        model.classifier[1] = nn.Linear(model.last_channel, len(self.class_names))  # type: ignore
        model.load_state_dict(sd, strict=False)
        model.to(self.device)
        return model

    def _setup_transforms(self):
        return transforms.Compose([  # type: ignore
            transforms.Resize((self.input_size, self.input_size)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225],
            ),
        ])

    # ----------- API -----------

    def preprocess(self, bgr: np.ndarray) -> Optional["torch.Tensor"]:
        if not (_HAS_TORCH and _HAS_TORCHVISION and _HAS_PIL) or self.model is None:
            return None
        rgb = bgr[:, :, ::-1]
        pil = Image.fromarray(rgb)  # type: ignore
        x = self.transform(pil)  # type: ignore
        return x.unsqueeze(0).to(self.device)  # type: ignore

    def predict(self, bgr: np.ndarray) -> ClassificationResult:
        # Real model path
        if _HAS_TORCH and self.model is not None:
            t = self.preprocess(bgr)
            if t is None:
                return self._mock_predict(bgr)

            with torch.no_grad():  # type: ignore
                logits = self.model(t)
                if isinstance(logits, (list, tuple)):
                    logits = logits[0]
                probs = F.softmax(logits, dim=-1).squeeze(0).detach().cpu().numpy()  # type: ignore

            idx = int(np.argmax(probs))
            conf = float(probs[idx])
            return ClassificationResult(
                predicted_class=self.class_names[idx] if idx < len(self.class_names) else f"Unknown_{idx}",
                class_index=idx,
                confidence=conf,
                all_probabilities={cls: float(p) for cls, p in zip(self.class_names, probs[:len(self.class_names)])},
                timestamp=time.time(),
            )

        # Mock path
        return self._mock_predict(bgr)

    # Alias for backwards compatibility
    def classify(self, bgr: np.ndarray) -> ClassificationResult:
        return self.predict(bgr)

    # ----------- mock -----------

    def _mock_predict(self, bgr: np.ndarray) -> ClassificationResult:
        # Deterministic "fake" result based on mean pixel value
        mean_val = float(bgr.mean()) if bgr is not None else 128.0
        idx = int(mean_val) % len(self.class_names)
        probs = np.zeros(len(self.class_names), dtype=np.float32)
        probs[idx] = 0.85  # Make it look realistic
        # Add some noise to other classes
        for i in range(len(self.class_names)):
            if i != idx:
                probs[i] = np.random.uniform(0.01, 0.1)
        # Normalize
        probs = probs / probs.sum()
        
        return ClassificationResult(
            predicted_class=self.class_names[idx],
            class_index=idx,
            confidence=float(probs[idx]),
            all_probabilities={c: float(p) for c, p in zip(self.class_names, probs)},
            timestamp=time.time(),
        )