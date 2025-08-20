# src/core/classifier.py
"""
Core classification module - handles model loading and inference.
Decoupled from camera and display logic for better modularity.
"""

import torch
import torch.nn.functional as F
from torchvision import transforms
from PIL import Image
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)

@dataclass
class ClassificationResult:
    """Structured result from classification inference."""
    predicted_class: str
    class_index: int
    confidence: float
    all_probabilities: Dict[str, float]
    timestamp: float

class RecyclingClassifier:
    """
    Core classification engine.
    Handles model loading, preprocessing, and inference.
    Independent of camera/display systems for better modularity.
    """
    
    DEFAULT_CLASSES = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
    
    def __init__(
        self, 
        model_path: str,
        class_names: Optional[List[str]] = None,
        device: Optional[str] = None,
        input_size: int = 224
    ):
        self.model_path = model_path
        self.class_names = class_names or self.DEFAULT_CLASSES
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.input_size = input_size
        
        logger.info(f"Initializing classifier on device: {self.device}")
        
        # Load model and setup transforms
        self.model = self._load_model()
        self.model.eval()
        self.transform = self._setup_transforms()
        
        logger.info("Classifier initialized successfully")
    
    def _load_model(self) -> torch.nn.Module:
        """Load PyTorch model from checkpoint."""
        try:
            checkpoint = torch.load(
                self.model_path, 
                map_location=self.device, 
                weights_only=False
            )
            
            logger.debug(f"Loaded checkpoint type: {type(checkpoint)}")
            
            # Handle different checkpoint formats
            if isinstance(checkpoint, dict):
                if 'state_dict' in checkpoint:
                    state_dict = checkpoint['state_dict']
                else:
                    state_dict = checkpoint
                
                # Create model architecture and load state
                model = self._create_model_architecture()
                model.load_state_dict(state_dict)
                logger.info("Loaded model from state dict")
                
            else:
                # Full model object
                model = checkpoint
                logger.info("Loaded complete PyTorch model")
            
            model.to(self.device)
            return model
            
        except Exception as e:
            logger.error(f"Failed to load model from {self.model_path}: {e}")
            raise
    
    def _create_model_architecture(self) -> torch.nn.Module:
        """Create model architecture. Override this for custom models."""
        try:
            # Try importing from project first
            from src.classifier.model import get_model
            return get_model(num_classes=len(self.class_names))
        except ImportError:
            # Fallback to standard torchvision model
            from torchvision import models
            from torch import nn
            
            model = models.mobilenet_v2(weights=None)
            model.classifier[1] = nn.Linear(
                model.last_channel, 
                len(self.class_names)
            )
            return model
    
    def _setup_transforms(self) -> transforms.Compose:
        """Setup image preprocessing transforms."""
        try:
            # Try importing from project
            from src.classifier.utils import get_transforms
            _, transform = get_transforms(input_size=self.input_size)
            return transform
        except ImportError:
            # Standard transforms
            return transforms.Compose([
                transforms.Resize((self.input_size, self.input_size)),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225]
                ),
            ])
    
    def preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        """
        Preprocess image for inference.
        
        Args:
            image: RGB numpy array (H, W, 3)
            
        Returns:
            Preprocessed tensor ready for model
        """
        pil_image = Image.fromarray(image)
        tensor = self.transform(pil_image).unsqueeze(0).to(self.device)
        return tensor
    
    @torch.inference_mode()
    def classify(self, image: np.ndarray) -> ClassificationResult:
        """
        Classify a single image.
        
        Args:
            image: RGB numpy array (H, W, 3)
            
        Returns:
            ClassificationResult with prediction details
        """
        import time
        
        # Preprocess
        input_tensor = self.preprocess_image(image)
        
        # Inference
        logits = self.model(input_tensor)
        if isinstance(logits, (list, tuple)):
            logits = logits[0]
            
        probabilities = F.softmax(logits, dim=1)
        confidence, class_idx = torch.max(probabilities, dim=1)
        
        # Extract results
        class_idx = class_idx.item()
        confidence = float(confidence.item())
        probs_array = probabilities[0].detach().cpu().numpy()
        
        # Build result
        predicted_class = (
            self.class_names[class_idx] 
            if class_idx < len(self.class_names) 
            else f"Unknown_{class_idx}"
        )
        
        all_probs = {
            self.class_names[i]: float(probs_array[i])
            for i in range(min(len(probs_array), len(self.class_names)))
        }
        
        return ClassificationResult(
            predicted_class=predicted_class,
            class_index=class_idx,
            confidence=confidence,
            all_probabilities=all_probs,
            timestamp=time.time()
        )