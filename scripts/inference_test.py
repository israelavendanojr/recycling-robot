import torch
import torch.nn.functional as F
from torchvision import transforms
from picamera2 import Picamera2
import cv2
import time
from PIL import Image
import numpy as np
import os
import sys

# Add project root to path to import your modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

# Import your model architecture
try:
    from src.classifier.model import get_model
    from src.classifier.utils import get_transforms
except ImportError as e:
    print(f"Warning: Could not import project modules: {e}")
    print("Falling back to standalone model loading...")
    get_model = None
    get_transforms = None

class RecyclingClassifier:
    def __init__(self, model_path="recycler.pth", device=None):
        """
        Initialize the recycling classifier
        
        Args:
            model_path: Path to the trained PyTorch model
            device: Device to run inference on (cuda/cpu)
        """
        self.device = device if device else ('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")
        
        # Define class names FIRST (needed for model loading)
        self.class_names = [
            'cardboard',
            'glass', 
            'metal',
            'plastic',
            'trash'
        ]
        
        # Check if model file exists
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        # Load the trained model
        self.model = self.load_model(model_path)
        self.model.eval()
        
        # Define preprocessing transforms (matching your training setup)
        self.transform = self.get_preprocessing_transforms()
        
        # Initialize camera
        self.setup_camera()
    
    def load_model(self, model_path):
        """Load the model with proper error handling for .pth files"""
        try:
            # First, peek at what's in the file
            checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
            print(f"Loaded checkpoint type: {type(checkpoint)}")
            
            # If it's an OrderedDict, it's a state dict
            if isinstance(checkpoint, dict) and 'state_dict' not in checkpoint:
                print("Detected state dict format, need to reconstruct model...")
                # This is a state dict, we need the model architecture
                if get_model is not None:
                    model = get_model(num_classes=len(self.class_names))
                    model.load_state_dict(checkpoint)
                    model.to(self.device)
                    model.eval()
                    print("✓ Loaded model from state dict using project architecture")
                    return model
                else:
                    # Fallback: try to create a basic MobileNetV2 model
                    print("Project modules not available, trying to create MobileNetV2...")
                    from torchvision import models
                    from torch import nn
                    
                    model = models.mobilenet_v2(weights=None)  # Don't load pretrained weights
                    model.classifier[1] = nn.Linear(model.last_channel, len(self.class_names))
                    model.load_state_dict(checkpoint)
                    model.to(self.device)
                    model.eval()
                    print("✓ Loaded model using fallback MobileNetV2 architecture")
                    return model
                    
            # If it has 'state_dict' key, extract it
            elif isinstance(checkpoint, dict) and 'state_dict' in checkpoint:
                print("Found 'state_dict' key in checkpoint...")
                state_dict = checkpoint['state_dict']
                if get_model is not None:
                    model = get_model(num_classes=len(self.class_names))
                    model.load_state_dict(state_dict)
                    model.to(self.device)
                    model.eval()
                    print("✓ Loaded model from checkpoint state dict")
                    return model
                else:
                    print("Project modules not available, trying fallback...")
                    from torchvision import models
                    from torch import nn
                    
                    model = models.mobilenet_v2(weights=None)
                    model.classifier[1] = nn.Linear(model.last_channel, len(self.class_names))
                    model.load_state_dict(state_dict)
                    model.to(self.device)
                    model.eval()
                    print("✓ Loaded model using fallback architecture")
                    return model
                    
            # If it's already a model object
            else:
                checkpoint.eval()
                checkpoint.to(self.device)
                print("✓ Loaded complete PyTorch model (.pth)")
                return checkpoint
                
        except Exception as e:
            print(f"✗ Error loading model: {e}")
            import traceback
            traceback.print_exc()
            raise Exception(f"Could not load model from {model_path}")
    
    def get_preprocessing_transforms(self):
        """Get preprocessing transforms matching your training setup"""
        if get_transforms is not None:
            # Use the transforms from your project
            _, transform = get_transforms(input_size=224)
            return transform
        else:
            # Fallback to manual transform definition
            return transforms.Compose([
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                                   std=[0.229, 0.224, 0.225])  # ImageNet normalization
            ])
    
    def setup_camera(self):
        """Initialize and configure the camera"""
        try:
            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration({
                "format": "RGB888", 
                "size": (1280, 720)
            }))
            self.picam2.start()
            time.sleep(0.5)  # Allow camera to warm up
            print("✓ Camera initialized successfully")
        except Exception as e:
            print(f"✗ Failed to initialize camera: {e}")
            print("Make sure you're running on a Raspberry Pi with camera enabled")
            raise
    
    def capture_frame(self):
        """Capture a frame from the camera"""
        frame = self.picam2.capture_array()
        return frame
    
    def preprocess_frame(self, frame):
        """
        Preprocess the captured frame for model input
        
        Args:
            frame: RGB numpy array from camera
            
        Returns:
            preprocessed tensor ready for model inference
        """
        # Convert numpy array to PIL Image
        pil_image = Image.fromarray(frame)
        
        # Apply preprocessing transforms
        preprocessed = self.transform(pil_image)
        
        # Add batch dimension and move to device
        preprocessed = preprocessed.unsqueeze(0).to(self.device)
        
        return preprocessed
    
    def run_inference(self, preprocessed_frame):
        """
        Run model inference on preprocessed frame
        
        Args:
            preprocessed_frame: Preprocessed tensor
            
        Returns:
            predicted_class_idx: Index of predicted class
            confidence: Confidence score (0-1)
            all_probabilities: All class probabilities
        """
        with torch.no_grad():
            # Run inference
            outputs = self.model(preprocessed_frame)
            
            # Apply softmax to get probabilities
            probabilities = F.softmax(outputs, dim=1)
            
            # Get predicted class and confidence
            confidence, predicted_class_idx = torch.max(probabilities, 1)
            
            return predicted_class_idx.item(), confidence.item(), probabilities[0].cpu().numpy()
    
    def classify_current_view(self, save_frame=False, show_all_probs=False):
        """
        Capture frame, run inference, and return results
        
        Args:
            save_frame: Whether to save the captured frame
            show_all_probs: Whether to show probabilities for all classes
            
        Returns:
            dict: Classification results
        """
        # Capture frame
        frame = self.capture_frame()
        print(f"Captured frame: {frame.shape}")
        
        # Save frame if requested
        if save_frame:
            cv2.imwrite("latest_capture.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            print("Frame saved as 'latest_capture.jpg'")
        
        # Preprocess frame
        preprocessed = self.preprocess_frame(frame)
        
        # Run inference
        class_idx, confidence, all_probs = self.run_inference(preprocessed)
        
        # Get class name
        class_name = self.class_names[class_idx] if class_idx < len(self.class_names) else f"Unknown_Class_{class_idx}"
        
        results = {
            'predicted_class': class_name,
            'class_index': class_idx,
            'confidence': confidence,
            'frame': frame
        }
        
        if show_all_probs:
            results['all_probabilities'] = {
                self.class_names[i]: prob for i, prob in enumerate(all_probs)
            }
        
        return results
    
    def run_continuous_inference(self, delay_seconds=2, save_frames=False):
        """
        Run continuous inference loop
        
        Args:
            delay_seconds: Delay between inferences
            save_frames: Whether to save each frame
        """
        print("\n=== Starting Continuous Inference ===")
        print("Press Ctrl+C to stop...")
        
        frame_count = 0
        try:
            while True:
                results = self.classify_current_view(save_frame=save_frames, show_all_probs=True)
                
                frame_count += 1
                print(f"\n--- Frame {frame_count} ---")
                print(f"Predicted Class: {results['predicted_class']}")
                print(f"Confidence: {results['confidence']:.4f} ({results['confidence']*100:.2f}%)")
                
                # Show confidence level
                if results['confidence'] > 0.8:
                    print("High confidence prediction")
                elif results['confidence'] > 0.6:
                    print("Medium confidence prediction") 
                else:
                    print("Low confidence prediction")
                
                # Show all probabilities
                print("All class probabilities:")
                for class_name, prob in results['all_probabilities'].items():
                    print(f"  {class_name}: {prob:.4f} ({prob*100:.2f}%)")
                
                time.sleep(delay_seconds)
                
        except KeyboardInterrupt:
            print("\n\nStopping continuous inference...")
    
    def cleanup(self):
        """Clean up camera resources"""
        if hasattr(self, 'picam2'):
            self.picam2.close()
            print("✓ Camera closed")

def test_model_loading(model_path):
    """Test if the model can be loaded correctly"""
    print(f"Testing model loading from: {model_path}")
    
    if not os.path.exists(model_path):
        print(f"✗ Model file not found: {model_path}")
        return False
    
    try:
        # Load and inspect the file
        device = 'cpu'  # Use CPU for testing
        checkpoint = torch.load(model_path, map_location=device, weights_only=False)
        print(f"✓ File loaded successfully")
        print(f"Content type: {type(checkpoint)}")
        
        if isinstance(checkpoint, dict):
            if 'state_dict' in checkpoint:
                print("✓ Found checkpoint with 'state_dict' key")
                state_dict = checkpoint['state_dict']
            else:
                print("✓ Found state dictionary")
                state_dict = checkpoint
                
            # Try to create model and load state dict
            try:
                from torchvision import models
                from torch import nn
                
                model = models.mobilenet_v2(weights=None)
                model.classifier[1] = nn.Linear(model.last_channel, 5)  # 5 classes
                model.load_state_dict(state_dict)
                model.eval()
                
                print(f"✓ Model reconstructed successfully")
                print(f"Model type: {type(model)}")
                
                # Try a forward pass
                dummy_input = torch.randn(1, 3, 224, 224)
                with torch.no_grad():
                    output = model(dummy_input)
                print(f"✓ Forward pass successful, output shape: {output.shape}")
                print(f"✓ Number of classes detected: {output.shape[1]}")
                return True
                
            except Exception as model_error:
                print(f"✗ Error reconstructing model: {model_error}")
                return False
        else:
            # It's already a model
            model = checkpoint
            model.eval()
            print(f"✓ Complete model loaded")
            print(f"Model type: {type(model)}")
            
            # Try a forward pass
            dummy_input = torch.randn(1, 3, 224, 224)
            with torch.no_grad():
                output = model(dummy_input)
            print(f"✓ Forward pass successful, output shape: {output.shape}")
            print(f"✓ Number of classes detected: {output.shape[1]}")
            return True
            
    except Exception as e:
        print(f"✗ Error testing model: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main function to run inference"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Recycling Classification Inference')
    parser.add_argument('--model_path', type=str, default='recycler.pth',
                       help='Path to the model file')
    parser.add_argument('--test_model', action='store_true',
                       help='Test model loading without camera')
    parser.add_argument('--single_shot', action='store_true',
                       help='Run single inference instead of continuous')
    parser.add_argument('--continuous', action='store_true', default=True,
                       help='Run continuous inference (default)')
    parser.add_argument('--delay', type=float, default=2.0,
                       help='Delay between inferences in continuous mode')
    parser.add_argument('--save_frames', action='store_true',
                       help='Save captured frames')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Recycling Classification System")
    print("=" * 60)
    
    # Test model loading if requested
    if args.test_model:
        success = test_model_loading(args.model_path)
        if success:
            print("✓ Model test passed!")
        else:
            print("✗ Model test failed!")
        return
    
    try:
        # Initialize classifier
        classifier = RecyclingClassifier(model_path=args.model_path)
        print("✓ Classifier initialized successfully")
        
        if args.single_shot:
            # Single inference
            print("\nRunning single inference...")
            results = classifier.classify_current_view(save_frame=args.save_frames, show_all_probs=True)
            
            print(f"\n--- Classification Results ---")
            print(f"Predicted Class: {results['predicted_class']}")
            print(f"Confidence: {results['confidence']:.4f} ({results['confidence']*100:.2f}%)")
            
            # Show confidence level
            if results['confidence'] > 0.8:
                print("High confidence prediction")
            elif results['confidence'] > 0.6:
                print("Medium confidence prediction") 
            else:
                print("Low confidence prediction")
                
            # Show all probabilities
            print("\nAll class probabilities:")
            for class_name, prob in results['all_probabilities'].items():
                print(f"  {class_name}: {prob:.4f} ({prob*100:.2f}%)")
                
        else:
            # Continuous inference
            classifier.run_continuous_inference(
                delay_seconds=args.delay, 
                save_frames=args.save_frames
            )
            
    except Exception as e:
        print(f"✗ Error during inference: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Clean up
        if 'classifier' in locals():
            classifier.cleanup()
        print("✓ Cleanup completed")

if __name__ == "__main__":
    main()