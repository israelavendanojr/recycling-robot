import torch
import torch.nn.functional as F
from torchvision import transforms
from picamera2 import Picamera2
import cv2
import time
from PIL import Image
import numpy as np

class RecyclingClassifier:
    def __init__(self, model_path="models/recycling_classifier.pt", device=None):
        """
        Initialize the recycling classifier
        
        Args:
            model_path: Path to the trained PyTorch model
            device: Device to run inference on (cuda/cpu)
        """
        self.device = device if device else ('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")
        
        # Load the trained model
        self.model = torch.load(model_path, map_location=self.device)
        self.model.eval()
        
        # Define preprocessing transforms (adjust based on your training setup)
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),  # Common input size - adjust if needed
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])  # ImageNet normalization
        ])
        
        # Define class names (update these based on your actual classes)
        self.class_names = [
            'plastic_bottle',
            'aluminum_can', 
            'glass_bottle',
            'paper',
            'cardboard',
            'organic_waste',
            'other'
        ]
        
        # Initialize camera
        self.setup_camera()
    
    def setup_camera(self):
        """Initialize and configure the camera"""
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration({
            "format": "RGB888", 
            "size": (1280, 720)
        }))
        self.picam2.start()
        time.sleep(0.5)  # Allow camera to warm up
        print("Camera initialized successfully")
    
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
        """
        with torch.no_grad():
            # Run inference
            outputs = self.model(preprocessed_frame)
            
            # Apply softmax to get probabilities
            probabilities = F.softmax(outputs, dim=1)
            
            # Get predicted class and confidence
            confidence, predicted_class_idx = torch.max(probabilities, 1)
            
            return predicted_class_idx.item(), confidence.item()
    
    def classify_current_view(self, save_frame=False):
        """
        Capture frame, run inference, and return results
        
        Args:
            save_frame: Whether to save the captured frame
            
        Returns:
            tuple: (class_name, confidence, frame)
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
        class_idx, confidence = self.run_inference(preprocessed)
        
        # Get class name
        class_name = self.class_names[class_idx] if class_idx < len(self.class_names) else f"Unknown_Class_{class_idx}"
        
        return class_name, confidence, frame
    
    def cleanup(self):
        """Clean up camera resources"""
        if hasattr(self, 'picam2'):
            self.picam2.close()
            print("Camera closed")

def main():
    """Main function to run inference"""
    try:
        # Initialize classifier
        classifier = RecyclingClassifier()
        
        print("\n=== Recycling Classification System ===")
        print("Starting inference...")
        
        # Run classification
        class_name, confidence, frame = classifier.classify_current_view(save_frame=True)
        
        # Print results
        print(f"\n--- Classification Results ---")
        print(f"Predicted Class: {class_name}")
        print(f"Confidence: {confidence:.4f} ({confidence*100:.2f}%)")
        
        # Additional info
        if confidence > 0.8:
            print("🟢 High confidence prediction")
        elif confidence > 0.6:
            print("🟡 Medium confidence prediction") 
        else:
            print("🔴 Low confidence prediction")
            
    except Exception as e:
        print(f"Error during inference: {e}")
        
    finally:
        # Clean up
        if 'classifier' in locals():
            classifier.cleanup()

if __name__ == "__main__":
    main()
