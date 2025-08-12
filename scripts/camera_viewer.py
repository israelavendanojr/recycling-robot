#!/usr/bin/env python3
"""
Simple camera viewer with live classification overlay
Works locally on the Pi with a connected display
"""

import cv2
import torch
import torch.nn.functional as F
from torchvision import transforms
from picamera2 import Picamera2
from PIL import Image
import numpy as np
import time
import os
from torchvision import models
from torch import nn

class SimpleCameraViewer:
    def __init__(self, model_path="models/recycler.pth"):
        self.device = 'cpu'
        self.class_names = ['cardboard', 'glass', 'metal', 'plastic', 'trash']
        
        # Load model
        self.model = self.load_model(model_path)
        
        # Setup transforms
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        
        # Initialize camera
        self.setup_camera()
        
        print("Camera viewer initialized!")
        print("Controls:")
        print("  Press 'q' to quit")
        print("  Press 's' to save current frame")
        print("  Press 'p' to pause/unpause classification")
        
    def load_model(self, model_path):
        """Load the PyTorch model"""
        checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
        
        if isinstance(checkpoint, dict):
            model = models.mobilenet_v2(weights=None)
            model.classifier[1] = nn.Linear(model.last_channel, len(self.class_names))
            model.load_state_dict(checkpoint)
        else:
            model = checkpoint
            
        model.eval()
        return model
    
    def setup_camera(self):
        """Initialize camera"""
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration({
            "format": "RGB888", 
            "size": (640, 480)
        }))
        self.picam2.start()
        time.sleep(0.5)
    
    def classify_frame(self, frame):
        """Classify the current frame"""
        pil_image = Image.fromarray(frame)
        input_tensor = self.transform(pil_image).unsqueeze(0)
        
        with torch.no_grad():
            outputs = self.model(input_tensor)
            probabilities = F.softmax(outputs, dim=1)
            confidence, predicted = torch.max(probabilities, 1)
            
            return {
                'class': self.class_names[predicted.item()],
                'confidence': float(confidence.item()),
                'probabilities': probabilities[0].numpy()
            }
    
    def add_overlay(self, frame, result):
        """Add classification results to frame"""
        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Create semi-transparent overlay background
        overlay = frame_bgr.copy()
        cv2.rectangle(overlay, (10, 10), (450, 150), (0, 0, 0), -1)
        frame_bgr = cv2.addWeighted(frame_bgr, 0.7, overlay, 0.3, 0)
        
        # Add text
        cv2.putText(frame_bgr, f"Prediction: {result['class'].upper()}", 
                   (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        confidence_pct = result['confidence'] * 100
        cv2.putText(frame_bgr, f"Confidence: {confidence_pct:.1f}%", 
                   (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Confidence indicator
        if confidence_pct > 80:
            color = (0, 255, 0)  # Green
            status = "HIGH CONFIDENCE"
        elif confidence_pct > 60:
            color = (0, 255, 255)  # Yellow  
            status = "MEDIUM CONFIDENCE"
        else:
            color = (0, 0, 255)  # Red
            status = "LOW CONFIDENCE"
            
        cv2.putText(frame_bgr, status, 
                   (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Add top 3 predictions
        y_offset = 130
        sorted_indices = np.argsort(result['probabilities'])[::-1]
        for i, idx in enumerate(sorted_indices[:3]):
            if i == 0:
                continue  # Skip the top prediction (already shown)
            prob_pct = result['probabilities'][idx] * 100
            cv2.putText(frame_bgr, f"{self.class_names[idx]}: {prob_pct:.1f}%", 
                       (20, y_offset + i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                       (200, 200, 200), 1)
        
        return frame_bgr
    
    def run(self):
        """Main display loop"""
        classify_active = True
        last_classification_time = 0
        current_result = {
            'class': 'Loading...',
            'confidence': 0.0,
            'probabilities': np.zeros(len(self.class_names))
        }
        
        try:
            while True:
                # Get frame
                frame = self.picam2.capture_array()
                
                # Classify every 1 second to avoid lag
                current_time = time.time()
                if classify_active and (current_time - last_classification_time > 1.0):
                    try:
                        current_result = self.classify_frame(frame)
                        last_classification_time = current_time
                    except Exception as e:
                        print(f"Classification error: {e}")
                
                # Add overlay
                display_frame = self.add_overlay(frame, current_result)
                
                # Add instructions at bottom
                cv2.putText(display_frame, "Controls: 'q'=quit, 's'=save, 'p'=pause classification", 
                           (10, display_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.4, (255, 255, 255), 1)
                
                # Display
                cv2.imshow('Recycling Robot Camera', display_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    # Save current frame
                    filename = f"capture_{int(time.time())}.jpg"
                    cv2.imwrite(filename, display_frame)
                    print(f"Frame saved as {filename}")
                elif key == ord('p'):
                    # Toggle classification
                    classify_active = not classify_active
                    status = "ENABLED" if classify_active else "PAUSED"
                    print(f"Classification {status}")
                    
        except KeyboardInterrupt:
            print("Interrupted by user")
            
        finally:
            cv2.destroyAllWindows()
            self.picam2.close()
            print("Camera viewer stopped")

def main():
    print("=" * 50)
    print("Simple Camera Viewer with Classification")
    print("=" * 50)
    
    # Check if display is available
    if not os.environ.get('DISPLAY'):
        print("⚠️  No display detected!")
        print("For headless Pi, use SSH with X forwarding:")
        print("ssh -X pi@your_pi_ip")
        print("Or use VNC for remote desktop")
        return
    
    viewer = SimpleCameraViewer()
    viewer.run()

if __name__ == '__main__':
    main()