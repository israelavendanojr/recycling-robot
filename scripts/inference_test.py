# scripts/inference_test.py
import os, sys, time, cv2, torch, torch.nn.functional as F
from torchvision import transforms
from picamera2 import Picamera2
from PIL import Image
import numpy as np

# Allow "from src..." imports if your project has them
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

try:
    from src.classifier.model import get_model
    from src.classifier.utils import get_transforms
except Exception as e:
    print(f"Warning: Could not import project modules: {e}")
    print("Falling back to standalone model loading...")
    get_model = None
    get_transforms = None


class RecyclingClassifier:
    def __init__(self, model_path="recycler.pth", device=None, input_size=224, cam_size=(640, 480)):
        self.device = device if device else ('cuda' if torch.cuda.is_available() else 'cpu')
        self.input_size = input_size
        print(f"Using device: {self.device}")

        self.class_names = ['cardboard', 'glass', 'metal', 'plastic', 'trash']

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")

        self.model = self._load_model(model_path)
        self.model.eval()

        self.transform = self._get_transforms()
        self._setup_camera(cam_size)

    def _load_model(self, model_path):
        try:
            checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
            print(f"Loaded checkpoint type: {type(checkpoint)}")

            # State dict only
            if isinstance(checkpoint, dict) and 'state_dict' not in checkpoint:
                if get_model is not None:
                    model = get_model(num_classes=len(self.class_names))
                else:
                    from torchvision import models
                    from torch import nn
                    model = models.mobilenet_v2(weights=None)
                    model.classifier[1] = nn.Linear(model.last_channel, len(self.class_names))
                model.load_state_dict(checkpoint)
                model.to(self.device)
                print("✓ Loaded model from state dict")
                return model

            # Checkpoint with 'state_dict'
            if isinstance(checkpoint, dict) and 'state_dict' in checkpoint:
                state = checkpoint['state_dict']
                if get_model is not None:
                    model = get_model(num_classes=len(self.class_names))
                else:
                    from torchvision import models
                    from torch import nn
                    model = models.mobilenet_v2(weights=None)
                    model.classifier[1] = nn.Linear(model.last_channel, len(self.class_names))
                model.load_state_dict(state)
                model.to(self.device)
                print("✓ Loaded model from checkpoint state dict")
                return model

            # Full model object
            checkpoint.to(self.device)
            print("✓ Loaded complete PyTorch model")
            return checkpoint

        except Exception as e:
            print(f"✗ Error loading model: {e}")
            raise

    def _get_transforms(self):
        if get_transforms is not None:
            _, t = get_transforms(input_size=self.input_size)
            return t
        return transforms.Compose([
            transforms.Resize((self.input_size, self.input_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])

    def _setup_camera(self, size):
        try:
            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration(
                {"format": "RGB888", "size": size}
            ))
            self.picam2.start()
            time.sleep(0.4)
            print("✓ Camera initialized")
        except Exception as e:
            print(f"✗ Failed to initialize camera: {e}")
            raise

    def capture_frame(self):
        # Returns RGB ndarray (H,W,3)
        return self.picam2.capture_array()

    def preprocess_frame(self, frame_rgb):
        pil = Image.fromarray(frame_rgb)
        x = self.transform(pil).unsqueeze(0).to(self.device)
        return x

    @torch.inference_mode()
    def run_inference(self, x):
        logits = self.model(x)
        if isinstance(logits, (list, tuple)):
            logits = logits[0]
        probs = F.softmax(logits, dim=1)
        conf, idx = torch.max(probs, dim=1)
        return idx.item(), float(conf.item()), probs[0].detach().cpu().numpy()

    def classify_current_view(self, save_frame=False, show_all_probs=False):
        frame = self.capture_frame()
        if save_frame:
            cv2.imwrite("latest_capture.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            print("Saved latest_capture.jpg")
        x = self.preprocess_frame(frame)
        idx, conf, probs = self.run_inference(x)
        name = self.class_names[idx] if idx < len(self.class_names) else f"Unknown_{idx}"

        out = {
            "predicted_class": name,
            "class_index": idx,
            "confidence": conf,
            "frame": frame,
        }
        if show_all_probs:
            out["all_probabilities"] = {self.class_names[i]: float(probs[i])
                                        for i in range(min(len(probs), len(self.class_names)))}
        return out

    def create_overlay_text(self, frame_rgb, results):
        img = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
        conf = results["confidence"]
        color = (0, 255, 0) if conf > 0.8 else ((0, 165, 255) if conf > 0.6 else (0, 0, 255))
        text = f"{results['predicted_class']} ({conf*100:.1f}%)"
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        cv2.rectangle(img, (10, 10), (tw + 20, th + 30), (0, 0, 0), -1)
        cv2.putText(img, text, (15, 35), font, scale, color, thick, cv2.LINE_AA)

        if 'all_probabilities' in results:
            y = 70
            for cls, p in results['all_probabilities'].items():
                t = f"{cls}: {p*100:.1f}%"
                c = color if cls == results['predicted_class'] else (255, 255, 255)
                cv2.putText(img, t, (15, y), font, 0.5, c, 1, cv2.LINE_AA)
                y += 22

        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(img, ts, (15, img.shape[0] - 15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def cleanup(self):
        if hasattr(self, "picam2"):
            self.picam2.close()
            print("✓ Camera closed")


def test_display_capability():
    print("Testing display capability...")
    if 'DISPLAY' not in os.environ:
        print("✗ DISPLAY not set")
        return False
    try:
        img = np.zeros((240, 320, 3), np.uint8)
        cv2.putText(img, "Display OK - press any key", (15, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("test", img); cv2.waitKey(0); cv2.destroyAllWindows()
        print("✓ Display working")
        return True
    except Exception as e:
        print(f"✗ Cannot create window: {e}")
        return False


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Recycling Classification Inference")
    parser.add_argument('--model_path', type=str, default='recycler.pth')
    parser.add_argument('--test_model', action='store_true')
    parser.add_argument('--test_display', action='store_true')
    parser.add_argument('--single_shot', action='store_true')
    parser.add_argument('--display', action='store_true', help='X11 display with cv2.imshow')
    parser.add_argument('--console_only', action='store_true')
    parser.add_argument('--delay', type=float, default=2.0)
    parser.add_argument('--save_frames', action='store_true')
    # web dashboard flags (served by scripts/web_dashboard.py)
    parser.add_argument('--web', action='store_true', help='Serve dashboard at http://<pi-ip>:8000')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--infer_hz', type=float, default=5.0)
    parser.add_argument('--cam_w', type=int, default=640)
    parser.add_argument('--cam_h', type=int, default=480)
    args = parser.parse_args()

    print("=" * 60)
    print("Recycling Classification System")
    print("=" * 60)

    if args.test_display:
        test_display_capability()
        return

    if args.test_model:
        # Quick load test on CPU only
        try:
            _ = torch.load(args.model_path, map_location='cpu', weights_only=False)
            print("✓ Model file can be loaded on CPU")
        except Exception as e:
            print(f"✗ Model load failed: {e}")
        return

    classifier = None
    try:
        classifier = RecyclingClassifier(
            model_path=args.model_path,
            input_size=224,
            cam_size=(args.cam_w, args.cam_h)
        )
        print("✓ Classifier initialized")

        if args.web:
            print(f"\n🌐 Starting web dashboard on 0.0.0.0:{args.port}")
            print("Open: http://<pi-ip>:%d/" % args.port)
            # Import here to avoid Flask dependency unless needed
            from web_dashboard import start_web_dashboard
            start_web_dashboard(classifier, host="0.0.0.0", port=args.port, infer_hz=args.infer_hz)
            return

        if args.single_shot:
            res = classifier.classify_current_view(save_frame=args.save_frames, show_all_probs=True)
            print("\n--- Classification Results ---")
            print(f"Predicted Class: {res['predicted_class']}")
            print(f"Confidence: {res['confidence']:.4f} ({res['confidence']*100:.2f}%)")
            if 'all_probabilities' in res:
                print("\nAll class probabilities:")
                for k, v in res['all_probabilities'].items():
                    print(f"  {k}: {v:.4f} ({v*100:.2f}%)")
            return

        if args.display and not args.console_only:
            print("\n🖥️  X11 display mode...")
            print(f"DISPLAY={os.environ.get('DISPLAY','<not set>')}")
            if test_display_capability():
                print("Press 'q' to quit.")
                last_t = 0.0
                while True:
                    frame = classifier.capture_frame()
                    t = time.time()
                    if t - last_t >= max(0.1, args.delay):
                        x = classifier.preprocess_frame(frame)
                        idx, conf, probs = classifier.run_inference(x)
                        name = classifier.class_names[idx]
                        last_t = t
                        res = {
                            "predicted_class": name,
                            "class_index": idx,
                            "confidence": conf,
                            "frame": frame,
                            "all_probabilities": {classifier.class_names[i]: float(probs[i])
                                                  for i in range(min(len(probs), len(classifier.class_names)))}
                        }
                    disp = classifier.create_overlay_text(frame, res if 'res' in locals() else {
                        "predicted_class":"...", "confidence":0.0, "frame":frame})
                    cv2.imshow("Recycling Classifier", disp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                cv2.destroyAllWindows()
            else:
                print("Display test failed. Falling back to console mode...")

        # Console mode (continuous)
        print("\n=== Console Mode (Ctrl+C to stop) ===")
        while True:
            res = classifier.classify_current_view(save_frame=args.save_frames, show_all_probs=True)
            print(f"{time.strftime('%H:%M:%S')} | {res['predicted_class']} "
                  f"({res['confidence']*100:.1f}%)")
            time.sleep(args.delay)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if classifier:
            classifier.cleanup()
        print("✓ Cleanup completed")


if __name__ == "__main__":
    main()
