from picamera2 import Picamera2
import cv2, time

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration({"format":"RGB888","size":(1280,720)}))
picam2.start(); time.sleep(0.5)
frame = picam2.capture_array()                # RGB image
print("Captured:", frame.shape)
cv2.imwrite("frame.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
picam2.close()
