from picamera2 import Picamera2
import time
import numpy as np
import cv2

# Initialize the camera
camera = Picamera2()

# Use a simple still image configuration for the preview (video preview mode)
camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))

# Start the camera preview
camera.start()

# Create a window for the preview using OpenCV
cv2.namedWindow("Camera Preview", cv2.WINDOW_NORMAL)

# Run the preview loop
try:
    while True:
        # Capture a frame from the camera
        frame = camera.capture_array()

        # Display the frame in the OpenCV window
        cv2.imshow("Camera Preview", frame)

        # Check for a key press to exit the preview
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    pass

# Stop the camera and close the window
camera.stop()
cv2.destroyAllWindows()
