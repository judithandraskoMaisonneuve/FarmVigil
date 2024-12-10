import cv2
import numpy as np
import mediapipe as mp
from picamera2 import Picamera2
import serial
import threading
import time

# Initialize serial communication
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(15)  # Wait for the Arduino to be ready

# Load YOLOv4 model and classes
net = cv2.dnn.readNet("yolo/yolov4-tiny.weights", "yolo/yolov4-tiny.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
with open("coco/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Initialize Mediapipe Hand Detection
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Initialize camera
camera = Picamera2()
camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))

# Global state variables
distance = float('inf')  # Initialize as a high value
buzzer_state = False
_led_state = False
camera_started = False
information = ""
stop_event = threading.Event()

# Load only the "person" class
classes = ["person"]

def detect_objects():
    global distance, _led_state, buzzer_state, camera_started, information
    try:
        while not stop_event.is_set():
            if distance <= 20.0 and not camera_started:
                _led_state = True
                ser.write(b'L')
                camera.start()  # Start the camera when the person is detected
                camera_started = True
                time.sleep(0.2)

            if distance > 20.0 and camera_started:
                _led_state = False
                ser.write(b'F')
                camera.stop()  # Stop the camera immediately
                camera_started = False
                cv2.destroyAllWindows()  # Close the window
                time.sleep(1)  # Cooldown time

            if camera_started:
                frame = camera.capture_array()
                if frame is None:
                    print("Failed to capture frame!")
                    continue

                # Display the frame in a window
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                
                # Mediapipe Hand Detection
                results = hands.process(frame_rgb)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_draw.draw_landmarks(frame_bgr, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                cv2.imshow("FarmVigil", frame_bgr)  # Show the live feed

                # Check for key press to close the preview
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                blob = cv2.dnn.blobFromImage(frame_bgr, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
                net.setInput(blob)
                outs = net.forward(output_layers)

                # Process detections
                person_detected = False
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]

                        # Check only for "person" class
                        if class_id == 0 and confidence > 0.5:  # 0 since "person" is the only item in classes
                            person_detected = True
                            break

                # Update buzzer based on detection
                if person_detected:
                    ser.write(b'P')
                    information = "PERSONNE DETECTE"
                    buzzer_state = False

                else:
                    ser.write(b'N')
                    information = "MOVEMENT DETECTE"
                    buzzer_state = True

                time.sleep(1)
    finally:
        _led_state = False
        buzzer_state = False
        if camera_started:
            camera.stop()
            cv2.destroyAllWindows()

def get_Info():
    return information

def set_led(state):
    global _led_state
    _led_state = state

def get_led():
    return _led_state

def get_distance():
    return distance

def set_buzzer_state(state):
    global buzzer_state
    buzzer_state = state

def get_buzzer_state():
    return buzzer_state

def read_serial():
    global distance
    while not stop_event.is_set():
        if ser.in_waiting > 0:
            data = ser.readline().decode().strip()
            try:
                # Assuming the serial input contains "distance:<value>", we split by ":" and parse the float
                print(data)  # Get the value after the colon
                distance = float(data)
            except (ValueError, IndexError):
                pass  # Ignore invalid data
        time.sleep(1)

# Start threads for object detection and serial communication
def start_threads():
    detection_thread = threading.Thread(target=detect_objects)
    serial_thread = threading.Thread(target=read_serial)
    detection_thread.start()
    serial_thread.start()
    return detection_thread, serial_thread

def stop_threads():
    stop_event.set()
    ser.close()

# Run the threads
if __name__ == "__main__":
    detection_thread, serial_thread = start_threads()

    try:
        while True:
            time.sleep(1)  # Main thread runs while detection and serial threads work
    except KeyboardInterrupt:
        stop_threads()
        hands.close()  # Close Mediapipe hands instance
        cv2.destroyAllWindows()
