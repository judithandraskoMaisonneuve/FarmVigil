import cv2
import numpy as np
from picamera2 import Picamera2
import serial
import time
import threading

# Initialize serial communication (replace '/dev/ttyACM0' with your actual serial port if necessary)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

time.sleep(15)

# Load YOLOv4 weights, config, and coco.names
net = cv2.dnn.readNet("yolo/yolov4-tiny.weights", "yolo/yolov4-tiny.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Load coco.names (the classes YOLOv4 can detect)
with open("coco/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Initialize the PiCamera using Picamera2
camera = Picamera2()
camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))

# Flag for clean thread termination
stop_event = threading.Event()
distance = float('inf')  # Initialize distance as infinite
buzzer_state = 0  # Initialize buzzer state as 0 (off)
_led_state = False
def detect_objects():
    global distance
    try:
        camera_started = False  # Flag to track whether the camera is started
        while not stop_event.is_set():
            # Only open the camera if the distance is 50 or less
            if distance <= 50 and not camera_started:
                camera.start()
                camera_started = True
                print("Camera started.")
                time.sleep(0.2)  # Allow camera to warm up

            # Capture a frame from the camera
            if camera_started:
                set_led(True)
                frame = camera.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

                # Prepare the frame for YOLO detection
                blob = cv2.dnn.blobFromImage(frame, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
                net.setInput(blob)
                outs = net.forward(output_layers)

                # Post-processing: detect objects
                class_ids, confidences, boxes = [], [], []
                height, width, _ = frame.shape

                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        
                        # Filter for 'person' class with high confidence
                        if confidence > 0.5 and classes[class_id] == "person":
                            center_x = int(detection[0] * width)
                            center_y = int(detection[1] * height)
                            w = int(detection[2] * width)
                            h = int(detection[3] * height)
                            x = int(center_x - w / 2)
                            y = int(center_y - h / 2)

                            boxes.append([x, y, w, h])
                            confidences.append(float(confidence))
                            class_ids.append(class_id)

                # Apply non-maxima suppression to reduce overlapping boxes
                indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
                person_detected = False  # Flag to check if any person is detected

                if len(indexes) > 0:
                    for i in indexes.flatten():
                        x, y, w, h = boxes[i]
                        if class_ids[i] < len(classes):
                            label = str(classes[class_ids[i]])
                            if label == "person":
                                person_detected = True
                            color = (0, 255, 0)  # Green for detected persons
                            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

                # Update buzzer state based on detection
                if person_detected or (distance > 50 and camera_start):
                    ser.write(b'P')
                    # Stop the camera when a person is detected and reset the flag
                    camera.stop()
                    camera_started = False
                    cv2.destroyAllWindows()
                    time.sleep(30)
                    set_led(False)
                else:
                    ser.write(b'N')
                
                # Show the frame with detections
                cv2.imshow("FarmVigil", frame)

                # Exit loop on 'q' key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_event.set()
                    set_led(False)
                    break

    finally:
        camera.stop()
        cv2.destroyAllWindows()

def set_led(state):
    global _led_state
    _led_state = state

def get_led():
    return _led_state
        
def get_distance():
    return distance

def get_buzzer_state():
    return buzzer_state

def read_serial():
    global distance, buzzer_state
    try:
        while not stop_event.is_set():
            if ser.in_waiting > 0:
                data = ser.readline().decode().strip()
                try:
                    # Extract distance and buzzer state values from the serial data
                    distance_str, buzzer_state_str = data.split(":")
                    distance = float(distance_str)
                    buzzer_state = int(buzzer_state_str)
                except (ValueError, IndexError):
                    pass  # Handle cases where the data format is incorrect
            time.sleep(1)  # This should be indented to the main loop
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

# Start the detection and serial reading in separate threads
try:
    detection_thread = threading.Thread(target=detect_objects)
    serial_thread = threading.Thread(target=read_serial)

    detection_thread.start()
    serial_thread.start()

    # Wait for both threads to complete
    detection_thread.join()
    serial_thread.join()
except KeyboardInterrupt:
        print("Exiting...")
