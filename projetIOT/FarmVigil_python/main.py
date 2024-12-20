import cv2
import numpy as np
import mediapipe as mp
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

# Initialize MediaPipe hand tracking
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Load YOLOv4 weights, config, and coco.names
net = cv2.dnn.readNet("yolo/yolov4.weights", "yolo/yolov4.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Load coco.names (the classes YOLOv4 can detect)
with open("coco/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Initialize the PiCamera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 24
raw_capture = PiRGBArray(camera, size=(640, 480))

# Allow camera to warm up
time.sleep(0.1)

def count_fingers(hand_landmarks):
    fingers = []
    tip_ids = [4, 8, 12, 16, 20]

    # Thumb
    if (hand_landmarks.landmark[tip_ids[0]].y < hand_landmarks.landmark[tip_ids[0] - 1].y):
        fingers.append(1)
    else:
        fingers.append(0)

    # Fingers
    for id in range(1, 5):
        if (hand_landmarks.landmark[tip_ids[id]].y < hand_landmarks.landmark[tip_ids[id] - 2].y):
            fingers.append(1)
        else:
            fingers.append(0)

    return fingers.count(1)


for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # Grab the image array
    image = frame.array

    # Hand tracking
    results = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Count fingers
            num_fingers = count_fingers(hand_landmarks)
            print(f"L'utilisateur montre {num_fingers} doigts")

    # Prepare the frame for YOLO detection
    blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Post-processing: detect objects
    class_ids = []
    confidences = []
    boxes = []
    height, width, channels = image.shape

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # Set a threshold for detection
                # Get the coordinates of the bounding box
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates (x, y, w, h)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Apply non-maxima suppression to reduce overlapping boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    # Draw the detected bounding boxes
    if len(indexes) > 0:
        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            color = (0, 255, 0)  # Green for detected objects
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # Show the frame with detections and hand tracking
    cv2.imshow("Scarecrow Detection with Hand Tracking", image)

    # Clear the stream for the next frame
    raw_capture.truncate(0)

    # Exit loop on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
hands.close()