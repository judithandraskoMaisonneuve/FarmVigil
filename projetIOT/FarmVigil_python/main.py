import cv2
import time
import serial
from picamzero import Camera

ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

# Update this path to the correct location of the Haar cascade file
human_cascade = cv2.CascadeClassifier('/home/vincent/Desktop/FarmVigil/haarcascade_fullbody.xml')

def record_with_detection():
    cam = Camera()
    cam.start_preview()

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"/home/vincent/Desktop/FarmVigil/videos/new_{timestamp}.mp4"
    cam.record_video(filename, duration=10)

    # Initialize video capture for human detection
    cap = cv2.VideoCapture(filename)

    human_detected = False

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        humans = human_cascade.detectMultiScale(gray, 1.1, 4)

        if len(humans) > 0:
            human_detected = True
            print("Human detected! Stopping recording...")
            break

    cap.release()
    cam.stop_preview()
    
    return human_detected

while True:
    line = ser.readline().decode().strip()
    print(line)

    if "Distance:" in line:
        distance = int(line.split()[1])

        if distance <= 5:
            print("Object detected within 5 cm! Starting video recording...")
            if record_with_detection():
                print("Human detected. Not activating servo or buzzer.")
            else:
                print("No human detected. Activating servo or buzzer...")
                # Add your servo/buzzer activation code here

    time.sleep(0.5)

ser.close()
