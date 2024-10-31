from picamzero import Camera
import time
import serial

ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

def record():
    cam = Camera()
    cam.start_preview()
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"/home/vincent/Desktop/FarmVigil/videos/new_{timestamp}.mp4"
    cam.record_video(filename, duration=10)
    print(f"Filming video: {filename}")
    cam.stop_preview()

while True:
    # Read distance from Arduino
    line = ser.readline().decode().strip()  # Read from Arduino
    print(line)  # Print the distance reading

    if "Distance:" in line:
        # Extract distance value
        distance = int(line.split()[1])  # Get the distance value

        # Check if distance is 5 cm or less
        if distance <= 5:
            print("Object detected within 5 cm! Starting video recording...")
            record()

    time.sleep(0.5)  # Add a small delay to avoid flooding the serial port

ser.close()
