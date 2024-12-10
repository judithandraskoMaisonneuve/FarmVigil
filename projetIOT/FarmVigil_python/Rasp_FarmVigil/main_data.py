import time
import aiot_camera
from aliot.aliot_obj import AliotObj

# Initialize Aliot object
Rasp_FarmVigil = AliotObj("Rasp_FarmVigil")

time.sleep(10)


def update_data():
    while True:
        try:
            # Fetching data from aiot_camera
            distance = aiot_camera.get_distance()
            buzzer_state = aiot_camera.get_buzzer_state()
            led_state = aiot_camera.get_led()
            information = aiot_camera.get_Info()
            
            if buzzer_state == True:
                Rasp_FarmVigil.update_component("BUZZER",200)
            
            # Printing the fetched data with context
            print(f"Distance: {distance} cm")  # Assuming distance is in centimeters
            print(f"Buzzer State: {'ON' if buzzer_state else 'OFF'}")  # Assuming True/False for buzzer state
            print(f"LED State: {'ON' if led_state else 'OFF'}")  # Assuming True/False for LED state
            print(f"logs: {information}")  # Assuming distance is in centimeters

            # Update the server with the data
            Rasp_FarmVigil.update_doc({
                "/doc/distance": distance,
                "/doc/buzzer_state": buzzer_state,
                "/doc/led": led_state
            })
            
            Rasp_FarmVigil.update_component("Detection",information)

            
            # Sleep for a second before the next update
            time.sleep(1)
        
        except Exception as e:
            # Print error with context if something goes wrong
            print(f"Error while updating data: {e}")
            time.sleep(1)


Rasp_FarmVigil.on_start(callback=update_data)

# Start detection and serial threads
detection_thread, serial_thread = aiot_camera.start_threads()

# Connect to server
try:
    Rasp_FarmVigil.run()
finally:
    aiot_camera.stop_threads()
    detection_thread.join()
    serial_thread.join()
