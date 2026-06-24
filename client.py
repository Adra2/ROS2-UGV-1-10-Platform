import socket
import time
import keyboard  # pip install keyboard

PI_IP = "104.194.97.117"
PORT = 9999

THROTTLE_MAX = 1.0
STEER_MAX = 1.0
SEND_RATE_HZ = 20  # how many times per second we send a command

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((PI_IP, PORT))
print("Connected to the Pi. Use W/S for throttle, A/D for steering. ESC to quit.")

try:
    while True:
        throttle = 0.0
        steer = 0.0

        if keyboard.is_pressed('w'):
            throttle = THROTTLE_MAX
        elif keyboard.is_pressed('s'):
            throttle = -THROTTLE_MAX

        if keyboard.is_pressed('a'):
            steer = -STEER_MAX
        elif keyboard.is_pressed('d'):
            steer = STEER_MAX

        msg = f"{steer},{throttle}\n"
        client.sendall(msg.encode())

        if keyboard.is_pressed('esc'):
            print("Exiting.")
            break

        time.sleep(1 / SEND_RATE_HZ)

finally:
    # Send a final stop command before closing, just in case
    client.sendall(b"0.0,0.0\n")
    client.close()