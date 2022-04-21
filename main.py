import os
import cv2
from gpiozero import Device, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import socket
import asyncio
import board
import adafruit_mlx90614

Device.pin_factory = PiGPIOFactory()
TEMP_THRESHOLD = 'TEMP_THRESHOLD'
SERVO_PIN_NUMBER = 11
OPEN_TIME = 5
HOST = "192.168.2.73"
PORT = 12345
fn_haar = 'haarcascade_frontalface_default.xml'
size = 2
door_servo = Servo(SERVO_PIN_NUMBER)
servo_lock = False
i2c = board.I2C()
mlx = adafruit_mlx90614.MLX90614(i2c)


async def open_and_close_door(time_to_open):
    servo_lock = True
    door_servo.min()
    sleep(0.5)
    door_servo.mid()
    sleep(0.5)
    door_servo.max()
    sleep(0.5)
    sleep(time_to_open)
    door_servo.mid()
    sleep(0.5)
    door_servo.min()
    sleep(0.5)
    servo_lock = False


def main():
    haar_cascade = cv2.CascadeClassifier(fn_haar)
    webcam = cv2.VideoCapture(0)
    door_servo.min()

    while True:
        temp_threshold = float(os.getenv(TEMP_THRESHOLD, '38.0').lower())

        # TODO: Detect faces from camera
        rval = False
        while (not rval):
            # Put the image from the webcam into 'frame'
            (rval, frame) = webcam.read()
            if (not rval):
                print("Failed to open webcam. Trying again...")

        # Flip the image (optional)
        frame = cv2.flip(frame, 1, 0)

        # Convert to grayscalel
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Resize to speed up detection (optinal, change size above)
        mini = cv2.resize(
            gray, (int(gray.shape[1] / size), int(gray.shape[0] / size)))
        # Detect faces and loop through each one
        faces = haar_cascade.detectMultiScale(mini)

        if (len(faces) > 0):
            face_i = faces[0]
            #temp = mlx.object_temperature
            temp = 36.2

            # Coordinates of face after scaling back by `size`
            (x, y, w, h) = [v * size for v in face_i]

            # TODO: Draw box around faces and show it
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            # TODO: Print temperature on faces
            cv2.putText(frame, 'Temperature: {}'.format(temp), (x - 10, y - 10),
                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))

            noFever = True#(temp > 20.0 and temp < temp_threshold)

            if (noFever and not servo_lock):
                # TODO: Send signal to servo motor to unlock the door
                #asyncio.run(open_and_close_door(OPEN_TIME))
                print("Passed face temperature")
        else:
            print("No face detected")

        cv2.imshow("Face Temp Detection", frame)

        key = cv2.waitKey(3)

        if key == 27:
            break

    cv2.destroyAllWindows()
    webcam.release()


if __name__ == "__main__":
    main()
