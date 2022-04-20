import board
import adafruit_mlx90614
import os
import cv2
from gpiozero import Servo
from time import sleep

TEMP_THRESHOLD = 'TEMP_THRESHOLD'
SERVO_PIN_NUMBER = 11
OPEN_TIME = 5
fn_haar = 'haarcascade_frontalface_default.xml'
i2c = board.I2C()
mlx = adafruit_mlx90614.MLX90614(i2c)
size = 2


def main():
    haar_cascade = cv2.CascadeClassifier(fn_haar)
    webcam = cv2.VideoCapture(0)
    servo = Servo(SERVO_PIN_NUMBER)
    servo.min()
    is_open = False

    while True:
        temp_threshold = float(os.getenv(TEMP_THRESHOLD, '38.0').lower())

        if is_open:
            sleep(5)
            is_open = False
            
            
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
        mini = cv2.resize(gray, (int(gray.shape[1] / size), int(gray.shape[0] / size)))
        # Detect faces and loop through each one
        faces = haar_cascade.detectMultiScale(mini)
        
        

        if (len(faces) > 0):
            face_i = faces[0]

            # Coordinates of face after scaling back by `size`
            (x, y, w, h) = [v * size for v in face_i]

            # TODO: Draw box around faces and show it
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            # TODO: Print temperature on faces
            cv2.putText(frame, 'Temperature: {}', (x - 10, y - 10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))

            noFever = (mlx.object_temperature < temp_threshold)

            if (noFever):
                # TODO: Send signal to servo motor to unlock the door
                servo.max()
                is_open = True
                print("Passed face temperature")
        else:
            print("No face detected")
            
        cv2.imshow("Face Temp Detection", frame)
        
        key = cv2.waitKey(3)
        
        if key == 27:
            break
    
    webcam.release()


if __name__ == "__main__":
    main()