from gpiozero import Servo
from time import sleep

myGPIO = 17
TIME_TO_OPEN = 5
servo = Servo(myGPIO)

servo.min()
sleep(0.5)
servo.mid()
sleep(0.5)
servo.max()
sleep(0.5)
sleep(TIME_TO_OPEN)
servo.mid()
sleep(0.5)
servo.min()
sleep(0.5)
