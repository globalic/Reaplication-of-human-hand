import RPi.GPIO as GPIO
import time

servoPIN = 17
servoPIN2 = 18
servoPIN3 = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
GPIO.setup(servoPIN2, GPIO.OUT)
GPIO.setup(servoPIN3, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(5) # Initialization
p2 = GPIO.PWM(servoPIN2, 50) # GPIO 18 for PWM with 50Hz
p2.start(5) # Initialization
p3 = GPIO.PWM(servoPIN3, 50) # GPIO 27 for PWM with 50Hz
p3.start(5) # Initialization
try:
  while True:
    p3.ChangeDutyCycle(6.0)
    time.sleep(1)
    p3.ChangeDutyCycle(3.0)
    time.sleep(1)
    print("Cycle completed !! ")
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()
