#!/usr/bin/python

import time
import RPi.GPIO as GPIO
import signal
import atexit

atexit.register(GPIO.cleanup)  

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define GPIO to use on Pi
GPIO_ECHO    = 0
GPIO_TRIGGER = 5
#GPIO_VCC     = 6

print "Ultrasonic Measurement"

# Set pins as output and input 
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger 

GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo

#GPIO.setup(GPIO_VCC,GPIO.OUT)      # VCC
#GPIO.output(GPIO_VCC, GPIO.HIGH)

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, GPIO.LOW)

# Allow module to settle 
time.sleep(0.5)

while True:
# Send 10us pulse to trigger
  GPIO.output(GPIO_TRIGGER, GPIO.HIGH)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER, GPIO.LOW)
  start = time.time()

  while GPIO.input(GPIO_ECHO)==0:
    start = time.time()

  while GPIO.input(GPIO_ECHO)==1:
    stop = time.time()

  # Calculate pulse length
  elapsed = stop-start

  # Distance pulse travelled in that time is time
  # multiplied by the speed of sound (cm/s)
  distance = elapsed * 34300

  # That was the distance there and back so halve the value
  distance = distance / 2
  if distance > 400 :
    continue

  print "Distance : %.1f cm" % distance 
  time.sleep(0.5)

