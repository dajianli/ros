#!/usr/bin/env python  


import RPi.GPIO as GPIO
import time
import signal
import atexit

atexit.register(GPIO.cleanup)  

servopin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(servopin, GPIO.OUT, initial=False)
p = GPIO.PWM(servopin,50) #50HZ
p.start(0)
time.sleep(2)

while(True):
  for i in range(0,181,10):
    p.ChangeDutyCycle(2.5 + 10 * i / 180)
    time.sleep(0.02)
    p.ChangeDutyCycle(0)
    time.sleep(0.5)
  for i in range(181,0,-10):
    p.ChangeDutyCycle(2.5 + 10 * i / 180)
    time.sleep(0.02)
    p.ChangeDutyCycle(0)
    time.sleep(0.5)

'''
p.ChangeDutyCycle(2.5)
time.sleep(0.02)
time.sleep(2)

p.ChangeDutyCycle(7.5)
time.sleep(0.02)
time.sleep(2)

p.ChangeDutyCycle(12.5)
time.sleep(0.02)
time.sleep(2)
'''

#def servopulse(int angle):
#  int pulsewidth=(angle*11)+500   
#  GPIO.write(servopin,HIGH)     
#  time.delay(pulsewidth)   
#  GPIO.write((servopin,LOW)    
#  time.delay(20000-pulsewidth)



