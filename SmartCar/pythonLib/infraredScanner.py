#!/usr/bin/env python  


import RPi.GPIO as GPIO 

class infraredScanner:
    def __init__(self, VCC_Pin, Signal_Pin):
        self.VCC_Pin = VCC_Pin
        self. Signal_Pin =  Signal_Pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(VCC_Pin, GPIO.OUT, initial=True)
        GPIO.output(VCC_Pin, GPIO.HIGH)
        GPIO.setup(Signal_Pin, GPIO.IN, initial=False)

    def IsBlocking(self):
        reading = GPIO.input(self.Signal_Pin)
        return reading == GPIO.LOW

    
