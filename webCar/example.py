# The original is aboudou ,the Source code is here : https://goddess-gate.com/dc2/index.php/pages/raspiledmeter.en
# The modifier is ukonline2000
#!/usr/bin/python
import RPi.GPIO as GPIO
import time
import signal
import atexit

atexit.register(GPIO.cleanup)  
  
GPIO.setmode(GPIO.BCM)  
  
IN1 = 15  
IN2 = 18
IN3 = 23  
IN4 = 24  

GPIO.setup(IN1, GPIO.OUT, initial=False)  
GPIO.setup(IN2, GPIO.OUT, initial=False)
GPIO.setup(IN3, GPIO.OUT, initial=False)  
GPIO.setup(IN4, GPIO.OUT, initial=False)
pwm_IN1 = GPIO.PWM(IN1, 100)
pwm_IN3 = GPIO.PWM(IN3, 100)
pwm_IN1.start(0)
pwm_IN3.start(0)

while True:
  i = 30
  while i <= 100: 
    pwm_IN1.ChangeDutyCycle(i);
    pwm_IN3.ChangeDutyCycle(i);
    i += 10
    time.sleep(2)
