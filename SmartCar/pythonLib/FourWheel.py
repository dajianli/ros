#!/usr/bin/env python  


import RPi.GPIO as GPIO
import sys

class FourWheel:
  def __init__(self, IN1, IN2, IN3, IN4):
     self.IN1 = IN1
     self.IN2 = IN2
     self.IN3 = IN3
     self.IN4 = IN4
     GPIO.setmode(GPIO.BCM)
     GPIO.setup(IN1, GPIO.OUT, initial=False)  
     GPIO.setup(IN2, GPIO.OUT, initial=False)
     GPIO.setup(IN3, GPIO.OUT, initial=False)  
     GPIO.setup(IN4, GPIO.OUT, initial=False)

     self.pwm_IN1 = GPIO.PWM(IN1, 100)
     self.pwm_IN2 = GPIO.PWM(IN2, 100)
     self.pwm_IN3 = GPIO.PWM(IN3, 100)
     self.pwm_IN4 = GPIO.PWM(IN4, 100)
     self.pwm_IN1.start(0)
     self.pwm_IN2.start(0)
     self.pwm_IN3.start(0)
     self.pwm_IN4.start(0)

  def front(self, speed, xWheel):
     lRatio,rRatio,lReverse,rReverse = wheelRotateRatio( xWheel )
     self.pwm_IN1.ChangeDutyCycle(speed * lRatio);
     self.pwm_IN3.ChangeDutyCycle(speed * rRatio);
     self.pwm_IN2.ChangeDutyCycle(speed * lReverse);
     self.pwm_IN4.ChangeDutyCycle(speed * rReverse);

     sys.stdout.write('front  ' + str(speed) + ', wheel ' + str(xWheel) + '\n') 
     sys.stdout.flush()

  def back(self, speed, xWheel):
     lRatio,rRatio,lReverse,rReverse = wheelRotateRatio( xWheel )
     self.pwm_IN1.ChangeDutyCycle(speed * lReverse);
     self.pwm_IN3.ChangeDutyCycle(speed * rReverse);
     self.pwm_IN2.ChangeDutyCycle(speed * lRatio);
     self.pwm_IN4.ChangeDutyCycle(speed * rRatio);

     sys.stdout.write('back  ' + str(speed) + ', wheel ' + str(xWheel) + '\n') 
     sys.stdout.flush()
  
  def stop(self):
     GPIO.output(self.IN1, False)  
     GPIO.output(self.IN2, False)

     self.pwm_IN1.ChangeDutyCycle(0);
     self.pwm_IN3.ChangeDutyCycle(0);
     self.pwm_IN2.ChangeDutyCycle(0);
     self.pwm_IN4.ChangeDutyCycle(0);

     sys.stdout.write('stop  \n')
     sys.stdout.flush() 
  
def wheelRotateRatio(xWheel):

    lRatio = 0
    rRatio = 0
    lReverse = 0
    rReverse = 0
    absWheel = abs(xWheel)
    if absWheel<=1 : #go straight, no turning the wheel
	lRatio = 1
	rRatio = 1
    elif absWheel == 2 : #light rurn
	if  xWheel > 0 :
	   lRatio = 1
	   rRatio = 0.5
	else:
	   lRatio = 0.5
	   rRatio = 1
    elif absWheel ==3 : #hard turn
	if xWheel > 0 :
	   lRatio = 1
	   rRatio = 0
	else:
	   lRatio = 0
	   rRatio = 1
    elif absWheel == 4: #turn without moving
	if xWheel > 0 :
	   lRatio = 1
	   rRatio = 0
           lReverse = 0
	   rReverse = 1
	else:
	   lRatio = 0
	   rRatio = 1
	   lReverse = 1
	   rReverse = 0
  
    return lRatio,rRatio,lReverse,rReverse

