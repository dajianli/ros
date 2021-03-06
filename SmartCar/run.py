#!/usr/bin/env python

import sys
import time
from pythonLib import infraredScanner
from pythonLib import FourWheel

IN1 = 15  
IN2 = 18  
IN3 = 23
IN4 = 24

#control the car with two infrared detected on front left and front right
wheelControl = FourWheel.FourWheel(IN1,IN2,IN3,IN4)
leftBlock = infraredScanner.infraredScanner(17,4)
rightBlock = infraredScanner.infraredScanner(12,16)
wheelControl.front(100, 0)
turnwheeltime = 0.8

while(True):
  
  IsLeftBlocking = False
  IsRightBlocking = False
  
  if(leftBlock.IsBlocking()):
    IsLeftBlocking = True
    sys.stdout.write('left blocked \n') 
    sys.stdout.flush()
     
  if(rightBlock.IsBlocking()):
    IsRightBlocking = True
    sys.stdout.write('right blocked \n')
    sys.stdout.flush()
  if(IsLeftBlocking and IsRightBlocking):
    # Blocked both sides, move backward a little bit and turn right 
    wheelControl.back(100,0)
    time.sleep(turnwheeltime)
    wheelControl.front(100, 4)
    time.sleep(turnwheeltime)
  elif IsLeftBlocking:
    # Blocked on the left, turn right
    wheelControl.front(100, 4)
    time.sleep(turnwheeltime)
  elif IsRightBlocking:
    # Blocked on the right, turn left
    wheelControl.front(100, -4)
    time.sleep(turnwheeltime)
  else:
    wheelControl.front(100, 0)
  
  time.sleep(0.01)

