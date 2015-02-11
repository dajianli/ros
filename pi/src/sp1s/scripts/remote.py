#!/usr/bin/env python  
# -*- coding: utf-8 -*-  
# @Author: darkbull  
# @Date:   2014-05-31 10:02:21  
# @Last Modified by:   darkbull  
# @Last Modified time: 2014-06-02 15:29:20  
  
"""Web控制树莓派小车 
"""
import rospy
from sp1s.msg import wheels_dir
import tornado.web  
import tornado.ioloop  
import codecs  
import RPi.GPIO as GPIO  
import time  
import atexit  
import math
import os
import thread

g_lForward = True
g_rForward = True

atexit.register(GPIO.cleanup)  

  
GPIO.setmode(GPIO.BCM)  

IN1 = 18  
IN2 = 23  
IN3 = 25
IN4 = 24
  
GPIO.setup(IN1, GPIO.OUT, initial=False)  
GPIO.setup(IN2, GPIO.OUT, initial=False)
GPIO.setup(IN3, GPIO.OUT, initial=False)  
GPIO.setup(IN4, GPIO.OUT, initial=False)

pwm_IN1 = GPIO.PWM(IN1, 100)
pwm_IN2 = GPIO.PWM(IN2, 100)
pwm_IN3 = GPIO.PWM(IN3, 100)
pwm_IN4 = GPIO.PWM(IN4, 100)
pwm_IN1.start(0)
pwm_IN2.start(0)
pwm_IN3.start(0)
pwm_IN4.start(0)

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

def front(speed,xWheel):  

    lRatio,rRatio,lReverse,rReverse = wheelRotateRatio( xWheel )
    pwm_IN1.ChangeDutyCycle(speed * lRatio);
    pwm_IN3.ChangeDutyCycle(speed * rRatio);
    pwm_IN2.ChangeDutyCycle(speed * lReverse);
    pwm_IN4.ChangeDutyCycle(speed * rReverse);
    global g_lForward
    global g_rForward
    g_lForward = True
    g_rForward = True
    if lReverse == 1:
	g_lForward = False
    elif rReverse == 1:
	g_rForward = False

def back(speed,xWheel):  

    lRatio,rRatio,lReverse,rReverse = wheelRotateRatio( xWheel )
    pwm_IN1.ChangeDutyCycle(speed * lReverse);
    pwm_IN3.ChangeDutyCycle(speed * rReverse);
    pwm_IN2.ChangeDutyCycle(speed * lRatio);
    pwm_IN4.ChangeDutyCycle(speed * rRatio);
    global g_lForward
    global g_rForward
    g_lForward = False
    g_rForward = False
    if lReverse == 1:
	g_lForward = True
    elif rReverse == 1:
	g_rForward = True
  
def stop():  
    GPIO.output(IN1, False)  
    GPIO.output(IN2, False)

    pwm_IN1.ChangeDutyCycle(0);
    pwm_IN3.ChangeDutyCycle(0);
    pwm_IN2.ChangeDutyCycle(0);
    pwm_IN4.ChangeDutyCycle(0);

    sys.stdout.write('stop  \n')
    sys.stdout.flush() 
  
  
def run(wheel, y, r, division):   
  
    speed_min = 40  
    speed_max = 100  
    speed_delta = ( speed_max - speed_min  ) / ( division - 1 )
    # x轴 wheel 为小车的运行方向，速度以y轴的值为基准 
    
    yShift = int(math.ceil( abs ( y ) * division / r ) )
    speed = speed_min + speed_delta * yShift
    speed =  abs ( speed ) 
    if y < 0:  
        back(speed,wheel)  
    elif y > 0:  
        front(speed,wheel)  
    else:  
        stop()  
        return  


class CarController(tornado.web.RequestHandler):  
    def get(self):  
        with codecs.open('webcar.html', 'r', encoding='utf-8') as fp:  
            self.write(fp.read())  
  
    def post(self):  
        r = int(self.get_argument('r'))  
        wheel = int(self.get_argument('wheel'))  
        y = int(self.get_argument('y'))
        division = int(self.get_argument('division'))
        run(wheel, y, r, division)  
        self.write('ok')

def webservice_thread():

    app = tornado.web.Application([('/', CarController)], static_path='static', debug=True)  
    app.listen(8080)
    tornado.ioloop.IOLoop.instance().start()   

if __name__ == '__main__':
    rospy.init_node('remote', anonymous=True)
    engaged = rospy.get_param('remote', True)
    if engaged == False:
      sys.exit(0) 

    wheels_dir_pub = rospy.Publisher('wheels_dir', wheels_dir, queue_size=10)    
    print os.getcwd()

    thread.start_new_thread(webservice_thread, ())  
    
    lForward = True
    rForward = True
    msg = wheels_dir()
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
         if g_lForward != lForward or g_rForward != rForward:           
           msg.left_forward = g_lForward
           msg.right_forward = g_rForward    
           wheels_dir_pub.publish(msg) 
           rForward = g_rForward
           lForward = g_lForward

         rate.sleep()
   
    
       
    
