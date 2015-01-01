#!/usr/bin/env python  
# -*- coding: utf-8 -*-  
# @Author: darkbull  
# @Date:   2014-05-31 10:02:21  
# @Last Modified by:   darkbull  
# @Last Modified time: 2014-06-02 15:29:20  
  
"""Web控制树莓派小车 
"""  
import tornado.web  
import tornado.ioloop  
import codecs  
#import RPi.GPIO as GPIO  
import time  
import atexit  
import math
import sys

##atexit.register(GPIO.cleanup)  
##
##  
##GPIO.setmode(GPIO.BCM)  
##
##IN1 = 15  
##IN2 = 18  
##IN3 = 23
##IN4 = 24
##  
##GPIO.setup(IN1, GPIO.OUT, initial=False)  
##GPIO.setup(IN2, GPIO.OUT, initial=False)
##GPIO.setup(IN3, GPIO.OUT, initial=False)  
##GPIO.setup(IN4, GPIO.OUT, initial=False)
##
##pwm_IN1 = GPIO.PWM(IN1, 100)
##pwm_IN2 = GPIO.PWM(IN2, 100)
##pwm_IN3 = GPIO.PWM(IN3, 100)
##pwm_IN4 = GPIO.PWM(IN4, 100)
##pwm_IN1.start(0)
##pwm_IN2.start(0)
##pwm_IN3.start(0)
##pwm_IN4.start(0)
 
def front(speed):  
#    GPIO.output(IN1, True)  
#    GPIO.output(IN2, False)

##    pwm_IN1.ChangeDutyCycle(speed);
##    pwm_IN3.ChangeDutyCycle(speed);
##    pwm_IN2.ChangeDutyCycle(0);
##    pwm_IN4.ChangeDutyCycle(0);

    sys.stdout.write('front  ' + str(speed) + '\n') 
    sys.stdout.flush()

def back(speed):  
#    GPIO.output(IN1, False)  
#    GPIO.output(IN2, True)

##    pwm_IN1.ChangeDutyCycle(0);
##    pwm_IN3.ChangeDutyCycle(0);
##    pwm_IN2.ChangeDutyCycle(speed);
##    pwm_IN4.ChangeDutyCycle(speed);

    sys.stdout.write('back  ' + str(speed) + '\n') 
    sys.stdout.flush()
  
# def left():  
#     # GPIO.output(IN1, False)  
#     # GPIO.output(IN2, True)  
#     GPIO.output(IN1, False)  
#     GPIO.output(IN2, False)  
#     GPIO.output(IN3, True)  
#     GPIO.output(IN4, False)  
  
# def right():  
#     # GPIO.output(IN1, True)  
#     # GPIO.output(IN2, False)  
#     GPIO.output(IN1, False)  
#     GPIO.output(IN2, False)  
#     GPIO.output(IN3, False)  
#     GPIO.output(IN4, True)  
  
def stop():  
#    GPIO.output(IN1, False)  
#    GPIO.output(IN2, False)

##    pwm_IN1.ChangeDutyCycle(0);
##    pwm_IN3.ChangeDutyCycle(0);
##    pwm_IN2.ChangeDutyCycle(0);
##    pwm_IN4.ChangeDutyCycle(0);

    sys.stdout.write('stop  \n')
    sys.stdout.flush() 
  
  
def run(x, y, r, division):  
    """小车运行 
    """  
    if x * x + y * y > r * r:  
        return  
  
    speed_min = 40  
    speed_max = 100  
    speed_delta = ( speed_max - speed_min  ) / ( division - 1 )
    # y轴为小车的运行方向，速度以y轴的值为基准
    
    nShift = int(math.ceil( abs ( y ) * division / r ) )
    speed = speed_min + speed_delta * nShift
    speed =  abs ( speed ) 
    if y < 0:  
        back(speed)  
    elif y > 0:  
        front(speed)  
    else:  
        stop()  
        return  
    """
    t = abs(y) * 100.0 / r  
    if t < 20:  # 电压太小，驱动不起来  
        t = 20   
    d = math.atan2(x, y) * 180 / math.pi  
      
    if x < 0: # 往左边偏, 右轮的速度比左轮快  
        t2 = t * (abs(90 + d) / 90)  
        pwm_r.ChangeDutyCycle(t)  
        pwm_l.ChangeDutyCycle(t2)  
    elif x > 0: # 往右边偏, 左边的速度比右轮快  
        t2 = t * (abs(90 - d) / 90)  
        pwm_l.ChangeDutyCycle(t)  
        pwm_r.ChangeDutyCycle(t2)  
    print x, y, t, t2  
    """

class CarController(tornado.web.RequestHandler):  
    def get(self):  
        with codecs.open('webcar.html', 'r', encoding='utf-8') as fp:  
            self.write(fp.read())  
  
    def post(self):  
        r = int(self.get_argument('r'))  
        x = int(self.get_argument('x'))  
        y = int(self.get_argument('y'))
        division = int(self.get_argument('division'))
        run(x, y, r, division)  
        self.write('ok')

if __name__ == '__main__':  
    app = tornado.web.Application([('/', CarController)], static_path='static', debug=True)  
    app.listen(8080)  
    tornado.ioloop.IOLoop.instance().start()  
