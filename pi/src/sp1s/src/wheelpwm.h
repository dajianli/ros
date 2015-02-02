#pragma once

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#define uchar unsigned char

class CWheelpwm
{
public:
  CWheelpwm(int pinForward, int pinBackward)
  {
    if(wiringPiSetup() == -1){//when initialize wiring failed,print message to screen
    	printf("setup wiringPi failed !");
    	return;
    }

    m_pinForward = pinForward;
    m_pinBackward = pinBackward;
    l298n_Init();
  }
  
  ~CWheelpwm()
  {
    softPwmWrite(m_pinForward, 0);
    softPwmWrite(m_pinBackward, 0);
  }

  void forward(int pwmValue)
  {
    softPwmWrite(m_pinForward, pwmValue);
    softPwmWrite(m_pinBackward, 0);
  }

  void backward(int pwmValue)
  {
    softPwmWrite(m_pinForward, 0);
    softPwmWrite(m_pinBackward, pwmValue);
  }

  int GetMaxPWM() { return 100;}

private:
  void l298n_Init(void)
  {
    softPwmCreate(m_pinForward,  0, 100);
    softPwmCreate(m_pinBackward,  0, 100);
  }
  
  int m_pinForward;
  int m_pinBackward;
};
