#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#define uchar unsigned char

#define L298N_ENA          1
#define L298N_IN1          15
#define L298N_IN2          16

void l298n_Init(void)
{
  softPwmCreate(L298N_ENA,  0, 100);
  softPwmCreate(L298N_IN1,  0, 100);
  softPwmCreate(L298N_IN2,  0, 100);
}

void forward(int pwmValue)
{
  softPwmWrite(L298N_IN1, pwmValue);
  softPwmWrite(L298N_IN2, 0);
}

void backward(int pwmValue)
{
  softPwmWrite(L298N_IN1, 0);
  softPwmWrite(L298N_IN2, pwmValue);
}

int main(void)
{
  int i;

  if(wiringPiSetup() == -1){      //when initialize wiring failed,print message to screen
    printf("setup wiringPi failed !");
    return 1; 
  }

  l298n_Init();
  int pwm = 0;

  while(1){
    
    for( pwm = 0; pwm = 100; pwm += 10 )
    {
	forward(pwm);
    	delay(500);
    }

    for( pwm = 0; pwm = 100; pwm += 10 )
    {
	backward(pwm);
    	delay(500);
    }
  }

  return 0;
}
