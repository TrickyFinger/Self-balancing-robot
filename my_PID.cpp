#include<Arduino.h>
#include"my_PID.h"

void PID::manual_Set(float p, float i, float d)
{
  Kp = p;
  Ki = i;
  Kd = d;
}

float PID::outPut(float real, float target, String mode)
{
  manual_Set(Kp, Ki, Kd);
  if(mode == str2)  //澧為噺鍨婸ID
  {                                                //error2: error this time
    static float error0, error1, error2, PWM;      //error1: error last time
    error2 = target - real;
    PWM += Kp*(error2 - error1) + Ki*error2 + Kd*(error2 - (2*error1) +error0);

    error0 = error1;
    error1 = error2;
    return PWM;
  }

  else if(mode == str1)  //浣嶇疆鍨婸ID
  {
    static float error1, error2, error_sum, PWM;
    error2 = target - real;
    error_sum += error2;
    PWM = Kp*error2 + Ki*error_sum + Kd*(error2 - error1);
    error1 = error2;
    return PWM;
  }
  else
  {
    return 0;
  }

}
