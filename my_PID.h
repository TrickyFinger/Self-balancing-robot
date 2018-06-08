#include<Arduino.h>

class PID
{
  private:
    ;     //remain empty
  public:
    float Kp, Ki, Kd;
    String str1 = "POSITIONAL";
    String str2 = "INCREMENTAL";
    void init();

    void manual_Set(float p, float i, float d);

    float outPut(float error, float target, String mode);
};
        
