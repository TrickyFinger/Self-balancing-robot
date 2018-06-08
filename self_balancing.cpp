
/*------------------------------------------------------------
                     Self_balancing robot
Hardware list
*Arduino 2560
*TB6612FNG X 2
*MPU6050

PINs connection
*TB6612--Arduino
       PWMA--8
       AIN1--6
       AIN2--7
ENCONDER_1A--2(interrupt 0)
ENCONDER_1B--9
       STBY--5
       PWMB--13
       BIN1--34
       BIN2--35
ENCONDER_2A--3(interrupt 1)
ENCONDER_2B--4

*MPU6050--Arduino
        INT--19
---------------------------------------------------------------
CHANGE LOG
2018.3.30   start
---------------------------------------------------------------*/


#include<MsTimer2.h>
#include<I2Cdev.h>
#include<MPU6050_6Axis_MotionApps20.h>
#include<Wire.h>

#include"my_PID.h"

PID motorPID;
MPU6050 mpu;


#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL


//pins definition
int PWM_A = 8; //PWM for motor 1
int STBY = 5;  //TB6612 STBY
int AIN1 = 6;  //TB6612 AIN1
int AIN2 = 7;  //TB6612 AIN2
int M1PA = 2;  //motor 1 phase A
int M1PB = 9;

int PWM_B = 13;
int BIN1 = 34;
int BIN2 = 35;
int M2PA = 3;
int M2PB = 4;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//motor control vars
float PWM;
float theta;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // join I2C bus
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 4)..."));
        attachInterrupt(4, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    motor_Initialize();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // other program behavior stuff here

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        // display real acceleration, adjusted to remove gravity

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
           // Serial.print("ypr\t");
           // Serial.print(ypr[0] * 180/M_PI);
           // Serial.print("\t");
           // Serial.print(ypr[1] * 180/M_PI);
           // Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
         /*
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);*/
        theta = ypr[2] * 180/M_PI;
        motor_BalanceControl(theta);

    }
}


void motor_Initialize()
{
  // configure motor and ENCODER
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(M1PA, INPUT);
    pinMode(M1PB, INPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(M2PA, INPUT);
    pinMode(M2PB, INPUT);

    digitalWrite(STBY, HIGH);  //MOTOR start

 //   attachInterrupt(0, doEncoder, CHANGE);
 //   MsTimer2::set(sample_time, get_speed);
 //   MsTimer2::start();

}


void motor_BalanceControl(float theta)
{
  motorPID.manual_Set(17.0, 0.0, 6.0);

  if(abs(theta)>35 || theta == 0.0)
  {
      PWM = 0;
      analogWrite(PWM_A, PWM);
      analogWrite(PWM_B, PWM);
  }
  else
  {
      if(theta<0)
      {
          //run forward
          digitalWrite(AIN1,HIGH);
          digitalWrite(AIN2,LOW);
          digitalWrite(BIN1,HIGH);
          digitalWrite(BIN2,LOW);
      }
      else
      {
          //run backward
          digitalWrite(AIN1,LOW);
          digitalWrite(AIN2,HIGH);
          digitalWrite(BIN1,LOW);
          digitalWrite(BIN2,HIGH);
      }

      PWM = motorPID.outPut(theta, 0.0, "POSITIONAL");  //positional PID
      PWM = abs(PWM);
      Serial.print("\t");
      Serial.println(PWM);
      if(PWM>255)
      {
          PWM = 255;
          analogWrite(PWM_A, PWM);
          analogWrite(PWM_B, PWM);
      }
      else
      {
          analogWrite(PWM_A, PWM);
          analogWrite(PWM_B, PWM);
      }
   }
}
