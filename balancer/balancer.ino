#include <QTRSensors.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "DualMC33926MotorShield.h"

MPU6050 mpu;
DualMC33926MotorShield md;

#define OUTPUT_READABLE_YAWPITCHROLL
#define PWM_L 9
#define PWM_R 10

//IR pin definitions
#define encoderPinAI  2
#define encoderPinBI  3
#define encoderPinA  4
#define encoderPinB  5

//ODOMETRY VARIABLES
//encoder trackers
volatile int encoderLeftPosition = 0;   //NEED TO FIGURE OUT WHICH IS WHICH
volatile int encoderRightPosition = 0;

float  DIAMETER  = 61  ;         // wheel diameter (in mm)  NEED TO CHANGE TO OURS

float distanceLeftWheel, distanceRightWheel, Dc, Orientation_change;

float ENCODER_RESOLUTION = 333.3;      //encoder resolution (in pulses per revolution)  where in Rover 5,  1000 state changes per 3 wheel rotations NEED TO CHANGE TO OURS

int x = 0;           // x initial coordinate of mobile robot 
int y = 0;           // y initial coordinate of mobile robot 
float Orientation  = 0;       // The initial orientation of mobile robot 
float WHEELBASE=183  ;       //  the wheelbase of the mobile robot in mm, NEED TO CHANGE TO OURS
float CIRCUMSTANCE =PI * DIAMETER  ;

//BALANCING VARIABLES
float K=50;
float B=50;
int pwm,pwm_l,pwm_r;
int i =0;
float angle, angular_rate, angle_offset;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
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
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector





// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    md.init();
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing MPU devices..."));
    mpu.initialize();
    mpu.setXGyroOffset(129);
    mpu.setYGyroOffset(-26); 
    mpu.setZGyroOffset(10);
    mpu.setZAccelOffset(1327); // 1688 factory default for my test chip
    mpu.getMotion();
    


    //Empty the Buffer
    while (Serial.available() && Serial.read()); // empty buffer




    //Pin stuff
    pinMode(PWM_L, OUTPUT);//
    pinMode(PWM_R, OUTPUT);//

    Serial.println("Setting up IR Pins");
    pinMode(encoderPinAI, INPUT); 
    digitalWrite(encoderPinAI, HIGH);       // turn on pull-up resistor
    pinMode(encoderPinA, INPUT); 
    digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
    pinMode(encoderPinBI, INPUT); 
    digitalWrite(encoderPinBI, HIGH);       // turn on pull-up resistor
    pinMode(encoderPinB, INPUT); 
    digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor

    attachInterrupt(0, encoderA, CHANGE);//Bind interupt pin2
    attachInterrupt(1, encoderB, CHANGE);//Bind interupt pin3

    //delay 10 seconds
    delay(10000);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    
//        #ifdef OUTPUT_READABLE_YAWPITCHROLL
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetGyro(gyro, fifoBuffer);
//            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//        #endif
        mpu.getMotion(&ypr[0],&ypr[1],&ypr[2],&gyro[0],&gyro[1],&gyro[2]);
        
        angle = ypr[1] + (.084); 
        angular_rate = -((double)gyro[1]/131.0); // converted to radian
        if(angular_rate<0.01 and angular_rate>-0.01){
          angular_rate=0;
        }
        
        Serial.print("Gyro: ");
        Serial.print(angular_rate);
        Serial.print("   Angle: ");
        Serial.print(ypr[1]);

      odometry();
      //calculate pwm
      pwm_out();

}

void pwm_out(){
  
     pwm += -K*(0.10 - angle)+B*(angular_rate);
     
    //set max and min to 400 and -400 change value for next project to leave power for turning
        if(pwm<-400){
          pwm=-400;
        }
        else if(pwm>400){
          pwm=400;
        }
       Serial.print("   PWM SIgnal: ");
       Serial.print(pwm);
       Serial.print("   K: ");
       Serial.print(K);
       Serial.print("   B: ");
       Serial.print(B);
       Serial.println();
       pwm_l = pwm;
       pwm_r = pwm;
       set_Motors(pwm_l, pwm_r);
}

void set_Motors(int l_val, int r_val){
  
      md.setM1Speed(l_val);
      md.setM2Speed(r_val);
}

//interupt method for first wheel
void encoderA(){
  if (digitalRead(encoderPinAI) == HIGH) 
  {   // found a low-to-high on channel A
    if (digitalRead(encoderPinA) == LOW) 
    {  // check channel B to see which way
      Serial.println("Counterclockwise and backward");
      encoderLeftPosition = encoderLeftPosition - 1;
    } 
    else 
    {
      Serial.println("Clockwise and forward");
      encoderLeftPosition = encoderLeftPosition + 1;         
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoderPinA) == LOW) 
    {   // check channel B to see which way
                                              // encoder is turning  
      Serial.println("Clockwise and forward");
      encoderLeftPosition = encoderLeftPosition + 1;          
    } 
    else 
    {
      Serial.println("Counterclockwise and backward");
      encoderLeftPosition = encoderLeftPosition - 1;         
    }

  }
  Serial.println (encoderPosA, DEC);   
}

//interupt method for other wheel
void encoderB(){
  if (digitalRead(encoderPinBI) == HIGH) 
  {   // found a low-to-high on channel A
    if (digitalRead(encoderPinB) == LOW) 
    {  // check channel B to see which way
      Serial.println("Counterclockwise and backward");      // encoder is turning
      encoderRightPosition = encoderRightPosition - 1;         
    } 
    else 
    {
      Serial.println("Clockwise and forward");
      encoderRightPosition = encoderRightPosition + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoderPinB) == LOW) 
    {   // check channel B to see which way
                                              // encoder is turning  
      Serial.println("Clockwise and forward");
      encoderRightPosition = encoderRightPosition + 1;          
    } 
    else 
    {
      Serial.println("Counterclockwise and backward");
      encoderRightPosition = encoderRightPosition - 1;         
    }

  }
  Serial.println (encoderPosB, DEC);   
}

//Calculate Odometry Values
void odometry(){
  
  Dl= Pi * dia * (encoderLPos/ ER);       // Dl & Dr are travel distance for the left and right wheel respectively 
  Dr= Pi * dia * (encoderRPos/ ER);     // which equal to pi * diameter of wheel * (encoder counts / encoder resolution ) 
  Dc=( Dl + Dr) /2 ;            // incremental linear displacement of the robot's centerpoint C
  Ori_ch=(Dr - Dl)/b;          // the robot's incremental change of orientation , where b is the wheelbase of the mobile robot ,
  Ori = Ori + Ori_ch ;          //  The robot's new relative orientation 
  x = x + Dc * cos (Ori);      // the relative position of the centerpoint for mobile robot 
  y = y + Dc * sin(Ori);
    
}




