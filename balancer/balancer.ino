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
#define encoderPinA  5
#define encoderPinB  6

//ODOMETRY VARIABLES
//encoder trackers
volatile int encoderLeftPosition = 0;   //NEED TO FIGURE OUT WHICH IS WHICH
volatile int encoderRightPosition = 0;

float  DIAMETER  = 58;         // wheel diameter (in mm)
float distanceLeftWheel, distanceRightWheel, Dc, Orientation_change;

float ENCODER_RESOLUTION = 36;      //encoder resolution (in pulses per revolution)

int x = 0;           // x initial coordinate of mobile robot 
int y = 0;           // y initial coordinate of mobile robot 
float Orientation  = 0;       // The initial orientation of mobile robot 
float WHEELBASE = 5;       //  the wheelbase of the mobile robot in mm
float CIRCUMSTANCE =PI * DIAMETER;
float Dl, Dr, theta, Ori_ch;

//BALANCING VARIABLES
float K=50;
float B=50;
int pwm,pwm_l,pwm_r;
int i =0;
float angle, angular_rate, angle_offset;
int16_t gyro[3];        // [x, y, z]            gyro vector
int16_t ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector





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
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing MPU devices..."));
    mpu.initialize();
    mpu.setXGyroOffset(129);
    mpu.setYGyroOffset(-26); 
    mpu.setZGyroOffset(10);
    mpu.setZAccelOffset(1327); // 1688 factory default for my test chip

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
    //delay(10000);
    Serial.print("Done with setup");

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
        mpu.getMotion6(&ypr[0],&ypr[1],&ypr[2],&gyro[0],&gyro[1],&gyro[2]);
        
        angle = ypr[1] + (.084); 
        angular_rate = -((double)gyro[1]/131.0); // converted to radian
        if(angular_rate<0.01 and angular_rate>-0.01){
          angular_rate=0;
        }
        
//        Serial.print("Gyro: ");
//        Serial.print(angular_rate);
//        Serial.print("   Angle: ");
//        Serial.print(ypr[1]);
        
      //update our odometry values every loop
      update_Odometry();
      //calculate pwm
      //pwm_out();

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
   Serial.println("I Happened First Bitch");
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
  Serial.println (encoderLeftPosition, DEC);   
}

//interupt method for other wheel
void encoderB(){
  Serial.println("I Happen");
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
  Serial.println (encoderRightPosition, DEC);   
}

//Calculate Odometry Values
void update_Odometry(){
  
  distanceLeftWheel = CIRCUMSTANCE * (encoderLeftPosition / ENCODER_RESOLUTION);        //  travel distance for the left and right wheel respectively 
  distanceRightWheel = CIRCUMSTANCE * (encoderRightPosition / ENCODER_RESOLUTION);       // which equal to pi * diameter of wheel * (encoder counts / encoder resolution ) 
  theta = (distanceLeftWheel + distanceRightWheel) /2 ;                                 // incremental linear displacement of the robot's centerpoint C
  Orientation_change = (distanceRightWheel - distanceLeftWheel)/WHEELBASE;              // the robot's incremental change of orientation , where b is the wheelbase of the mobile robot ,
  Orientation = Orientation + Orientation_change ;                                     //  The robot's new relative orientation 
  x = x + theta * cos(Orientation);                                              // the relative position of the centerpoint for mobile robot 
  y = y + theta * sin(Orientation);
  
  //if statments to make sure theta is within 2 Pi
  if(theta > 6.28)
    theta -= 6.28;
  else if(theta < -6.28)
    theta += 6.28;
    
}




