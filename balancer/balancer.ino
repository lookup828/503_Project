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

//MOVEMENT VARIABLES
//distance in mm
int paths[4][3] = {{0,10000,0},{0,0,0},{0,0,0},{0,0,0}};
int current = 0;
double left_output = 0;
double right_output = 0;
float angle_increase = 0;
double scalar;
float needed_distance = 0;
float x_tracker = 0;
float y_tracker = 0;
float theata_tracker = 0;

//ODOMETRY VARIABLES
//encoder trackers
volatile int encoderLeftPosition = 0;   //NEED TO FIGURE OUT WHICH IS WHICH
volatile int encoderRightPosition = 0;

float  DIAMETER  = 58;         // wheel diameter (in mm)
float distanceLeftWheel, distanceRightWheel, Dc, Orientation_change;

float ENCODER_RESOLUTION = 64;      //encoder resolution (in pulses per revolution)

int x = 0;           // x initial coordinate of mobile robot 
int y = 0;           // y initial coordinate of mobile robot 
float Orientation  = 0;       // The initial orientation of mobile robot 
float WHEELBASE = 5;       //  the wheelbase of the mobile robot in mm
float CIRCUMSTANCE =PI * DIAMETER;
float Dl, Dr, avg_dist, theta;

//BALANCING VARIABLES
float K=20;
float B=5;
int pwm,pwm_l,pwm_r;
int i =0;
float angle, angular_rate, angle_offset = .34;
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
    mpu.getFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setXGyroOffset(129);
    mpu.setYGyroOffset(-26); 
    mpu.setZGyroOffset(10);
    mpu.setZAccelOffset(1327); // 1688 factory default for my test chip

    //Empty the Buffer
    while (Serial.available() && Serial.read()); // empty buffer

    //Pin stuff
    pinMode(PWM_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);

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
        
        angle = atan2(ypr[1], ypr[2]) + (.084); 
        angular_rate = -(((float)gyro[1]/16.0)*(3.14/180.0));  //angular_rate = -((double)gyro[1]/131.0); // converted to radian
        if(angular_rate<0.01 and angular_rate>-0.01){
          angular_rate=0;
        }
        
//        Serial.print("Gyro: ");
//        Serial.print(angular_rate);
//        Serial.print("   Angle: ");
//        Serial.print(ypr[1]);
        
      //update our odometry values every loop
      update_Odometry();
      //method to move the robot
      //move_Bot();
      //calculate pwm
      //pwm_Out();

}

void pwm_Out(){
  
     pwm += -K*( (angle_offset + angle_increase) - angle)+B*(angular_rate);
     
    //set max and min to 400 and -400 change value for next project to leave power for turning
        if(pwm<-300){
          pwm=-300;
        }
        else if(pwm>300){
          pwm=300;
        }
//       Serial.print("   PWM SIgnal: ");
//       Serial.print(pwm);
//       Serial.print("   K: ");
//       Serial.print(K);
//       Serial.print("   B: ");
//       Serial.print(B);
//       Serial.println();
       pwm_l = pwm + right_output;
       pwm_r = pwm + left_output;
       set_Motors(pwm_l, pwm_r);
}

void set_Motors(int l_val, int r_val){
  
      md.setM1Speed(l_val + left_translation + left_rotation);
      md.setM2Speed(r_val + right_translation + right_rotation);
}

//interupt method for first wheel
void encoderA(){
   Serial.println("I Happened First");
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
  avg_dist = (distanceLeftWheel + distanceRightWheel) /2 ;                                 // incremental linear displacement of the robot's centerpoint C
  theta = atan2(distanceRightWheel, distanceLeftWheel);              // the robot's incremental change of orientation , where b is the wheelbase of the mobile robot ,
  Orientation = Orientation + theta;                                     //  The robot's new relative orientation 
  x = x + avg_dist * cos(Orientation);                                              // the relative position of the centerpoint for mobile robot 
  y = y + avg_dist * sin(Orientation);
  
  //if statments to make sure theta is within 2 Pi
  if(Orientation > PI)
    Orientation -= PI;
  else if(Orientation < PI)
    Orientation += PI;
  
  Serial.print("X:");
  Serial.print(x);
  Serial.print("  ");
  Serial.print("Y:");
  Serial.print(y);
  Serial.print("  ");
  Serial.print("Theta:");
  Serial.println(theta);
    
}
//method to move our robot
void move_Bot(){
    //put values into translation and rotation values
    //we have a 100pwm allowance need to cap that
    //beginning of segment
    if(needed_distance == -1){
      x_tracker = x;
      y_tracker = y;
      theta_tracker = theta;
      scalar = -60 - sqrt(paths[current][1]);   //calcuate the scalar for x in the motor output equation
      needed_distance = avg_dist + paths[current][1]//assuming the type is 0; will implement for type 1 later
    }

    // y = scalar(-x^2) + 60;
    right_output = ( scalar * pow(-distanceRightWheel, 2) )+ 100;
    left_output  = ( scalar * pow(-distanceLeftWheel,  2) )+ 100;
    
    //values automatically capped between 0 and 100 pwm which is our limit
    // will need to calculate the individual wheel distance travel for turning
    if(right_output >= left_output){
      angle_increase = left_output;
      right_output = right_output - left_output;
    }else{
      angle_increase = right_output;
      left_output = left_output - right_output;
    }
    error_correction();
    
    //check if we have finished the segment
    //if so we move the current path up one and set the begining of path flag
    if( avg_dist >= needed_dist ){
      current++;
      needed_dist = -1;
      delay(20000);
    }
    //MOVE THE ROBOT USING ANGLE OFSSET NOT PWM
    
}
//this will be our method for tweaking values so the robot
//will handle errors in movement
//for type 0: if we are farther than we should be lower pwm, if we are not as far as we should be up the pwm
//for type 1: if the angle is higher or lower than it should be change the output for that specific wheel
void error_correction(){}





