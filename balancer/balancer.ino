#include "QTRSensors.h"
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

//error for pin
int lastSignal_L = -1;
int lastSignal_R = -1;
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
float theta_tracker = 0;

//ODOMETRY VARIABLES
//encoder trackers
volatile int encoderLeftPosition = 0;   //NEED TO FIGURE OUT WHICH IS WHICH
volatile int encoderRightPosition = 0;

float  DIAMETER  = 70.2;         // wheel diameter (in mm)
float distanceLeftWheel, distanceRightWheel, deltaDistance=0, delta_theta_world=0, r_prev=0, l_prev=0;
 float  deltaRight =0; 
  float  deltaLeft =0;

float ENCODER_RESOLUTION = 32;      //encoder resolution (in pulses per revolution)

float x = 0.0;           // x initial coordinate of mobile robot 
float y = 0.0;           // y initial coordinate of mobile robot 
float theta_world  = 0;       // The initial theta_world of mobile robot 
float baseToWheel = 111.2;       //  the wheelbase of the mobile robot in mm
float CIRCUMFERENCE =PI * DIAMETER;
float Dl, Dr, avg_dist, theta;

//BALANCING VARIABLES
float K=20;
float B=5;
int pwm,pwm_l,pwm_r;
int i =0;
float angle, angular_rate, angle_offset = .34;
int16_t gyro[3];        // [x, y, z]            gyro vector
int16_t ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//odometry cap
int o_cap =0;





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
//    mpu.initialize();
//    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
//    mpu.setXGyroOffset(129);
//    mpu.setYGyroOffset(-26); 
//    mpu.setZGyroOffset(10);
//    mpu.setZAccelOffset(1327); // 1688 factory default for my test chip

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
    attachInterrupt(1, encoderB, HIGH);//Bind interupt pin3

    //delay 10 seconds
    //delay(10000);
    Serial.print("Done with setup");

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    
        //IF YOU INPLUG THE IMU TO TEST OTHER PARTS YOU NEED TO UNCOMMENT THE NEXT LINE TO RUN PAST IT
        mpu.getMotion6(&ypr[0],&ypr[1],&ypr[2],&gyro[0],&gyro[1],&gyro[2]);
 
        angle = atan2(ypr[1], ypr[2]) + (.084); 
        angular_rate = -(((float)gyro[1]/16.0)*(3.14/180.0));  //angular_rate = -((double)gyro[1]/131.0); // converted to radian
        if(angular_rate<0.01 and angular_rate>-0.01){
          angular_rate=0;
        }


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

       pwm_l = pwm + right_output;
       pwm_r = pwm + left_output;
       set_Motors(pwm_l, pwm_r);
}

void set_Motors(int l_val, int r_val){
  
      md.setM1Speed(l_val);
      md.setM2Speed(r_val);
}

//interupt method for first wheel
void encoderA(){
    if(digitalRead(encoderPinA) == lastSignal_R){
      return;
    }
  if (digitalRead(encoderPinAI) == HIGH) 
  {   
    if (digitalRead(encoderPinA) == LOW) 
    {  
      encoderLeftPosition = encoderLeftPosition + 1;
    } 
    else 
    {
      encoderLeftPosition = encoderLeftPosition - 1;         
    }
  }
  else                                        
  { 
    if (digitalRead(encoderPinA) == LOW) 
    {   
      encoderLeftPosition = encoderLeftPosition - 1;          
    } 
    else 
    {
      encoderLeftPosition = encoderLeftPosition + 1;         
    }

  }

    lastSignal_R = digitalRead(encoderPinA);  
    update_Odometry();
    Serial.print(x);
  Serial.print("   ");
  Serial.print(y);
  Serial.print("   ");
  Serial.print(deltaLeft);
  Serial.print("    ");
  Serial.print(deltaRight);
  Serial.print("    ");
  Serial.println(theta_world);
//   Serial.println(encoderRightPosition);
}

//interupt method for other wheel
void encoderB(){
  
  if(digitalRead(encoderPinB) == lastSignal_L){
      return;
    }
  if (digitalRead(encoderPinBI) == HIGH) 
  {   
    if (digitalRead(encoderPinB) == LOW) 
    {  
      encoderRightPosition = encoderRightPosition - 1;         
    } 
    else 
    {
      encoderRightPosition = encoderRightPosition + 1;
    }
  }
  else                                       
  { 
    if (digitalRead(encoderPinB) == LOW) 
    {                                       
        encoderRightPosition = encoderRightPosition + 1;          
    } 
    else 
    {
      //Serial.println("Counterclockwise and backward");
      encoderRightPosition = encoderRightPosition - 1;         
    }

  }
  lastSignal_L = digitalRead(encoderPinB); 
  update_Odometry();
  Serial.print(x);
  Serial.print("   ");
  Serial.print(y);
  Serial.print("   ");
  Serial.print(deltaLeft);
  Serial.print("    ");
  Serial.print(deltaRight);
  Serial.print("    ");
  Serial.println(theta_world);
//   Serial.println(encoderRightPosition);
}

//Calculate Odometry Values
void update_Odometry(){

  distanceLeftWheel = CIRCUMFERENCE * (encoderLeftPosition / ENCODER_RESOLUTION);        //  travel distance for the left and right wheel respectively 
  distanceRightWheel = CIRCUMFERENCE * (encoderRightPosition / ENCODER_RESOLUTION);       // which equal to pi * diameter of wheel * (encoder counts / encoder resolution )
  deltaRight = distanceRightWheel - r_prev; 
  deltaLeft = distanceLeftWheel - l_prev;
  deltaDistance = (deltaRight + deltaLeft) / 2;
  //Serial.println(deltaRight);
  delta_theta_world = atan2(abs(deltaRight - deltaDistance), baseToWheel);
  if(deltaLeft>deltaRight){
      delta_theta_world=-1*delta_theta_world;
  }
  theta_world = theta_world + delta_theta_world;
  x = x + deltaDistance * cos(theta_world);
  y = y + deltaDistance * sin(theta_world); 

  
  //if statments to make sure theta is within 2 Pi
  if(theta_world > 2*PI){
    theta_world -= 2*PI;
  }
  else if(theta_world < 0){
    theta_world += 2*PI;
  }

  r_prev = distanceRightWheel;
  l_prev = distanceLeftWheel;
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
      scalar = 100 / sqrt(paths[current][1]);   //calcuate the scalar for x in the motor output equation
      needed_distance = avg_dist + paths[current][1];//assuming the type is 0; will implement for type 1 later
    }

    // y = scalar(-x^2) + 100;
    right_output = ( scalar * pow(-distanceRightWheel, 2) ) + 100;
    left_output  = ( scalar * pow(-distanceLeftWheel,  2) ) + 100;
    
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
    if( avg_dist >= needed_distance ){
      current++;
      needed_distance = -1;
      delay(20000);
    }
    //MOVE THE ROBOT USING ANGLE OFSSET NOT PWM
    
}
//this will be our method for tweaking values so the robot
//will handle errors in movement
//for type 0: if we are farther than we should be lower pwm, if we are not as far as we should be up the pwm
//for type 1: if the angle is higher or lower than it should be change the output for that specific wheel
void error_correction(){}

