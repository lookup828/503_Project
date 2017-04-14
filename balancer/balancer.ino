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
int path_signal = -1;
//MOVEMENT VARIABLES
//distance in mm
int paths[7][3] = {{0,2000,0},{1,500,-90},{0,500,0},{1,1000,-270},{0,2000,0},{1,500,180},{0,3000,0}};
int numberOf = 7;
int current = 0;
double left_output = 0;
double right_output = 0;
double scalar;

float x_tracker = 0.0;
float y_tracker = 0.0;
float theta_tracker = 0.0;
float end_time=0.0;
float start_time = 0.0;
float time_step=0.05;

//ODOMETRY VARIABLES
//encoder trackers
volatile int encoderLeftPosition = 0;   //NEED TO FIGURE OUT WHICH IS WHICH
volatile int encoderRightPosition = 0;

float  DIAMETER  = 70.2;         // wheel diameter (in mm)
float distanceLeftWheel, distanceRightWheel, deltaDistance=0, delta_theta_world=0, r_prev=0, l_prev=0;
 float  deltaRight =0.0; 
  float  deltaLeft =0.0;
  float theta_world_dot=0;
float   delta_angle_translate = 0.0;

float delta_pwm_rotate=0;
float ENCODER_RESOLUTION = 40;      //encoder resolution (in pulses per revolution)

float x = 0.0;           // x initial coordinate of mobile robot 
float y = 0.0;           // y initial coordinate of mobile robot 
float theta_world  = 0.01;       // The initial theta_world of mobile robot 
float theta_world_offset = 0.02;
float baseToWheel = 111.2;       //  the wheelbase of the mobile robot in mm
float CIRCUMFERENCE =PI * DIAMETER;
float Dl, Dr, avg_dist, theta;

//BALANCING VARIABLES
float K=12;//12 
float B=2;//1.8 
float Kr=3;
float Br=1;
float Kt=0.01;
float Bt=0.009;
float distance = 0.0;
float distance_ref=0.0;
float distance_dot=0.0;
float distance_track=0.0;
float theta_world_track=0.0;
int pwm,pwm_l,pwm_r;
int i =0;
float angle, angular_rate, angle_offset = .09;  //0.195
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
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setXGyroOffset(129);
    mpu.setYGyroOffset(-26); 
    mpu.setZGyroOffset(10);
    mpu.setZAccelOffset(1327); // 1688 factory default for my test chip

    //Empty the Buffer
    while (Serial.available() && Serial.read()); // empty buffer

    //set angles for non turning paths
//    for(int i=0; i<numberOf; i++){
//      if(i != 0){
//          if(paths[i][0] == 0){
//            paths[i][2] == paths[i-1][2];  
//          }
//      }  
//    }

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
    
        //IF YOU INPLUG THE IMU TO TEST OTHER PARTS YOU NEED TO UNCOMMENT THE NEXT LINE TO RUN PAST IT
        mpu.getMotion6(&ypr[0],&ypr[1],&ypr[2],&gyro[0],&gyro[1],&gyro[2]);
 
        //angle = atan2(ypr[0], ypr[2]) + (.084); 
        angle = atan2(ypr[0], ypr[2]);
        angular_rate = -(((float)gyro[1]/16.0)*(3.14/180.0));  //angular_rate = -((double)gyro[1]/131.0); // converted to radian
        if(angular_rate<0.01 and angular_rate>-0.01){
          angular_rate=0;
        }

   
        //distance_ref theta_world_offset
//        if(path_signal = -1){
//            distance_ref = paths[current][1];
//            theta_world_offset = paths[current][2];
//            needed_distance += distance_ref;
//            needed_theta = paths[current][2];  
//        }
       
        
        
        start_time=millis();

        if((start_time-end_time)>=50){

          end_time=start_time;
          distance_dot=distance_track/time_step;
          theta_world_dot=theta_world_track/time_step;
          theta_world_track=0;
          distance_track=0;
        }
        
        //rotate();
        translate();
        pwm_Out();
//          Serial.print(distance);
//          Serial.print("   ");
//          Serial.print(delta_angle_translate);
//         
//          Serial.print("   ");
//          Serial.println(pwm);
//          Serial.print("   ");
//          Serial.println(angle);


}

void pwm_Out(){
  

      pwm += K*(angle_offset+delta_angle_translate-angle) - B*(angular_rate); 
    

    //set max and min to 400 and -400 change value for next project to leave power for turning
        if(pwm<-350){
          pwm=-350;
        }
        else if(pwm>350){
          pwm=350;
        }
        
       pwm_l = pwm - delta_pwm_rotate;
       pwm_r = pwm + delta_pwm_rotate;
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
    //time_step=start_time-end_time;   
    //Serial.println(encoderLeftPosition); 
    update_Odometry();

          
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
  //Serial.println(encoderRightPosition); 
  update_Odometry();

   

}

//Calculate Odometry Values
void update_Odometry(){

  distanceLeftWheel = CIRCUMFERENCE * (encoderLeftPosition / ENCODER_RESOLUTION);        //  travel distance for the left and right wheel respectively 
  distanceRightWheel = CIRCUMFERENCE * (encoderRightPosition / ENCODER_RESOLUTION);       // which equal to pi * diameter of wheel * (encoder counts / encoder resolution )
  deltaRight = distanceRightWheel - r_prev; 
  deltaLeft = distanceLeftWheel - l_prev;
  deltaDistance = (deltaRight + deltaLeft) / 2;
  distance = distance + deltaDistance;
  
  
  delta_theta_world = atan2((deltaRight - deltaDistance), baseToWheel);

  //distance = atan2(distanceRightWheel-(distanceRightWheel+distanceLeftWheel)/2,baseToWheel);
  
  distance_track=distance_track-deltaDistance;
  theta_world_track = theta_world_track-delta_theta_world;

  theta_world = theta_world - delta_theta_world; //try mapping 0-5.5 rad to 0 - 2 PI
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



void rotate(){

  delta_pwm_rotate = Kr*(theta_world_offset - theta_world) - Br *(theta_world_dot);
  if(delta_pwm_rotate>50){
    delta_pwm_rotate=50;
  }
  else if(delta_pwm_rotate<-50){
    delta_pwm_rotate=-50;
  }
}

void translate(){

  delta_angle_translate = Kt*(distance_ref - distance) - Bt *(distance_dot);

  
  if((delta_angle_translate)>0.075){
    delta_angle_translate=0.075;
  }
  if (delta_angle_translate< -0.075){
    delta_angle_translate= -0.075;
  }
}

