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

#define arr_len( x )  ( sizeof( x ) / sizeof( *x ) )

//IR pin definitions
#define encoderPinAI  2
#define encoderPinBI  3
#define encoderPinA  5
#define encoderPinB  6

//PING pin def
#define pingPin_F 11
#define pingPin_R 13

#define traversal 0
#define turn_Right 1
#define turn_Left 2
#define turn_Around 3
#define find_Wall 4


//MAZE TRAVERSAL Vars
int maze_state = 00; //FOUR STATES: 11, 10, 01, 00 BIT ONE IS FRONT PING, BIT 2 IS RIGHT PING
int robot_state = traversal;
int state_buffer[3] = {0,0,0};

//Ping Vars
float cm_F, cm_R;

//error for pin
int lastSignal_L = -1;
int lastSignal_R = -1;

//MOVEMENT VARIABLES
//distance in mm

//NEED TO HARD CODE 90 DEGREE RIGHT AND 180 DEGREE TURNS
int current = -1;
int path_length = -1;
float right_path[][3]= {{0,-3.141/2},{25,0}};
float left_path[][2] = {{0,3.141/2},{25,0}};
float turn_around[] = {3.141};

/*float paths[][3] = {
                      //circle path
                      {1,30,5.50*3.141/8.00},{0,100,3.141},
                      
                      {1,30,5.70*3.141/8.00},{0,100,3.141},

                      {0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},

                      {1,100,6.75*3.141/8.00},{0,100,3.141},{0,100,3.141},

                      {1,100,6.75*3.141/8.00},{0,100,3.141},{0,100,3.141},

                      {1,100,7.0*3.141/8.00},{0,100,3.141},{0,100,3.141},{0,50,3.141},
                      
                      {1,100,7.0*3.141/8.00},{0,100,3.141},{0,100,3.141},

                      {1,100,7.0*3.141/8.00},{0,100,3.141},{0,100,3.141},
                      
                      {0,100,3.141},{0,100,3.141},{0,100,3.141},

                      {1,100,6.50*3.141/8.00},{0,100,3.141},

                      {1,100,6.50*3.141/8.00},{0,70,3.141},

                      {1,100,6.50*3.141/8.00},

                      {1,100,6.00*3.141/8.00},

                      {1,100,6.4*3.141/8.00},


                      //back to staright line
                      {0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},
                      {0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},
                      {0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},
                      {0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},
                      {0,100,3.141},{0,100,3.141},{0,100,3.141},{0,100,3.141},

                      //second u turn path

                      {1,80,11*3.141/8.00}, 
                      {1,80,11*3.141/8.00}, 
                      {1,80,10.8*3.141/8.00}, 
                      
                      }; */
                      
//8 turns
//int N = arr_len(paths);
int straight_line_counter = 0;

double left_output = 0;
double right_output = 0;
double scalar;

float x_tracker = 0.0;
float y_tracker = 0.0;
float theta_tracker = 0.0;
float end_time=0.0;
float start_time = 0.0;
float time_step=0.0;

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
float ENCODER_RESOLUTION_LEFT = 32;//IMU SIDE      //encoder resolution (in pulses per revolution)
float ENCODER_RESOLUTION_RIGHT = 32;

float x = 0.0;           // x initial coordinate of mobile robot 
float y = 0.0;           // y initial coordinate of mobile robot 
float theta_world  = 3.141;       // The initial theta_world of mobile robot 
float theta_world_offset =3.141;
float last_theta_diff = 0.0;
float last_theta_world = 3.141;
float baseToWheel = 111.2;       //  the wheelbase of the mobile robot in mm
float CIRCUMFERENCE =PI * DIAMETER;
float Dl, Dr, avg_dist, theta;

//BALANCING VARIABLES
float K=40;//30 
float B=7;//5
float Kr=50;
float Br=0;
float Kt=0.00125; //0.00125
float Bt=0.00015; //0.00015
float distance = 0.0;
float distance_stick = 0.0;
float distance_ref=100.0;
float distance_dot=0.0;
float distance_track=0.0;
float theta_world_track=0.0;
float last_distance = 0;
float last_distance_diff = 0;
int pwm,pwm_l,pwm_r;
int i =0;
float angle, angular_rate, angle_offset = .008;  //0445
int16_t gyro[3];        // [x, y, z]            gyro vector
int16_t ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float theta_IMU = 0.0;
float theta_dot_IMU=0.0;
int path_count=-1;
int last_straight_counter = 0;


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

    pinMode(A2,OUTPUT);
    digitalWrite(A2,LOW);

    attachInterrupt(0, encoderA, CHANGE);//Bind interupt pin2
    attachInterrupt(1, encoderB, CHANGE);//Bind interupt pin3

    
    //delay 10 seconds
    //delay(10000);
    Serial.print("Done with setup");
    end_time=millis();

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
        theta_dot_IMU=(((float)gyro[2]/16.0)*(3.14/180.0))-0.012;
        if(theta_dot_IMU<0.015 and theta_dot_IMU>-0.015){
          theta_dot_IMU=0;
        }
        //theta_IMU = theta_IMU + theta_dot_IMU;
        
        if(angular_rate<0.01 and angular_rate>-0.01){
          angular_rate=0;
        }

            
       if((end_time - start_time)>=50){
          time_step = (end_time - start_time)/((float)1000.00);
          last_distance_diff = -1*(last_distance - distance);
          last_theta_diff = last_theta_world-theta_world;
          start_time=end_time;
          distance_dot=last_distance_diff / time_step;
          theta_world_dot=-1*last_theta_diff/ time_step;
          last_distance = distance;
          last_theta_world = theta_world;
        }

        //gets the ping values and sets the state of the robot
        getPingData_F();
        getPingData_R();
        set_Bot();
        move_Bot();    
        pwm_Out();
        end_time = millis();
}

void pwm_Out(){

      pwm += K*(angle_offset+delta_angle_translate - angle) - B*(angular_rate); 

    //set max and min to 400 and -400 change value for next project to leave power for turning
        if(pwm<-350){
          pwm=-350;
        }
        else if(pwm>350){
          pwm=350;
        }
        
       pwm_l = pwm + delta_pwm_rotate;
       pwm_r = pwm - delta_pwm_rotate;
       set_Motors(pwm_l, pwm_r);
}

void set_Motors(int l_val, int r_val){
  
      md.setM1Speed(l_val);
      md.setM2Speed(r_val);
}

//interupt method for first wheel
void encoderA(){
//
    if(digitalRead(encoderPinA) == lastSignal_R){
      return;
    }
  if (digitalRead(encoderPinAI) == HIGH) 
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
  else                                        
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

    lastSignal_R = digitalRead(encoderPinA);  
    update_Odometry();
    //Serial.println(x);
          
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
  //Serial.println(x);
}

//Calculate Odometry Values
void update_Odometry(){
  distanceLeftWheel = CIRCUMFERENCE * (encoderLeftPosition / ENCODER_RESOLUTION_LEFT);        //  travel distance for the left and right wheel respectively 
  distanceRightWheel = CIRCUMFERENCE * (encoderRightPosition / ENCODER_RESOLUTION_RIGHT);       // which equal to pi * diameter of wheel * (encoder counts / encoder resolution )
  deltaRight = distanceRightWheel - r_prev; 
  deltaLeft = distanceLeftWheel - l_prev;
  deltaDistance = (deltaRight + deltaLeft) / 2;
  distance = distance + deltaDistance;
  distance_stick =  distance_stick + deltaDistance;
  delta_theta_world = atan2((deltaRight - deltaDistance), baseToWheel);

  //distance = atan2(distanceRightWheel-(distanceRightWheel+distanceLeftWheel)/2,baseToWheel);
  
  theta_world = theta_world + delta_theta_world; //try mapping 0-5.5 rad to 0 - 2 PI
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
  float theta_diff = theta_world_offset - theta_world;
  if(theta_diff > PI){
    theta_diff = -2*PI+theta_diff;
  }
  if(theta_diff < -PI){
    theta_diff = 2*PI+theta_diff;
  }
  delta_pwm_rotate = Kr*(theta_diff) - Br *(theta_dot_IMU);
  delta_pwm_rotate = constrain(delta_pwm_rotate,-50,50);
  
}

void translate(){
  
  delta_angle_translate =  Kt*(distance_ref - distance_stick) - Bt *(distance_dot);  
  delta_angle_translate = constrain(delta_angle_translate,-0.075,0.075);
  
}

float microsecondsToCentimeters(float microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29.0 / 2.0;
}

void getPingData_F(){
        float duration_F = -1;
        pinMode(pingPin_F, OUTPUT);
        digitalWrite(pingPin_F, LOW);
        delayMicroseconds(2);
        digitalWrite(pingPin_F, HIGH);
        delayMicroseconds(5);
        digitalWrite(pingPin_F, LOW);

        pinMode(pingPin_F, INPUT);
        duration_F = pulseIn(pingPin_F, HIGH,4500);
        if(duration_F > 0){   
          cm_F = microsecondsToCentimeters(duration_F);
          maze_state = 10;
        }else{
          cm_F = -1;  
          maze_state = 0;
        }

}

void getPingData_R(){
        float duration_R = -1;
        pinMode(pingPin_R, OUTPUT);
        digitalWrite(pingPin_R, LOW);
        delayMicroseconds(2);
        digitalWrite(pingPin_R, HIGH);
        delayMicroseconds(5);
        digitalWrite(pingPin_R, LOW);

        pinMode(pingPin_R, INPUT);
        duration_R = pulseIn(pingPin_R, HIGH, 3000);
        if(duration_R > 0){   
          cm_R = microsecondsToCentimeters(duration_R);
          maze_state = maze_state + 1;
        }else{
          cm_R = -1;  
        }
 }

 int setRobotState(){
    if( maze_state == 11 && cm_F <= 20){
      
      return turn_Left;
      
    }else if(maze_state == 10){
      
        return turn_Right;
      
    }else if(maze_state == 01){
      
      return traversal;
    
    }else if(maze_state == 00){

        return turn_Right;
      
    }
    return traversal;
  }

  void move_Bot(){
        rotate();
        translate();
  }

  void set_Bot(){
    
    int state = setRobotState();
    state = buffer_Helper(state);
    if(robot_state == traversal){
      robot_state = state;  
    }
    if(robot_state == turn_Around){
      forward();  
    }else if(robot_state == turn_Right){
      rotate_90R();  
    }
    else if(robot_state==turn_Left){
      rotate_90L();
    }
    else{
      forward();  
    }
  }

  int buffer_Helper(int state){
     int state_total = 0;
     for(int j =0; j<2; j++){
       state_buffer[j] = state_buffer[j+1];
       state_total = state_total + state_buffer[j];  
     }
     state_buffer[2] = state;
     state_total = state_total + state;
     if(state_total == 3){
       return turn_Right;  
     }else if(state_total == 6){
       return turn_Left;
     } else{
       return traversal;  
     }
  }
   
  void rotate_180(){
    //straight up 180 degree turn
    if(current == -1){
      current = 0;
      path_length = arr_len(turn_around);
      theta_world_offset = turn_around[current];
    }
      
    //if we have turned correctly move to next rotate point, or switch back to traversal
    if( abs(theta_world_offset - theta_world) < 0.1 ){
     if(current >= path_length ){
       robot_state = traversal;
       theta_world_offset = 3.141;
       theta_world = 3.141;
       current = -1; 
     }else{
        current = current +1;
        theta_world_offset = turn_around[current]; 
     }
    }  
  };

  void rotate_90R(){
    //forward a few centimeters then 90 degree turn
    if(current == -1){
      current = 0;
      digitalWrite(A2,HIGH);
      path_length = arr_len(right_path);
      theta_world_offset = theta_world + right_path[current][1];
      distance_stick = 0;
      if(right_path[current][0]==0){
        distance_ref = 0;
      }
      else{
        distance_ref = right_path[current][0];
      }
      
    }

    if(right_path[current][0]==0){//means we are only turning no translate
      if( abs(theta_world_offset/theta_world) > 0.95 && abs(theta_world_offset/theta_world) < 1.05){
       if(current >= path_length){
         robot_state = traversal;
         for(int j =0; j<3;j++){
          state_buffer[j] = 0; 
         }
         current = -1;
         digitalWrite(A2,LOW); 
       }
       else{
          current = current +1;
          theta_world_offset = theta_world + right_path[current][1];
          distance_stick = 0;
          distance_ref = right_path[current][0]; 
       }
      }
    }

    
    //translation parts
    if(right_path[current][0]>0){
      if((distance_stick/distance_ref) > 0.95 && (distance_stick/distance_ref)<1.05){
        
         if(current >= path_length){
           robot_state = traversal;
           for(int j =0; j<3;j++){
            state_buffer[j] = 0; 
           }
           current = -1; 
           digitalWrite(A2,LOW);
         }
         else{
            current = current +1;
            theta_world_offset = theta_world + right_path[current][1];
            distance_stick = 0;
            distance_ref = right_path[current][0]; 
         }
        } 
      
    }  

    }

  void rotate_90L(){
    //forward a few centimeters then 90 degree turn
    if(current == -1){
      digitalWrite(A2,HIGH);
      current = 0;
      path_length = arr_len(left_path);
      theta_world_offset = theta_world + left_path[current][1];
      distance_stick = 0;
      if(left_path[current][0]==0){
        distance_ref = 0;
      }
      else{
        distance_ref = left_path[current][0];
      }      
    }

      
    if(left_path[current][0]==0){//means we are only turning no translate
      if( abs(theta_world_offset/theta_world) > 0.95 && abs(theta_world_offset/theta_world) < 1.05){
       if(current >= path_length ){
         robot_state = traversal;
         for(int j =0; j<3;j++){
          state_buffer[j] = 0; 
         }
         current = -1; 
         digitalWrite(A2,LOW);
       }
       else{
          current = current +1;
          theta_world_offset = theta_world + left_path[current][1];
          distance_stick = 0;
          distance_ref = left_path[current][0]; 
       }
      }
    }

    
    //translation part
    if(left_path[current][0]>0){
      if((distance_stick/distance_ref) > 0.95 && (distance_stick/distance_ref)<1.05){
        
         if(current >= path_length){
           robot_state = traversal;
           current = -1;
           for(int j =0; j<3;j++){
            state_buffer[j] = 0; 
           } 
            digitalWrite(A2,LOW);
         }
         else{
            current = current +1;
            theta_world_offset = theta_world + left_path[current][1];
            distance_stick = 0;
            distance_ref = left_path[current][0]; 
         }
        } 
 

  }
  }

  void forward(){
      distance_stick = 0;
      distance_ref = 25;
      if(cm_R > 11.0){
        theta_world_offset = theta_world - (3.14/64.0);
      }else if(cm_R < 9.0){
        theta_world_offset = theta_world + (3.14/64.0);
      }else{
        theta_world_offset = theta_world;  
      }
  
  }

