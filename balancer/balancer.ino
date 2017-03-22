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

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

//ir pins are going to be digital 3,5,6,11

float K=50;
float B=50;
int pwm,pwm_l,pwm_r;
int i =0;
bool blinkState = false;
float angle, angular_rate, angle_offset;
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
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    //Empty the Buffer
    while (Serial.available() && Serial.read()); // empty buffer

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(129);
    mpu.setYGyroOffset(-26); 
    mpu.setZGyroOffset(10);
    mpu.setZAccelOffset(1327); // 1688 factory default for my test chip

    //Pin stuff
    pinMode(PWM_L, OUTPUT);//
    pinMode(PWM_R, OUTPUT);//

    //delay 10 seconds
    delay(10000);
    //set on LED
    digitalWrite(ledPin, HIGH);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetGyro(gyro, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
        
        angle = ypr[1] + (.084); 
        angular_rate = -((double)gyro[1]/131.0); // converted to radian
        if(angular_rate<0.01 and angular_rate>-0.01){
          angular_rate=0;
        }

        //change k and b values while running TAKE OUT FOR FINAL
        if(Serial.available()> 1){
         char c = Serial.read();
          switch(c){
            case 'q':
              K += 1;
            break;

            case 'w':
              K -= 1;
            break;

            case 'a':
              B += 1;
            break;

            case 's':
              B -= 1;
            break;
          }
        }
        
        Serial.print("Gyro: ");
        Serial.print(angular_rate);
        Serial.print("   Angle: ");
        Serial.print(ypr[1]);

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




