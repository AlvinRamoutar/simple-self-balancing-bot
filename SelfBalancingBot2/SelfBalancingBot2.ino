/******************************************************************************
# Simple self-balancing robot code for Arduino w. MPU 6050
# Implemented based off Jeff Rowberg's MPU 6050 + Teapot impl.
# Sheridan College Robotics, 2019 
******************************************************************************/

#include "math.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
int j = 0;
float correct = 0.0f;


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// L298 Motor Shield Config
#define LMOTOR 4 
#define LMOTOR_PWM 5 
#define RMOTOR 7       
#define RMOTOR_PWM 6        

// Gyro Offsets
#define GYRO_X_OFFSET   -8
#define GYRO_Y_OFFSET   37
#define GYRO_Z_OFFSET   68
#define ACCEL_X_OFFSET  1635
#define ACCEL_Y_OFFSET  -4236
#define ACCEL_Z_OFFSET  1788



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

void setup() {

    // Motor Shield
    pinMode(LMOTOR, OUTPUT);
    pinMode(LMOTOR_PWM, OUTPUT);
    pinMode(RMOTOR, OUTPUT);
    pinMode(RMOTOR_PWM, OUTPUT);
  
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

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(GYRO_X_OFFSET);
    mpu.setYGyroOffset(GYRO_Y_OFFSET);
    mpu.setZGyroOffset(GYRO_Z_OFFSET);
    mpu.setXAccelOffset(ACCEL_X_OFFSET);
    mpu.setYAccelOffset(ACCEL_Y_OFFSET);
    mpu.setZAccelOffset(ACCEL_Z_OFFSET); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        
        // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  	if(fifoCount < packetSize){
  	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
  			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  	}
   
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
      	while(fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
      		mpu.getFIFOBytes(fifoBuffer, packetSize);
      		// track FIFO count here in case there is > 1 packet available
      		// (this lets us immediately read more without waiting for an interrupt)
      		fifoCount -= packetSize;
      	}

       //drive(ypr[1] * 180/M_PI);
       
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          // Yaw, Pitch, Roll values - Radians to degrees
          ypr[0] = ypr[0] * 180 / M_PI;
          ypr[1] = ypr[1] * 180 / M_PI;
          ypr[2] = ypr[2] * 180 / M_PI;
          
          // Skip 300 readings (self-calibration process)
          if (j <= 300) {
            correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
            j++;
          }
          // After 300 readings
          else {
            ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
            // Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180
            int servo0Value = map(ypr[0], -90, 90, 0, 180);
            int servo1Value = map(ypr[1], -90, 90, 0, 180);
            int servo2Value = map(ypr[2], -90, 90, 180, 0);

            newDrive(servo1Value);
            
            // Control the servos according to the MPU6050 orientation
            // Debug
            //servo0.write(servo0Value);
            //servo1.write(servo1Value);
            //servo2.write(servo2Value);
            //Serial.print("x\t");
            //Serial.print(servo0Value);
            //Serial.print("y\t");
            //Serial.print(servo1Value);
            //Serial.print("z\t");
            //Serial.println(servo2Value);
          }
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

/**
 * Based on pitch, discern:
 * - Relative pitch
 * - Motor drive power
 * - Direction (opposite of pitch dir)
 */
void newDrive(int pitch) {

  int relPitch = 90 - pitch;
  int pitchPwr = abs(relPitch) + 50;

  String debugMsg = "RawPitch=" + String(pitch) + " RelPitch=" + String(relPitch) + " PitchPower=" + String(pitchPwr) + " TheoreticalPowerCalc=" + String(powerCalc(relPitch));

  if(relPitch < 0) {
    Serial.println(debugMsg);
    driveForward(powerCalc(pitchPwr));
    
  } else if(relPitch > 0) {
    Serial.println(debugMsg);
    driveReverse(powerCalc(pitchPwr));
  
  } else {
    halt();
  }
  
}


void drive(float pitch) {

  float threshold = 4;

  String msg;

  if(pitch <= -threshold) {
    msg = "TURN FORWARD " + String("(") + String(pitch, 5) + String(")") + " WITH PWM OF " + String(powerCalc(pitch));
    Serial.println(msg);
    driveForward(pitch);
  } else if(pitch >= threshold) {
    msg = "TURN BACK " + String("(") + String(pitch, 5) + String(")") + " WITH PWM OF " + String(powerCalc(pitch));
    Serial.println(msg);
    driveReverse(pitch);
  } else {
    halt();
  }
}

/**
 * Derive PWM signal for L293 enable
 * Takes into account PWN range, and through some really bad math,
 *  smoothens (is this even a word) the signal on rise/drops
 */
int powerCalc(float pitch) {

  int powerDrop = 0;
  // just like most things, based on a pure, dumb, assumption
  int multiplier = 64;

  // note to self, perhaps consider quadratic model? logarithmetic too steep...
  float floatPwr = multiplier * log10(abs(pitch)) - powerDrop;

  return (int) floatPwr;
}


void driveForward(int power) {
    digitalWrite(LMOTOR, HIGH);
    digitalWrite(LMOTOR_PWM, power);
    
    digitalWrite(RMOTOR, LOW);
    digitalWrite(RMOTOR_PWM, power);
}


void driveReverse(int power) {
    digitalWrite(LMOTOR, LOW);
    digitalWrite(LMOTOR_PWM, power);
    
    digitalWrite(RMOTOR, HIGH);
    digitalWrite(RMOTOR_PWM, power);
}

/**
 * Motor halt via enable low on both EN1 & EN2
 */
void halt() {
    digitalWrite(LMOTOR, 0);
    digitalWrite(RMOTOR, 0);
}


