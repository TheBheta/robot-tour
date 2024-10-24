//Robot Tour by Chinmay Govind & Reuben James for Cumberland Valley Science Olympiad

#define PIN_MOTOR_RIGHT_PWM   5 //analog
#define PIN_MOTOR_RIGHT_IN    7 //digital
#define PIN_MOTOR_LEFT_PWM    6 //analog
#define PIN_MOTOR_LEFT_IN     8 //digital
#define PIN_ENCODER_LEFT_FRONT_A    18 //digital (yellow wire), goes high on encoder pulse
#define PIN_ENCODER_RIGHT_FRONT_A    19 //digital (yellow wire), goes high on encoder pulse
#define PIN_ENCODER_LEFT_FRONT_B 22 //digital (white wire), high on forward tick, low on reverse tick
#define PIN_ENCODER_RIGHT_FRONT_B 24 //digital (white wire), high on forward tick, low on reverse tick
#define PIN_ENCODER_LEFT_BACK_B 26 //digital (white wire), high on forward tick, low on reverse tick
#define PIN_ENCODER_RIGHT_BACK_B 28 //digital (white wire), high on forward tick, low on reverse tick
#define PIN_MOTOR_ENABLE      3 //digital

#define PIN_TRIGGER 2
long startMillis = 0;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define INTERRUPT_PIN 20

MPU6050 mpu;
double GYRO_TURN_CONSTANT = 0.0000109;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <FastLED.h>
#define NUM_LEDS 1    // Number of LEDs in strip (just 1)
#define PIN_LED 4    // Pin for data communications

#define NORTH 0
#define EAST -90
#define WEST 90
#define SOUTH 180
// Define the array of leds
CRGB leds[NUM_LEDS];

int encoderBPins[] = {PIN_ENCODER_LEFT_FRONT_B, PIN_ENCODER_RIGHT_FRONT_B, PIN_ENCODER_LEFT_BACK_B, PIN_ENCODER_RIGHT_BACK_B};
int lastEncoderTicks[] = {0, 0, 0, 0};//LF, RF, LB, RB
int encoderTicks[] = {0, 0, 0, 0};//LF, RF, LB, RB

int STRAIGHT_SLOW = 60;//60 80 default, 100 120 on hard surfaces
int STRAIGHT_FAST = 80;
int STRAIGHT_BASE_SLOW = 300;
int STRAIGHT_BASE_FAST = 320;
double FORWARD_LR_TOLERANCE = 0.01;
int TURN_SPEED = 600;
int TURN_SLOW = 60;
int FULL_TURN_TICKS = 740;
double TURN_TOLERANCE = 0.04;
double TURN_TIME = 2000;

double robotX = 0;
double robotY = 0;
double robotTheta = 0;
double targetTheta = 0;
double thetaOffset = 0;
double TICKS_PER_CM = 9.51;
double TICKS_PER_REV = 740;
double WHEEL_RADIUS = 7.7;

Quaternion quat;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];  

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, PIN_LED>(leds, NUM_LEDS);  // Configure the LED    
  FastLED.setBrightness(1);  
  for (int i = 0; i < 3; i++) {
    leds[0] = CRGB::Black;
    FastLED.show();    
    delay(200);
    leds[0] = CRGB::Red;
    FastLED.show();     
    delay(200); 
  }
  Serial.println("\nWaiting for Trigger Press...");
  while (digitalRead(PIN_TRIGGER) == HIGH) {
    //wait
  }
  Serial.println("Trigger Pressed...");
  leds[0] = CRGB::Yellow;
  FastLED.show();  
  
  //attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_FRONT_A), readLFEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_FRONT_A), readRFEncoder, RISING);
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_BACK_A), readLBEncoder, RISING);
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_BACK_A), readRBEncoder, RISING);

  //mpu setup
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.println("Testing MPU connection...");
  mpu.initialize();
  leds[0] = CRGB::Orange;
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("Calibrating MPU...");
  //mpu.CalibrateGyro();
  //mpu.CalibrateAccel();

  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(10);
      mpu.CalibrateGyro(10);
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
  delay(1000);//DO NOT REMOVE
  leds[0] = CRGB::Green;
  FastLED.show();  
  Serial.println("Starting...");
  startMillis = millis();
  //RUN CODE GOES HERE (TRIPATHI)
  //start: 16
  //gates: 4, 5, 12
  //time: 58
  //end: 13
  robotX = 0;
  robotY = 0;
  robotTheta = 0;
  setSpeed(200);
  TURN_TIME = 2000;
  bank(EAST, true, true);
  forward(39 + 50);//forward to A
  bank(NORTH, false, true);//into A
  forward(70);
  backward(70);
  bank(SOUTH, false, false);
  turnTo(WEST, true);
  forward(50);
  bank(NORTH, true, true);
  forward(50);
  bank(SOUTH, true, true);
  bank(NORTH, false, false);
  backward(50 + 39);

} 

void setSpeed(int val) {
  STRAIGHT_SLOW = val;
  STRAIGHT_FAST = val + 20;
}


void readLFEncoder() {
  readEncoder(0);
}
void readRFEncoder() {
  readEncoder(1);
}
void readLBEncoder() {
  readEncoder(2);
}
void readRBEncoder() {
  readEncoder(3);
}
void readEncoder(int wheel) {//0, 1, 2, 3 = LF, RF, LB, RB
  //if going forward, increase motor ticks. otherwise, decrement
  if (digitalRead(encoderBPins[wheel]) == (wheel%2)) { //even wheels need LOW, odd wheels need HIGH
    encoderTicks[wheel]++;
    //Serial.println("Incrementing encoder ticks...");
  } else {
    encoderTicks[wheel]--;
    //Serial.println("Decrementing encoder ticks...");
  }
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setMotors(int left, int right) {
  digitalWrite(PIN_MOTOR_ENABLE, HIGH);//enable motor controller
  digitalWrite(PIN_MOTOR_LEFT_IN, (left > 0) ? LOW : HIGH);//set right wheels to CW/CCW
  analogWrite(PIN_MOTOR_LEFT_PWM, abs(left));//set right wheels to left speed
  digitalWrite(PIN_MOTOR_RIGHT_IN, (right > 0) ? LOW : HIGH);//set right wheels to CW/CCW
  analogWrite(PIN_MOTOR_RIGHT_PWM, abs(right));//set right wheels to right speed
}

void findThetaOffset() {
  thetaOffset = 0;
  while (robotTheta + thetaOffset > PI/4) {
    thetaOffset -= PI/2;
  }
  while (robotTheta + thetaOffset < -PI/4) {
    thetaOffset += PI/2;
  }
}

void forward(double distance) {
  robotX = 0;
  robotY = 0;
  //find theta offset
  while (robotX < distance) {
    findThetaOffset();
    printInfo();
    updatePose();
    bool slowMode = distance - robotX < 8;
    int slowSpeed = slowMode ? STRAIGHT_BASE_SLOW : STRAIGHT_SLOW;
    int fastSpeed = slowMode ? STRAIGHT_BASE_FAST : STRAIGHT_FAST;
    if ((robotTheta - targetTheta) > FORWARD_LR_TOLERANCE) {//going left, correct right
      //Serial.println("Correcting left...");
      setMotors(fastSpeed, slowSpeed);
    } else if ((robotTheta - targetTheta) < -FORWARD_LR_TOLERANCE) {
      //Serial.println("Correcting right...");
      setMotors(slowSpeed, fastSpeed);
    } else {
      setMotors(slowSpeed, slowSpeed);
    }
  }
  setMotors(0, 0);
}

void forward(double distance, double time) {
  robotX = 0;
  robotY = 0;
  double newDist = distance - 8;//cm
  double newTime = time - 1;//s
  double speed = min(200, 5 * (newDist / newTime));//PWM / (cm/s) * cm/s
  double forwardStart = millis();
  //find theta offset
  while (robotX < distance) {
    findThetaOffset();
    printInfo();
    updatePose();
    bool slowMode = distance - robotX < 8;
    int slowSpeed = slowMode ? STRAIGHT_BASE_SLOW : speed;
    int fastSpeed = slowMode ? STRAIGHT_BASE_FAST : speed + 20;
    if ((robotTheta - targetTheta) > FORWARD_LR_TOLERANCE) {//going left, correct right
      //Serial.println("Correcting left...");
      setMotors(fastSpeed, slowSpeed);
    } else if ((robotTheta - targetTheta) < -FORWARD_LR_TOLERANCE) {
      //Serial.println("Correcting right...");
      setMotors(slowSpeed, fastSpeed);
    } else {
      setMotors(slowSpeed, slowSpeed);
    }
  }
  setMotors(0, 0);
  while (millis() - forwardStart < time * 1000) {
    printInfo();
    delay(1);
  }
}

void forwardStall(double distance, double stallTo, double startTime) {
  double time = (millis() - startTime)/1000;
  if (time > stallTo) {
    setSpeed(200);
    forward(distance);
  } else {
    forward(distance, stallTo - time);
  }
}


void backward(double distance) {
  robotX = 0;
  robotY = 0;
  //find theta offset
  while (robotX > -distance) {
    findThetaOffset();
    printInfo();
    updatePose();
    bool slowMode = robotX + distance < 8;
    int slowSpeed = slowMode ? STRAIGHT_BASE_SLOW : STRAIGHT_SLOW;
    int fastSpeed = slowMode ? STRAIGHT_BASE_FAST : STRAIGHT_FAST;
    double differential = robotTheta - targetTheta;
    if (robotTheta == SOUTH) differential *= -1;
    if (differential > FORWARD_LR_TOLERANCE) {//going left, correct right
      setMotors(-slowSpeed, -fastSpeed);
    } else if (differential < -FORWARD_LR_TOLERANCE) {
      setMotors(-fastSpeed, -slowSpeed);
    } else { 
      setMotors(-slowSpeed, -slowSpeed);
    }
  }
  setMotors(0, 0);
}

void backward(double distance, double time) {
  robotX = 0;
  robotY = 0;
  double newDist = distance - 8;//cm
  double newTime = time - 1;//s
  double speed = min(255, 5 * (newDist / newTime));//PWM / (cm/s) * cm/s
  double backwardStart = millis();
  //find theta offset
  while (robotX > -distance) {
    findThetaOffset();
    printInfo();
    updatePose();
    bool slowMode = distance - robotX < 8;
    int slowSpeed = slowMode ? STRAIGHT_BASE_SLOW : speed;
    int fastSpeed = slowMode ? STRAIGHT_BASE_FAST : speed + 20;
    if ((robotTheta + thetaOffset) > FORWARD_LR_TOLERANCE) {//going left, correct right
      //Serial.println("Correcting left...");
      setMotors(-slowSpeed, -fastSpeed);
    } else if ((robotTheta + thetaOffset) < -FORWARD_LR_TOLERANCE) {
      //Serial.println("Correcting right...");
      setMotors(-fastSpeed, -slowSpeed);
    } else {
      setMotors(-slowSpeed, -slowSpeed);
    }
  }
  setMotors(0, 0);
  while (millis() - backwardStart < time * 1000) {
    printInfo();
    delay(10);
  }
}


//bank turns
void bank(double angle, bool right, bool forward) {
  double targetAngle = angle * (PI / 180.0);
  targetTheta = targetAngle;
  int lowSpeed = 100;
  int highSpeed = (int) (lowSpeed * (34 / 16.0));//scale wheel powers
  double leftSpeed = right ? highSpeed : lowSpeed;
  double rightSpeed = right ? lowSpeed : highSpeed;
  if (!forward) {
    leftSpeed *= -1;
    rightSpeed *= -1;
  }
  while (angularDistance(robotTheta, targetAngle) > TURN_TOLERANCE) {
    updatePose();
    setMotors(leftSpeed, rightSpeed);
  }
  setMotors(0, 0);
}

//TURNING FUNC
void turnTo(double angle, bool ccw) {
  double targetAngle = angle * (PI / 180.0);
  targetTheta = targetAngle;
  //double startAngle = robotTheta;
  //find theta offset
  //findThetaOffset();
  //double targetAngle = fstartAngle + angleRad;
  //double targetAngle = angleRad;
  double sign = ccw ? 1 : -1;
  double turnStart = millis();
  while (angularDistance(robotTheta, targetAngle) > TURN_TOLERANCE) {
    updatePose();
    //printInfo();
    //Serial.println(angularDistance(robotTheta, targetAngle));
    double k = sign * TURN_SPEED;
    if (angularDistance(robotTheta, targetAngle) < 0.5) {
      k = sign * TURN_SLOW;
    }
    setMotors(-k, k);//positive k means left turn means positive angle change
  }
  setMotors(0, 0);
  while (millis() - turnStart < TURN_TIME) {
    printInfo();
    delay(1);  
  }
}

double angularDistance(double a, double b) {
  return min(abs(a - b), abs(min(a, b) + 2 * PI - max(a, b)));
}

double lastTime = 0;
double lastGyroTheta = 0;
void updatePose() {
  int differentials[] = {encoderTicks[0] - lastEncoderTicks[0], 
  encoderTicks[1] - lastEncoderTicks[1], 
  encoderTicks[2] - lastEncoderTicks[2], 
  encoderTicks[3] - lastEncoderTicks[3]};

  double dForward = (differentials[0] + differentials[1])/2.0 / TICKS_PER_CM;//forward change in cm
  //double dTime = (millis() - lastTime) / 1000.0;

  lastTime = millis();
  robotX += dForward * cos(robotTheta + thetaOffset);
  robotY += dForward * sin(robotTheta + thetaOffset);
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
  // display Euler angles in degrees
          mpu.dmpGetQuaternion(&quat, fifoBuffer);
          mpu.dmpGetEuler(euler, &quat);
  }
  robotTheta = -euler[0];
  for (int i = 0; i < 4; i++) lastEncoderTicks[i] = encoderTicks[i];
}

void printInfo() {
  Serial.print("Encoder ticks: ");
  Serial.print(encoderTicks[0]);
  Serial.print(", ");
  Serial.print(encoderTicks[1]);
  Serial.print(" | Pose: ");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print(", ");
  Serial.print(robotTheta + thetaOffset);
  Serial.print("(rad) / ");
  Serial.print((robotTheta + thetaOffset) * (180 / PI));
  Serial.print(" (deg)");
  Serial.println();
}

void stop() {
    Serial.println("Stop Triggered, Shutting Down...");
    digitalWrite(PIN_MOTOR_ENABLE, LOW);//disable motor controller
    leds[0] = CRGB::Red;
    FastLED.show();  
    delay(250);
    exit(0);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(PIN_TRIGGER) == LOW && (millis() - startMillis > 1500)) {
    stop();
  }
  updatePose();
  printInfo();
}
