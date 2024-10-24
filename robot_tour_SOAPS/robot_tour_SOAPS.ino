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
#include "MPU6050.h"
MPU6050 mpu;
double GYRO_TURN_CONSTANT = 0.0000109;
//double GYRO_TURN_CONSTANT = 1;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


#include <FastLED.h>
#define NUM_LEDS 1    // Number of LEDs in strip (just 1)
#define PIN_LED 4    // Pin for data communications
// Define the array of leds
CRGB leds[NUM_LEDS];

int encoderBPins[] = {PIN_ENCODER_LEFT_FRONT_B, PIN_ENCODER_RIGHT_FRONT_B, PIN_ENCODER_LEFT_BACK_B, PIN_ENCODER_RIGHT_BACK_B};
int lastEncoderTicks[] = {0, 0, 0, 0};//LF, RF, LB, RB
int encoderTicks[] = {0, 0, 0, 0};//LF, RF, LB, RB

int STRAIGHT_SLOW = 60;
int STRAIGHT_FAST = 80;
double FORWARD_TOLERANCE = 2;
double FORWARD_LR_TOLERANCE = 0.01;
double FORWARD_LATERAL_TOLERANCE = 0.25;
int TURN_SPEED = 600;
int TURN_SLOW = 350;
int FULL_TURN_TICKS = 740;
double TURN_TOLERANCE = 0.02;

double robotX = 0;
double robotY = 0;
double robotTheta = 0;
double TICKS_PER_CM = 11.66;
double TICKS_PER_REV = 740;
double WHEEL_RADIUS = 7.7;


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
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("Calibrating MPU...");
  mpu.CalibrateGyro();

  delay(1000);//DO NOT REMOVE
  leds[0] = CRGB::Green;
  FastLED.show();  
 
  Serial.println("Starting...");
  startMillis = millis();
  //2, 8, 9, 15
  forward(190);//thru A
  turn(-90);//right to D
  forward(50);//to D
  turn(-90);//right to C
  forward(150);//to C
  turn(90);//left to C
  forward(50);//to C
  turn(90);//left to B
  forward(100);//to B
  turn(-90);//right to B
  forward(30);//kiss B
  backward(30);//pull out of B
  turn(90);//left to end
  forward(30);//to end

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

void setMotors(int left, int right) {
  digitalWrite(PIN_MOTOR_ENABLE, HIGH);//enable motor controller
  digitalWrite(PIN_MOTOR_LEFT_IN, (left > 0) ? LOW : HIGH);//set right wheels to CW/CCW
  analogWrite(PIN_MOTOR_LEFT_PWM, abs(left));//set right wheels to left speed
  digitalWrite(PIN_MOTOR_RIGHT_IN, (right > 0) ? LOW : HIGH);//set right wheels to CW/CCW
  analogWrite(PIN_MOTOR_RIGHT_PWM, abs(right));//set right wheels to right speed
}

void forward(double distance) {
  robotX = 0;
  robotY = 0;
  //find theta offset
  while (robotTheta > PI/4) {
    robotTheta -= PI/2;
  }
  while (robotTheta < -PI/4) {
    robotTheta += PI/2;
  }
  robotTheta = 0;
  while (robotX < distance) {
    printInfo();
    updatePose();
    if (robotTheta > FORWARD_LR_TOLERANCE) {//going left, correct right
      Serial.println("Correcting left...");
      setMotors(STRAIGHT_FAST, STRAIGHT_SLOW);
    } else if (robotTheta < -FORWARD_LR_TOLERANCE) {
      Serial.println("Correcting right...");
      setMotors(STRAIGHT_SLOW, STRAIGHT_FAST);
    } else {
      setMotors(STRAIGHT_SLOW, STRAIGHT_SLOW);
    }
  }
  setMotors(0, 0);
}

void backward(double distance) {
  robotX = 0;
  robotY = 0;
  //find theta offset
  while (robotTheta > PI/4) {
    robotTheta -= PI/2;
  }
  while (robotTheta < -PI/4) {
    robotTheta += PI/2;
  }
  while (robotX > -distance) {
    printInfo();
    updatePose();
    if (robotTheta > FORWARD_LR_TOLERANCE) {//going left, correct right
      setMotors(-STRAIGHT_SLOW, -STRAIGHT_FAST);
    } else if (robotTheta < -FORWARD_LR_TOLERANCE) {
      setMotors(-STRAIGHT_FAST, -STRAIGHT_SLOW);
    } else {
      setMotors(-STRAIGHT_SLOW, -STRAIGHT_SLOW);
    }
  }
  setMotors(0, 0);
}

//TURNING FUNC
void turn(double angle) {
  double angleRad = angle * (PI / 180.0);
  double startAngle = robotTheta;
  //find theta offset
  while (robotTheta > PI/4) {
    robotTheta -= PI/2;
  }
  while (robotTheta < -PI/4) {
    robotTheta += PI/2;
  }
  //double targetAngle = startAngle + angleRad;
  double targetAngle = angleRad;
  while (abs(robotTheta - targetAngle) > TURN_TOLERANCE) {
    updatePose();
    printInfo();
    Serial.println(robotTheta - targetAngle);
    double k = (robotTheta < (targetAngle) ? 1 : -1);
    if (abs(robotTheta - targetAngle) < 0.5) {
      k *= 0.5;
    }
    setMotors(-k * TURN_SPEED, k * TURN_SPEED);//positive k means left turn means positive angle change
  }
  setMotors(0, 0);
}


double lastTime = 0;
void updatePose() {
  int differentials[] = {encoderTicks[0] - lastEncoderTicks[0], 
  encoderTicks[1] - lastEncoderTicks[1], 
  encoderTicks[2] - lastEncoderTicks[2], 
  encoderTicks[3] - lastEncoderTicks[3]};

  double dForward = (differentials[0] + differentials[1])/2.0 / TICKS_PER_CM;//forward change in cm
  //double dTheta = (((differentials[1] + differentials[3]) - (differentials[0] + differentials[2]))/4.0 / TICKS_PER_CM) / WHEEL_RADIUS;
  //double dTime = (millis() - lastTime) / 1000.0;
  lastTime = millis();
  double dTheta = mpu.getRotationZ() * GYRO_TURN_CONSTANT;
  if (mpu.getIntDMPStatus() == 2) {
    
  }
  robotX += dForward * cos(robotTheta);
  robotY += dForward * sin(robotTheta);
  robotTheta += dTheta;
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
  Serial.print(robotTheta);
  Serial.print("(rad) / ");
  Serial.print(robotTheta * (180 / PI));
  Serial.print(" (deg)");
  Serial.println();
}

double DEGREES_TO_RADIANS = PI / 180.0;
double targetX = 100;
double targetY = 0;
double targetTheta = 90 * DEGREES_TO_RADIANS;
double DIST_TOLERANCE = 2;
double THETA_TOLERANCE = 0.05;
int trajState = 0;

void toTarget() {
  double distX = targetX - robotX;
  double distY = targetY - robotY;
  double distTheta = targetTheta - robotTheta;
  double dist = sqrt(distX*distX + distY*distY);
  if (dist > DIST_TOLERANCE) {
      if (robotTheta > 0) {//veering left, correct right
        setMotors(STRAIGHT_SLOW, STRAIGHT_FAST);
      } else {
        setMotors(STRAIGHT_FAST, STRAIGHT_SLOW);
      }
  } else {
    trajState = 1;
    if (abs(robotTheta - targetTheta) > THETA_TOLERANCE) {
      if (robotTheta < targetTheta) {//facing right, turn left
        setMotors(-TURN_SPEED, TURN_SPEED);
      } else {//facing left, turn right
        setMotors(TURN_SPEED, -TURN_SPEED);
      }
    } else {
      trajState = 2;
      setMotors(0, 0);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(PIN_TRIGGER) == LOW && (millis() - startMillis > 1500)) {
    Serial.println("Stop Trigger Pressed, Shutting Down...");
    digitalWrite(PIN_MOTOR_ENABLE, LOW);//disable motor controller
    delay(250);
    exit(0);
  }
  updatePose();
  printInfo();
}
