#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Ultrasonic.h>
#include <Pixy2.h>
#include <Servo.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
Ultrasonic ultrasonicFront(48, 49);  
Pixy2 pixy;
Servo myServo;

// Pin Definitions
const int PWM_FL = 5;  const int IN1_FL = 23; const int IN2_FL = 22;
const int PWM_FR = 6;  const int IN1_FR = 25; const int IN2_FR = 24;
const int PWM_RL = 7;  const int IN1_RL = 26; const int IN2_RL = 27;
const int PWM_RR = 8;  const int IN1_RR = 28; const int IN2_RR = 29;
const int SERVO_PIN = 2;

// Control Parameters
int currentSpeed = 200;        
const float KP_HEADING = 3.0;  

// Global State Variables
float globalTargetHeading = 0.0;
float startupHeading = 0.0;

// Memorize grabbed item signature (0=default, 1=Sig1, 4=Sig4)
int grabbedSignature = 0;

unsigned long previousPixyMillis = 0;
const unsigned long pixyInterval = 200;

// State Machine Definition
enum RobotState {
  STATE_INIT,               // Init wait
  STATE_WAIT,               // Non-blocking wait
  STATE_FORWARD_1,          // Drive forward 1.2s
  STATE_TURN_TO_GRAB,       // Steer back to grabbing lane
  STATE_FORWARD_TO_GRAB,    // Drive forward until 10cm from wall
  STATE_WAIT_FOR_ITEM,      // Wait for item to enter grab coordinates
  STATE_GRAB_ITEM,          // Close gripper
  STATE_VERIFY_GRAB,        // Verify item grabbed successfully
  STATE_MISGRAB_FAILSAFE_1, // Dump white cube out of lane
  STATE_MISGRAB_FAILSAFE_2, // Return to lane after dump
  STATE_BACKWARD,           // Reverse
  STATE_TURN_TO_PLACE,      // Steer to placement zone based on signature
  STATE_FORWARD_TO_PLACE,   // Drive to placement zone
  STATE_PLACE_ITEM,         // Release item
  STATE_TURN_180_AND_RETURN // Turn 180 degrees and restart
};

RobotState currentState = STATE_INIT;

// State Machine Timer Variables
RobotState nextStateAfterWait = STATE_INIT;
unsigned long waitStartTime = 0;
unsigned long currentWaitDuration = 0;

// Helper function for non-blocking delay
void startWait(unsigned long ms, RobotState nextState) {
  waitStartTime = millis();
  currentWaitDuration = ms;
  nextStateAfterWait = nextState;
  currentState = STATE_WAIT;
}

void setup() {
  myServo.attach(SERVO_PIN);
  myServo.write(0); // Ensure gripper is released at boot

  int pixyRetry = 0;
  while (pixy.init() != 0 && pixyRetry < 5) {
    delay(1000); 
    pixyRetry++;
  }

  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    while (1);
  }
  bno.setExtCrystalUse(true);

  for(int pin = 22; pin <= 29; pin++) pinMode(pin, OUTPUT);
  pinMode(PWM_FL, OUTPUT); pinMode(PWM_FR, OUTPUT);
  pinMode(PWM_RL, OUTPUT); pinMode(PWM_RR, OUTPUT);
  stopMotors();

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  while (gyro < 3) {
    bno.getCalibration(&system, &gyro, &accel, &mag);
  }

  startupHeading = getHeading();
  globalTargetHeading = startupHeading;
}

void loop() {
  switch (currentState) {

    case STATE_WAIT:
      if (millis() - waitStartTime + 100 >= currentWaitDuration) {
        currentState = nextStateAfterWait;
      }
      break;

    case STATE_INIT:
      startWait(0, STATE_FORWARD_1);
      break;

    case STATE_FORWARD_1:
      driveStraightForTime(1000);
      startWait(200, STATE_TURN_TO_GRAB);
      break;

    case STATE_TURN_TO_GRAB:
      if (grabbedSignature == 1) {
        turnDegrees(-90.0);
      } else {
        turnDegrees(90.0);
      }
      startWait(200, STATE_FORWARD_TO_GRAB);
      break;

    case STATE_FORWARD_TO_GRAB:
      currentSpeed = 85;
      driveStraightUntilDistance(4);
      currentSpeed = 200;
      startWait(200, STATE_WAIT_FOR_ITEM);
      break;

    case STATE_WAIT_FOR_ITEM: {
      if (millis() - previousPixyMillis >= pixyInterval) {
        previousPixyMillis = millis();
        pixy.ccc.getBlocks();
        
        if (pixy.ccc.numBlocks) {
          for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            int x = pixy.ccc.blocks[i].m_x;
            int y = pixy.ccc.blocks[i].m_y;
            int signature = pixy.ccc.blocks[i].m_signature;

            // Check if target signature entered the grab zone
            if ((signature == 1 || signature == 4) && (x >= 235 && x <= 280) && (y >= 20 && y <= 60)) {
              grabbedSignature = signature;
              currentState = STATE_GRAB_ITEM;
            }
          }
        }
      }
      break;
    }

    case STATE_GRAB_ITEM:
      myServo.write(160);
      startWait(2000, STATE_VERIFY_GRAB); // Wait for servo to close
      break;

    case STATE_VERIFY_GRAB: {
      grabbedSignature = verifyTargetInZone(70, 180, 70, 200);
      
      // Verify if valid target was grabbed
      if (grabbedSignature == 0) {
        currentState = STATE_MISGRAB_FAILSAFE_1;
      } else {
        currentState = STATE_BACKWARD;
      }
      break;
    }

    case STATE_MISGRAB_FAILSAFE_1:
      driveBackwardForTime(50);
      turnDegrees(-90.0);       // Turn to dump cube
      myServo.write(0);         // Open gripper
      startWait(500, STATE_MISGRAB_FAILSAFE_2); 
      break;
    
    case STATE_MISGRAB_FAILSAFE_2:
      turnDegrees(90.0);        // Turn back to lane
      currentState = STATE_FORWARD_TO_GRAB;
      break;

    case STATE_BACKWARD:
      driveBackwardForTime(250);
      startWait(200, STATE_TURN_TO_PLACE);
      break;

    case STATE_TURN_TO_PLACE:
      if (grabbedSignature == 1) {
        turnDegrees(-90.0);
      } else if (grabbedSignature == 4) {
        turnDegrees(90.0);
      } else {
        turnDegrees(-90.0);
      }
      startWait(200, STATE_FORWARD_TO_PLACE);
      break;

    case STATE_FORWARD_TO_PLACE:
      driveStraightUntilDistance(28);
      startWait(200, STATE_PLACE_ITEM);
      break;

    case STATE_PLACE_ITEM:
      myServo.write(0);
      startWait(800, STATE_TURN_180_AND_RETURN);
      break;

    case STATE_TURN_180_AND_RETURN:
      turnDegrees(180.0);
      startWait(400, STATE_FORWARD_1); 
      break;
  }
}

// Auxiliary Movement Functions

void driveStraightForTime(unsigned long ms) {
  unsigned long startTime = millis();

  while (millis() - startTime < ms) {
    float error = getHeadingError(globalTargetHeading, getHeading());
    int correction = constrain((int)(error * KP_HEADING), -40, 40);

    int leftSpeed = constrain(currentSpeed - correction, 0, 255);
    int rightSpeed = constrain(currentSpeed + correction, 0, 255);

    setWheel(PWM_FL, IN1_FL, IN2_FL, true, leftSpeed);
    setWheel(PWM_RL, IN1_RL, IN2_RL, true, leftSpeed);
    setWheel(PWM_FR, IN1_FR, IN2_FR, true, rightSpeed);
    setWheel(PWM_RR, IN1_RR, IN2_RR, true, rightSpeed);
  }
  stopMotors();
}

void driveBackwardForTime(unsigned long ms) {
  unsigned long startTime = millis();

  while (millis() - startTime < ms) {
    float current = getHeading();
    float error = getHeadingError(globalTargetHeading, current);

    int correction = constrain((int)(error * KP_HEADING), -40, 40);

    int leftSpeed = constrain(currentSpeed + correction, 0, 255);
    int rightSpeed = constrain(currentSpeed - correction, 0, 255);

    setWheel(PWM_FL, IN1_FL, IN2_FL, false, leftSpeed);
    setWheel(PWM_RL, IN1_RL, IN2_RL, false, leftSpeed);
    setWheel(PWM_FR, IN1_FR, IN2_FR, false, rightSpeed);
    setWheel(PWM_RR, IN1_RR, IN2_RR, false, rightSpeed);
  }
  stopMotors();
}

void driveStraightUntilDistance(int targetCM) {
  unsigned long lastPingTime = 0;
  const int REQUIRED_CONFIRMATIONS = 3;
  int validCount = 0;                  

  int initialDist = ultrasonicFront.read();

  while (true) {
    // Read sensor every 10ms
    if (millis() - lastPingTime >= 10) {
      lastPingTime = millis();
      int distance = ultrasonicFront.read();

      if (distance > 0 && distance <= targetCM) {
        validCount++;
        if (validCount >= REQUIRED_CONFIRMATIONS) break;
      }
      else if (distance > targetCM) {
        validCount = 0;
      }
    }

    // Update motors continuously while waiting
    float current = getHeading();
    float error = getHeadingError(globalTargetHeading, current);

    int correction = constrain((int)(error * KP_HEADING), -40, 40);
    int leftSpeed = constrain(currentSpeed - correction, 0, 255);
    int rightSpeed = constrain(currentSpeed + correction, 0, 255);

    setWheel(PWM_FL, IN1_FL, IN2_FL, true, leftSpeed);
    setWheel(PWM_RL, IN1_RL, IN2_RL, true, leftSpeed);
    setWheel(PWM_FR, IN1_FR, IN2_FR, true, rightSpeed);
    setWheel(PWM_RR, IN1_RR, IN2_RR, true, rightSpeed);
  }
  stopMotors();
}

void turnDegrees(float angleChange) {
  globalTargetHeading += angleChange;

  while (globalTargetHeading >= 360.0) globalTargetHeading -= 360.0;
  while (globalTargetHeading < 0.0) globalTargetHeading += 360.0;

  const float KP = 2.5;  
  const float KI = 0.5;  
  const float KD = 1.5;  

  const int MAX_TURN_SPEED = 130;
  const int MIN_BASE_SPEED = 65;  

  float integral = 0;
  float lastError = 0;

  unsigned long stableTime = 0;  
  unsigned long lastTime = millis();
  unsigned long turnStartTime = millis();

  while (true) {
    if (millis() - turnStartTime > 2000) {
      break;
    }

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;              
    lastTime = now;

    float current = getHeading();
    float error = getHeadingError(globalTargetHeading, current);

    if (abs(error) <= 1.0) {
      stopMotors();
      if (stableTime == 0) stableTime = now;
      else if (now - stableTime > 200) break;
      continue;
    } else {
      stableTime = 0;
    }

    integral += error * dt;
    if (error * lastError < 0) integral = 0;
    integral = constrain(integral, -100.0, 100.0);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = (error * KP) + (integral * KI) + (derivative * KD);

    int turnSpeed = abs(output);
    turnSpeed = constrain(turnSpeed + MIN_BASE_SPEED, MIN_BASE_SPEED, MAX_TURN_SPEED);

    if (output > 0) {
      setWheel(PWM_FL, IN1_FL, IN2_FL, true, turnSpeed);
      setWheel(PWM_RL, IN1_RL, IN2_RL, true, turnSpeed);
      setWheel(PWM_FR, IN1_FR, IN2_FR, false, turnSpeed);
      setWheel(PWM_RR, IN1_RR, IN2_RR, false, turnSpeed);
    } else {        
      setWheel(PWM_FL, IN1_FL, IN2_FL, false, turnSpeed);
      setWheel(PWM_RL, IN1_RL, IN2_RL, false, turnSpeed);
      setWheel(PWM_FR, IN1_FR, IN2_FR, true, turnSpeed);
      setWheel(PWM_RR, IN1_RR, IN2_RR, true, turnSpeed);
    }
  }

  stopMotors();
}

float getHeading() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.x();
}

float getHeadingError(float target, float current) {
  float error = target - current;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

void setWheel(int pwmPin, int in1Pin, int in2Pin, bool isForward, int speed) {
  if (speed == 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 255);
  } else if (isForward) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, speed);
  }
}

void stopMotors() {
  setWheel(PWM_FL, IN1_FL, IN2_FL, true, 0);
  setWheel(PWM_FR, IN1_FR, IN2_FR, true, 0);
  setWheel(PWM_RL, IN1_RL, IN2_RL, true, 0);
  setWheel(PWM_RR, IN1_RR, IN2_RR, true, 0);
}

int verifyTargetInZone(int minX, int maxX, int minY, int maxY) {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int checkX = pixy.ccc.blocks[i].m_x;
      int checkY = pixy.ccc.blocks[i].m_y;

      // Return signature if block is within specified bounds
      if (checkX > minX && checkX < maxX && checkY > minY && checkY < maxY) {
        return pixy.ccc.blocks[i].m_signature;
      }
    }
  }

  // Return 0 if no block found in zone
  return 0;
}
