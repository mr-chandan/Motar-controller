#include <PID_v1.h>
#include <math.h>

// Motor Pins (L298N)
const int ENA_L = 9;    // Enable pin (PWM) for Left motor
const int IN1_L = 5;    // Motor direction pin 1 for left motor
const int IN2_L = 6;    // Motor direction pin 2 for left motor
const int ENA_R = 10;   // Enable pin (PWM) for Right motor
const int IN1_R = 7;    // Motor direction pin 1 for right motor
const int IN2_R = 8;    // Motor direction pin 2 for right motor

// Encoder Pins
const int encoderA_L = 2;   // Encoder A pin for left motor (interrupt)
const int encoderB_L = 3;   // Encoder B pin for left motor
const int encoderA_R = 18;  // Encoder A pin for right motor (interrupt)(A4)
const int encoderB_R = 19;  // Encoder B pin for right motor(A5)
volatile long encoderPosL = 0;  // Tracks left encoder position
volatile long encoderPosR = 0;  // Tracks right encoder position

// Encoder Constants
const double degreesPerPulse = 360.0 / 750.0; // Degrees per encoder pulse
const double radiansPerPulse = 2 * M_PI / 750.0; // Radians per encoder pulse


// PID Variables
double setpointL = 0;      // Desired position in degrees for the left motor
double inputL = 0;         // Current position in degrees for the left motor
double outputL = 0;        // PID output for the left motor
double setpointR = 0;      // Desired position in degrees for the right motor
double inputR = 0;         // Current position in degrees for the right motor
double outputR = 0;        // PID output for the right motor

double Kp = 0.6, Ki = 0.0001, Kd = 0.01;
PID myPIDL(&inputL, &outputL, &setpointL, Kp, Ki, Kd, DIRECT); //PID for Left Motor
PID myPIDR(&inputR, &outputR, &setpointR, Kp, Ki, Kd, DIRECT); //PID for Right Motor
// Serial Command Variables
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
double arg1;
double arg2;

const int MIN_PWM = 82;
const int MAX_PWM = 90; // Added Maximum PWM constant
unsigned long lastMotorCommand = 0;

const int AUTO_STOP_INTERVAL = 2000;
bool moving = false;

void setup() {
  // Motor Pins
  pinMode(ENA_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(ENA_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  // Encoder Pins
  pinMode(encoderA_L, INPUT_PULLUP);
  pinMode(encoderB_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA_L), encoderISR_L, CHANGE);

  pinMode(encoderA_R, INPUT_PULLUP);
  pinMode(encoderB_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA_R), encoderISR_R, CHANGE);

  // PID Setup
  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetOutputLimits(-255, 255); // Output range for PWM for Left motor
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetOutputLimits(-255, 255); // Output range for PWM for Right motor

  // Serial Communication
  Serial.begin(57600);
}

void loop() {
  // Serial Input
  while (Serial.available() > 0) {
    chr = Serial.read();
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    } else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    } else {
      if (arg == 0) {
        cmd = chr;
      } else if (arg == 1) {
        argv1[index] = chr;
        index++;
      } else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  // Update position
  inputL = convertEncoderToDegrees(encoderPosL);
  inputR = convertEncoderToDegrees(encoderPosR);

  // Check if auto stop condition is met
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    motorControl(0, 0); // Stop both motors if timeout
    moving = false;
    return;
  }
  // Check if within tolerance
  double tolerance = 0.5;

  if (moving)
  {
    if (abs(setpointL - inputL) <= tolerance && abs(setpointR - inputR) <= tolerance) {
      // Stop the motor if within tolerance
      motorControl(0, 0); // Ensure both motors stop
      moving = false;
      return;
    }

    // Compute PID output
    myPIDL.Compute();
    myPIDR.Compute();
    motorControl(outputL, outputR);
  }
  // Serial.print("Left Motor Position (Deg): ");
  //  Serial.println(inputL,2);
  //   Serial.print("Right Motor Position (Deg): ");
  //  Serial.println(inputR,2);
}

// Interrupt Service Routine (ISR) for Left Encoder
void encoderISR_L() {
  static int lastBState = LOW;
  int bState = digitalRead(encoderB_L);
  if (bState != lastBState) {
    encoderPosL += (digitalRead(encoderA_L) == bState) ? 1 : -1;
    lastBState = bState;
  }
}

// Interrupt Service Routine (ISR) for Right Encoder
void encoderISR_R() {
  static int lastBState = LOW;
  int bState = digitalRead(encoderB_R);
  if (bState != lastBState) {
    encoderPosR += (digitalRead(encoderA_R) == bState) ? 1 : -1;
    lastBState = bState;
  }
}

// Motor Control based on PID Output
void motorControl(double speedL, double speedR) {
  int motorSpeedL = abs(speedL);
  int motorSpeedR = abs(speedR);

  // Apply minimum PWM
  if (motorSpeedL > 0 && motorSpeedL < MIN_PWM) {
    motorSpeedL = MIN_PWM;
  }
  if (motorSpeedR > 0 && motorSpeedR < MIN_PWM) {
    motorSpeedR = MIN_PWM;
  }
  // Limit maximum PWM
  if (motorSpeedL > MAX_PWM) {
    motorSpeedL = MAX_PWM;
  }
  if (motorSpeedR > MAX_PWM) {
    motorSpeedR = MAX_PWM;
  }

  if (speedL > 0) {
    setMotorDirection(1, 0); // 1 means forward, 0 means left
  } else if (speedL < 0) {
    setMotorDirection(-1, 0); // -1 means reverse, 0 means left
  } else {
    setMotorDirection(0, 0); // 0 means stop, 0 means left
  }
  if (speedR > 0) {
    setMotorDirection(1, 1); // 1 means forward, 1 means right
  } else if (speedR < 0) {
    setMotorDirection(-1, 1); // -1 means reverse, 1 means right
  } else {
    setMotorDirection(0, 1); // 0 means stop, 1 means right
  }

  analogWrite(ENA_L, motorSpeedL);
  analogWrite(ENA_R, motorSpeedR);
}

void setMotorDirection(int direction, int motor) {
  if (motor == 0)
  {
    switch (direction) {
    case 1:
      digitalWrite(IN1_L, HIGH);
      digitalWrite(IN2_L, LOW);
      break;
    case -1:
      digitalWrite(IN1_L, LOW);
      digitalWrite(IN2_L, HIGH);
      break;
    case 0:
    default:
      digitalWrite(IN1_L, LOW);
      digitalWrite(IN2_L, LOW);
    }
  }
  else if (motor == 1)
  {
    switch (direction) {
    case 1:
      digitalWrite(IN1_R, HIGH);
      digitalWrite(IN2_R, LOW);
      break;
    case -1:
      digitalWrite(IN1_R, LOW);
      digitalWrite(IN2_R, HIGH);
      break;
    case 0:
    default:
      digitalWrite(IN1_R, LOW);
      digitalWrite(IN2_R, LOW);
    }
  }
}

double convertEncoderToDegrees(long encoderValue) {
  return encoderValue * degreesPerPulse;
}

double convertEncoderToRadians(long encoderValue) {
  return encoderValue * radiansPerPulse;
}

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

void runCommand() {
  arg1 = atof(argv1); // Use atof for double conversion
  arg2 = atof(argv2);

  switch (cmd) {
    case 'e':
      Serial.print("Left Encoder Value: ");
      Serial.println(encoderPosL);
       Serial.print("Left Position (Deg): ");
      Serial.println(convertEncoderToDegrees(encoderPosL), 2);
      Serial.print("Left Position (Rad): ");
      Serial.println(convertEncoderToRadians(encoderPosL), 2);
      Serial.print("Right Encoder Value: ");
      Serial.println(encoderPosR);
      Serial.print("Right Position (Deg): ");
      Serial.println(convertEncoderToDegrees(encoderPosR), 2);
       Serial.print("Right Position (Rad): ");
      Serial.println(convertEncoderToRadians(encoderPosR), 2);
      break;
    case 'p':
      setpointL = arg1 * 180.0 / M_PI; // Convert radians to degrees
      setpointR = arg2 * 180.0 / M_PI;
       moving = true;
      lastMotorCommand = millis();
       Serial.print("Target Position set to: Left = ");
      Serial.print(arg1,2);
       Serial.print(" Rad, Right = ");
      Serial.println(arg2,2);
      break;
    case 'm': // Set speed command (still interpreted as degrees per loop internally)
      setpointL = arg1 * 180.0 / M_PI;
      setpointR = arg2 * 180.0 / M_PI;
      lastMotorCommand = millis();
      moving = true;
      Serial.print("Target Speed set to: Left = ");
      Serial.print(arg1,2);
        Serial.print(" Rad, Right = ");
      Serial.println(arg2,2);
      break;
    case 'r': // Reset encoders
      encoderPosL = 0;
      encoderPosR = 0;
      Serial.println("Encoders reset.");
      break;
    case 'o':  // Set raw pwm
       lastMotorCommand = millis();
      motorControl(arg1, arg2);
      Serial.print("PWM set to: Left = ");
      Serial.print(arg1);
      Serial.print(", Right = ");
      Serial.println(arg2);
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}
