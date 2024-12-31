#include <PID_v1.h>

// Motor Pins
const int IN1 = 5;        // Motor direction pin 1
const int IN2 = 6;        // Motor direction pin 2

// Encoder Pins
const int encoderA = 2;   // Encoder A pin (interrupt)
const int encoderB = 3;   // Encoder B pin
volatile long encoderPos = 0;  // Tracks encoder position

// Encoder Constants
const double degreesPerPulse = 360.0 / 1500.0; // Degrees per encoder pulse

// PID Variables
double setpoint = 0;  // Desired position in degrees
double input = 0;     // Current position in degrees
double output = 0;    // PID output
double Kp = 1, Ki = 0.5, Kd = 0.1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Serial Command Variables
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;

const int MIN_PWM = 82;

void setup() {
  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Encoder Pins
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);

  // PID Setup
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Output range for PWM

  // Serial Communication
  Serial.begin(57600);
}

void loop() {
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  // Current position is in encoder counts
  input = encoderPos;

  // Compute PID output
  myPID.Compute();

  // Apply PID output to motor
  motorControl(output);

  // Send data over Serial port in comma delimited values, the last character should be new line.
  Serial.print(encoderPos); // First data
  Serial.print(",");       // Separator
  Serial.print(input);      // Second data
  Serial.print(",");       // Separator
  Serial.print(output);     // Third data
  Serial.print(",");        //Separator
  Serial.println(setpoint);   //Fourth data, followed by new line char
}

// Interrupt Service Routine (ISR) for Encoder
void encoderISR() {
  static int lastBState = LOW;
  int bState = digitalRead(encoderB);
  if (bState != lastBState) {
    encoderPos += (digitalRead(encoderA) == bState) ? 1 : -1;
    lastBState = bState;
  }
}

// Motor Control based on PID Output
void motorControl(double speed) {
    int motorSpeed = abs(speed);
    
    // Apply minimum PWM
    if (motorSpeed > 0 && motorSpeed < MIN_PWM){
        motorSpeed = MIN_PWM;
    }
   if (speed > 0) {
     analogWrite(IN1, motorSpeed);
     digitalWrite(IN2, LOW);
   } else if (speed < 0) {
     analogWrite(IN2, motorSpeed);
     digitalWrite(IN1, LOW);
   } else {
     digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
   }
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
   arg1 = atoi(argv1);
    
    switch(cmd) {
    case 'e':
        Serial.print("Encoder Value: ");
        Serial.println(encoderPos);
         Serial.print("Position (Degrees): ");
        Serial.println(encoderPos * (360.0 / 1500.0),2);
        break;
    case 'p':
        setpoint = arg1;
        Serial.print("Target Position set to: ");
        Serial.println(setpoint);
        break;
    default:
        Serial.println("Invalid Command");
        break;
    }
}