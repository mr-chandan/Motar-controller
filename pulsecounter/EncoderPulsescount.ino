// Encoder Pins
const int encoderA = 2;   // Encoder A pin (interrupt)
const int encoderB = 3;   // Encoder B pin
volatile long encoderPos = 0;  // Tracks encoder position

void setup() {
  // Encoder Pins
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);

  // Serial Communication
  Serial.begin(57600);
}

void loop() {
  // Display encoder values
  Serial.print("Encoder Pulses: ");
  Serial.println(encoderPos);

  delay(500); // Adjust refresh rate for display
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
