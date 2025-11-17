// L298N Driver Pin
#define MOTOR_ENA 4   // Use a PWM-capable pin
#define MOTOR_IN1 27
#define MOTOR_IN2 32

// Encoder Pins
#define ENCODER_PINA 34
#define ENCODER_PINB 35

// Encoder Counter
volatile long encoderCount = 0;
volatile double position = 0;

// Serial Monitor command for rotation direction
String command;

// Encoder interrupt function
void IRAM_ATTR encoderInterrupt() {
  if (digitalRead(ENCODER_PINA) != digitalRead(ENCODER_PINB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Function to calculate position in degrees
double getPosition() {
  // Replace 1000.0 with actual counts per revolution
  //const float countsPerRevolution = 1000.0; // <-- Replace this after observation
  //position = float(encoderCount) * 360.0 / 193 countsPerRevolution;
  position = float(encoderCount)*360.0/194;
  // Normalize position to [0, 360)
  if (position < 0) {
    position += 360;
  } else if (position >= 360) {
    position -= 360;
  }

  return position;
}

// Serial print function
void getState() {
  Serial.print("Count: ");
  Serial.println(encoderCount);
  Serial.print("Position: ");
  Serial.println(getPosition());
  delay(50); // For better readability in Serial Monitor
}

void setup() {
  // Set L298N motor pins
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Set encoder pins
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);

  // Attach interrupt to encoder pin A
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderInterrupt, CHANGE);

  // Start serial communication
  Serial.begin(115200);
}

void loop() {
  // Read user command from Serial Monitor
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim(); // Clean whitespace and carriage return

    if (command == "F") {
      // Forward direction
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_ENA, 120); // Set motor speed
    } else if (command == "B") {
      // Backward direction
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
      analogWrite(MOTOR_ENA, 120); // Set motor speed
    }
  }

  // Print position and encoder count
  Serial.print("Count: ");
  Serial.println(encoderCount);
  Serial.print("Position: ");
  Serial.println(getPosition());
  getState();
  delay(10);

  // Optional: Reset encoder count if out of expected bounds
  if (position >= 360 || position < 0) {
    encoderCount = 0;
  }
}