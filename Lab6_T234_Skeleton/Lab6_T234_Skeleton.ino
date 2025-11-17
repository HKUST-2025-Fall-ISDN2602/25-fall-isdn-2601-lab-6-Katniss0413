//L298N Driver Pin 
#define MOTOR_ENA 4  
#define MOTOR_IN1 27  
#define MOTOR_IN2 32  

//Encoder Pin 
#define ENCODER_PINA 34 
#define ENCODER_PINB 35 

//Encoder Counter
volatile long encoderCount = 0; 
volatile double position = 0.0; 
String command;

//PID constants
double kp = 1.0;    // Tuned for better response
double ki = 0.01;   // Tuned for better response
double kd = 0.5;    // Tuned for better response

//PID Variables 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double output;
double cumError, rateError;

// set desired position to 90 degrees
double setPoint = 90.0;

int Task = 4 ; 

void TaskConfig(){
  if(Task == 2){
    ki = 0;
    kd = 0;
  }
  else if(Task == 3)
    kd = 0;
} 

//PID Controller 
double computePID(double inp){     
  currentTime = millis();                              
  elapsedTime = (double)(currentTime - previousTime);  
  
  error = setPoint - inp;                              
  cumError += error * elapsedTime;                     
  rateError = (error - lastError) / elapsedTime;        

  double out = kp * error + ki * cumError + kd * rateError;  

  lastError = error;                                   
  previousTime = currentTime;                          

  return out;                                          
}


void IRAM_ATTR encoderInterrupt() {
  if (digitalRead(ENCODER_PINA) != digitalRead(ENCODER_PINB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}


//Serial Display Function 
void serialGraph(){
  Serial.print("Position:");
  Serial.print(getPosition());
  Serial.print(",");
  Serial.print("PID_output:");
  Serial.print(int(output));
  Serial.print(","); 
  Serial.print("setPoint:"); 
  Serial.print(setPoint);
  Serial.println("\t"); 
}

// To get the current position
double getPosition() {
  // Calculate position (assuming 400 counts per revolution - common for encoders)
  position = (double)encoderCount * 360.0 / 194; 

  // Normalize position to 0-360 degrees
  position = fmod(position, 360.0);
  if (position < 0) {
    position += 360.0;
  }
  
  return position;
}

void setup() {
  
  // Set motor control pins as outputs
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Encoder A pin mode for interrupt
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderInterrupt, CHANGE);

  // Encoder B pin mode
  pinMode(ENCODER_PINB, INPUT_PULLUP);

  // Configure PID for different tasks
  TaskConfig(); 

  // Set up serial communication
  Serial.begin(115200);

  // Initialize PID timing
  previousTime = millis();
}

void loop() {
  error = setPoint - getPosition();         
  output = computePID(getPosition());       

  // Control motor direction based on PID output
  if (output < 0) {
    // Reverse direction
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  } else {
    // Forward direction
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }

  // Control motor speed (constrain to 0-255)
  int speed = constrain(abs(output), 0, 255);
  analogWrite(MOTOR_ENA, speed);

  // Display data
  serialGraph();

  // Reset encoder count when position wraps around
  if (position >= 360.0 || position < 0.0) {
    encoderCount = 0;
  } 

  delay(50);
}