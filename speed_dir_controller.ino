// Pin definitions
#define trigPin_R A0
#define echoPin_R A1
#define trigPin_F A2
#define echoPin_F A3
#define trigPin_L A5
#define echoPin_L A4

#define WindowSize 5
#define motorR1  8
#define motorR2  7
#define motorENR 6
#define motorL1  9
#define motorL2  10
#define motorENL  11
#define maxSPEED 255

// Global variables
int SPEED;
float DataArr[WindowSize][3];
long duration;
float distanceCm;
float difference;
float Kp = 0.9;
float Ki = 0.1;
float Kd = 0.2;
float d;
float v;
float lastError = 0;
float integral = 0;
char dir;

// Arrays for pin configuration
char TrigArr[]={trigPin_R,trigPin_F,trigPin_L};
char EchoArr[]={echoPin_R,echoPin_F,echoPin_L};

// Function declarations
void DataInit(void);
float UltrasonicRead(int Ultranum);
float UltrasonicRead_WithAverage(int Ultranum);
void Mov(int SPEED, char dir);
float PIDControl(float error);

void setup() {
  // Pin mode configuration
  for (int i = 0; i < 3; i++) {
    pinMode(TrigArr[i], OUTPUT);
    pinMode(EchoArr[i], INPUT);
  }
  
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorENR, OUTPUT);
  pinMode(motorENL, OUTPUT);
  
  // Serial communication initialization
  Serial.begin(9600);
  
  // Initialize data array
  DataInit();
}

void loop() {
  // Read ultrasonic sensor values
  float UR = UltrasonicRead_WithAverage(0);
  float UF = UltrasonicRead_WithAverage(1);
  float UL = UltrasonicRead_WithAverage(2);
  
  // Determine direction based on sensor readings
  if (UL > UR) {
    dir = 'L';
    difference = UL - UR;
  } else if (UL < UR) {
    dir = 'R';
    difference = UR - UL;
  } else if (UR == UL) {
    dir = 'C';
    difference = 0;
  }
  
  // Check if obstacle is detected in front
  if (UF > 50) {
    digitalWrite(motorR1, LOW);
    digitalWrite(motorR2, HIGH);
    digitalWrite(motorL1, LOW);
    digitalWrite(motorL2, HIGH);
  } else if (UF < 50) {
    digitalWrite(motorR1, LOW);
    digitalWrite(motorR2, LOW);
    digitalWrite(motorL1, LOW);
    digitalWrite(motorL2, LOW);
  }
  
  // Calculate error for PID control
  float error = difference;
  
  // Perform PID control
  float pidOutput = PIDControl(error);
  
  // Calculate final speed
  SPEED = maxSPEED - abs(pidOutput);
  
  // Print sensor readings and calculated values
  Serial.print(" ,L; ");
  Serial.print(UL);
  Serial.print(" ,F: ");
  Serial.print(UF);
  Serial.print(" ,R: ");
  Serial.print(UR);
  Serial.print(" ,diff: ");
  Serial.print(difference);
  Serial.print(" ,spd: ");
  Serial.println(SPEED);

  // Move the robot based on calculated speed and direction
  Mov(SPEED, dir);
}

// Function to control motor movement
void Mov(int SPEED, char dir) {
  if (dir == 'R') {
    analogWrite(motorENR, SPEED);
    analogWrite(motorENL, maxSPEED);
  } else if (dir == 'C') {
    analogWrite(motorENR, maxSPEED);
    analogWrite(motorENL, maxSPEED);
  } else if (dir == 'L') {
    analogWrite(motorENR, maxSPEED);
    analogWrite(motorENL, SPEED);
  }
}

// Function to initialize data array
void DataInit(void) {
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < WindowSize; i++) {
      DataArr[i][j] = UltrasonicRead(j);
    }
  }
}

// Function to read distance from ultrasonic sensor
float UltrasonicRead(int Ultranum) {
  digitalWrite(TrigArr[Ultranum], LOW);
  delayMicroseconds(2);
  digitalWrite(TrigArr[Ultranum], HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigArr[Ultranum], LOW);
  
  duration = pulseIn(EchoArr[Ultranum], HIGH);
  distanceCm = (duration * 0.0343) / 2.0;
  
  return distanceCm;
} 

// Function to take multiple ultrasonic readings and calculate average
float UltrasonicRead_WithAverage(int Ultranum) {
  float sum = 0;
  
  for (int i = 0; i < WindowSize - 1; i++) {
    DataArr[i][Ultranum] = DataArr[i + 1][Ultranum];
    sum += DataArr[i][Ultranum];
  }
  
  DataArr[WindowSize - 1][Ultranum] = UltrasonicRead(Ultranum);
  sum += DataArr[WindowSize - 1][Ultranum];
  
  return (sum / WindowSize);
}

// Function to perform PID control
float PIDControl(float error) {
  float derivative = error - lastError;
  integral += error;
  lastError = error;
  
  return (Kp * error + Ki * integral + Kd * derivative);
}
