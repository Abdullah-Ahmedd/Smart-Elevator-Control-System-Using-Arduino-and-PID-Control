// =============================================
// Smart Elevator Project - PID Position Control
// =============================================

// -- Encoder Pins --
const int encoderPinA = 2;   // Yellow - Phase A
const int encoderPinB = 3;   // Green  - Phase B

// -- L298N Pins --
const int PWM_PIN = 5;   // ENB
const int IN3     = 4;   // IN3
const int IN4     = 7;   // IN4

// -- Ultrasonic and Potentiometer Pins --
const int trigPin = 8;
const int echoPin = 9;
const int potPin  = A0;

// -- Motor Constants (GA25-370) --
const double PPR = 1496.0;

// -- PWM and Control Limits --
const int MAX_PWM = 130;
const int MIN_PWM = 40;
const double TOLERANCE = 15.0;

// -- Soft Start / Soft Stop Parameters --
// First revolution = RAMP_IN_PULSES pulses
// Last two revolutions = RAMP_OUT_PULSES pulses
const double RAMP_IN_PULSES  = 1.0 * PPR;
const double RAMP_OUT_PULSES = 2.0 * PPR;

// Speed reduction factor inside ramp zones (0.0 - 1.0)
const double RAMP_SPEED_FACTOR = 0.45;

// -- Ultrasonic Emergency Limits --
const int EMERGENCY_MIN_CM = 4;   // Too close to the ceiling
const int EMERGENCY_MAX_CM = 70;  // Too close to the floor

// -- Encoder Variables --
volatile long encoderCount = 0;

// -- PID Variables --
double setpoint   = 0;
double input      = 0;
double output     = 0;
double error      = 0;
double lastError  = 0;
double integral   = 0;
double derivative = 0;

double Kp = 4.5;
double Ki = 0.1;
double Kd = 0.15;

unsigned long lastTime      = 0;
unsigned long lastPrintTime = 0;

// -- Ultrasonic Variables --
long duration;
int distanceCm;

// -- Emergency Flag --
bool emergencyStop = false;

// =============================================
void setup() {
  Serial.begin(9600);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN3,     OUTPUT);
  pinMode(IN4,     OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), readEncoderB, CHANGE);

  stopMotor();
  lastTime = millis();

  Serial.println("=== Smart Elevator PID System Ready ===");
}

// =============================================
void loop() {

  // 0. Check emergency conditions before executing any control logic
  measureDistance();
  checkEmergency();

  if (emergencyStop) {
    stopMotor();
    integral  = 0;
    lastError = 0;

    if (millis() - lastPrintTime >= 250) {
      Serial.print("!!! EMERGENCY STOP !!! Distance: ");
      Serial.print(distanceCm);
      Serial.println(" cm");
      lastPrintTime = millis();
    }
    return; // Immediately stop all further execution
  }

  // 1. Read potentiometer and determine target floor
  int potValue = analogRead(potPin);
  int targetFloor = 1;

  if (potValue < 340) {
    targetFloor = 1;
    setpoint = 0;
  }
  else if (potValue < 680) {
    targetFloor = 2;
    setpoint = PPR * 10;
  }
  else {
    targetFloor = 3;
    setpoint = PPR * 20;
  }

  // 2. Read encoder position
  noInterrupts();
  input = (double)encoderCount;
  interrupts();

  error = setpoint - input;

  // 3. PID Computation
  unsigned long now = millis();
  double deltaTime = (double)(now - lastTime) / 1000.0;
  lastTime = now;

  if (deltaTime > 0 && deltaTime <= 0.5) {

    if (abs(error) > TOLERANCE) {

      integral += error * deltaTime;
      integral = constrain(integral, -150.0, 150.0);

      derivative = (error - lastError) / deltaTime;

      output = (Kp * error) +
               (Ki * integral) +
               (Kd * derivative);

      output = constrain(output, -MAX_PWM, MAX_PWM);

      // 4. Apply Soft Start and Soft Stop profile
      output = applyRamp(output, input, setpoint);

      driveMotor(output);
    }
    else {
      stopMotor();
      integral = 0;
    }

    lastError = error;
  }

  // 5. Print system status every 250 ms
  if (millis() - lastPrintTime >= 250) {
    Serial.print("Floor: ");     Serial.print(targetFloor);
    Serial.print(" | SP: ");     Serial.print(setpoint, 0);
    Serial.print(" | Pos: ");    Serial.print(input, 0);
    Serial.print(" | Dist: ");   Serial.print(distanceCm);
    Serial.print("cm | PWM: ");  Serial.println(output, 0);

    lastPrintTime = millis();
  }
}

// =============================================
// Soft Start + Soft Stop Function
// Reduces speed during acceleration and deceleration
// =============================================
double applyRamp(double rawOutput, double currentPos, double targetPos) {

  double distanceTraveled  = 0;
  double distanceRemaining = abs(targetPos - currentPos);
  double totalDistance     = abs(targetPos - 0);

  double s0 = 0;
  double s1 = PPR * 10;
  double s2 = PPR * 20;

  if (targetPos == s1) {

    distanceTraveled = abs(currentPos - s0);
    totalDistance    = abs(s1 - s0);

  } else if (targetPos == s2) {

    if (currentPos >= s1) {
      distanceTraveled = abs(currentPos - s1);
      totalDistance    = abs(s2 - s1);
    } else {
      distanceTraveled = abs(currentPos - s0);
      totalDistance    = abs(s2 - s0);
    }

  } else {

    if (currentPos >= s1) {
      distanceTraveled = abs(currentPos - s2);
      totalDistance    = abs(s2 - s0);
    } else {
      distanceTraveled = abs(currentPos - s1);
      totalDistance    = abs(s1 - s0);
    }
  }

  bool inRampIn  = (distanceTraveled  < RAMP_IN_PULSES);
  bool inRampOut = (distanceRemaining < RAMP_OUT_PULSES);

  if (inRampIn || inRampOut) {

    double factor = 1.0;

    if (inRampIn) {
      factor = min(factor,
                   distanceTraveled / RAMP_IN_PULSES);
    }

    if (inRampOut) {
      factor = min(factor,
                   distanceRemaining / RAMP_OUT_PULSES);
    }

    factor = max(factor, RAMP_SPEED_FACTOR);

    rawOutput *= factor;
  }

  return rawOutput;
}

// =============================================
// Emergency Condition Check
// =============================================
void checkEmergency() {

  if (distanceCm == -1) {
    return;
  }

  if (distanceCm < EMERGENCY_MIN_CM ||
      distanceCm > EMERGENCY_MAX_CM) {

    emergencyStop = true;

  } else {

    emergencyStop = false;
  }
}

// =============================================
// Encoder Interrupt Service Routines
// =============================================
void readEncoderA() {

  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);

  encoderCount += (a == b) ? 1 : -1;
}

void readEncoderB() {

  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);

  encoderCount += (a != b) ? 1 : -1;
}

// =============================================
// Motor Drive Function
// =============================================
void driveMotor(double pwmVal) {

  if (abs(pwmVal) < MIN_PWM) {
    stopMotor();
    return;
  }

  int rawPWM   = (int)abs(pwmVal);
  int finalPWM = constrain(rawPWM, MIN_PWM, MAX_PWM);

  if (pwmVal > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(PWM_PIN, finalPWM);
}

// =============================================
// Stop Motor Function
// =============================================
void stopMotor() {

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(PWM_PIN, 0);
}

// =============================================
// Ultrasonic Distance Measurement
// =============================================
void measureDistance() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 15000);

  if (duration == 0) {
    distanceCm = -1;
  } else {
    distanceCm = duration * 0.034 / 2;
  }
}