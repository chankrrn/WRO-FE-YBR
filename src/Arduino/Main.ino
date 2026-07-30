#include <PID_v2.h>
#include <Servo.h>

#define ENCODER_A 2
#define ENCODER_B 6
#define PWM_PIN 11   // speed
#define DIR_PIN 13   // direction

// EDIT THESE to match your motor:
#define GEAR_RATIO 19       // your CHP-20GP-180's gear ratio
#define PPR 11              // encoder pulses per motor-shaft revolution
#define TICKS_PER_REV (GEAR_RATIO * PPR * 2.0) // x4 from quadrature decoding

int minDuty = 60;      // minimum PWM to overcome friction
int stopZoneDeg = 1;   // degrees considered "arrived"
int resumeZoneDeg = 2; // must drift this far from target before correcting again
bool arrived = false;
float error = 0;
float corr = 0;

Servo servo;


volatile long motorPosition = 0;

double currentDeg, targetDeg, output;
double kp = 0.33, ki = 0.0, kd = 0.007;
PID myPID(&currentDeg, &output, &targetDeg, kp, ki, kd, DIRECT);

void updateMotorPosition() {
  if (digitalRead(ENCODER_B) != digitalRead(ENCODER_A)) {
    motorPosition++;
  } else {
    motorPosition--;
  }
}

void setup() {
  Serial.begin(115200);
  servo.attach(9);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(10);

  servo.write(cal_head(-2));
  motor_degree(2060);
  // delay(800);
  // Serial.println(currentDeg);

  // motor_dc(255);
}
void motor_degree(float angle) {
  targetDeg = angle; // set the GLOBAL setpoint the PID actually reads

  do {
    currentDeg = motorPosition * 360.0 / TICKS_PER_REV; // update the GLOBAL currentDeg
    error = abs(targetDeg - currentDeg);

    myPID.Compute();
    corr = constrain(abs((int)output), 0, 100);
    if (corr > 0 && corr < minDuty && error > 5) corr = minDuty;
    digitalWrite(DIR_PIN, output >= 0 ? HIGH : LOW);
    analogWrite(PWM_PIN, corr);

    Serial.print("target: ");
    Serial.print(targetDeg);
    Serial.print("  current: ");
    Serial.print(currentDeg);
    Serial.print("  duty: ");
    Serial.print(corr);
    Serial.print("  error: ");
    Serial.println(error);
  } while (error > 2);

  analogWrite(PWM_PIN, 0);
}

void motor_dc(int duty){
  
  digitalWrite(DIR_PIN, duty >= 0 ? HIGH : LOW);
  analogWrite(PWM_PIN, constrain(abs(duty), 0, 255));
}

int cal_head(int degree){
  return ((degree + 90)/2);
}

void loop() {
  // Serial.println(servo.read());
  
  // servo.write(cal_head(-50));
  // delay(500);
  // servo.write(cal_head(50));
  // delay(500);
  
 
}