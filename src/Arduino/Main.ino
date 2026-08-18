#include <PID_v2.h>
#include <Servo.h>

#define ENCODER_A 2
#define ENCODER_B 3
#define PWM_PIN 11   // speed
#define DIR_PIN 13   // direction


#define GEAR_RATIO 19       // Our CHP-20GP-180's gear ratio
#define PPR 11              // encoder pulses per motor-shaft revolution
#define TICKS_PER_REV (GEAR_RATIO * PPR * 4.0) // x4 quadrature decoding

int minDuty = 60;      // minimum PWM to overcome friction
int stopZoneDeg = 1;   // degrees considered "arrived"
int resumeZoneDeg = 2; // must drift this far from target before correcting again
bool arrived = false;
float error = 0;
float corr = 0;

Servo servo;

volatile long motorPosition = 0;
volatile uint8_t lastEncoded = 0;

double currentDeg, targetDeg, output;
double kp = 0.33, ki = 0.0, kd = 0.007;
PID myPID(&currentDeg, &output, &targetDeg, kp, ki, kd, DIRECT);

// ---------- Serial command buffer ----------
String serialBuf = "";

void updateMotorPosition() {
  uint8_t MSB = digitalRead(ENCODER_A);
  uint8_t LSB = digitalRead(ENCODER_B);
  uint8_t encoded = (MSB << 1) | LSB;
  uint8_t sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) motorPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) motorPosition--;

  lastEncoded = encoded;
}

void setup() {
  Serial.begin(115200);
  servo.attach(9);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateMotorPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateMotorPosition, CHANGE);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(10);

  servo.write(cal_head(0)); // center steering
  serialBuf.reserve(32);

  Serial.println("READY");
  StartButton();
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
  } while (error > 2);

  analogWrite(PWM_PIN, 0);
}

void motor_dc(int duty) {
  digitalWrite(DIR_PIN, duty >= 0 ? HIGH : LOW);
  analogWrite(PWM_PIN, constrain(abs(duty), 0, 255));
}

// Calculate the seering angle to make it useable with our servo
int cal_head(int degree) {
  return ((degree + 180) / 2);
}

// ---------- Serial command handling ----------
// Protocol (newline-terminated ASCII lines), comma-separated tuple:
//   <servoAngle>,<speed>,<distance>
//     servoAngle : int, degrees, passed through cal_head() and applied immediately
//     speed      : int, raw PWM duty -255..255 (sign = direction)
//     distance   : float, degrees of motor travel
//                    0    -> just drive continuously at <speed>, replies "OK" right away (non-blocking)
//                    !=0  -> drive at <speed> until |distance| degrees have been traveled
//                            (measured from encoder position at command start), then stop
//                            and print "t"
// Malformed lines reply "ERR"
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  int c1 = cmd.indexOf(',');
  int c2 = (c1 >= 0) ? cmd.indexOf(',', c1 + 1) : -1;
  if (c1 < 0 || c2 < 0) {
    Serial.println("ERR");
    return;
  }

  int servoAngle = cmd.substring(0, c1).toInt();
  int speed = cmd.substring(c1 + 1, c2).toInt();
  float distance = cmd.substring(c2 + 1).toFloat();

  servo.write(cal_head(servoAngle));

  if (distance == 0) {
    motor_dc(speed);
    Serial.println("OK");
  } else {
    long startPos = motorPosition;
    motor_dc(speed);

    float traveled;
    do {
      traveled = abs((motorPosition - startPos) * 360.0 / TICKS_PER_REV);
    } while (traveled < abs(distance));

    analogWrite(PWM_PIN, 0);
    Serial.println("t");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      serialBuf += c;
      // guard against runaway buffer if newline never arrives
      if (serialBuf.length() > 40) serialBuf = "";
    }
  }
}

void StartButton(){
  while (analogRead(0) > 500){
    delay(10);
  }
  delay(800);
  Serial.println("Start");

}

void loop() {
  readSerialCommands();
}
