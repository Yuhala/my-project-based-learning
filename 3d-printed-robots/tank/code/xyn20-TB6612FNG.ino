
                                                                                                                                                                                                                                                                                      +/*
  Two N20 motors on TB6612FNG (tank) + ESP32-S3.

  Board: ESP32S3 Dev Module

  Only Red/White per motor. Leave Black/Blue/Yellow/Green disconnected.

  ---------------------------------------------------------------------------
  POWER
  ---------------------------------------------------------------------------
  Battery +  ------------->  TB6612 VM
  Battery -  ------------->  TB6612 GND  ----+
  ESP32 GND  --------------------------------+---- common ground
  ESP32 3V3  ------------->  TB6612 VCC

  ---------------------------------------------------------------------------
  ESP32  --->  TB6612  (control lines)
  ---------------------------------------------------------------------------
  ESP32 GPIO8   ---------->  TB6612 STBY
  ESP32 GPIO9   ---------->  TB6612 PWMA     (left speed)
  ESP32 GPIO10  ---------->  TB6612 AIN1     (left direction)
  ESP32 GPIO11  ---------->  TB6612 AIN2     (left direction)
  ESP32 GPIO12  ---------->  TB6612 PWMB     (right speed)
  ESP32 GPIO13  ---------->  TB6612 BIN1     (right direction)
  ESP32 GPIO14  ---------->  TB6612 BIN2     (right direction)

  ---------------------------------------------------------------------------
  TB6612  --->  LEFT N20  (channel A)
  ---------------------------------------------------------------------------
  TB6612 AO1  ------------>  Left N20 Red
  TB6612 AO2  ------------>  Left N20 White

  ---------------------------------------------------------------------------
  TB6612  --->  RIGHT N20  (channel B)
  ---------------------------------------------------------------------------
  TB6612 BO1  ------------>  Right N20 Red
  TB6612 BO2  ------------>  Right N20 White

  If one track runs backward, swap that motor's Red/White, or set
  LEFT_INVERT / RIGHT_INVERT below.
*/

const int STBY = 8;

// Channel A = left track
const int PWMA = 9;
const int AIN1 = 10;
const int AIN2 = 11;

// Channel B = right track
const int PWMB = 12;
const int BIN1 = 13;
const int BIN2 = 14;

const bool LEFT_INVERT = false;
const bool RIGHT_INVERT = true;  // tank sides face opposite; flip if still wrong

void setup() {
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, HIGH);
  stopMotors();

  Serial.begin(115200);
  Serial.println("tank motors ready");
}

void driveChannel(int pwmPin, int in1, int in2, int speed, bool invert) {
  if (invert) {
    speed = -speed;
  }
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void driveLeft(int speed) {
  driveChannel(PWMA, AIN1, AIN2, speed, LEFT_INVERT);
}

void driveRight(int speed) {
  driveChannel(PWMB, BIN1, BIN2, speed, RIGHT_INVERT);
}

void driveTank(int leftSpeed, int rightSpeed) {
  driveLeft(leftSpeed);
  driveRight(rightSpeed);
}

void stopMotors() {
  driveTank(0, 0);
}

// Runs LEFT alone, then RIGHT alone. Use this to see which side is dead.
void sideCheckRoutine() {
  Serial.println("LEFT only");
  driveLeft(180);
  driveRight(0);
  delay(2500);
  stopMotors();
  delay(1000);

  Serial.println("RIGHT only");
  driveLeft(0);
  driveRight(180);
  delay(2500);
  stopMotors();
  delay(1000);
}

void testRoutine() {
  Serial.println("test: both forward");
  driveTank(180, 180);
  delay(2000);
  stopMotors();
  delay(1000);

  Serial.println("test: both reverse");
  driveTank(-180, -180);
  delay(2000);
  stopMotors();
  delay(1000);
}

void spinRoutine() {
  const int speeds[] = {64, 128, 192, 255};
  const int speedCount = sizeof(speeds) / sizeof(speeds[0]);

  for (int i = 0; i < speedCount; i++) {
    Serial.print("spin both: speed=");
    Serial.println(speeds[i]);
    driveTank(speeds[i], speeds[i]);
    delay(2000);
    stopMotors();
    delay(2000);
  }
}

void turnRoutine() {
  Serial.println("turn: left");
  driveTank(-160, 160);
  delay(1500);
  stopMotors();
  delay(1000);

  Serial.println("turn: right");
  driveTank(160, -160);
  delay(1500);
  stopMotors();
  delay(1000);
}

void loop() {
  //sideCheckRoutine();   // diagnose one-wheel problem first
   testRoutine();
  // spinRoutine();
  // turnRoutine();
}
