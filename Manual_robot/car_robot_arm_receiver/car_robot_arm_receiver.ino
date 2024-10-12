#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ArduinoJson.h>

#define enPin 69
#define stepPin 2
#define dirPin1 67
#define dirPin2 68
#define dirPin3 15
#define dirPin4 19

#define v_arm 2000
#define a_arm 1000

#define end_X 3
#define end_Y 14
#define end_Z 18

int pulsestep = 40;
int numstep = 3200;

int step_X = 54;
int Dir_X = 55;
int ena_X = 38;
int step_Y = 60;
int Dir_Y = 61;
int ena_Y = 56;
int step_Z = 46;
int Dir_Z = 48;
int ena_Z = 62;

int motorX;
int motorY;
int motorZ;
int servo4;

int smode;
int d_range = 150;

// create an RF24 object
RF24 radio(49, 53); // CE, CSN

// address through which two modules communicate.
const byte address[6] = "11001";

int rightJoyX;
int rightJoyY;
bool rightJoySw;
int leftJoyX;
int leftJoyY;
bool leftJoySw;
int potX;
int potY;
int potZ;
bool switch1;
bool switch2;
bool switchAB;

int enPinState = 0;
int lastButtonRState = 0;
int buttonRState = 0;
int ModeState = 0;
int lastButtonLState = 0;
int buttonLState = 0;
unsigned long timer;
int control[8];

AccelStepper Step_X(1, step_X, Dir_X, ena_X);
AccelStepper Step_Y(1, step_Y, Dir_Y, ena_Y);
AccelStepper Step_Z(1, step_Z, Dir_Z, ena_Z);

Servo servo_red;
Servo servo_green;


void setup() {
  pinMode(enPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(dirPin4, OUTPUT);

  servo_red.attach(5);
  servo_green.attach(6);

  pinMode(end_X, INPUT_PULLUP);
  pinMode(end_Y, INPUT_PULLUP);
  pinMode(end_Z, INPUT_PULLUP);

  Step_X.setEnablePin(ena_X);
  Step_X.setPinsInverted(false, false, true);

  Step_Y.setEnablePin(ena_Y);
  Step_Y.setPinsInverted(false, false, true);

  Step_Z.setEnablePin(ena_Z);
  Step_Z.setPinsInverted(false, false, true);

  Step_X.setMaxSpeed(v_arm);
  Step_X.setAcceleration(a_arm);

  Step_Y.setMaxSpeed(v_arm);
  Step_Y.setAcceleration(a_arm);

  Step_Z.setMaxSpeed(v_arm);
  Step_Z.setAcceleration(a_arm);

  home_one();

  Serial.begin(115200);

  radio.begin();

  // set the address
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_2MBPS);

  // Set module as receiver
  radio.startListening();
}


void loop() {
  radio_check();
  check_to_stop_arm();
  // run(95, potX, potY, potZ);
  
  if (potX > 512) smode = 1;
  else smode = 0;

  switch (smode) {
    case 0:
      dichuyen();
      break;
    case 1:
      // chạy mode cánh tay
      // run(95, potX, potY, potZ);
      dieu_chinh_arm();
      break;
  }
  check_arm_and_run_mode();
}

void radio_check() {
  // Read the data if available in buffer
  if (radio.available()) {
    radio.read(&control, sizeof(control));
    rightJoyX = control[0];
    // rightJoyY = control[1];
    rightJoySw = control[2];

    leftJoyX = control[3];
    leftJoyY = control[4];            
    leftJoySw = control[5];

    potX = control[6];
    potY = control[7];
    potZ = control[1];

    // Serial.print(leftJoyX);
    // Serial.print("-");
    // Serial.println(leftJoyY);
  }
}

void stop_arm() {
  Step_X.run();
  Step_Y.run();
  Step_Z.run();
}

void check_arm_and_run_mode() {
  if (Serial.available()){
    StaticJsonDocument<300> doc;
    DeserializationError err = deserializeJson(doc, Serial);
    
    if (err == DeserializationError::Ok) {
      motorX = doc["motor1"].as<int>();
      motorY = doc["motor2"].as<int>();
      motorZ = doc["motor3"].as<int>();
      servo4 = doc["servo4"].as<int>();
      run(servo4, motorX, motorY, motorZ);
    }
    else {
      // Serial.print("deserializeJson() returned ");
      // Serial.println(err.c_str());
      while (Serial.available() > 0)
        Serial.read();
      }
  }
}

void run(byte Ser, int X, int Y, int Z) {
  Step_X.moveTo(X);
  Step_Y.moveTo(Y);
  Step_Z.moveTo(Z);

  while (Step_X.distanceToGo() != 0 or Step_Y.distanceToGo() != 0 or Step_Z.distanceToGo() != 0)
  {
    Step_X.run();
    Step_Y.run();
    Step_Z.run();
  }
  servo_red.write(Ser);
}


void home_YZ() {
  int homeY = 0;
  int homeZ = 0;

  Step_Y.setMaxSpeed(v_arm);
  Step_Y.setAcceleration(a_arm);
  Step_Z.setMaxSpeed(v_arm);
  Step_Z.setAcceleration(a_arm);
  Step_Y.enableOutputs();
  Step_Z.enableOutputs();

  while (digitalRead(end_Y) == 1 and digitalRead(end_Z) == 1) {
    Step_Y.moveTo(homeY);
    Step_Z.moveTo(homeZ);
    homeY++;
    homeZ--;
    Step_Y.run();
    Step_Z.run();
  }

  while (digitalRead(end_Z) == 1) {
    Step_Z.moveTo(homeZ);
    homeZ--;
    Step_Z.run();
  }
  Step_Z.setCurrentPosition(0);

  while (digitalRead(end_Y) == 1) {
    Step_Y.moveTo(homeY);
    homeY++;
    Step_Y.run();
  }
  Step_Y.setCurrentPosition(0);

  homeY = 0;
  homeZ = 0;
}


void home_X() {
  int homeX = 0;

  Step_X.setMaxSpeed(v_arm);
  Step_X.setAcceleration(a_arm);
  Step_X.enableOutputs();

  while (digitalRead(end_X) == 1) {
    Step_X.moveTo(homeX);
    homeX++;
    Step_X.run();
  }
  Step_X.setCurrentPosition(0);

  homeX = 0;
}


void home_one() {
  home_YZ();
  home_X();
}

void dieu_chinh_arm() {
  if (leftJoyX > 723) {
    Step_X.setSpeed(1500);
    Step_X.runSpeed();
  }
  else if (leftJoyX < 300) {
    Step_X.setSpeed(-1500);
    Step_X.runSpeed();
  }
  else {
    Step_X.stop();
  }
}

void dichuyen() {
  motor_hold();
  
  if ((leftJoyY - leftJoyX > 0 + d_range) & (leftJoyY + leftJoyX > 1023 + d_range)) {
    motor_tien();
  }
  if ((leftJoyY - leftJoyX < 0 - d_range) & (leftJoyY + leftJoyX < 1023 - d_range)) {
    motor_lui();
  }
  if ((leftJoyY - leftJoyX < 0 - d_range) & (leftJoyY + leftJoyX > 1023 + d_range)) {
    motor_phai();
  }
  if ((leftJoyY - leftJoyX > 0 + d_range) & (leftJoyY + leftJoyX < 1023 - d_range)) {
    motor_trai();
  }

  // if (leftJoyY > 723) motor_tien();
  // if (leftJoyY < 300) motor_lui();

  // if (leftJoyX > 723) motor_phai();
  // if (leftJoyX < 300) motor_trai();

  if (rightJoyX > 723) motor_quay_phai();
  if (rightJoyX < 300) motor_quay_trai();

  digitalWrite(enPin, enPinState);
}

void check_to_stop_arm() {
  if ((leftJoyY > 723) | (leftJoyY < 300) | (leftJoyX > 723) | (leftJoyX < 300) | (rightJoyX > 723) | (rightJoyX < 300)) {
    smode = 0;
  }
  else smode = 1;
}


void motor_hold() {
  buttonRState = rightJoySw;
  if (buttonRState != lastButtonRState) {
    if (buttonRState == HIGH) {
      enPinState = !enPinState;
    }
    delay(50); // Chống nhiễu
  }
  lastButtonRState = rightJoySw; // Cập nhật trạng thái nút nhấn trước đó
}


void motor_tien() {
    numstep = map(control[4], 723, 1023, 0, 300);
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    digitalWrite(dirPin3, HIGH);
    digitalWrite(dirPin4, HIGH);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(pulsestep);
    }
}


void motor_lui() {
    numstep = map(control[4], 300, 0, 0, 300);
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    digitalWrite(dirPin3, LOW);
    digitalWrite(dirPin4, LOW);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(pulsestep);
    }
}

void motor_phai() {
    numstep = map(control[3], 723, 1023, 0, 300);
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    digitalWrite(dirPin3, HIGH);
    digitalWrite(dirPin4, LOW);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(pulsestep);
    }
}


void motor_trai() {
    numstep = map(control[3], 300, 0, 0, 300);
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    digitalWrite(dirPin3, LOW);
    digitalWrite(dirPin4, HIGH);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(pulsestep);
    }
}


void motor_quay_phai() {
    numstep = map(control[0], 723, 1023, 0, 300);
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    digitalWrite(dirPin3, HIGH);
    digitalWrite(dirPin4, HIGH);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(pulsestep);
    }
}


void motor_quay_trai() {
    numstep = map(control[0], 300, 0, 0, 300);
    digitalWrite(enPin, LOW);
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    digitalWrite(dirPin3, LOW);
    digitalWrite(dirPin4, LOW);
    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(pulsestep);
    }
}
