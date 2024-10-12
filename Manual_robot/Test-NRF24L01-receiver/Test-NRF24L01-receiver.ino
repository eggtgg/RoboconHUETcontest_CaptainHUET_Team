#include <SPI.h>
#include "RF24.h"
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
#define pulsestep 90


#define v_arm 3500
#define a_arm 3500

int motorX;
int motorY;
int motorZ;
int servo4;

int step_X = 54; int Dir_X = 55; int ena_X = 38;
int step_Y = 60; int Dir_Y = 61; int ena_Y = 56;
int step_Z = 46; int Dir_Z = 48; int ena_Z = 62;

int numstep ;

const uint64_t pipe = 0xE8E8F0F0E1LL; // địa chỉ phát
RF24 radio(49,53);//thay 10 thành 53 với mega
int control[9];

  int rightJoyX;  int rightJoyY;  bool rightJoySw;
  int leftJoyX;  bool leftJoySw;
  int potX;  int potY;  int potZ;
  bool switch1;

int enPinState=0; int lastButtonRState=0; int buttonRState=0;
int ModeState=0; int lastButtonLState=0; int buttonLState=0;


AccelStepper Step_X(1, step_X, Dir_X, ena_X);
AccelStepper Step_Y(1, step_Y, Dir_Y, ena_Y);
AccelStepper Step_Z(1, step_Z, Dir_Z, ena_Z);
Servo servo_kep;
#define end_X  3
#define end_Y  14
#define end_Z  18

unsigned long timer;       //millis timer

void setup(){
  pinMode(enPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(dirPin4, OUTPUT);

  
  servo_kep.attach(5);
  pinMode(end_X, INPUT_PULLUP);
  pinMode(end_Y, INPUT_PULLUP);
  pinMode(end_Z, INPUT_PULLUP);

  Step_X.setEnablePin(38);
  Step_X.setPinsInverted(false, false, true);
  Step_Y.setEnablePin(56);
  Step_Y.setPinsInverted(false, false, true);
  Step_Z.setEnablePin(62);
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
  radio.setAutoAck(1);              
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(10);               // Đặt kênh
  radio.openReadingPipe(1,pipe);     
  radio.startListening();            
}
 
void loop(){
  if (radio.available()){
    while (radio.available()){
      if (millis() - timer >= 20) {  //activate every 50 milliseconds (20Hz)
        timer = millis();  //new timestamp
        radio.read(&control, sizeof(control));
        rightJoyX = control[0];
        rightJoyY = control[1];
        rightJoySw = control[2];
        leftJoyX = control[3];
        leftJoySw = control[4];
        Serial.println(control[5]);
      }

      dichuyen();
      mode_arm();
    }
  }
  // bt 95, servo càng tăng càng kẹp
  // run(110, -3000, -3000, 3000);
  check_arm_and_run_mode();
}

// kiem tra tin hieu Serial, chi nhan kieu json
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
          Serial.print("deserializeJson() returned ");
          Serial.println(err.c_str());
          while (Serial.available() > 0)
            Serial.read();
        }
    }
}

void vitri() {
  digitalWrite(ena_X,LOW);
  digitalWrite(ena_Y,LOW);
  digitalWrite(ena_Z,LOW); 
  run(180, -5000, -4000, 4000); //1
  // delay(5);
  run(0, -2200, -3000, 2500); //3
  // delay(5);
  digitalWrite(ena_X,HIGH);
  digitalWrite(ena_Y,HIGH);
  digitalWrite(ena_Z,HIGH);
  ModeState = !ModeState;    
}

void set_vitri(){
 digitalWrite(ena_X,LOW);
 digitalWrite(ena_Y,LOW);
 digitalWrite(ena_Z,LOW);
  potX = map(control[5],0,1023,0,-6000);
  potY = map(control[6],0,1023,0,-6000);
  potZ = map(control[7],0,1023,0,6000);
  run(180, potX, potY, potZ);
  delay(5);
}

void mode_arm(){
  buttonLState = leftJoySw;
  if (buttonLState != lastButtonLState) {
    if (buttonLState == HIGH) {
      ModeState = !ModeState;
    }
    delay(50); // Chống nhiễu
  }
  lastButtonLState = leftJoySw; // Cập nhật trạng thái nút nhấn trước đó
}

void run(byte Ser, int X, int Y, int Z) {
  Step_X.moveTo(X);
  Step_Y.moveTo(Y);
  Step_Z.moveTo(Z);
  while (Step_X.distanceToGo() != 0 or Step_Y.distanceToGo() != 0 or Step_Z.distanceToGo() != 0) {
    Step_X.run();
    Step_Y.run();
    Step_Z.run();
  }
  servo_kep.write(Ser);
}
void home_YZ() {
  int homeY = 0; int homeZ = 0;

  Step_Y.setMaxSpeed(v_arm);
  Step_Y.setAcceleration(a_arm);
  Step_Z.setMaxSpeed(v_arm);
  Step_Z.setAcceleration(a_arm);  
  Step_Y.enableOutputs();
  Step_Z.enableOutputs();

  while(digitalRead(end_Y)==1 and digitalRead(end_Z)==1) {
    Step_Y.moveTo(homeY);
    Step_Z.moveTo(homeZ);
    homeY ++;
    homeZ --;
    Step_Y.run();
    Step_Z.run();
  }

  while(digitalRead(end_Y)==1) {
    Step_Y.moveTo(homeY);
    homeY ++;
    Step_Y.run();
  }
  Step_Y.setCurrentPosition(0);

  while(digitalRead(end_Z)==1) {
    Step_Z.moveTo(homeZ);
    homeZ --;
    Step_Z.run();        
  }
  Step_Z.setCurrentPosition(0);
    
  homeY = 0;
  homeZ = 0;
}
void home_X() {
  int homeX = 0;
  Step_X.setMaxSpeed(v_arm);
  Step_X.setAcceleration(a_arm);  
  Step_X.enableOutputs();

  while(digitalRead(end_X)==1) {
    Step_X.moveTo(homeX);
    homeX ++;
    Step_X.run();
  }
  Step_X.setCurrentPosition(0);

  homeX = 0;
}


void home_one() {
  home_YZ();
  home_X();
}


void dichuyen() {
  motor_hold();
  motor_tien();
  motor_lui();
  motor_phai();
  motor_trai();
  motor_quay_phai();
  motor_quay_trai();
  digitalWrite(enPin,enPinState);
}


void motor_hold(){
  buttonRState = rightJoySw;
  if (buttonRState != lastButtonRState) {
    if (buttonRState == HIGH) {
      enPinState = !enPinState;
    }
    delay(50); // Chống nhiễu
  }
  lastButtonRState = rightJoySw; // Cập nhật trạng thái nút nhấn trước đó
}


void motor_tien(){
  if (rightJoyY > 723) {
    numstep = map(control[1],723,1023,0,300);
    
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin1,HIGH);
    digitalWrite(dirPin2,HIGH);
    digitalWrite(dirPin3,HIGH);
    digitalWrite(dirPin4,HIGH);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(pulsestep);
    }
  }  
}


void motor_lui(){
  if (rightJoyY < 300){
    numstep = map(control[1],300,0,0,300);
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin1,LOW);
    digitalWrite(dirPin2,LOW);
    digitalWrite(dirPin3,LOW);
    digitalWrite(dirPin4,LOW);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(pulsestep);
    }
  }
}


void motor_phai() {
  if (rightJoyX > 723){
    numstep = map(control[0],723,1023,0,300);
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin1,LOW);
    digitalWrite(dirPin2,HIGH);
    digitalWrite(dirPin3,HIGH);
    digitalWrite(dirPin4,LOW);
  
    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(pulsestep);
    }
  }
}


void motor_trai(){
  if (rightJoyX < 300){
    numstep = map(control[0],300,0,0,300);
    
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin1,HIGH);
    digitalWrite(dirPin2,LOW);
    digitalWrite(dirPin3,LOW);
    digitalWrite(dirPin4,HIGH);

    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(pulsestep);
    }
  }
}


void motor_quay_phai() {
  if (leftJoyX > 723){
    numstep = map(control[3],723,1023,0,300);
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin1,LOW);
    digitalWrite(dirPin2,LOW);
    digitalWrite(dirPin3,HIGH);
    digitalWrite(dirPin4,HIGH);
    
    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(pulsestep);
    }
  }
}
void motor_quay_trai() {
  if (leftJoyX < 300){
    numstep = map(control[3],300,0,0,300);
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin1,HIGH);
    digitalWrite(dirPin2,HIGH);
    digitalWrite(dirPin3,LOW);
    digitalWrite(dirPin4,LOW);
  
    for (int i = 0; i < numstep; i++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(pulsestep);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(pulsestep);
    }
  }
}
