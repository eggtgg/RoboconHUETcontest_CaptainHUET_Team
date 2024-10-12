#include <SPI.h>
#include "RF24.h"
#include <nRF24L01.h>

const byte rightJoyXPin = A1;   //right joystick x
const byte rightJoyYPin = A0;   //right joystick y
const byte rightJoySwPin = 6;  //right joystick sw
const byte leftJoyXPin = A2;    //left joystick x
//const byte leftJoyYPin = A1;    //left joystick y
const byte leftJoySwPin = 7;   //left joystick sw

const byte potXPin = A3;        //potentiometer 1
const byte potYPin = A6;        //potentiometer 2
const byte potZPin = A7;        //potentiometer 1

const byte switch1Pin = 8;     //switch 1

  int leftJoyX;
  bool leftJoySw;
  
  int rightJoyX;
  int rightJoyY;
  bool rightJoySw;
  
  int potX;
  int potY;
  int potZ;

  bool switch1;
  
const uint64_t pipe = 0xE8E8F0F0E1LL; // địa chỉ để phát

unsigned long timer;       //millis timer


RF24 radio(4,5); //thay 10 thành 53 với mega
int control[9];

void setup(){ 
  pinMode(leftJoySwPin, INPUT_PULLUP);
  pinMode(rightJoySwPin, INPUT_PULLUP);
  pinMode(switch1Pin, INPUT_PULLUP);
  
  pinMode(leftJoyXPin, INPUT);
//  pinMode(leftJoyYPin, INPUT);
  pinMode(rightJoyXPin, INPUT);
  pinMode(rightJoyYPin, INPUT);
  pinMode(potXPin, INPUT);
  pinMode(potYPin, INPUT);
  pinMode(potZPin, INPUT);

  //============================================================Module NRF24
  radio.begin();
  Serial.begin(115200);                  
  radio.setAutoAck(1);               
  radio.setRetries(1,1);             
  radio.setDataRate(RF24_250KBPS);    // Tốc độ truyền
  radio.setPALevel(RF24_PA_MAX);      // Dung lượng tối đa
  radio.setChannel(10);               // Đặt kênh
  radio.openWritingPipe(pipe);        // mở kênh
}
void loop(){
  if(millis() - timer >= 20){  //activate every 50 milliseconds (20Hz)
    timer = millis();  //new timestamp
 
  rightJoyX = analogRead(rightJoyXPin); control[0] = rightJoyX;
  rightJoyY = analogRead(rightJoyYPin); control[1] = rightJoyY;
  rightJoySw = digitalRead(rightJoySwPin); control[2] = rightJoySw;
   
  leftJoyX = analogRead(leftJoyXPin); control[3] = leftJoyX;
  leftJoySw = digitalRead(leftJoySwPin); control[4] = leftJoySw;
    
  potX = analogRead(potXPin); control[5] = potX;
  potY = analogRead(potYPin); control[6] = potY;
  potZ = analogRead(potZPin); control[7] = potZ;

  switch1 = digitalRead(switch1Pin); control[8] = switch1;
  
  radio.write(&control, sizeof(control));
  }
}
