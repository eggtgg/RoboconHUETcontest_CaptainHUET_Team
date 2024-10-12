#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//create an RF24 object
RF24 radio(4, 5);  // CE, CSN

//address through which two modules communicate.
const byte address[6] = "11001";

const byte rightJoyXPin = 34;   //right joystick x
const byte rightJoyYPin = 35;   //right joystick y
const byte rightJoySwPin = 2;  //right joystick sw
const byte leftJoyXPin = 13;    //left joystick x
const byte leftJoyYPin = 12;    //left joystick y
const byte leftJoySwPin = 15;   //left joystick sw

const byte potXPin = 14;        //potentiometer 1
const byte potYPin = 33;        //potentiometer 2
const byte potZPin = 32;        //potentiometer 1

const byte switch1Pin = 27;     //switch 1
const byte switch2Pin = 26;     //switch 2

const byte phaA = 25;
const byte phaB = 17;
const byte switchABPin = 16;
int pulstep = 90; int bandau = LOW; int hientai=LOW;

  int leftJoyX;
  bool leftJoySw;
  int leftJoyY;
  
  int rightJoyX;
  int rightJoyY;
  bool rightJoySw;
  
  int potX;
  int potY;
  int potZ;

  bool switch1;
  bool switch2;
  bool switchAB;

  int control[12];
  
void setup() {
  Serial.begin(9600);
  
  pinMode(leftJoySwPin, INPUT_PULLUP);
  pinMode(rightJoySwPin, INPUT_PULLUP);
  
  pinMode(switch1Pin, INPUT_PULLUP);
  pinMode(switch2Pin, INPUT_PULLUP);
  
  pinMode(leftJoyXPin, INPUT);
  pinMode(leftJoyYPin, INPUT);
  pinMode(rightJoyXPin, INPUT);  
  pinMode(rightJoyYPin, INPUT);
  
  pinMode(potXPin, INPUT);
  pinMode(potYPin, INPUT);
  pinMode(potZPin, INPUT);

  pinMode(phaA,INPUT);
  pinMode(phaB,INPUT);
  pinMode(switchAB,INPUT);
      
  bandau = digitalRead(phaA);   

  radio.begin();
  
  //set the address
  radio.openWritingPipe(address);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MAX); 
  //Set module as transmitter
  radio.stopListening();
}

void loop() {
  int control[8];
  //Send message to receiver
  rightJoyX = analogRead(rightJoyXPin); control[0] = map(rightJoyX,0,4095,0,1023);
  //rightJoyY = analogRead(rightJoyYPin); control[1] =  map(rightJoyY,0,4095,0,1023);
  
  rightJoySw = digitalRead(rightJoySwPin); control[2] = rightJoySw;
  leftJoyX = analogRead(leftJoyXPin); control[3] = map(leftJoyX,0,4095,0,1023);
  leftJoyY = analogRead(leftJoyYPin); control[4] = map(leftJoyY,0,4095,0,1023);
  leftJoySw = digitalRead(leftJoySwPin); control[5] = leftJoySw;
  

  potX = analogRead(potXPin); control[6] = map(potX,0,4095,1900,0);
  potY = analogRead(potYPin); control[7] = map(potY,0,4095,0,-1000);
  potZ = analogRead(potZPin); control[1] = map(potZ,0,4095,0,900);

  radio.write(&control, sizeof(control));
  Serial.print(leftJoyX);Serial.print("--");Serial.println(leftJoyY);
}
