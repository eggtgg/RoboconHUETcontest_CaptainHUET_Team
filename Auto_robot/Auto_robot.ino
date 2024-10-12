#include <TimerOne.h>

#define EncoderA1 2
#define EncoderA2 3

#define BRAKE 0
#define CW    1
#define CCW   2
//#define CS_THRESHOLD 15   // Definition of safety current (Check: "1.3 Monster Shield Example").

//MOTOR 1
#define MOTOR_A1_PIN A3
#define MOTOR_B1_PIN A5

//MOTOR 2
#define MOTOR_A2_PIN A2
#define MOTOR_B2_PIN 7

#define PWM_MOTOR_1 5
#define PWM_MOTOR_2 6

//#define CURRENT_SEN_1 A2
//#define CURRENT_SEN_2 A3

//#define EN_PIN_1 A0
//#define EN_PIN_2 A1

#define MOTOR_1 0
#define MOTOR_2 1

const int trig = 8;     // chân trig của HC-SR04
const int echo = 9;     // chân echo của HC-SR04

short usSpeed1 = 225;  //default motor speed
short usSpeed2 = 255;  //default motor speed
unsigned short usMotor_Status = BRAKE;

int xung1 = 0;
float v1 = 0;
int xung2 = 0;
float v2 = 0;
 
void setup() {

  pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo,INPUT);    // chân echo sẽ nhận tín hiệu

  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

//  pinMode(CURRENT_SEN_1, OUTPUT);
//  pinMode(CURRENT_SEN_2, OUTPUT);  
//
//  pinMode(EN_PIN_1, OUTPUT);
//  pinMode(EN_PIN_2, OUTPUT);

pinMode(EncoderA1,INPUT_PULLUP);
pinMode(EncoderA2,INPUT_PULLUP);
attachInterrupt (1, ngat_dem_xung1, FALLING);
attachInterrupt (0, ngat_dem_xung2, FALLING);
Timer1.initialize(1000000);
Timer1.attachInterrupt(ngat_timer1);

  Serial.begin(9600);     // Initiates the serial to do the monitoring 
  Serial.println(); //Print function list for user selection
  Serial.println("Enter number for control option:");
  Serial.println("1. STOP");
  Serial.println("2. FORWARD");
  Serial.println("3. REVERSE");
  Serial.println("4. READ CURRENT");
  Serial.println("+. INCREASE SPEED");
  Serial.println("-. DECREASE SPEED");
  Serial.println();

}

void loop() 
{
  char user_input;   
  while(Serial.available()){
    user_input = Serial.read(); //Read user input and trigger appropriate function
//    digitalWrite(EN_PIN_1, HIGH);
//    digitalWrite(EN_PIN_2, HIGH); 
     
    if (user_input =='1'){
       Stop();
    }
    else if(user_input =='2'){
      Forward();
    }
    else if(user_input =='3'){
      Reverse();
    }
    else if(user_input =='+'){
      IncreaseSpeed();
    }
    else if(user_input =='-'){
      DecreaseSpeed();
    }
    else{
      Serial.println("Invalid option entered.");
    }
  }
  int is_go = get_distance();
  if (is_go) {
    Reverse();
  }
  else {
    Rotate_Left();
    delay(1000);
  }

}
void ngat_dem_xung1(){
  xung1++;
}
void ngat_dem_xung2(){
  xung2++;
}
void ngat_timer1(){
  v1=(float)xung1*60/11;
  xung1=0;
  v2=(float)xung2*60/11;
  xung2=0;
}

void Stop()
{
  Serial.println("Stop");
  usMotor_Status = BRAKE;
  motorGo(MOTOR_1, usMotor_Status, 0);
  motorGo(MOTOR_2, usMotor_Status, 0);
}

void Forward()
{
  Serial.println("Forward");
  usMotor_Status = CW;
  motorGo(MOTOR_1, usMotor_Status, usSpeed1);
  motorGo(MOTOR_2, usMotor_Status, usSpeed2);
}

void Reverse()
{
  Serial.println("Reverse");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, usMotor_Status, usSpeed1);
  motorGo(MOTOR_2, usMotor_Status, usSpeed2);
}
void Rotate_Right()
{
  Serial.println("RoR");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, CCW, usSpeed1);
  motorGo(MOTOR_2, CW, usSpeed2);
}
void Rotate_Left()
{
  Serial.println("RoL");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, CW, usSpeed1);
  motorGo(MOTOR_2, CCW, usSpeed2);
}
void Half_Right()
{
  Serial.println("HaR");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, BRAKE, 0);
  motorGo(MOTOR_2, CW, usSpeed2);
}
void Half_Left()
{
  Serial.println("HaL");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, CW, usSpeed1);
  motorGo(MOTOR_2, BRAKE, 0);
}
void IncreaseSpeed()
{
  usSpeed1 = usSpeed1 + 10;
  if(usSpeed1 > 255)
  {
    usSpeed1 = 255;  
  }
  
  Serial.print("Speed +: ");
  Serial.println(usSpeed2);

  motorGo(MOTOR_1, usMotor_Status, usSpeed1);
  motorGo(MOTOR_2, usMotor_Status, usSpeed2);  
}

void DecreaseSpeed()
{
  usSpeed2 = usSpeed2 - 10;
  if(usSpeed2 < 0)
  {
    usSpeed2 = 0;  
  }
  
  Serial.print("Speed -: ");
  Serial.println(usSpeed1);

  motorGo(MOTOR_1, usMotor_Status, usSpeed1);
  motorGo(MOTOR_2, usMotor_Status, usSpeed2);  
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
{
  if(motor == MOTOR_1)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, pwm); 
    
  }
  else if(motor == MOTOR_2)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_2, pwm);
  }
}

int get_distance() {
    unsigned long duration; // biến đo thời gian
    int distance;           // biến lưu khoảng cách
    
    /* Phát xung từ chân trig */
    digitalWrite(trig,0);   // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(trig,1);   // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(trig,0);   // tắt chân trig
    
    /* Tính toán thời gian */
    // Đo độ rộng xung HIGH ở chân echo. 
    duration = pulseIn(echo,HIGH);  
    // Tính khoảng cách đến vật.
    distance = int(duration/2/29.412);
    if (distance < 10) {
      return 0;
    }
    else {
      return 1;
    }
}