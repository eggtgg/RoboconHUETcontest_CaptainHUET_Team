#define PWM_MOTOR_1 5
#define PWM_MOTOR_2 6

#define EncoderA1 2
#define EncoderA2 3

// cam bien hong ngoai
#define hongNgoai A7

//HC3
#define Trig1 A0
#define Echo1 A1

//HC2
#define Trig2 8
#define Echo2 9

//HC1
#define Trig3 11
#define Echo3 12

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

float d1 = 100;
float d2 = 100;
float d3 = 100;

int i = 1;

long previousTime1 = 0;
long previousTime2 = 0;

float ePrevious1 = 0;
float ePrevious2 = 0;

float eIntegral1 = 0;
float eIntegral2 = 0;

volatile long  preC1 = 0;
volatile long  preC2 = 0;

float kp = 5;
float kd = 0.75;
float ki = 0.15;

float pidController1(int target, float kp, float kd, float ki) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime1)) / 1.0e6;

  float deltaC = preC1 - encoder1Count;

  int e = deltaC - target;
  float eDerivative = (e - ePrevious1) / deltaT;
  eIntegral1 = eIntegral1 + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral1);

  previousTime1 = currentTime;
  ePrevious1 = e;

  preC1 = encoder1Count;
  return u;
}

float pidController2(int target, float kp, float kd, float ki) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime2)) / 1.0e6;

  int deltaC = preC2 - encoder2Count;

  int e = deltaC - target;
  float eDerivative = (e - ePrevious2) / deltaT;
  eIntegral2 = eIntegral2 + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral2);

  previousTime2 = currentTime;
  ePrevious2 = e;

  preC2 = encoder2Count;
  return u;
}
#define BRAKE 0
#define CW    1
#define CCW   2
#define MOTOR_A1_PIN A3
#define MOTOR_B1_PIN A5
#define MOTOR_A2_PIN A2
#define MOTOR_B2_PIN 7
#define MOTOR_1 0
#define MOTOR_2 1
short usSpeed1 = 150;  //default motor speed
short usSpeed2 = 255;  //default motor speed
unsigned short usMotor_Status = CW;
void setup() {
  Serial.begin(9600);
  // delay(500);
  // pinmode
  pinMode(A7, INPUT);

  pinMode(Trig1,OUTPUT);
  pinMode(Echo1,INPUT);

  pinMode(Trig2,OUTPUT);
  pinMode(Echo2,INPUT);

  pinMode(Trig3,OUTPUT);
  pinMode(Echo3,INPUT);

  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT); 
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(EncoderA1, INPUT);
  pinMode(EncoderA2, INPUT);

  attachInterrupt(digitalPinToInterrupt(EncoderA2), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderA1), handleEncoder2, RISING);

  while (d1 >= 4) {
    d1 = get_distance(Trig1, Echo1);
    d2 = get_distance(Trig2, Echo2);
    d3 = get_distance(Trig3, Echo3);
  }

}

void loop() {
  Serial.print(encoder1Count); Serial.print("--");
  Serial.println(encoder2Count);

  d1 = get_distance(Trig1, Echo1);
  d2 = get_distance(Trig2, Echo2);
  d3 = get_distance(Trig3, Echo3);
  thang();
}

void thang() {
  int target1 = 10;
  int target2 = 10;
  float u1 = pidController1(target1, kp, kd, ki);
  float u2 = pidController2(target2, kp, kd, ki);
  // Serial.print("cc");
}

void handleEncoder1() {
    encoder1Count++;
}
void handleEncoder2() {
    encoder2Count++;
}

void moveMotor2(int dirPin, int pwmPin, float u){
  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }
  if(speed <= 0){
    speed = 0;
  }
    if(dirPin == CW)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(dirPin == CCW)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
  analogWrite(pwmPin, speed);
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
void Stop()
{
  // Serial.println("Stop");
  usMotor_Status = BRAKE;
  motorGo(MOTOR_1, usMotor_Status, 0);
  motorGo(MOTOR_2, usMotor_Status, 0);
}

void Forward()
{
  // Serial.println("Forward");
  usMotor_Status = CW;
  motorGo(MOTOR_1, usMotor_Status, usSpeed1);
  motorGo(MOTOR_2, usMotor_Status, usSpeed2);
}

void Reverse()
{
  // Serial.println("Reverse");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, usMotor_Status, usSpeed1);
  motorGo(MOTOR_2, usMotor_Status, usSpeed2);
}
void Rotate_Right()
{
  // Serial.println("RoR");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, CCW, usSpeed1);
  motorGo(MOTOR_2, CW, usSpeed2);
}
void Rotate_Left()
{
  // Serial.println("RoL");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, CW, usSpeed1);
  motorGo(MOTOR_2, CCW, usSpeed2);
}
void Half_Right()
{
  // Serial.println("HaR");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, BRAKE, 0);
  motorGo(MOTOR_2, CW, usSpeed2);
}
void Half_Left()
{
  // Serial.println("HaL");
  usMotor_Status = CCW;
  motorGo(MOTOR_1, CW, usSpeed1);
  motorGo(MOTOR_2, BRAKE, 0);
}

int get_distance(int Trig, int Echo) {
    unsigned long duration; // biến đo thời gian
    float distance;           // biến lưu khoảng cách
    
    /* Phát xung từ chân trig */
    digitalWrite(Trig,0);   // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(Trig,1);   // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(Trig,0);   // tắt chân trig
    
    /* Tính toán thời gian */
    // Đo độ rộng xung HIGH ở chân echo. 
    duration = pulseIn(Echo,HIGH);  
    // Tính khoảng cách đến vật.
    distance = float(duration/2/29.412);
    // Serial.println(distance);
      return distance;
}