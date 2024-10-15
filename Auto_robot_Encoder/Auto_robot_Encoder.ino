#define PWM_MOTOR_1 5
#define PWM_MOTOR_2 6

#define EncoderA1 2
#define EncoderA2 3

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

float pidController(int target, float kp, float kd, float ki) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

  int e = encoder2Count - target;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  previousTime = currentTime;
  ePrevious = e;

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
short usSpeed1 = 100;  //default motor speed
short usSpeed2 = 255;  //default motor speed
unsigned short usMotor_Status = CW;
void setup() {
  Serial.begin(9600);
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
}

void loop() {
  int target = encoder1Count;
  motorGo(MOTOR_1, usMotor_Status, usSpeed1);
  
  float kp = 2;
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);

  moveMotor(CW, PWM_MOTOR_2, u);

  Serial.print(encoder1Count);
  Serial.print(", ");
  Serial.print(encoder2Count);
  Serial.println(", ");
}


void handleEncoder1() {
    encoder1Count++;
}
void handleEncoder2() {
    encoder2Count++;
}
void moveMotor(int dirPin, int pwmPin, float u){
  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }
 if (encoder2Count > encoder1Count){
  speed =0;
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
