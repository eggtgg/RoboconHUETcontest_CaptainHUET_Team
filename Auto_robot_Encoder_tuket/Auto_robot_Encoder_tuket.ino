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

long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

float kp = 3;
float kd = 0.1;
float ki = 0;

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
short usSpeed1 = 150;  //default motor speed
short usSpeed2 = 255;  //default motor speed
unsigned short usMotor_Status = CW;

int start_mode = 0;
void setup() {
  // Serial.begin(9600);
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


  while (true) {
    d1 = get_distance(Trig1, Echo1);
    d2 = get_distance(Trig2, Echo2);
    d3 = get_distance(Trig3, Echo3);

    if ((d1 <= 10) and (d3 > 10)) {
      start_mode = 3;
      break;
    }
    
    if ((d1 > 10) and (d3 <= 10)) {
      start_mode = 1;
      break;
    }
  }

  while (true) {
    d1 = get_distance(Trig1, Echo1);
    d2 = get_distance(Trig2, Echo2);
    d3 = get_distance(Trig3, Echo3);
    if (start_mode == 3) {
      if (d3 <= 10) {
        break;
      }
    }
    else {
      if (start_mode == 1) {
        if (d1 <= 10) {
          break;
        }
      }
    }
  }
}

void loop() {
  // Serial.print(encoder1Count); Serial.print("--");
  // Serial.println(encoder2Count);

  // start_mode=3;
  d1 = get_distance(Trig1, Echo1); //phai
  d2 = get_distance(Trig2, Echo2); //giua
  d3 = get_distance(Trig3, Echo3); //ben trai

  // Serial.print("d1: "); Serial.print(d1); Serial.print(";");
  // Serial.print("d3: "); Serial.print(d3); Serial.println(";");

  switch (start_mode) {
    case 1:
      // if(d2<2 && d2>850) lui();
      if (d2 > 3.0 && d2 < 900){
        thang();
        if(d1<=2.4 or d1>900) trai_15();
        if(d3<=2.0 or d3>900) phai_15();
      }
      else
      if ( d2 <= 3.0 | d2 >900) {
        //if ((d1 > 10) and (d1 < 850) and (d3 <= 10)) phai_90();

        //chung ket
        if(d1>=10 && d1<=850 && (d3<10 | d3 >850)) phai_90();
        else 
        if(d3>=10 && d3<=850 && (d1<10 | d1 >850)) trai_90();
        
        // cut
        if ((d1<6 | d1 >850) && (d3<6 | d3 >850)){
          trai_90();
        }
        //nga ba
        if ((d1>=15 && d1<850) && (d3>=15 && d3<850)){
          phai_90();
        }
      }
      break;
      
    case 3:
      // if(d2<2 && d2>850) lui();    
      if(d2 > 3  && d2 < 900){
        thang();
        if(d1<=2.4 or d1 >900) trai_15();
        if(d3<=2.0 or d3 >900 ) phai_15();
      }        
      else
      if (d2 <= 3 | d2 > 900) {
        //if ((d1 > 10) and (d1 < 850) and (d3 <= 10)) phai_90();
        if(d1>=15 && d1<=850 && (d3<15 | d3 >850)) phai_90();
        else 
        if(d3>=15 && d3<=850 && (d1<15 | d1 >850)) trai_90();
        
        // cut
        if ((d1<6 | d1 >850) && (d3<6 | d3 >850)){
          phai_90();
        }
        //nga ba
        if ((d1>=15 && d1<850) && (d3>=15 && d3<850)){
          trai_90();
        }
      }
      break;
  }
}

void thang() {
  int target = encoder1Count;
  motorGo(MOTOR_1, CW, 230);
  float u = pidController(target, kp, kd, ki);
  // Serial.print("cc");
  moveMotor(CW, PWM_MOTOR_2, u);
}


void lui() {
  int target = encoder1Count;
  motorGo(MOTOR_1, CCW, 230);
  float u = pidController(target, kp, kd, ki);
  // Serial.print("cc");
  moveMotor(CCW, PWM_MOTOR_2, u);
}


void trai_90() {
  motorGo(MOTOR_1, CCW, 0);
  motorGo(MOTOR_2, CCW, 0);
  encoder1Count=0;
  encoder2Count=0;
  delay(10);
  int target = encoder1Count;
  // 890
  while (target <= 920) {
    
    motorGo(MOTOR_1, CW, 200);
    float u = pidController(target, kp, kd, ki);
    // Serial.print("cc");
    moveMotor(CCW, PWM_MOTOR_2, u);
    target = encoder1Count;
  }
  motorGo(MOTOR_1, CCW, 0);
  motorGo(MOTOR_2, CW, 0);
  delay(10);
}
void trai_15() {
  // motorGo(MOTOR_1, CCW, 0);
  // motorGo(MOTOR_2, CCW, 0);
  encoder1Count=0;
  encoder2Count=0;
  // delay(5);
  int target = encoder1Count;
  while (target <= 70) {
    motorGo(MOTOR_1, CW, 150);
    float u = pidController(target, kp, kd, ki);
    // Serial.print("cc");
    moveMotor(CCW, PWM_MOTOR_2, u);
    target = encoder1Count;
  }
  // motorGo(MOTOR_1, CCW, 0);
  // motorGo(MOTOR_2, CW, 0);
  // delay(10);
  encoder1Count=0;
  encoder2Count=0;
}

void phai_90() {
  motorGo(MOTOR_1, CCW, 0);
  motorGo(MOTOR_2, CCW, 0);
  encoder1Count=0;
  encoder2Count=0;
  int target = encoder1Count;
  delay(10);
  // 920
  while (target <= 920) {
    motorGo(MOTOR_1, CCW, 200);
    float u = pidController(target, kp, kd, ki);
    // Serial.print("cc");
    moveMotor(CW, PWM_MOTOR_2, u);
    target = encoder1Count;
  }
  motorGo(MOTOR_1, CW, 0);
  motorGo(MOTOR_2, CCW, 0);
  delay(10);
}

void phai_15() {
  // motorGo(MOTOR_1, CCW, 0);
  // motorGo(MOTOR_2, CCW, 0);
  encoder1Count=0;
  encoder2Count=0;
  int target = encoder1Count;
  // delay(10);
  while (target <= 70) {
    motorGo(MOTOR_1, CCW, 150);
    float u = pidController(target, kp, kd, ki);
    // Serial.print("cc");
    moveMotor(CW, PWM_MOTOR_2, u);
    target = encoder1Count;
  }
  // motorGo(MOTOR_1, CW, 0);
  // motorGo(MOTOR_2, CCW, 0);
  // delay(10);
  encoder1Count=0;
  encoder2Count=0;
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


float get_distance(int Trig, int Echo) {
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