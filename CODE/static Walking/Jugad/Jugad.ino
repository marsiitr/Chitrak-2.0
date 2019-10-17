#include <PinChangeInt.h>

#define a1_1 19
#define b1_1 A3
#define a2_1 18
#define b2_1 51

#define a1_2 20
#define b1_2 A0
#define a2_2 21
#define b2_2 A1

#define a1_3 A14
#define b1_3 A15
#define a2_3 A12
#define b2_3 A13

#define a1_4 2
#define b1_4 15
#define a2_4 3
#define b2_4 14

int A1_1 = 4, A2_1 = 5, A1_2 = 3, A2_2 = 2, A1_4 = 0, A2_4= 1;  //Change as per need

#define motor1_1  43
#define motor1pwm_1  12
#define motor2_1  41
#define motor2pwm_1  11

#define motor1_2  40
#define motor1pwm_2  8
#define motor2_2  42
#define motor2pwm_2  7

#define motor1_3  38
#define motor1pwm_3  10
#define motor2_3  17
#define motor2pwm_3  9

#define motor1_4  32
#define motor1pwm_4  5
#define motor2_4  28
#define motor2pwm_4  4

double pwm1_1, pwm2_1, pwm1_2, pwm2_2, pwm1_3, pwm2_3, pwm1_4, pwm2_4;

volatile int temp1_1 , counter1_1 = 0;
volatile int temp2_1 , counter2_1 = 0;

volatile int temp1_2 , counter1_2 = 0;
volatile int temp2_2 , counter2_2 = 0;

volatile int temp1_3 , counter1_3 = 0;
volatile int temp2_3 , counter2_3 = 0;

volatile int temp1_4 , counter1_4 = 0;
volatile int temp2_4 , counter2_4 = 0;

double theta1c_1, theta2c_1;
double theta1c_2, theta2c_2;
double theta1c_3, theta2c_3;
double theta1c_4, theta2c_4;

double theta1_1 = -6.2;
double theta2_1 = -15;
double theta1_2 = 7;
double theta2_2 = -15;
double theta1_3 = 6.2;
double theta2_3 = -15;
double theta1_4 = -7;
double theta2_4 = -15;

int s1_1, s2_1, s1_2, s2_2, s1_3, s2_3, s1_4, s2_4;
int t = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(a1_1, INPUT_PULLUP);
  pinMode(a2_1, INPUT_PULLUP);
  pinMode(b1_1, INPUT_PULLUP);
  pinMode(b2_1, INPUT_PULLUP);
  
  
  attachInterrupt(A1_1, ai1_1, CHANGE);
  attachInterrupt(A2_1, ai2_1, CHANGE);
  
  pinMode(a1_2, INPUT_PULLUP);
  pinMode(a2_2, INPUT_PULLUP);
  pinMode(b1_2, INPUT_PULLUP);
  pinMode(b2_2, INPUT_PULLUP);

  attachInterrupt(A1_2, ai1_2, CHANGE);
  attachInterrupt(A2_2, ai2_2, CHANGE);
  
  pinMode(a1_3, INPUT_PULLUP);
  pinMode(a2_3, INPUT_PULLUP);
  pinMode(b1_3, INPUT_PULLUP);
  pinMode(b2_3, INPUT_PULLUP);

  PCintPort::attachInterrupt(a1_3, ai2_3, CHANGE);
  PCintPort::attachInterrupt(a2_3, ai3_3, CHANGE);
  
  pinMode(a1_4, INPUT_PULLUP);
  pinMode(a2_4, INPUT_PULLUP);
  pinMode(b1_4, INPUT_PULLUP);
  pinMode(b2_4, INPUT_PULLUP);

  attachInterrupt(A1_4, ai4_4, CHANGE);
  attachInterrupt(A2_4, ai5_4, CHANGE);

  pinMode(motor1_1, OUTPUT);
  pinMode(motor1pwm_1, OUTPUT);
  pinMode(motor2_1, OUTPUT);
  pinMode(motor2pwm_1, OUTPUT);

  pinMode(motor1_2, OUTPUT);
  pinMode(motor1pwm_2, OUTPUT);
  pinMode(motor2_2, OUTPUT);
  pinMode(motor2pwm_2, OUTPUT);
  
  pinMode(motor1_3, OUTPUT);
  pinMode(motor1pwm_3, OUTPUT);
  pinMode(motor2_3, OUTPUT);
  pinMode(motor2pwm_3, OUTPUT);

  pinMode(motor1_4, OUTPUT);
  pinMode(motor1pwm_4, OUTPUT);
  pinMode(motor2_4, OUTPUT);
  pinMode(motor2pwm_4, OUTPUT);
  t = millis();
  while(millis()-t<15000){
    
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  s1_1 = 0;
  s2_1 = 0;
  s1_2 = 0;
  s2_2 = 0;
  s1_3 = 0;
  s2_3 = 0;
  s1_4 = 0;
  s2_4 = 0;

//1st and 4th legs are in ellipse
  pwm1_3 = 250;
  pwm1_2 = 250;  
  while(s1_1 < 2 || s2_1 < 2 || s1_2 < 1 || s1_3 < 1 || s1_4 < 2 || s2_4 < 2){
    
    if ( counter1_1 != temp1_1 ) {
      temp1_1 = counter1_1;

      if (counter1_1 > 1200)
      {
        counter1_1 = 0;
      }
      theta1c_1 = (counter1_1 * 0.3);
    }

    if ( counter2_1 != temp2_1 ) {
      temp2_1 = counter2_1;

      if (counter2_1 > 1200)
      {
        counter2_1 = 0;
      }
      theta2c_1 = (counter2_1 * 0.3);
    }

    if ( counter1_2 != temp1_2 ) {
      temp1_2 = counter1_2;

      if (counter1_2 > 1200) {
        counter1_2 = 0;
      }

      theta1c_2 = (counter1_2 * 0.3);
    }

    if ( counter2_2 != temp2_2 ) {
      temp2_2 = counter2_2;

      if (counter2_2 > 1200) {
        counter2_2 = 0;
      }
      theta2c_2 = -(counter2_2 * 0.3);
    }

    if ( counter1_3 != temp1_3 ) {
      temp1_3 = counter1_3;

      if (counter1_3 > 1200)
        counter1_3 = 0;

      theta1c_3 = (counter1_3 * 0.3);
    }

    if ( counter2_3 != temp2_3 ) {
      temp2_3 = counter2_3;

      if (counter2_3 > 1200)
        counter2_3 = 0;

      theta2c_3 = -(counter2_3 * 0.3);
    }

    if ( counter1_4 != temp1_4 ) {
      temp1_4 = counter1_4;

      if (counter1_4 > 1200)
        counter1_4 = 0;

      theta1c_4 = (counter1_4 * 0.3);
    }

    if ( counter2_4 != temp2_4 ) {
      temp2_4 = counter2_4;

      if (counter2_4 > 1200)
        counter2_4 = 0;

      theta2c_4 = -(counter2_4 * 0.3);
    }

//-------------------------111111111111111111111111111111--------------------------------

    if(s2_1 == 0 && theta2c_1 <= theta2_1){          // theta2_1 is -ve
      analogWrite(motor2_1, 0);
      s2_1 = 1;
      s1_1 = 1;
    }

    if(s1_1 == 1 && theta1c_1 <= theta1_1){          // thata1_1 is -ve
      analogWrite(motor1pwm_1, 0);
      s1_1 = 2;
    }

    if(s2_1 == 1 && theta2c_1 >= 0){
      analogWrite(motor2pwm_1, 0);
      s2_1 = 2;
    }

//-----------------------222222222222222222222222222222222--------------------------------

    if(s1_2 == 0 && theta1c_2 >= theta1_2){
      analogWrite(motor1pwm_2, 0);
      s1_2 = 1;
    }

//----------------------33333333333333333333333333333333333-------------------------------

    if(s1_3 == 0 && theta1c_3 >= theta1_3){
      analogWrite(motor1pwm_3, 0);
      s1_3 = 1;
    }

//-----------------------4444444444444444444444444444444444------------------------------

    if(s2_4 == 0 && theta2c_4 <= theta2_4){          // theta2_4 is -ve
      analogWrite(motor2_4, 0);
      s2_4 = 1;
      s1_4 = 1;
    }

    if(s1_4 == 1 && theta1c_4 <= theta1_4){          // thata1_4 is -ve
      analogWrite(motor1pwm_4, 0);
      s1_4 = 2;
    }

    if(s2_4 == 1 && theta2c_4 >= 0){
      analogWrite(motor2pwm_4, 0);
      s2_4 = 2;
    }
//----------------------------------------------------------------------------------------
//1

    if(s1_1 == 1){
      upr_mtr_fwd_1();
      pwm1_1 = 45;
      analogWrite(motor1pwm_1, pwm1_1);
    }
    
    if(s2_1 == 0){
      lwr_mtr_fwd_1();
      pwm2_1 = 250;
      analogWrite(motor2pwm_1, pwm2_1);
    }
    else if(s2_1 == 1){
      lwr_mtr_bwd_1();
      pwm2_1 = 250;
      analogWrite(motor2pwm_1, pwm2_1);
    }

//2      

    if(s1_2 == 0){
      upr_mtr_bwd_2();
      pwm1_2 += 7;
      analogWrite(motor1pwm_2, pwm1_2);
    }

//3

    if(s1_3 == 0){
      upr_mtr_bwd_3();
      pwm1_3 += 7;
      analogWrite(motor1pwm_3, pwm1_3);
    }

//4

    if(s1_4 == 1){
      upr_mtr_fwd_4();
      pwm1_4 = 45;
      analogWrite(motor1pwm_4, pwm1_4);
    }
    
    if(s2_4 == 0){
      lwr_mtr_fwd_4();
      pwm2_4 = 250;
      analogWrite(motor2pwm_4, pwm2_4);
    }
    else if(s2_4 == 1){
      lwr_mtr_bwd_4();
      pwm2_4 = 250;
      analogWrite(motor2pwm_4, pwm2_4);
    }
  
  }
  
  s1_1 = 0;
  s2_1 = 0;
  s1_2 = 0;
  s2_2 = 0;
  s1_3 = 0;
  s2_3 = 0;
  s1_4 = 0;
  s2_4 = 0;
  delay(100);

//2nd and 3rd legs are in ellipse
  pwm1_1 = 250;
  pwm1_4 = 250;
  while(s1_1 < 1 || s1_2 < 2 || s2_2 < 2 || s1_3 < 2 || s2_3 < 2 || s1_4 < 1){
    
    if ( counter1_1 != temp1_1 ) {
      temp1_1 = counter1_1;

      if (counter1_1 > 1200)
      {
        counter1_1 = 0;
      }
      theta1c_1 = (counter1_1 * 0.3);
    }

    if ( counter2_1 != temp2_1 ) {
      temp2_1 = counter2_1;

      if (counter2_1 > 1200)
      {
        counter2_1 = 0;
      }
      theta2c_1 = (counter2_1 * 0.3);
    }

    if ( counter1_2 != temp1_2 ) {
      temp1_2 = counter1_2;

      if (counter1_2 > 1200) {
        counter1_2 = 0;
      }

      theta1c_2 = (counter1_2 * 0.3);
    }

    if ( counter2_2 != temp2_2 ) {
      temp2_2 = counter2_2;

      if (counter2_2 > 1200) {
        counter2_2 = 0;
      }
      theta2c_2 = -(counter2_2 * 0.3);
    }

    if ( counter1_3 != temp1_3 ) {
      temp1_3 = counter1_3;

      if (counter1_3 > 1200)
        counter1_3 = 0;

      theta1c_3 = (counter1_3 * 0.3);
    }

    if ( counter2_3 != temp2_3 ) {
      temp2_3 = counter2_3;

      if (counter2_3 > 1200)
        counter2_3 = 0;

      theta2c_3 = -(counter2_3 * 0.3);
    }

    if ( counter1_4 != temp1_4 ) {
      temp1_4 = counter1_4;

      if (counter1_4 > 1200)
        counter1_4 = 0;

      theta1c_4 = (counter1_4 * 0.3);
    }

    if ( counter2_4 != temp2_4 ) {
      temp2_4 = counter2_4;

      if (counter2_4 > 1200)
        counter2_4 = 0;

      theta2c_4 = -(counter2_4 * 0.3);
    }

//-------------------------111111111111111111111111111111--------------------------------

    if(s1_1 == 0 && theta1c_1 >= 0){
      analogWrite(motor1pwm_1, 0);
      s1_1 = 1;
    }

//-----------------------222222222222222222222222222222222--------------------------------

    if(s2_2 == 0 && theta2c_2 <= theta2_2){          // theta2_2 is -ve
      analogWrite(motor2_2, 0);
      s2_2 = 1;
      s1_2 = 1;
    }

    if(s1_2 == 1 && theta1c_2 <= 0){          // thata1_2 is -ve
      analogWrite(motor1pwm_2, 0);
      s1_2 = 2;
    }

    if(s2_2 == 1 && theta2c_2 >= 0){
      analogWrite(motor2pwm_2, 0);
      s2_2 = 3;
    }

//----------------------33333333333333333333333333333333333-------------------------------

    if(s2_3 == 0 && theta2c_3 <= theta2_3){          // theta2_3 is -ve
      analogWrite(motor2_3, 0);
      s2_3 = 1;
      s1_3 = 1;
    }

    if(s1_3 == 1 && theta1c_3 <= 0){          // thata1_3 is -ve
      analogWrite(motor1pwm_3, 0);
      s1_3 = 2;
    }

    if(s2_3 == 1 && theta2c_3 >= 0){
      analogWrite(motor2pwm_3, 0);
      s2_3 = 3;
    }

//-----------------------4444444444444444444444444444444444------------------------------

    if(s1_4 == 0 && theta1c_4 >= 0){
      analogWrite(motor1pwm_4, 0);
      s1_4 = 1;
    }
//----------------------------------------------------------------------------------------

//1

    if(s1_1 == 0){
      upr_mtr_bwd_1();
      pwm1_1 += 7;
      analogWrite(motor1pwm_1, pwm1_1);
    }

//2      

    if(s1_2 == 1){
      upr_mtr_fwd_2();
      pwm1_2 = 45;
      analogWrite(motor1pwm_2, pwm1_2);
    }
    
    if(s2_2 == 0){
      lwr_mtr_fwd_2();
      pwm2_2 = 250;
      analogWrite(motor2pwm_2, pwm2_2);
    }
    else if(s2_2 == 1){
      lwr_mtr_bwd_2();
      pwm2_2 = 250;
      analogWrite(motor2pwm_2, pwm2_2);
    }

//3

    if(s1_3 == 1){
      upr_mtr_fwd_3();
      pwm1_3 = 45;
      analogWrite(motor1pwm_3, pwm1_3);
    }
    
    if(s2_3 == 0){
      lwr_mtr_fwd_3();
      pwm2_3 = 250;
      analogWrite(motor2pwm_3, pwm2_3);
    }
    else if(s2_3 == 1){
      lwr_mtr_bwd_3();
      pwm2_3 = 250;
      analogWrite(motor2pwm_3, pwm2_3);
    }

//4

    if(s1_4 == 0){
      upr_mtr_bwd_4();
      pwm1_4 += 7;
      analogWrite(motor1pwm_4, pwm1_4);
    }
  
  }

  delay(100);
  
}

void upr_mtr_fwd_1() {
  digitalWrite(motor1_1, HIGH);
}

void upr_mtr_bwd_1() {
  digitalWrite(motor1_1, LOW);
}

void lwr_mtr_fwd_1() {
  digitalWrite(motor2_1, HIGH);
}

void lwr_mtr_bwd_1() {
  digitalWrite(motor2_1, LOW);
}

void upr_mtr_fwd_2() {
  digitalWrite(motor1_2, HIGH);
}

void upr_mtr_bwd_2() {
  digitalWrite(motor1_2, LOW);
}

void lwr_mtr_fwd_2() {
  digitalWrite(motor2_2, HIGH);
}

void lwr_mtr_bwd_2() {
  digitalWrite(motor2_2, LOW);
}

void upr_mtr_fwd_3(){
  digitalWrite(motor1_3, HIGH);
}

void upr_mtr_bwd_3(){
  digitalWrite(motor1_3, LOW);
}

void lwr_mtr_fwd_3(){
  digitalWrite(motor2_3, HIGH);
}

void lwr_mtr_bwd_3(){
  digitalWrite(motor2_3, LOW);
}

void upr_mtr_fwd_4(){
  digitalWrite(motor1_4, HIGH);
}

void upr_mtr_bwd_4(){
  digitalWrite(motor1_4, LOW);
}

void lwr_mtr_fwd_4(){
  digitalWrite(motor2_4, HIGH);
}

void lwr_mtr_bwd_4(){
  digitalWrite(motor2_4, LOW);
}

void ai1_1() {
  if (digitalRead(b1_1) == !digitalRead(a1_1)) {
    counter1_1++;
  }
  else {
    counter1_1--;
  }
//  s1_1 = !s1_1;
}

void ai2_1() {
  if (digitalRead(b2_1) == !digitalRead(a2_1)) {
    counter2_1++;
  }
  else {
    counter2_1--;
  }
//  s2_1 = !s2_1;
}

void ai1_2() {
  if (digitalRead(b1_2) == !digitalRead(a1_2)) {
    counter1_2++;
  }
  else {
    counter1_2--;
  }
//  s1_2 = !s1_2;
}

void ai2_2() {
  if (digitalRead(b2_2) == !digitalRead(a2_2)) {
    counter2_2++;
  }
  else {
    counter2_2--;
  }
//  s2_2 = !s2_2;
}

void ai2_3() {
  if (digitalRead(b1_3) == !digitalRead(a1_3)) {
    counter1_3++;
  }
  else {
    counter1_3--;
  }
//  s1_3 = !s1_3;
}

void ai3_3() {
  if (digitalRead(b2_3) == !digitalRead(a2_3)) {
    counter2_3++;
  }
  else {
    counter2_3--;
  }
//  s2_3 = !s2_3;
}

void ai4_4() {
  if (digitalRead(b1_4) == !digitalRead(a1_4)) {
    counter1_4++;
  }
  else {
    counter1_4--;
  }
//  s1_4 = !s1_4;
}

void ai5_4() {
  if (digitalRead(b2_4) == !digitalRead(a2_4)) {
    counter2_4++;
  }
  else {
    counter2_4--;
  }
//  s2_4 = !s2_4;
}
