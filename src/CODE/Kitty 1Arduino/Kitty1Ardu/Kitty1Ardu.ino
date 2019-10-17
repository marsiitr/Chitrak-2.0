#include <PinChangeInt.h>

#define a1_1 20
#define a2_1 19
#define a1_2 21
#define a2_2 A11
#define a1_3 3
#define a2_3 A12
#define a1_4 18
#define a2_4 A14

int A1_1 = 3, A1_2 = 2, A1_3 = 1, A1_4 = 5;
int A2_1 = 4;
#define b1_1 A15
#define b2_1 51
#define b1_2 A2
#define b2_2 A0
#define b1_3 48
#define b2_3 53
#define b1_4 52
#define b2_4 47

#define motor1_1  43
#define motor2_1  40
#define motor1pwm_1  2
#define motor2pwm_1  11

#define motor1_2  31
#define motor2_2  29
#define motor1pwm_2  5
#define motor2pwm_2  10

#define motor1_3  14
#define motor2_3  22
#define motor1pwm_3  6
#define motor2pwm_3  7

#define motor1_4  36
#define motor2_4  37
#define motor1pwm_4  8
#define motor2pwm_4  9

bool state1_1 = true;
bool state2_1 = true;
bool state1_2 = true;
bool state2_2 = true;
bool state1_3 = true;
bool state2_3 = true;
bool state1_4 = true;
bool state2_4 = true;

double alpha_1;
double w = 0.1, theta1c_1 = 0.0 , theta2c_1 = 0.0, theta1_1, theta2_1, error1_1, error2_1, correction1_1, correction2_1, c1_1, c2_1;
double dif_error1_1 , prev_error1_1 = 0.0 , dif_error2_1 , prev_error2_1 = 0.0;

double alpha_2;
double theta1c_2 = 0.0 , theta2c_2 = 0.0, theta1_2, theta2_2, error1_2, error2_2, correction1_2, correction2_2, c1_2, c2_2;
double dif_error1_2 , prev_error1_2 = 0.0 , dif_error2_2 , prev_error2_2 = 0.0;

double alpha_3, theta1c_3 = 0.0 , theta2c_3 = 0.0, theta1_3, theta2_3, error1_3, error2_3, correction1_3, correction2_3, c1_3, c2_3;
double dif_error1_3 , prev_error1_3 = 0.0 , dif_error2_3 , prev_error2_3 = 0.0;

double alpha_4, theta1c_4 = 0.0 , theta2c_4 = 0.0, theta1_4, theta2_4, error1_4, error2_4, correction1_4, correction2_4, c1_4, c2_4;
double dif_error1_4 , prev_error1_4 = 0.0 , dif_error2_4 , prev_error2_4 = 0.0;

volatile int temp1_1 , counter1_1 = 0;
volatile int temp2_1 , counter2_1 = 0;

volatile int temp1_2 , counter1_2 = 0;
volatile int temp2_2 , counter2_2 = 0;

volatile int temp1_3 , counter1_3 = 0;
volatile int temp2_3 , counter2_3 = 0;

volatile int temp1_4 , counter1_4 = 0;
volatile int temp2_4 , counter2_4 = 0;

double Kp1 = 1.5, Kp2 = 1.5, Kd1 = 2.0, Kd2 = 2.0 ;
int l1 = 25, l2 = 25;

double theta1AT1 = 53.728, theta1AT2 = 45.78, theta1AT3 = 69.71, theta1_AT1 = 0, theta1_AT2 = 0.1, theta1_AT3 = 0, theta1__AT1 = 0, theta1__AT2 = 0.1, theta1__AT3 = 0;
double theta2AT1 = 42.68, theta2AT2 = 76.73, theta2AT3 = 50.75, theta2_AT1 = 0, theta2_AT2 = 0.1, theta2_AT3 = 0, theta2__AT1 = 0, theta2__AT2 = 0.1, theta2__AT3 = 0;

double theta1AT1_2 = 54.68, theta1AT2_2 = 48.41, theta1AT3_2 = 72.815, theta1_AT1_2 = 0, theta1_AT2_2 = 0.1, theta1_AT3_2 = 0, theta1__AT1_2 = 0, theta1__AT2_2 = 0.1, theta1__AT3_2 = 0;
double theta2AT1_2 = 45.57, theta2AT2_2 = 77.29, theta2AT3_2 = 49.55, theta2_AT1_2 = 0, theta2_AT2_2 = 0.1, theta2_AT3_2 = 0, theta2__AT1_2 = 0, theta2__AT2_2 = 0.1, theta2__AT3_2 = 0;

double theta1AT1_3=53.728,theta1AT2_3=45.78,theta1AT3_3=69.71,theta1_AT1_3=0,theta1_AT2_3=0.1,theta1_AT3_3=0,theta1__AT1_3=0,theta1__AT2_3=0.1,theta1__AT3_3=0;
double theta2AT1_3=42.68,theta2AT2_3=76.73,theta2AT3_3=50.75,theta2_AT1_3=0,theta2_AT2_3=0.1,theta2_AT3_3=0,theta2__AT1_3=0,theta2__AT2_3=0.1,theta2__AT3_3=0;

double theta1AT1_4=54.68,theta1AT2_4=48.41,theta1AT3_4=72.815,theta1_AT1_4=0,theta1_AT2_4=0.1,theta1_AT3_4=0,theta1__AT1_4=0,theta1__AT2_4=0.1,theta1__AT3_4=0;
double theta2AT1_4=45.57,theta2AT2_4=77.29,theta2AT3_4=49.55,theta2_AT1_4=0,theta2_AT2_4=0.1,theta2_AT3_4=0,theta2__AT1_4=0,theta2__AT2_4=0.1,theta2__AT3_4=0;

void setup()
{
  Serial.begin(9600);
  pinMode(a1_1, INPUT_PULLUP);
  pinMode(a2_1, INPUT_PULLUP);
  pinMode(a1_2, INPUT_PULLUP);
  pinMode(a2_2, INPUT_PULLUP);
  pinMode(a1_3, INPUT_PULLUP);
  pinMode(a2_3, INPUT_PULLUP);
  pinMode(a1_4, INPUT_PULLUP);
  pinMode(a2_4, INPUT_PULLUP);

  pinMode(b1_1, INPUT_PULLUP);
  pinMode(b2_1, INPUT_PULLUP);
  pinMode(b1_2, INPUT_PULLUP);
  pinMode(b2_2, INPUT_PULLUP);
  pinMode(b1_3, INPUT_PULLUP);
  pinMode(b2_3, INPUT_PULLUP);
  pinMode(b1_4, INPUT_PULLUP);
  pinMode(b2_4, INPUT_PULLUP);

  attachInterrupt(A1_1, ai1_1, CHANGE);
  attachInterrupt(A2_1, ai2_1, CHANGE);
  attachInterrupt(A1_2, ai1_2, CHANGE);
  PCintPort::attachInterrupt(a2_2, ai2_2, CHANGE);
  attachInterrupt(A1_3, ai1_3, CHANGE);
  PCintPort::attachInterrupt(a2_3, ai2_3, CHANGE);
  attachInterrupt(A1_4, ai1_4, CHANGE);
  PCintPort::attachInterrupt(a2_4, ai2_4, CHANGE);
  state1_1 = digitalRead(a1_1);
  state2_1 = digitalRead(a2_1);
  state1_2 = digitalRead(a1_2);
  state2_2 = digitalRead(a2_2);
  state1_3 = digitalRead(a1_3);
  state2_3 = digitalRead(a2_3);
  state1_4 = digitalRead(a1_4);
  state2_4 = digitalRead(a2_4);

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
}

void loop()
{
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    double xe_2 = 4.66 + u ;
    double ye_2 = -45 ;

    double xe_3 = 1.3333 + u ;
    double ye_3 = -45 ;

    double xe_4 = -6 + u ;
    double ye_4 = -45;

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


    if (atan(ye_2 / xe_2) > 0)
      alpha_2 = atan(ye_2 / xe_2) - 3.14159;

    else
      alpha_2 = atan(ye_2 / xe_2);

    if (atan(ye_3 / xe_3) > 0)
      alpha_3 = atan(ye_3 / xe_3) - PI;

    else
      alpha_3 = atan(ye_3 / xe_3);

    if (atan(ye_4 / xe_4) > 0)
      alpha_4 = atan(ye_4 / xe_4) - PI;

    else
      alpha_4 = atan(ye_4 / xe_4);

    //----------------------------------------------1111111111111111111------------------------------------------------------------------

    if (t < 0.5) {
      theta1_1 = (-1) * theta(theta1AT1, theta1_AT1, theta1__AT1, theta1AT2, theta1_AT2, theta1__AT2, t);
      theta2_1 = (-1) * theta(theta2AT1, theta2_AT1, theta2__AT1, theta2AT2, theta2_AT2, theta2__AT2, t);
    }
    else {
      theta1_1 = (-1) * theta(theta1AT2, theta1_AT2, theta1__AT2, theta1AT3, theta1_AT3, theta1__AT3, t - 0.5);
      theta2_1 = (-1) * theta(theta2AT2, theta2_AT2, theta2__AT2, theta2AT3, theta2_AT3, theta2__AT3, t - 0.5);
    }

    error1_1 = theta1_1 - theta1c_1 + 53.728;
    error2_1 = theta2_1 - theta2c_1 / 4 + 42.68;

    dif_error1_1 = error1_1 - prev_error1_1;
    dif_error2_1 = error2_1 - prev_error2_1;

    if (t < 0.5)
      c1_1 = (Kp1 - 0.5) * error1_1 + Kd1 * (dif_error1_1);
    else
      c1_1 = (Kp1 - 0.9) * error1_1 + Kd1 * (dif_error1_1);

    c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

    prev_error1_1 = error1_1;
    prev_error2_1 = error2_1;

    correction1_1 = map(abs(c1_1), 0, 70, 0, 80);
    if (t < 0.5)
      correction2_1 = map(abs(c2_1), 0, 90, 0, 220);
    else
      correction2_1 = map(abs(c2_1), 0, 90, 0, 225);

    //------------------------------------------------22222222222222-------------------------------------------------------------------------

    theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

    error1_2 = theta1_2 - theta1c_2 + 65.8795;
    error2_2 = theta2_2 - theta2c_2 / 4 + 50.4;

    dif_error1_2 = error1_2 - prev_error1_2;
    dif_error2_2 = error2_2 - prev_error2_2;

    c1_2 = (Kp1 - 0.5) * error1_2 + (Kd1 + 0.5) * (dif_error1_2);
    c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

    prev_error1_2 = error1_2;
    prev_error2_2 = error2_2;

    correction1_2 = map(abs(c1_2), 0, 30, 0, 50);     //50
    correction2_2 = map(abs(c2_2), 0, 35, 0, 150);    //175

    //----------------------------------------------3333333333333333333333--------------------------------------------------------------------

    theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
    theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));


    error1_3 = theta1_3 - theta1c_3 + 62.513;
    error2_3 = theta2_3 - theta2c_3 / 4 + 51.58;

    dif_error1_3 = error1_3 - prev_error1_3;
    prev_error1_3 = error1_3;
    dif_error2_3 = error2_3 - prev_error2_3;
    prev_error2_3 = error2_3;

    c1_3 = (Kp1 - 0.5) * error1_3 + (Kd1 + 0.5) * (dif_error1_3);
    c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

    correction1_3 = map(abs(c1_3), 0, 30, 0, 50); //50
    correction2_3 = map(abs(c2_3), 0, 35, 0, 150);//175

    //----------------------------------------------------44444444444444444444-----------------------------------------------------------------------

    theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

    error1_4 = theta1_4 - theta1c_4 + 72.815;
    error2_4 = theta2_4 - theta2c_4 / 4 + 49.55;

    dif_error1_4 = error1_4 - prev_error1_4;
    prev_error1_4 = error1_4;
    dif_error2_4 = error2_4 - prev_error2_4;
    prev_error2_4 = error2_4;

    c1_4 = (Kp1 - 0.5) * error1_4 + (Kd1 + 0.5) * (dif_error1_4);
    c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

    correction1_4 = map(abs(c1_4), 0, 30, 0, 50);     //50
    correction2_4 = map(abs(c2_4), 0, 35, 0, 150);    //175

    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - 53.728);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - 42.68);
    Serial.print("theta2_1=");
    Serial.println(theta2_1);
    Serial.print("c1_1=");
    Serial.println(c1_1);
    Serial.print("c2_1=");
    Serial.println(c2_1);
    Serial.print("pwm1=");
    Serial.println(correction1_1);
    Serial.print("pwm2=");
    Serial.println(correction2_1);
    Serial.println("------------------------");


    if (error1_1 < 0 ) {
      upr_mtr_fwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    else if (error1_1 > 0) {
      upr_mtr_bwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    if (error2_1 < 0) {
      lwr_mtr_fwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    else if (error2_1 > 0) {
      lwr_mtr_bwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    if (error1_2 < 0 ) {
      upr_mtr_fwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    else if (error1_2 > 0) {
      upr_mtr_bwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    if (error2_2 < 0) {
      lwr_mtr_fwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    else if (error2_2 > 0) {
      lwr_mtr_bwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    if (error1_3 < 0 ) {
      upr_mtr_fwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    else if (error1_3 > 0) {
      upr_mtr_bwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    if (error2_3 < 0) {
      lwr_mtr_fwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    else if (error2_3 > 0) {
      lwr_mtr_bwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    if (error1_4 < 0 ) {
      upr_mtr_fwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    else if (error1_4 > 0) {
      upr_mtr_bwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    if (error2_4 < 0) {
      lwr_mtr_fwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

    else if (error2_4 > 0) {
      lwr_mtr_bwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {

    double xe_1 = -4 + u ;
    double ye_1 = -45 ;

    double xe_3 = 1.3333 + u + 5.3333 ;
    double ye_3 = -45 ;

    double xe_4 = -6 + u + 5.3333;
    double ye_4 = -45;

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

      if (counter2_2 > 1200)
      {
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


    if (atan(ye_1 / xe_1) > 0)
      alpha_1 = atan(ye_1 / xe_1) - 3.14159;

    else
      alpha_1 = atan(ye_1 / xe_1);

    if (atan(ye_3 / xe_3) > 0)
      alpha_3 = atan(ye_3 / xe_3) - PI;

    else
      alpha_3 = atan(ye_3 / xe_3);

    if (atan(ye_4 / xe_4) > 0)
      alpha_4 = atan(ye_4 / xe_4) - PI;

    else
      alpha_4 = atan(ye_4 / xe_4);

    //------------------------------------------------------11111111111111111111----------------------------------------------------------------

    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

    error1_1 = theta1_1 - theta1c_1 + 53.728;
    error2_1 = theta2_1 - theta2c_1 / 4 + 42.68;

    dif_error1_1 = error1_1 - prev_error1_1;
    dif_error2_1 = error2_1 - prev_error2_1;

    c1_1 = (Kp1 - 0.5) * error1_1 + (Kd1 + 0.5) * (dif_error1_1);
    c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

    prev_error1_1 = error1_1;
    prev_error2_1 = error2_1;

    correction1_1 = map(abs(c1_1), 0, 35, 0, 90);       //40 // 30
    correction2_1 = map(abs(c2_1), 0, 40, 0, 175);      //35

    //-------------------------------------------------------------22222222222222222222---------------------------------------------------

    if (t < 0.5) {
      theta1_2 = (-1) * theta(theta1AT1_2, theta1_AT1_2, theta1__AT1_2, theta1AT2_2, theta1_AT2_2, theta1__AT2_2, t);
      theta2_2 = (-1) * theta(theta2AT1_2, theta2_AT1_2, theta2__AT1_2, theta2AT2_2, theta2_AT2_2, theta2__AT2_2, t);
    }
    else {
      theta1_2 = (-1) * theta(theta1AT2_2, theta1_AT2_2, theta1__AT2_2, theta1AT3_2, theta1_AT3_2, theta1__AT3_2, t - 0.5);
      theta2_2 = (-1) * theta(theta2AT2_2, theta2_AT2_2, theta2__AT2_2, theta2AT3_2, theta2_AT3_2, theta2__AT3_2, t - 0.5);
    }

    error1_2 = theta1_2 - theta1c_2 + 65.8795;
    error2_2 = theta2_2 - theta2c_2 / 4 + 50.4;

    dif_error1_2 = error1_2 - prev_error1_2;
    dif_error2_2 = error2_2 - prev_error2_2;

    if (t < 0.5)
      c1_2 = (Kp1 - 0.5) * error1_2 + Kd1 * (dif_error1_2);
    else
      c1_2 = (Kp1 - 0.9) * error1_2 + Kd1 * (dif_error1_2);
    c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

    prev_error1_2 = error1_2;
    prev_error2_2 = error2_2;

    correction1_2 = map(abs(c1_2), 0, 70, 0, 80 );
    if (t < 0.5)
      correction2_2 = map(abs(c2_2), 0, 90, 0, 200);
    else
      correction2_2 = map(abs(c2_2), 0, 90, 0, 210);

    //----------------------------------------------3333333333333333333333--------------------------------------------------------------------

    theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
    theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));


    error1_3 = theta1_3 - theta1c_3 + 62.513;
    error2_3 = theta2_3 - theta2c_3 / 4 + 51.58;

    dif_error1_3 = error1_3 - prev_error1_3;
    prev_error1_3 = error1_3;
    dif_error2_3 = error2_3 - prev_error2_3;
    prev_error2_3 = error2_3;

    c1_3 = (Kp1 - 0.5) * error1_3 + (Kd1 + 0.5) * (dif_error1_3);
    c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

    correction1_3 = map(abs(c1_3), 0, 30, 0, 50); //50
    correction2_3 = map(abs(c2_3), 0, 35, 0, 150);//175

    //----------------------------------------------------44444444444444444444-----------------------------------------------------------------------

    theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

    error1_4 = theta1_4 - theta1c_4 + 72.815;
    error2_4 = theta2_4 - theta2c_4 / 4 + 49.55;

    dif_error1_4 = error1_4 - prev_error1_4;
    prev_error1_4 = error1_4;
    dif_error2_4 = error2_4 - prev_error2_4;
    prev_error2_4 = error2_4;

    c1_4 = (Kp1 - 0.5) * error1_4 + (Kd1 + 0.5) * (dif_error1_4);
    c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

    correction1_4 = map(abs(c1_4), 0, 30, 0, 50);     //50
    correction2_4 = map(abs(c2_4), 0, 35, 0, 150);    //175

    Serial.print("x=");
    Serial.println(xe_1);
    Serial.print("y=");
    Serial.println(ye_1);
    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - 53.728);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - 42.68);
    Serial.print("theta2_1=");
    Serial.println(theta2_1);
    Serial.print("c1_1=");
    Serial.println(c1_1);
    Serial.print("c2_1=");
    Serial.println(c2_1);
    Serial.print("pwm1=");
    Serial.println(correction1_1);
    Serial.print("pwm2=");
    Serial.println(correction2_1);
    Serial.println("------------------------");


    if (error1_1 < 0 ) {
      upr_mtr_fwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    else if (error1_1 > 0) {
      upr_mtr_bwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    if (error2_1 < 0) {
      lwr_mtr_fwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    else if (error2_1 > 0) {
      lwr_mtr_bwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    if (error1_2 < 0 ) {
      upr_mtr_fwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    else if (error1_2 > 0) {
      upr_mtr_bwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    if (error2_2 < 0) {
      lwr_mtr_fwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    else if (error2_2 > 0) {
      lwr_mtr_bwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    if (error1_3 < 0 ) {
      upr_mtr_fwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    else if (error1_3 > 0) {
      upr_mtr_bwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    if (error2_3 < 0) {
      lwr_mtr_fwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    else if (error2_3 > 0) {
      lwr_mtr_bwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    if (error1_4 < 0 ) {
      upr_mtr_fwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    else if (error1_4 > 0) {
      upr_mtr_bwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    if (error2_4 < 0) {
      lwr_mtr_fwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

    else if (error2_4 > 0) {
      lwr_mtr_bwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {

    double xe_1 = 1.3333 + u ;
    double ye_1 = -45;

    double xe_2 = -6 + u;
    double ye_2 = -45;

    double xe_4 = 4.66 + u ;
    double ye_4 = -45;

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

      if (counter2_2 > 1200)
      {
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


    if (atan(ye_1 / xe_1) > 0)
      alpha_1 = atan(ye_1 / xe_1) - 3.14159;

    else
      alpha_1 = atan(ye_1 / xe_1);

    if (atan(ye_2 / xe_2) > 0)
      alpha_2 = atan(ye_2 / xe_2) - 3.14159;

    else
      alpha_2 = atan(ye_2 / xe_2);

    if (atan(ye_4 / xe_4) > 0)
      alpha_4 = atan(ye_4 / xe_4) - PI;

    else
      alpha_4 = atan(ye_4 / xe_4);

    //----------------------------------------------------------------------111111111111111111111----------------------------------------------------------------

    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

    error1_1 = theta1_1 - theta1c_1 + 53.728;
    error2_1 = theta2_1 - theta2c_1 / 4 + 42.68;

    dif_error1_1 = error1_1 - prev_error1_1;
    dif_error2_1 = error2_1 - prev_error2_1;

    c1_1 = (Kp1 - 0.5) * error1_1 + (Kd1 + 0.5) * (dif_error1_1);
    c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

    prev_error1_1 = error1_1;
    prev_error2_1 = error2_1;

    correction1_1 = map(abs(c1_1), 0, 30, 0, 130);//50
    correction2_1 = map(abs(c2_1), 0, 35, 0, 175);

    //---------------------------------------------------------------22222222222222222222-------------------------------------------------------------

    theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

    error1_2 = theta1_2 - theta1c_2 + 65.8795;
    error2_2 = theta2_2 - theta2c_2 / 4 + 50.4;

    dif_error1_2 = error1_2 - prev_error1_2;
    dif_error2_2 = error2_2 - prev_error2_2;

    c1_2 = (Kp1 - 0.5) * error1_2 + (Kd1 + 0.5) * (dif_error1_2);
    c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

    prev_error1_2 = error1_2;
    prev_error2_2 = error2_2;

    correction1_2 = map(abs(c1_2), 0, 35, 0, 40);   //50
    correction2_2 = map(abs(c2_2), 0, 40, 0, 150);    //175

    //------------------------------------------------------------333333333333333333333----------------------------------------------------------------

    if (t < 0.5) {
      theta1_3 = (-1) * theta(theta1AT1_3, theta1_AT1_3, theta1__AT1_3, theta1AT2_3, theta1_AT2_3, theta1__AT2_3, t);
      theta2_3 = (-1) * theta(theta2AT1_3, theta2_AT1_3, theta2__AT1_3, theta2AT2_3, theta2_AT2_3, theta2__AT2_3, t);
    }
    else {
      theta1_3 = (-1) * theta(theta1AT2_3, theta1_AT2_3, theta1__AT2_3, theta1AT3_3, theta1_AT3_3, theta1__AT3_3, t - 0.5);
      theta2_3 = (-1) * theta(theta2AT2_3, theta2_AT2_3, theta2__AT2_3, theta2AT3_3, theta2_AT3_3, theta2__AT3_3, t - 0.5);
    }

    error1_3 = theta1_3 - theta1c_3 + 62.513;
    error2_3 = theta2_3 - theta2c_3 / 4 + 51.58;

    dif_error1_3 = error1_3 - prev_error1_3;
    prev_error1_3 = error1_3;
    dif_error2_3 = error2_3 - prev_error2_3;
    prev_error2_3 = error2_3;

    if (t < 0.5)
      c1_3 = (Kp1 - 0.5) * error1_3 + Kd1 * (dif_error1_3);
    else
      c1_3 = (Kp1 - 0.9) * error1_3 + Kd1 * (dif_error1_3);
    c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

    if (t < 0.5) {
      correction1_3 = map(abs(c1_3), 0, 70, 0, 80);
      correction2_3 = map(abs(c2_3), 0, 90, 0, 200);//
    }
    else {
      correction1_3 = map(abs(c1_3), 0, 70, 0, 80);
      correction2_3 = map(abs(c2_3), 0, 90, 0, 190);//210
    }

    //-----------------------------------------------------------444444444444444444------------------------------------------------------------------

    theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

    error1_4 = theta1_4 - theta1c_4 + 72.815;
    error2_4 = theta2_4 - theta2c_4 / 4 + 49.55;

    dif_error1_4 = error1_4 - prev_error1_4;
    prev_error1_4 = error1_4;
    dif_error2_4 = error2_4 - prev_error2_4;
    prev_error2_4 = error2_4;

    c1_4 = (Kp1 - 0.5) * error1_4 + (Kd1 + 0.5) * (dif_error1_4);
    c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

    correction1_4 = map(abs(c1_4), 0, 30, 0, 50);     //50
    correction2_4 = map(abs(c2_4), 0, 35, 0, 150);    //175

    Serial.print("x=");
    Serial.println(xe_1);
    Serial.print("y=");
    Serial.println(ye_1);
    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - 53.728);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - 42.68);
    Serial.print("theta2_1=");
    Serial.println(theta2_1);
    Serial.print("c1_1=");
    Serial.println(c1_1);
    Serial.print("c2_1=");
    Serial.println(c2_1);
    Serial.print("pwm1=");
    Serial.println(correction1_1);
    Serial.print("pwm2=");
    Serial.println(correction2_1);
    Serial.println("------------------------");

    if (error1_1 < 0 ) {
      upr_mtr_fwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    else if (error1_1 > 0) {
      upr_mtr_bwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    if (error2_1 < 0) {
      lwr_mtr_fwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    else if (error2_1 > 0) {
      lwr_mtr_bwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    if (error1_2 < 0 ) {
      upr_mtr_fwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    else if (error1_2 > 0) {
      upr_mtr_bwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    if (error2_2 < 0) {
      lwr_mtr_fwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    else if (error2_2 > 0) {
      lwr_mtr_bwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    if (error1_3 < 0 ) {
      upr_mtr_fwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    else if (error1_3 > 0) {
      upr_mtr_bwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    if (error2_3 < 0) {
      lwr_mtr_fwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    else if (error2_3 > 0) {
      lwr_mtr_bwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    if (error1_4 < 0 ) {
      upr_mtr_fwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    else if (error1_4 > 0) {
      upr_mtr_bwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    if (error2_4 < 0) {
      lwr_mtr_fwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

    else if (error2_4 > 0) {
      lwr_mtr_bwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {

    double xe_1 = 1.3333 + u + 5.3333;
    double ye_1 = -45 ;

    double xe_2 = -6 + u + 5.3333;
    double ye_2 = -45;

    double xe_3 = -4 + u ;
    double ye_3 = -45 ;

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

      if (counter2_2 > 1200)
      {
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


    if (atan(ye_1 / xe_1) > 0)
      alpha_1 = atan(ye_1 / xe_1) - 3.14159;

    else
      alpha_1 = atan(ye_1 / xe_1);

    if (atan(ye_2 / xe_2) > 0)
      alpha_2 = atan(ye_2 / xe_2) - 3.14159;

    else
      alpha_2 = atan(ye_2 / xe_2);

    if (atan(ye_3 / xe_3) > 0)
      alpha_3 = atan(ye_3 / xe_3) - PI;

    else
      alpha_3 = atan(ye_3 / xe_3);

    //---------------------------------------------------------11111111111111111111----------------------------------------------------------------

    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

    error1_1 = theta1_1 - theta1c_1 + 53.728;
    error2_1 = theta2_1 - theta2c_1 / 4 + 42.68;

    dif_error1_1 = error1_1 - prev_error1_1;
    dif_error2_1 = error2_1 - prev_error2_1;

    c1_1 = (Kp1 - 0.5) * error1_1 + (Kd1 + 0.5) * (dif_error1_1);
    c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

    prev_error1_1 = error1_1;
    prev_error2_1 = error2_1;

    correction1_1 = map(abs(c1_1), 0, 35, 0, 130);     //50
    correction2_1 = map(abs(c2_1), 0, 40, 0, 175);    //175

    //---------------------------------------------------------------22222222222222222222-------------------------------------------------------------

    theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

    error1_2 = theta1_2 - theta1c_2 + 65.8795;
    error2_2 = theta2_2 - theta2c_2 / 4 + 50.4;

    dif_error1_2 = error1_2 - prev_error1_2;
    dif_error2_2 = error2_2 - prev_error2_2;

    c1_2 = (Kp1 - 0.5) * error1_2 + (Kd1 + 0.5) * (dif_error1_2);
    c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

    prev_error1_2 = error1_2;
    prev_error2_2 = error2_2;

    correction1_2 = map(abs(c1_2), 0, 30, 0, 40);   //50
    correction2_2 = map(abs(c2_2), 0, 35, 0, 150);    //175

    //---------------------------------------------------------33333333333333333------------------------------------------------------------------------

    theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
    theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

    error1_3 = theta1_3 - theta1c_3 + 62.513;
    error2_3 = theta2_3 - theta2c_3 / 4 + 51.58;

    dif_error1_3 = error1_3 - prev_error1_3;
    prev_error1_3 = error1_3;
    dif_error2_3 = error2_3 - prev_error2_3;
    prev_error2_3 = error2_3;


    c1_3 = (Kp1 - 0.5) * error1_3 + (Kd1 + 0.5) * (dif_error1_3);
    c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

    correction1_3 = map(abs(c1_3), 0, 30, 0, 40);
    correction2_3 = map(abs(c2_3), 0, 35, 0, 130);     //175

    //-----------------------------------------------------------44444444444444444444444---------------------------------------------------------------

    if (t < 0.5) {
      theta1_4 = (-1) * theta(theta1AT1_4, theta1_AT1_4, theta1__AT1_4, theta1AT2_4, theta1_AT2_4, theta1__AT2_4, t);
      theta2_4 = (-1) * theta(theta2AT1_4, theta2_AT1_4, theta2__AT1_4, theta2AT2_4, theta2_AT2_4, theta2__AT2_4, t);
    }
    else {
      theta1_4 = (-1) * theta(theta1AT2_4, theta1_AT2_4, theta1__AT2_4, theta1AT3_4, theta1_AT3_4, theta1__AT3_4, t - 0.5);
      theta2_4 = (-1) * theta(theta2AT2_4, theta2_AT2_4, theta2__AT2_4, theta2AT3_4, theta2_AT3_4, theta2__AT3_4, t - 0.5);
    }

    error1_4 = theta1_4 - theta1c_4 + 72.815;
    error2_4 = theta2_4 - theta2c_4 / 4 + 49.55;

    dif_error1_4 = error1_4 - prev_error1_4;
    prev_error1_4 = error1_4;
    dif_error2_4 = error2_4 - prev_error2_4;
    prev_error2_4 = error2_4;


    if (t < 0.5)
      c1_4 = (Kp1 - 0.5) * error1_4 + Kd1 * (dif_error1_4);
    else
      c1_4 = (Kp1 - 0.9) * error1_4 + Kd1 * (dif_error1_4);
    c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);


    if (t < 0.5)
    {
      correction1_4 = map(abs(c1_4), 0, 70, 0, 80);
      correction2_4 = map(abs(c2_4), 0, 90, 0, 200);//150
    }
    else
    {
      correction1_4 = map(abs(c1_4), 0, 70, 0, 80);
      correction2_4 = map(abs(c2_4), 0, 90, 0, 210);//150
    }

    Serial.print("x=");
    Serial.println(xe_1);
    Serial.print("y=");
    Serial.println(ye_1);
    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - 53.728);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - 42.68);
    Serial.print("theta2_1=");
    Serial.println(theta2_1);
    Serial.print("c1_1=");
    Serial.println(c1_1);
    Serial.print("c2_1=");
    Serial.println(c2_1);
    Serial.print("pwm1=");
    Serial.println(correction1_1);
    Serial.print("pwm2=");
    Serial.println(correction2_1);
    Serial.println("------------------------");

    if (error1_1 < 0 ) {
      upr_mtr_fwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    else if (error1_1 > 0) {
      upr_mtr_bwd_1();
      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    if (error2_1 < 0) {
      lwr_mtr_fwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    else if (error2_1 > 0) {
      lwr_mtr_bwd_1();
      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    if (error1_2 < 0 ) {
      upr_mtr_fwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    else if (error1_2 > 0) {
      upr_mtr_bwd_2();
      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    if (error2_2 < 0) {
      lwr_mtr_fwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    else if (error2_2 > 0) {
      lwr_mtr_bwd_2();
      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    if (error1_3 < 0 ) {
      upr_mtr_fwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    else if (error1_3 > 0) {
      upr_mtr_bwd_3();
      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    if (error2_3 < 0) {
      lwr_mtr_fwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    else if (error2_3 > 0) {
      lwr_mtr_bwd_3();
      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    if (error1_4 < 0 ) {
      upr_mtr_fwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    else if (error1_4 > 0) {
      upr_mtr_bwd_4();
      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    if (error2_4 < 0) {
      lwr_mtr_fwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

    else if (error2_4 > 0) {
      lwr_mtr_bwd_4();
      analogWrite(motor2pwm_4, abs(correction2_4));
    }

  }

}

double theta(double thet1, double thet1_, double thet1__, double thet2, double thet2_, double thet2__, double t) {
  double a, b, c, d, e, f;

  a = thet1;
  b = thet1_;
  c = thet1__ / 2;
  d = (20 * (thet2 - thet1) - (8 * thet2_ + 12 * thet1_) * 0.5 + (3 * thet1__ - thet2__) * 0.5 * 0.5) / (2 * 0.5 * 0.5 * 0.5);
  e = (30 * (thet1 - thet2) + (14 * thet2_ + 16 * thet1_) * 0.5 + (3 * thet1__ - 2 * thet2__) * 0.5 * 0.5) / (2 * 0.5 * 0.5 * 0.5 * 0.5);
  f = (12 * (thet2 - thet1) - (6 * thet2_ + 6 * thet1_) * 0.5 - (thet1__ - thet2__) * 0.5 * 0.5) / (2 * 0.5 * 0.5 * 0.5 * 0.5 * 0.5);

  return (a + b * t + c * t * t + d * t * t * t + e * t * t * t * t + f * t * t * t * t * t);
}

double cosine_rule(double c, double b, double a) {
  double x = ( a * a + b * b - c * c ) / ( 2 * a * b );
  return acos(x);
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
  if (digitalRead(b1_1) == state1_1) {
    counter1_1++;
  }
  else {
    counter1_1--;
  }
  state1_1 = !state1_1;
}

void ai2_1() {
  if (digitalRead(b2_1) == state2_1) {
    counter2_1++;
  }
  else {
    counter2_1--;
  }
  state2_1 = !state2_1;
}

void ai1_2() {
  if (digitalRead(b1_2) == state1_2) {
    counter1_2++;
  }
  else {
    counter1_2--;
  }
  state1_2 = !state1_2;
}

void ai2_2() {
  if (digitalRead(b2_2) == state2_2) {
    counter2_2++;
  }
  else {
    counter2_2--;
  }
  state2_2 = !state2_2;
}

void ai1_3() {
  if (digitalRead(b1_3) == state1_3) {
    counter1_3++;
  }
  else {
    counter1_3--;
  }
  state1_3 = !state1_3;
}

void ai2_3() {
  if (digitalRead(b2_3) == state2_3) {
    counter2_3++;
  }
  else {
    counter2_3--;
  }
  state2_3 = !state2_3;
}

void ai1_4() {
  if (digitalRead(b1_4) == state1_4) {
    counter1_4++;
  }
  else {
    counter1_4--;
  }
  state1_4 = !state1_4;
}

void ai2_4() {
  if (digitalRead(b2_4) == state2_4) {
    counter2_4++;
  }
  else {
    counter2_4--;
  }
  state2_4 = !state2_4;
}

