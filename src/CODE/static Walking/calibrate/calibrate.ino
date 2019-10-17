#include <PinChangeInt.h>

double pwm1_1;
double pwm2_1;
double pwm1_2;
double pwm2_2;
double pwm1_3;
double pwm2_3;
double pwm1_4;
double pwm2_4;

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

double n = 0.5;

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

double ll1=0.0, lm1=0.0, ul1=0.0, um1=0.0;
double ll2=0.0, lm2=0.0, ul2=0.0, um2=0.0;

double a = 26.0, b = 23.4, c = 0;
double d, y;

double alpha_1;
double theta1c_1 = 0.0 , theta2c_1 = 0.0, theta1_1, theta2_1,error1_1,error2_1, correction1_1, correction2_1, c1_1, c2_1, prev_error1_1 = 0.0 , prev_error2_1 = 0.0, zeroError1_1 = 56.6, zeroError2_1 = 39.0;

double alpha_2;
double theta1c_2 = 0.0 , theta2c_2 = 0.0, theta1_2, theta2_2,error1_2,error2_2, correction1_2, correction2_2, c1_2, c2_2, prev_error1_2 = 0.0 , prev_error2_2 = 0.0, zeroError1_2 = 70.8795, zeroError2_2=50.4;

double alpha_3;
double theta1c_3 = 0.0 , theta2c_3 = 0.0, theta1_3, theta2_3,error1_3,error2_3, correction1_3, correction2_3, c1_3, c2_3, prev_error1_3 = 0.0 , prev_error2_3 = 0.0, zeroError1_3 = 69.513, zeroError2_3=51.58;

double alpha_4;
double theta1c_4 = 0.0 , theta2c_4 = 0.0, theta1_4, theta2_4,error1_4,error2_4, correction1_4, correction2_4, c1_4, c2_4, prev_error1_4 = 0.0, prev_error2_4 = 0.0, zeroError1_4 = 53.815, zeroError2_4 = 39.55;

volatile int temp1_1 , counter1_1 = 0;
volatile int temp2_1 , counter2_1 = 0;

volatile int temp1_2 , counter1_2 = 0;
volatile int temp2_2 , counter2_2 = 0;

volatile int temp1_3 , counter1_3 = 0;
volatile int temp2_3 , counter2_3 = 0;

volatile int temp1_4 , counter1_4 = 0;
volatile int temp2_4 , counter2_4 = 0;

double Kp1 = 1.5, Kp2 = 1.5, Kd1 = 2.0, Kd2 = 2.0 ;
double l1 = 26.0, l2 = 23.4;

double theta1AT1_1 = 55.0, theta1AT2_1 = 45.94, theta1AT3_1 = 70.35, theta1_AT1_1 = 0, theta1_AT2_1 = 0.1, theta1_AT3_1 = 0, theta1__AT1_1 = 0, theta1__AT2_1 = 0.1, theta1__AT3_1 = 0;
double theta2AT1_1 = 39.3, theta2AT2_1 = 75.07, theta2AT3_1 = 47.79, theta2_AT1_1 = 0, theta2_AT2_1 = 0.1, theta2_AT3_1 = 0, theta2__AT1_1 = 0, theta2__AT2_1 = 0.1, theta2__AT3_1 = 0;

double theta1AT1_2 = 54.3, theta1AT2_2 = 45.34, theta1AT3_2 = 70.44, theta1_AT1_2 = 0, theta1_AT2_2 = 0.1, theta1_AT3_2 = 0, theta1__AT1_2 = 0, theta1__AT2_2 = 0.1, theta1__AT3_2 = 0;
double theta2AT1_2 = 43.5, theta2AT2_2 = 75.29, theta2AT3_2 = 50.55, theta2_AT1_2 = 0, theta2_AT2_2 = 0.1, theta2_AT3_2 = 0, theta2__AT1_2 = 0, theta2__AT2_2 = 0.1, theta2__AT3_2 = 0;

double theta1AT1_3 = 53.0,theta1AT2_3=45.94,theta1AT3_3=69.35, theta1_AT1_3=0,theta1_AT2_3=0.1,theta1_AT3_3=0,theta1__AT1_3=0,theta1__AT2_3=0.1,theta1__AT3_3=0;
double theta2AT1_3 = 42.3,theta2AT2_3=75.07,theta2AT3_3=51.79, theta2_AT1_3=0,theta2_AT2_3=0.1,theta2_AT3_3=0,theta2__AT1_3=0,theta2__AT2_3=0.1,theta2__AT3_3=0;

double theta1AT1_4 = 53.5356, theta1AT2_4 = 45.3491, theta1AT3_4 = 64.4462, theta1_AT1_4 = 0, theta1_AT2_4 = 0.1, theta1_AT3_4 = 0, theta1__AT1_4 = 0, theta1__AT2_4 = 0.1, theta1__AT3_4 = 0;
double theta2AT1_4 = 39.198, theta2AT2_4 = 75.6567, theta2AT3_4 = 51.5097, theta2_AT1_4 = 0, theta2_AT2_4 = 0.1, theta2_AT3_4 = 0, theta2__AT1_4 = 0, theta2__AT2_4 = 0.1, theta2__AT3_4 = 0;

void setup()
{
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
  
//1

  d = 0;
  y = -45;
  c = sqrt(d*d + y*y);
  double theta = atan(abs(y/d))*90/acos(0);
  zeroError1_1 = theta - cosine1();
  if(d>0){
    zeroError1_1 = 180 - theta - cosine1();
  }
  zeroError2_1 = 180 - cosine2();
  
//2
  
  d = -2;
  y = -45;
  c = sqrt(d*d + y*y);
  theta = atan(abs(y/d))*90/acos(0);
  zeroError1_2 = theta - cosine1();
  if(d>0){
    zeroError1_2 = 180 - theta - cosine1();
  }
  zeroError2_2 = 180 - cosine2();

//3
  
  d = 0;
  y = -45;
  c = sqrt(d*d + y*y);
  theta = atan(abs(y/d))*90/acos(0);
  zeroError1_3 = theta - cosine1();
  if(d>0){
    zeroError1_3 = 180 - theta - cosine1();
  }
  zeroError2_3 = 180 - cosine2();
  
//4

  d = -2;
  y = -45;
  c = sqrt(d*d + y*y);
  theta = atan(abs(y/d))*90/acos(0);
  zeroError1_4 = theta - cosine1();
  if(d>0){
    zeroError1_4 = 180 - theta - cosine1();
  }
  zeroError2_4 = 180 - cosine2();

  
}

void loop(){
  for (double t = 0.1, u = 1.2; t <= 1.1, u <= 12.1; t = t + 0.1, u = u + 1.2)
  {
    double xe_1 = 0 - u ;
    double ye_1 = -45 ;

    double xe_2 = 0 - u;
    double ye_2 = -45;
    
    double xe_3 = 0 - u ;
    double ye_3 = -45 ;
    
    double xe_4 = 0 - u;
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

    //----------------------------------------------1111111111111111111------------------------------------------------------------------
    
    
    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

    error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
    error2_1 = theta2_1 - theta2c_1 + zeroError2_1;
    
      c1_1 = PID(theta1_1, theta1c_1, zeroError1_1 , (Kp1+1.5) , (Kd1+2.2) , prev_error1_1);
    c2_1 = PID(theta2_1, theta2c_1, zeroError2_1 , Kp2 , Kd2 , prev_error2_1);

    ll1=0.0, lm1=40.0, ul1=30, um1=45.0;    
    ll2=0.0, lm2=90.0, ul2=100.0, um2=180.0;

    correction1_1 = ((um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1)*n;
    correction2_1 = ((um2-ul2)/(lm2-ll2)*abs(c2_1)+ul2)*n;
    
//    if(correction1_1 > um1)
//    correction1_1=um1;

    if(correction2_1 > 255)
    correction2_1=255;


    //------------------------------------------------22222222222222-------------------------------------------------------------------------
    theta1_2 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_2 * xe_2 + ye_2 * ye_2)) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

    error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
    error2_2 = theta2_2 - theta2c_2 + zeroError2_2;
    
    c1_2 = PID(theta1_2, theta1c_2, zeroError1_2 , (Kp1+1.2) , (Kd1+2.2) , prev_error1_2);
    c2_2 = PID(theta2_2, theta2c_2, zeroError2_2 , Kp2 , Kd2+2.4 , prev_error2_2);

    ll1=0.0, lm1=40.0, ul1=30, um1=45.0;    
    ll2=0.0, lm2=90.0, ul2=100.0, um2=180.0;

    correction1_2 = ((um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1)*n;
    correction2_2 = ((um2-ul2)/(lm2-ll2)*abs(c2_2)+ul2)*n;

//    if(correction1_2 > um1)
//    correction1_2=um1;

    if(correction2_2 > 255)
    correction2_2=255;

//    if(t>=0.9) correction1_2 = 55;

    //----------------------------------------------3333333333333333333333--------------------------------------------------------------------
    theta1_3 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_3 * xe_3 + ye_3 * ye_3)) + alpha_3);
    theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

    error1_3 = theta1_3 - theta1c_3 + zeroError1_3;
    error2_3 = theta2_3 - theta2c_3 + zeroError2_3;
    
    c1_3 = PID(theta1_3, theta1c_3, zeroError1_3 , (Kp1+1.3) , (Kd1+2.4) , prev_error1_3);
    c2_3 = PID(theta2_3, theta2c_3, zeroError2_3 , Kp2 , Kd2+1.5 , prev_error2_3);

    ll1=0.0, lm1=40.0, ul1=30, um1=45.0;    
    ll2=0.0, lm2=90.0, ul2=100.0, um2=180.0;

    correction1_3 = ((um1-ul1)/(lm1-ll1)*abs(c1_3)+ul1)*n;
    correction2_3 = ((um2-ul2)/(lm2-ll2)*abs(c2_3)+ul2)*n;
    
//    if(correction1_3 > um1)
//    correction1_3=um1;
   
    if(correction2_3 >255)
    correction2_3=255;

//    if(t>0.75) correction1_3 = 32;

     //----------------------------------------------------44444444444444444444-----------------------------------------------------------------------
    theta1_4 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_4 * xe_4 + ye_4 * ye_4)) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

    error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
    error2_4 = theta2_4 - theta2c_4 + zeroError2_4;
    
    c1_4 = PID(theta1_4, theta1c_4, zeroError1_4, (Kp1+1.8), (Kd1+1.8) , prev_error1_4);
    c2_4 = PID(theta2_4, theta2c_4, zeroError2_4, Kp2 , Kd2, prev_error2_4);

    ll1=0.0, lm1=30.0, ul1=30, um1=45;
    ll2=0.0, lm2=40.0, ul2=90.0, um2=180.0;

    correction1_4 = ((um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1)*n;
    correction2_4 = ((um2-ul2)/(lm2-ll2)*abs(c2_4)+ul2)*n;

//    correction1_4 = map(abs(c1_4), 0, 30, 0, 50);       //TODO
//    correction2_4 = map(abs(c2_4), 0, 40, 125, 260);      //TODO

//    if(t>=0.75) correction1_4 = 45;
    
//    if(correction1_4 > um1)
//    correction1_4=um1;

    if(correction2_4 > 255)
      correction2_4 = 255;
//-------------------------------------------------------------------------------------------------------

    Serial.println(t);
    Serial.print("theta1_2=");
    Serial.println(theta1_2);
    Serial.print("theta1c_2=");
    Serial.println(theta1c_2 - zeroError1_2);
    Serial.print("theta2c_2=");
    Serial.println(theta2c_2 - zeroError2_2);
    Serial.print("theta2_2=");
    Serial.println(theta2_2);
    Serial.print("c1_2=");
    Serial.println(c1_2);
    Serial.print("c2_2=");
    Serial.println(c2_2);
    Serial.print("pwm1=");
    Serial.println(correction1_2);
    Serial.print("pwm2=");
    Serial.println(correction2_2);
    Serial.println("------------------------");
    
//-------------------------------------------------------------------------------------------------------

    pwm1_1 = abs(correction1_1);
    pwm2_1 = abs(correction2_1);
    pwm1_2 = abs(correction1_2);
    pwm2_2 = abs(correction2_2);
    pwm1_3 = abs(correction1_3);
    pwm2_3 = abs(correction2_3);
    pwm1_4 = abs(correction1_4);
    pwm2_4 = abs(correction2_4);
    
    while(abs(error1_1)>0.5 || abs(error2_1)>0.5 || abs(error1_2)>0.5 || abs(error2_2)>0.5 || abs(error1_3)>0.5 || abs(error2_3)>0.5 || abs(error1_4)>0.5 || abs(error2_4)>0.5){
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

      error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
      error2_1 = theta2_1 - theta2c_1 + zeroError2_1;

      error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
      error2_2 = theta2_2 - theta2c_2 + zeroError2_2;

      error1_3 = theta1_3 - theta1c_3 + zeroError1_3;
      error2_3 = theta2_3 - theta2c_3 + zeroError2_3;

      error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
      error2_4 = theta2_4 - theta2c_4 + zeroError2_4;

      if (error1_1 < 0 ) {
      upr_mtr_fwd_1();
//      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    else if (error1_1 > 0) {
      upr_mtr_bwd_1();
//      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    if (error2_1 < 0) {
      lwr_mtr_fwd_1();
//      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    else if (error2_1 > 0) {
      lwr_mtr_bwd_1();
//      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    if (error1_2 < 0 ) {
      upr_mtr_fwd_2();
//      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    else if (error1_2 > 0) {
      upr_mtr_bwd_2();
//      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    if (error2_2 < 0) {
      lwr_mtr_fwd_2();
//      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    else if (error2_2 > 0) {
      lwr_mtr_bwd_2();
//      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    if (error1_3 < 0 ) {
      upr_mtr_fwd_3();
//      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    else if (error1_3 > 0) {
      upr_mtr_bwd_3();
//      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    if (error2_3 < 0) {
      lwr_mtr_fwd_3();
//      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    else if (error2_3 > 0) {
      lwr_mtr_bwd_3();
//      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    if (error1_4 < 0 ) {
      upr_mtr_fwd_4();
//      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    else if (error1_4 > 0) {
      upr_mtr_bwd_4();
//      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    if (error2_4 < 0) {
      lwr_mtr_fwd_4();
//      analogWrite(motor2pwm_4, abs(correction2_4));
    }

    else if (error2_4 > 0) {
      lwr_mtr_bwd_4();
//      analogWrite(motor2pwm_4, abs(correction2_4));
    }


      pwm1_1 += 3;
      pwm2_1 += 3;
      pwm1_2 += 3;
      pwm2_2 += 3;
      pwm1_3 += 3;
      pwm2_3 += 3;
      pwm1_4 += 3;
      pwm2_4 += 3;

      if(abs(error1_1)<0.5) pwm1_1 = 0;
      if(abs(error2_1)<0.5) pwm2_1 = 0;

      if(abs(error1_2)<0.5) pwm1_2 = 0;
      if(abs(error2_2)<0.5) pwm2_2 = 0;

      if(abs(error1_3)<0.5) pwm1_3 = 0;
      if(abs(error2_3)<0.5) pwm2_3 = 0;

      if(abs(error1_4)<0.5) pwm1_4 = 0;
      if(abs(error2_4)<0.5) pwm2_4 = 0;

      

      analogWrite(motor1pwm_1, pwm1_1);
      analogWrite(motor2pwm_1, pwm2_1);
      analogWrite(motor1pwm_2, pwm1_2);
      analogWrite(motor2pwm_2, pwm2_2);
      analogWrite(motor1pwm_3, pwm1_3);
      analogWrite(motor2pwm_3, pwm2_3);
      analogWrite(motor1pwm_4, pwm1_4);
      analogWrite(motor2pwm_4, pwm2_4);

    }
    
  }

delay(1000);

  for (double t = 0.1, u = 1.2; t <= 1.1, u <= 12.1; t = t + 0.1, u = u + 1.2)
  { 
    double xe_1 = -12 + u;
    double ye_1 = -45;
    
    double xe_2 = -12 + u ;
    double ye_2 = -45 ;

    double xe_3 = -12 + u;
    double ye_3 = -45 ;

    double xe_4 = -12 + u;
    double ye_4 = -45 ;

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

    if (atan(ye_4 / xe_4) > 0)
      alpha_4 = atan(ye_4 / xe_4) - PI;

    else
      alpha_4 = atan(ye_4 / xe_4);

    //------------------------------------------------------11111111111111111111----------------------------------------------------------------
    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

    error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
    error2_1 = theta2_1 - theta2c_1 + zeroError2_1;
    
      c1_1 = PID(theta1_1, theta1c_1, zeroError1_1 , (Kp1+1.5) , (Kd1+2.2) , prev_error1_1);
    c2_1 = PID(theta2_1, theta2c_1, zeroError2_1 , Kp2 , Kd2 , prev_error2_1);

    ll1=0.0, lm1=40.0, ul1=20, um1=30.0;    
    ll2=0.0, lm2=90.0, ul2=100.0, um2=180.0;

    correction1_1 = ((um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1)*n;
    correction2_1 = ((um2-ul2)/(lm2-ll2)*abs(c2_1)+ul2)*n;
    
//    if(correction1_1 > um1)
//    correction1_1=um1;

    if(correction2_1 > 255)
    correction2_1=255;

//    if(t>=0.7) correction1_1 = 34;

    //-------------------------------------------------------------22222222222222222222---------------------------------------------------
    theta1_2 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_2 * xe_2 + ye_2 * ye_2)) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

    error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
    error2_2 = theta2_2 - theta2c_2 + zeroError2_2;
    
    c1_2 = PID(theta1_2, theta1c_2, zeroError1_2 , (Kp1+1.2) , (Kd1+2.2) , prev_error1_2);
    c2_2 = PID(theta2_2, theta2c_2, zeroError2_2 , Kp2 , Kd2+2.4 , prev_error2_2);

    ll1=0.0, lm1=40.0, ul1=20, um1=30.0;    
    ll2=0.0, lm2=90.0, ul2=100.0, um2=180.0;

    correction1_2 = ((um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1)*n;
    correction2_2 = ((um2-ul2)/(lm2-ll2)*abs(c2_2)+ul2)*n;

//    if(correction1_2 > um1)
//    correction1_2=um1;

    if(correction2_2 > 255)
    correction2_2=255;

//    if(t>=0.9) correction1_2 = 55;

    //----------------------------------------------3333333333333333333333--------------------------------------------------------------------
    
    theta1_3 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_3 * xe_3 + ye_3 * ye_3)) + alpha_3);
    theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

    error1_3 = theta1_3 - theta1c_3 + zeroError1_3;
    error2_3 = theta2_3 - theta2c_3 + zeroError2_3;
    
    c1_3 = PID(theta1_3, theta1c_3, zeroError1_3 , (Kp1+1.3) , (Kd1+2.4) , prev_error1_3);
    c2_3 = PID(theta2_3, theta2c_3, zeroError2_3 , Kp2 , Kd2+1.5 , prev_error2_3);

    ll1=0.0, lm1=40.0, ul1=20, um1=30.0;    
    ll2=0.0, lm2=90.0, ul2=100.0, um2=180.0;

    correction1_3 = ((um1-ul1)/(lm1-ll1)*abs(c1_3)+ul1)*n;
    correction2_3 = ((um2-ul2)/(lm2-ll2)*abs(c2_3)+ul2)*n;
    
//    if(correction1_3 > um1)
//    correction1_3=um1;
   
    if(correction2_3 >255)
    correction2_3=255;

//    if(t>0.75) correction1_3 = 32;

    //----------------------------------------------------44444444444444444444-----------------------------------------------------------------------

    theta1_4 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_4 * xe_4 + ye_4 * ye_4)) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

    error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
    error2_4 = theta2_4 - theta2c_4 + zeroError2_4;
    
    c1_4 = PID(theta1_4, theta1c_4, zeroError1_4 , (Kp1+1.8) , (Kd1+1.8) , prev_error1_4);
    c2_4 = PID(theta2_4, theta2c_4, zeroError2_4 , Kp2 , Kd2 , prev_error2_4);

    ll1=0.0, lm1=30.0, ul1=20, um1=30;
    ll2=0.0, lm2=40.0, ul2=100.0, um2=180.0;

    correction1_4 = ((um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1)*n;
    correction2_4 = ((um2-ul2)/(lm2-ll2)*abs(c2_4)+ul2)*n;

//    correction1_4 = map(abs(c1_4), 0, 30, 0, 50);       //TODO
//    correction2_4 = map(abs(c2_4), 0, 40, 125, 260);      //TODO

//    if(t>=0.75) correction1_4 = 45;
    
//    if(correction1_4 > um1)
//    correction1_4=um1;

    if(correction2_4 > 255)
      correction2_4 = 255;
//--------------------------------------------------------------------------------------------------------------------------
    Serial.println(t);
    Serial.print("theta1_2=");
    Serial.println(theta1_2);
    Serial.print("theta1c_2=");
    Serial.println(theta1c_2 - zeroError1_2);
    Serial.print("theta2c_2=");
    Serial.println(theta2c_2 - zeroError2_2);
    Serial.print("theta2_2=");
    Serial.println(theta2_2);
    Serial.print("c1_2=");
    Serial.println(c1_2);
    Serial.print("c2_2=");
    Serial.println(c2_2);
    Serial.print("pwm1=");
    Serial.println(correction1_2);
    Serial.print("pwm2=");
    Serial.println(correction2_2);
    Serial.println("------------------------");
    
//---------------------------------------------------------------------------------------------------
    pwm1_1 = abs(correction1_1);
    pwm2_1 = abs(correction2_1);
    pwm1_2 = abs(correction1_2);
    pwm2_2 = abs(correction2_2);
    pwm1_3 = abs(correction1_3);
    pwm2_3 = abs(correction2_3);
    pwm1_4 = abs(correction1_4);
    pwm2_4 = abs(correction2_4);
    
    while(abs(error1_1)>0.5 || abs(error2_1)>0.5 || abs(error1_2)>0.5 || abs(error2_2)>0.5 || abs(error1_3)>0.5 || abs(error2_3)>0.5 || abs(error1_4)>0.5 || abs(error2_4)>0.5){

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

      error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
      error2_1 = theta2_1 - theta2c_1 + zeroError2_1;

      error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
      error2_2 = theta2_2 - theta2c_2 + zeroError2_2;

      error1_3 = theta1_3 - theta1c_3 + zeroError1_3;
      error2_3 = theta2_3 - theta2c_3 + zeroError2_3;

      error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
      error2_4 = theta2_4 - theta2c_4 + zeroError2_4;

    if (error1_1 < 0 ) {
      upr_mtr_fwd_1();
//      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    else if (error1_1 > 0) {
      upr_mtr_bwd_1();
//      analogWrite(motor1pwm_1, abs(correction1_1));
    }

    if (error2_1 < 0) {
      lwr_mtr_fwd_1();
//      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    else if (error2_1 > 0) {
      lwr_mtr_bwd_1();
//      analogWrite(motor2pwm_1, abs(correction2_1));
    }

    if (error1_2 < 0 ) {
      upr_mtr_fwd_2();
//      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    else if (error1_2 > 0) {
      upr_mtr_bwd_2();
//      analogWrite(motor1pwm_2, abs(correction1_2));
    }

    if (error2_2 < 0) {
      lwr_mtr_fwd_2();
//      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    else if (error2_2 > 0) {
      lwr_mtr_bwd_2();
//      analogWrite(motor2pwm_2, abs(correction2_2));
    }

    if (error1_3 < 0 ) {
      upr_mtr_fwd_3();
//      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    else if (error1_3 > 0) {
      upr_mtr_bwd_3();
//      analogWrite(motor1pwm_3, abs(correction1_3));
    }

    if (error2_3 < 0) {
      lwr_mtr_fwd_3();
//      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    else if (error2_3 > 0) {
      lwr_mtr_bwd_3();
//      analogWrite(motor2pwm_3, abs(correction2_3));
    }

    if (error1_4 < 0 ) {
      upr_mtr_fwd_4();
//      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    else if (error1_4 > 0) {
      upr_mtr_bwd_4();
//      analogWrite(motor1pwm_4, abs(correction1_4));
    }

    if (error2_4 < 0) {
      lwr_mtr_fwd_4();
//      analogWrite(motor2pwm_4, abs(correction2_4));
    }

    else if (error2_4 > 0) {
      lwr_mtr_bwd_4();
//      analogWrite(motor2pwm_4, abs(correction2_4));
    }


      pwm1_1 += 3;
      pwm2_1 += 3;
      pwm1_2 += 3;
      pwm2_2 += 3;
      pwm1_3 += 3;
      pwm2_3 += 3;
      pwm1_4 += 3;
      pwm2_4 += 3;

      if(abs(error1_1)<0.5) pwm1_1 = 0;
      if(abs(error2_1)<0.5) pwm2_1 = 0;

      if(abs(error1_2)<0.5) pwm1_2 = 0;
      if(abs(error2_2)<0.5) pwm2_2 = 0;

      if(abs(error1_3)<0.5) pwm1_3 = 0;
      if(abs(error2_3)<0.5) pwm2_3 = 0;

      if(abs(error1_4)<0.5) pwm1_4 = 0;
      if(abs(error2_4)<0.5) pwm2_4 = 0;  

      analogWrite(motor1pwm_1, pwm1_1);
      analogWrite(motor2pwm_1, pwm2_1);
      analogWrite(motor1pwm_2, pwm1_2);
      analogWrite(motor2pwm_2, pwm2_2);
      analogWrite(motor1pwm_3, pwm1_3);
      analogWrite(motor2pwm_3, pwm2_3);
      analogWrite(motor1pwm_4, pwm1_4);
      analogWrite(motor2pwm_4, pwm2_4);

    }
    
  }

delay(1000);  
}


double PID(double theta, double thetac, double zeroError, double KP, double KD, double &prevError) {
  double error = theta - thetac + zeroError;

  double diffError = error - prevError;

  prevError = error;

  return (KP * error) + (KD * diffError);

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
  if (digitalRead(b1_1) == !digitalRead(a1_1)) {
    counter1_1++;
  }
  else {
    counter1_1--;
  }
//  state1_1 = !state1_1;
}

void ai2_1() {
  if (digitalRead(b2_1) == !digitalRead(a2_1)) {
    counter2_1++;
  }
  else {
    counter2_1--;
  }
//  state2_1 = !state2_1;
}

void ai1_2() {
  if (digitalRead(b1_2) == !digitalRead(a1_2)) {
    counter1_2++;
  }
  else {
    counter1_2--;
  }
//  state1_2 = !state1_2;
}

void ai2_2() {
  if (digitalRead(b2_2) == !digitalRead(a2_2)) {
    counter2_2++;
  }
  else {
    counter2_2--;
  }
//  state2_2 = !state2_2;
}

void ai2_3() {
  if (digitalRead(b1_3) == !digitalRead(a1_3)) {
    counter1_3++;
  }
  else {
    counter1_3--;
  }
//  state1_3 = !state1_3;
}

void ai3_3() {
  if (digitalRead(b2_3) == !digitalRead(a2_3)) {
    counter2_3++;
  }
  else {
    counter2_3--;
  }
//  state2_3 = !state2_3;
}

void ai4_4() {
  if (digitalRead(b1_4) == !digitalRead(a1_4)) {
    counter1_4++;
  }
  else {
    counter1_4--;
  }
//  state1_4 = !state1_4;
}

void ai5_4() {
  if (digitalRead(b2_4) == !digitalRead(a2_4)) {
    counter2_4++;
  }
  else {
    counter2_4--;
  }
//  state2_4 = !state2_4;
}

double cosine1(){
  return (acos((a*a + c*c - b*b)/(2*a*c)))*90/acos(0);
}

double cosine2(){
  return (acos((a*a + b*b - c*c)/(2*a*b)))*90/acos(0);
}
