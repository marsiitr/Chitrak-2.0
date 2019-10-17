#include <PinChangeInt.h>

#define motor1_1  43
#define motor1pwm_1  2
#define motor2_1  40
#define motor2pwm_1  11
int A1_1 = 3, A2_1 = 4;

#define a1_1 20
#define b1_1 A15
#define a2_1 19
#define b2_1 51

double ll1=0.0, lm1=0.0, ul1=0.0, um1=0.0;
double ll2=0.0, lm2=0.0, ul2=0.0, um2=0.0;

double Kp1 = 1.5, Kp2 = 1.5;
double Kd1 = 2.0, Kd2 = 2.0;

int l1 = 25, l2 = 25;

//int A1_1 = 1;
//int A2_1 = 0;

//bool state1_1 = true;
//bool state2_1 = true;

double alpha_1;
double theta1c_1 = 0.0 , theta2c_1 = 0.0, theta1_1, theta2_1,error1_1,error2_1, correction1_1, correction2_1, c1_1, c2_1, prev_error1_1 = 0.0 , prev_error2_1 = 0.0, zeroError1_1 = 53.728, zeroError2_1 = 42.68;

volatile int temp1_1 , counter1_1 = 0;
volatile int temp2_1 , counter2_1 = 0;

double theta1AT1 = 53.73, theta1AT2 = 45.78, theta1AT3 = 69.71, theta1_AT1 = 0, theta1_AT2 = 0.1, theta1_AT3 = 0, theta1__AT1 = 0, theta1__AT2 = 0.1, theta1__AT3 = 0;
double theta2AT1 = 42.68, theta2AT2 = 76.73, theta2AT3 = 50.75, theta2_AT1 = 0, theta2_AT2 = 0.1, theta2_AT3 = 0, theta2__AT1 = 0, theta2__AT2 = 0.1, theta2__AT3 = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(a1_1, INPUT_PULLUP);
  pinMode(a2_1, INPUT_PULLUP);
  pinMode(b1_1, INPUT_PULLUP);
  pinMode(b2_1, INPUT_PULLUP);
  pinMode(motor1_1, OUTPUT);
  pinMode(motor2_1, OUTPUT);
  pinMode(motor1pwm_1, OUTPUT);
  pinMode(motor2pwm_1, OUTPUT);

  attachInterrupt(A1_1, ai1_1, CHANGE);
  attachInterrupt(A2_1, ai2_1, CHANGE);

//  state1_1 = digitalRead(a1_1);
//  state2_1 = digitalRead(a2_1);
}

void loop()
{
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  { 

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

    if (t < 0.5) {
      theta1_1 = (-1) * theta(theta1AT1, theta1_AT1, theta1__AT1, theta1AT2, theta1_AT2, theta1__AT2, t);
      theta2_1 = (-1) * theta(theta2AT1, theta2_AT1, theta2__AT1, theta2AT2, theta2_AT2, theta2__AT2, t);
    }
    else {
      theta1_1 = (-1) * theta(theta1AT2, theta1_AT2, theta1__AT2, theta1AT3, theta1_AT3, theta1__AT3, t - 0.5);
      theta2_1 = (-1) * theta(theta2AT2, theta2_AT2, theta2__AT2, theta2AT3, theta2_AT3, theta2__AT3, t - 0.5);
    }

    error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
    error2_1 = theta2_1 - theta2c_1 + zeroError2_1; 
    
    if(t<0.5)
    c1_1 = PID(theta1_1, theta1c_1, zeroError1_1 , Kp1+1.5 , Kd1+2.8 , prev_error1_1);
    else
    c1_1 = PID(theta1_1, theta1c_1, zeroError1_1 , Kp1+2.1 , (Kd1+2.2) , prev_error1_1);
    c2_1 = PID(theta2_1, theta2c_1, zeroError2_1 , Kp2+1.2 , Kd2 , prev_error2_1);
    ll1=0.0, lm1=70.0, ul1=45.0, um1=120.0;

//    correction1_1 = (um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1;
    
    if(t<0.5)
    correction1_1 = (um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1;
    else{
    ll1=0.0, lm1=70.0, ul1=25.0, um1=45.0;
    correction1_1 = (um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1;      //TODO   0-60
    }
    
    if(t<0.5){
    ll2=0.0, lm2=70.0, ul2=100.0, um2=225.0;
    correction2_1 = (um2-ul2)/(lm2-ll2)*abs(c2_1) + ul2;      //TODO
    }
    else{
      ll2=0.0, lm2=70.0, ul2=100.0, um2=225.0;
    correction2_1 = (um2-ul2)/(lm2-ll2)*abs(c2_1) + ul2;
    }
    

    if(correction1_1 > um1)
    correction1_1=um1;

    if(correction2_1 > um2)
    correction2_1=um2;
    
    if(t>0.85)
    correction1_1=11;
    
    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - zeroError1_1);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - zeroError2_1);
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

//    do {
//        if ( counter1_1 != temp1_1 ) {
//        temp1_1 = counter1_1;
//
//        if (counter1_1 > 1200)
//        {
//          counter1_1 = 0;
//        }
//        theta1c_1 = (counter1_1 * 0.3);
//      }
////      Serial.println("theta1c_1-----------------");
////      Serial.println(theta1c_1);
//
//    }while(t<0.5 && (theta1_1 - theta1c_1 + zeroError1_1)>0.5);
//
//    do {
//        if ( counter1_1 != temp1_1 ) {
//        temp1_1 = counter1_1;
//
//        if (counter1_1 > 1200)
//        {
//          counter1_1 = 0;
//        }
//        theta1c_1 = (counter1_1 * 0.3);
//      }
//
//    }while(t>0.5 && (theta1_1 - theta1c_1 + zeroError1_1)<-0.2);
//    
    delay(150);
  }
  
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-1");
    double xe_1 = -4 + u ;
    double ye_1 = -45 ;

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

    if (atan(ye_1 / xe_1) > 0)
      alpha_1 = atan(ye_1 / xe_1) - 3.14159;

    else
      alpha_1 = atan(ye_1 / xe_1);

    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

    error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
    error2_1 = theta2_1 - theta2c_1 + zeroError2_1;
    
    c1_1 = PID(theta1_1, theta1c_1, zeroError1_1 , (Kp1+2.0) , (Kd1+1.2) , prev_error1_1);
    c2_1 = PID(theta2_1, theta2c_1, zeroError2_1 , Kp2 , Kd2 , prev_error2_1);

    ll1=0.0, lm1=40.0, ul1=25, um1=53.0;    
    ll2=0.0, lm2=90.0, ul2=0.0, um2=150.0;

    correction1_1 = (um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1;
    correction2_1 = (um2-ul2)/(lm2-ll2)*abs(c2_1)+ul2;
    
    if(correction1_1 > um1)
    correction1_1=um1;

    if(correction2_1 > 255)
    correction2_1=255;

    if(u>3.5){
      correction1_1 = 10;
    }

    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - zeroError1_1);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - zeroError2_1);
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

//    do{
//      if ( counter1_1 != temp1_1 ) {
//        temp1_1 = counter1_1;
//
//        if (counter1_1 > 1200)
//        {
//          counter1_1 = 0;
//        }
//        theta1c_1 = (counter1_1 * 0.3);
//      }
//    }while((theta1_1 - theta1c_1 + zeroError1_1)>0.1);
//
  }
  
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-2");
    double xe_1 = 1.3333 + u ;
    double ye_1 = -45;

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

    if (atan(ye_1 / xe_1) > 0)
      alpha_1 = atan(ye_1 / xe_1) - 3.14159;

    else
      alpha_1 = atan(ye_1 / xe_1);

    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

     error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
    error2_1 = theta2_1 - theta2c_1 + zeroError2_1;
    
    c1_1 = PID(theta1_1, theta1c_1, zeroError1_1 , (Kp1+2.1) , (Kd1+0.6) , prev_error1_1);
    c2_1 = PID(theta2_1, theta2c_1, zeroError2_1 , Kp2 , Kd2 , prev_error2_1);

      ll1=0.0, lm1=30.0, ul1=35, um1=65.0;
    ll2=0.0, lm2=40.0, ul2=0.0, um2=150.0;

    correction1_1 = (um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1;
    correction2_1 = (um2-ul2)/(lm2-ll2)*abs(c2_1)+ul2;
    
//    correction1_1 = map(abs(c1_1), 0, 30, 0, 50);       //TODO
//    correction2_1 = map(abs(c2_1), 0, 40, 0, 375);      //TODO

    if(correction1_1 > um1)
    correction1_1=um1;

    if(correction2_1 > 255)
    correction2_1=255;
   
    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - zeroError1_1);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - zeroError2_1);
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

//     do{
//      if ( counter1_1 != temp1_1 ) {
//        temp1_1 = counter1_1;
//
//        if (counter1_1 > 1200)
//        {
//          counter1_1 = 0;
//        }
//        theta1c_1 = (counter1_1 * 0.3);
//      }
//    }while(theta1_1 - theta1c_1 + zeroError1_1>0.1);
  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-3");
    double xe_1 = 1.3333 + u + 5.3333;
    double ye_1 = -45 ;

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

    if (atan(ye_1 / xe_1) > 0)
      alpha_1 = atan(ye_1 / xe_1) - 3.14159;

    else
      alpha_1 = atan(ye_1 / xe_1);

    theta1_1 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
    theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

    error1_1 = theta1_1 - theta1c_1 + zeroError1_1;
    error2_1 = theta2_1 - theta2c_1 + zeroError2_1;
    
    c1_1 = PID(theta1_1, theta1c_1, zeroError1_1 , (Kp1+2.2) , (Kd1+0.6) , prev_error1_1);
    c2_1 = PID(theta2_1, theta2c_1, zeroError2_1 , Kp2 , Kd2 , prev_error2_1);

    ll1=0.0, lm1=30.0, ul1=34, um1=77.0;
    ll2=0.0, lm2=40.0, ul2=0.0, um2=150.0;

    correction1_1 = (um1-ul1)/(lm1-ll1)*abs(c1_1)+ul1;
    correction2_1 = (um2-ul2)/(lm2-ll2)*abs(c2_1)+ul2;

//    correction1_1 = map(abs(c1_1), 0, 30, 0, 50);       //TODO
//    correction2_1 = map(abs(c2_1), 0, 40, 125, 260);      //TODO

    if(correction1_1 > um1)
    correction1_1=um1;

    if(correction2_1 > 255)
    correction2_1=255;
    
    Serial.print("theta1_1=");
    Serial.println(theta1_1);
    Serial.print("theta1c_1=");
    Serial.println(theta1c_1 - zeroError1_1);
    Serial.print("theta2c_1=");
    Serial.println(theta2c_1 - zeroError2_1);
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

//      do{
//      if ( counter1_1 != temp1_1 ) {
//        temp1_1 = counter1_1;
//
//        if (counter1_1 > 1200)
//        {
//          counter1_1 = 0;
//        }
//        theta1c_1 = (counter1_1 * 0.3);
//      }
//    }while(theta1_1 - theta1c_1 + zeroError1_1>0.1);
  }
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

