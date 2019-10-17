#include <PinChangeInt.h>

#define motor1_2  31
#define motor1pwm_2  5
#define motor2_2  29
#define motor2pwm_2  10
int A1_2 = 2;

#define a1_2 21
#define b1_2 A2
#define a2_2 A11
#define b2_2 A0

double ll1=0.0, lm1=0.0, ul1=0.0, um1=0.0;
double ll2=0.0, lm2=0.0, ul2=0.0, um2=0.0;

double Kp1 = 1.5, Kp2 = 1.5;
double Kd1 = 2.0, Kd2 = 2.0;

int l1 = 25, l2 = 25;

//int A1_2 = 1;
//int A2_2 = 0;

bool state1_2 = true;
bool state2_2 = true;

double alpha_2;
double theta1c_2 = 0.0 , theta2c_2 = 0.0, theta1_2, theta2_2,error1_2,error2_2, correction1_2, correction2_2, c1_2, c2_2, prev_error1_2 = 0.0 , prev_error2_2 = 0.0, zeroError1_2 = 58.8795, zeroError2_2=50.4;

volatile int temp1_2 , counter1_2 = 0;
volatile int temp2_2 , counter2_2 = 0;

double theta1AT1 = 54.68, theta1AT2 = 48.41, theta1AT3 = 72.815, theta1_AT1 = 0, theta1_AT2 = 0.1, theta1_AT3 = 0, theta1__AT1 = 0, theta1__AT2 = 0.1, theta1__AT3 = 0;
double theta2AT1 = 45.57, theta2AT2 = 77.29, theta2AT3 = 49.55, theta2_AT1 = 0, theta2_AT2 = 0.1, theta2_AT3 = 0, theta2__AT1 = 0, theta2__AT2 = 0.1, theta2__AT3 = 0;

void setup()
{
  Serial.begin(9600);
  
  pinMode(a1_2, INPUT_PULLUP);                                           
  pinMode(b1_2, INPUT_PULLUP);
  attachInterrupt(A1_2, ai1_2, CHANGE);
  state1_2 = digitalRead(a1_2);
  pinMode(a2_2, INPUT_PULLUP);
  pinMode(b2_2, INPUT_PULLUP);
  PCintPort::attachInterrupt(a2_2, ai2_2, CHANGE);
  state2_2 = digitalRead(a2_2);
  
  pinMode(motor1_2, OUTPUT);
  pinMode(motor1pwm_2, OUTPUT);
  pinMode(motor2_2, OUTPUT);
  pinMode(motor2pwm_2, OUTPUT);
}

void loop()
{  
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-3");
    double xe_2 = 4.66+u;
    double ye_2 = -45 ;

    if ( counter1_2 != temp1_2 ) {
      temp1_2 = counter1_2;

      if (counter1_2 > 1200)
      {
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

    if (atan(ye_2 / xe_2) > 0)
      alpha_2 = atan(ye_2 / xe_2) - 3.14159;

    else
      alpha_2 = atan(ye_2 / xe_2);

    theta1_2 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_2 * xe_2 + ye_2 * ye_2)) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

    error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
    error2_2 = theta2_2 - theta2c_2 + zeroError2_2;
    
    c1_2 = PID(theta1_2, theta1c_2, zeroError1_2 , (Kp1+2.2) , (Kd1+0.6) , prev_error1_2);
    c2_2 = PID(theta2_2, theta2c_2, zeroError2_2 , Kp2 , Kd2 , prev_error2_2);

    ll1=0.0, lm1=30.0, ul1=25, um1=50.0;
    ll2=0.0, lm2=40.0, ul2=0, um2=60.0;

    correction1_2 = (um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1;
    correction2_2 = (um2-ul2)/(lm2-ll2)*abs(c2_2)+ul2;

    if(correction1_2 > um1)
    correction1_2=um1;

    if(correction2_2 > 255)
    correction2_2=255;
    
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

//      do{
//      if ( counter1_2 != temp1_2 ) {
//        temp1_2 = counter1_2;
//
//        if (counter1_2 > 1200)
//        {
//          counter1_2 = 0;
//        }
//        theta1c_2 = (counter1_2 * 0.3);
//      }
//    }while(theta1_2 - theta1c_2 + zeroError1_2>0.1);
  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  { 

    if ( counter1_2 != temp1_2 ) {
      temp1_2 = counter1_2;

      if (counter1_2 > 1200)
      {
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

    if (t < 0.5) {
      theta1_2 = (-1) * theta(theta1AT1, theta1_AT1, theta1__AT1, theta1AT2, theta1_AT2, theta1__AT2, t);
      theta2_2 = (-1) * theta(theta2AT1, theta2_AT1, theta2__AT1, theta2AT2, theta2_AT2, theta2__AT2, t);
    }
    else {
      theta1_2 = (-1) * theta(theta1AT2, theta1_AT2, theta1__AT2, theta1AT3, theta1_AT3, theta1__AT3, t - 0.5);
      theta2_2 = (-1) * theta(theta2AT2, theta2_AT2, theta2__AT2, theta2AT3, theta2_AT3, theta2__AT3, t - 0.5);
    }

    error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
    error2_2 = theta2_2 - theta2c_2 + zeroError2_2; 
    
    if(t<0.5)
    c1_2 = PID(theta1_2, theta1c_2, zeroError1_2 , Kp1+1.5 , Kd1+2.8 , prev_error1_2);
    else
    c1_2 = PID(theta1_2, theta1c_2, zeroError1_2 , Kp1+2.1 , (Kd1+2.2) , prev_error1_2);
    c2_2 = PID(theta2_2, theta2c_2, zeroError2_2 , Kp2+2 , Kd2 , prev_error2_2);
    ll1=0.0, lm1=70.0, ul1=38.0, um1=55.0;
//    correction1_2 = (um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1;
    
    if(t<0.5)
    correction1_2 = (um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1;
    else{
    ll1=0.0, lm1=70.0, ul1=32.0, um1=40.0;
    correction1_2 = (um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1;      //TODO   0-60
    }
    
    if(t<0.5){
    ll2=0.0, lm2=70.0, ul2=100.0, um2=225.0;
    correction2_2 = (um2-ul2)/(lm2-ll2)*abs(c2_2) + ul2;      //TODO
    }
    else{
      ll2=0.0, lm2=70.0, ul2=100.0, um2=225.0;
    correction2_2 = (um2-ul2)/(lm2-ll2)*abs(c2_2) + ul2;
    }
    

    if(correction1_2 > um1)
    correction1_2=um1;

    if(correction2_2 > 255)
    correction2_2=255;
    
    if(t>0.85)
    correction1_2=11;
    
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

//    do {
//        if ( counter1_2 != temp1_2 ) {
//        temp1_2 = counter1_2;
//
//        if (counter1_2 > 1200)
//        {
//          counter1_2 = 0;
//        }
//        theta1c_2 = (counter1_2 * 0.3);
//      }
////      Serial.println("theta1c_2-----------------");
////      Serial.println(theta1c_2);
//
//    }while(t<0.5 && (theta1_2 - theta1c_2 + zeroError1_2)>0.5);
//
//    do {
//        if ( counter1_2 != temp1_2 ) {
//        temp1_2 = counter1_2;
//
//        if (counter1_2 > 1200)
//        {
//          counter1_2 = 0;
//        }
//        theta1c_2 = (counter1_2 * 0.3);
//      }
//
//    }while(t>0.5 && (theta1_2 - theta1c_2 + zeroError1_2)<-0.2);
//    
//    delay(150);
  }
  
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-1");
    double xe_2 = -6 + u ;
    double ye_2 = -45 ;

    if ( counter1_2 != temp1_2 ) {
      temp1_2 = counter1_2;

      if (counter1_2 > 1200)
      {
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

    if (atan(ye_2 / xe_2) > 0)
      alpha_2 = atan(ye_2 / xe_2) - 3.14159;

    else
      alpha_2 = atan(ye_2 / xe_2);

    theta1_2 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_2 * xe_2 + ye_2 * ye_2)) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

    error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
    error2_2 = theta2_2 - theta2c_2 + zeroError2_2;
    
    c1_2 = PID(theta1_2, theta1c_2, zeroError1_2 , (Kp1+2.0) , (Kd1+1.2) , prev_error1_2);
    c2_2 = PID(theta2_2, theta2c_2, zeroError2_2 , Kp2 , Kd2 , prev_error2_2);

    ll1=0.0, lm1=40.0, ul1=12.3, um1=35.0;
    ll2=0.0, lm2=90.0, ul2=0.0, um2=60.0;

    correction1_2 = (um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1;
    correction2_2 = (um2-ul2)/(lm2-ll2)*abs(c2_2)+ul2;
    
    if(correction1_2 > um1)
    correction1_2=um1;

    if(correction2_2 > 255)
    correction2_2=255;

    if(u>3.5){
      correction1_2 = 10;
    }

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

//    do{
//      if ( counter1_2 != temp1_2 ) {
//        temp1_2 = counter1_2;
//
//        if (counter1_2 > 1200)
//        {
//          counter1_2 = 0;
//        }
//        theta1c_2 = (counter1_2 * 0.3);
//      }
//    }while((theta1_2 - theta1c_2 + zeroError1_2)>0.1);
//
  }
  
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-2");
    double xe_2 = -6 + 5.3333 + u ;
    double ye_2 = -45;

    if ( counter1_2 != temp1_2 ) {
      temp1_2 = counter1_2;

      if (counter1_2 > 1200)
      {
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
      theta2c_2 = (counter2_2 * 0.3);
    }

    if (atan(ye_2 / xe_2) > 0)
      alpha_2 = atan(ye_2 / xe_2) - 3.14159;

    else
      alpha_2 = atan(ye_2 / xe_2);

    theta1_2 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_2 * xe_2 + ye_2 * ye_2)) + alpha_2);
    theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

     error1_2 = theta1_2 - theta1c_2 + zeroError1_2;
    error2_2 = theta2_2 - theta2c_2 + zeroError2_2;
    
    c1_2 = PID(theta1_2, theta1c_2, zeroError1_2 , (Kp1+2.1) , (Kd1+0.6) , prev_error1_2);
    c2_2 = PID(theta2_2, theta2c_2, zeroError2_2 , Kp2 , Kd2 , prev_error2_2);

      ll1=0.0, lm1=30.0, ul1=16.5, um1=40.0;
    ll2=0.0, lm2=40.0, ul2=0.0, um2=50.0;

    correction1_2 = (um1-ul1)/(lm1-ll1)*abs(c1_2)+ul1;
    correction2_2 = (um2-ul2)/(lm2-ll2)*abs(c2_2)+ul2;
    
//    correction1_2 = map(abs(c1_2), 0, 30, 0, 50);       //TODO
//    correction2_2 = map(abs(c2_2), 0, 40, 0, 375);      //TODO

    if(correction1_2 > um1)
    correction1_2=um1;

    if(correction2_2 > 255)
    correction2_2=255;
   
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

//     do{
//      if ( counter1_2 != temp1_2 ) {
//        temp1_2 = counter1_2;
//
//        if (counter1_2 > 1200)
//        {
//          counter1_2 = 0;
//        }
//        theta1c_2 = (counter1_2 * 0.3);
//      }
//    }while(theta1_2 - theta1c_2 + zeroError1_2>0.1);
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

