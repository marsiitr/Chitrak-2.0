#include <PinChangeInt.h>

#define motor1_4  36
#define motor1pwm_4  8
#define motor2_4  37
#define motor2pwm_4  9

int A1_4 = 1, A2_4 = 3;

#define a1_4 3
#define b1_4 52
#define a2_4 A14
#define b2_4 47

double ll1=0.0, lm1=0.0, ul1=0.0, um1=0.0;
double ll2=0.0, lm2=0.0, ul2=0.0, um2=0.0;

double Kp1 = 1.5, Kp2 = 1.5;
double Kd1 = 2.0, Kd2 = 2.0;

int l1 = 25, l2 = 25;

//int A1_4 = 1;
//int A2_4 = 0;

bool state1_4 = true;
bool state2_4 = true;

double alpha_4;
double theta1c_4 = 0.0 , theta2c_4 = 0.0, theta1_4, theta2_4,error1_4,error2_4, correction1_4, correction2_4, c1_4, c2_4, prev_error1_4 = 0.0, prev_error2_4 = 0.0, zeroError1_4 = 72.815, zeroError2_4 = 49.55;

volatile int temp1_4 , counter1_4 = 0;
volatile int temp2_4 , counter2_4 = 0;

double theta1AT1_4=54.68,theta1AT2_4=48.41,theta1AT3_4=72.815,theta1_AT1_4=0,theta1_AT2_4=0.1,theta1_AT3_4=0,theta1__AT1_4=0,theta1__AT2_4=0.1,theta1__AT3_4=0;
double theta2AT1_4=45.57,theta2AT2_4=77.29,theta2AT3_4=49.55,theta2_AT1_4=0,theta2_AT2_4=0.1,theta2_AT3_4=0,theta2__AT1_4=0,theta2__AT2_4=0.1,theta2__AT3_4=0;

void setup()
{
  Serial.begin(9600);

  pinMode(a1_4, INPUT_PULLUP);
  pinMode(a2_4, INPUT_PULLUP);
  pinMode(b1_4, INPUT_PULLUP);
  pinMode(b2_4, INPUT_PULLUP);
  pinMode(motor1_4, OUTPUT);
  pinMode(motor2_4, OUTPUT);
  pinMode(motor1pwm_4, OUTPUT);
  pinMode(motor2pwm_4, OUTPUT);

  attachInterrupt(A1_4, ai4_4, CHANGE);
PCintPort::attachInterrupt(a2_4, ai5_4, CHANGE);
  state1_4 = digitalRead(a1_4);
  state2_4 = digitalRead(a2_4);
}

void loop()
{  
  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-1");
    double xe_4 = -6 + u ;
    double ye_4 = -45 ;

    if ( counter1_4 != temp1_4 ) {
      temp1_4 = counter1_4;

      if (counter1_4 > 1200)
      {
        counter1_4 = 0;
      }
      theta1c_4 = (counter1_4 * 0.3);
    }
    if ( counter2_4 != temp2_4 ) {
      temp2_4 = counter2_4;

      if (counter2_4 > 1200)
      {
        counter2_4 = 0;
      }
      theta2c_4 = -(counter2_4 * 0.3);
    }

    if (atan(ye_4 / xe_4) > 0)
      alpha_4 = atan(ye_4 / xe_4) - PI;

    else
      alpha_4 = atan(ye_4 / xe_4);

    theta1_4 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_4 * xe_4 + ye_4 * ye_4)) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

    error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
    error2_4 = theta2_4 - theta2c_4 + zeroError2_4;
    
    c1_4 = PID(theta1_4, theta1c_4, zeroError1_4 , (Kp1+2.0) , (Kd1+1.2) , prev_error1_4);
    c2_4 = PID(theta2_4, theta2c_4, zeroError2_4 , Kp2 , Kd2 , prev_error2_4);

    ll1=0.0, lm1=40.0, ul1=18.5, um1=45;
    ll2=0.0, lm2=90.0, ul2=0.0, um2=70.0;

    correction1_4 = (um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1;
    correction2_4 = (um2-ul2)/(lm2-ll2)*abs(c2_4)+ul2;
    
    if(correction1_4 > um1)
    correction1_4=um1;

    if(correction2_4 > 255)
    correction2_4=255;

    if(u>3.5){
      correction1_4 = 10;
    }

    Serial.print("theta1_4=");
    Serial.println(theta1_4);
    Serial.print("theta1c_4=");
    Serial.println(theta1c_4 - zeroError1_4);
    Serial.print("theta2c_4=");
    Serial.println(theta2c_4 - zeroError2_4);
    Serial.print("theta2_4=");
    Serial.println(theta2_4);
    Serial.print("c1_4=");
    Serial.println(c1_4);
    Serial.print("c2_4=");
    Serial.println(c2_4);
    Serial.print("pwm1=");
    Serial.println(correction1_4);
    Serial.print("pwm2=");
    Serial.println(correction2_4);
    Serial.println("------------------------");

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

//    do{
//      if ( counter1_4 != temp1_4 ) {
//        temp1_4 = counter1_4;
//
//        if (counter1_4 > 1200)
//        {
//          counter1_4 = 0;
//        }
//        theta1c_4 = (counter1_4 * 0.3);
//      }
//    }while((theta1_4 - theta1c_4 + zeroError1_4)>0.1);
//
  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-2");
    double xe_4 = -6+ 5.3333 + u ;
    double ye_4 = -45;

    if ( counter1_4 != temp1_4 ) {
      temp1_4 = counter1_4;

      if (counter1_4 > 1200)
      {
        counter1_4 = 0;
      }
      theta1c_4 = (counter1_4 * 0.3);
    }
    if ( counter2_4 != temp2_4 ) {
      temp2_4 = counter2_4;

      if (counter2_4 > 1200)
      {
        counter2_4 = 0;
      }
      theta2c_4 = -(counter2_4 * 0.3);
    }

    if (atan(ye_4 / xe_4) > 0)
      alpha_4 = atan(ye_4 / xe_4) - 3.14159;

    else
      alpha_4 = atan(ye_4 / xe_4);

    theta1_4 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_4 * xe_4 + ye_4 * ye_4)) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

     error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
    error2_4 = theta2_4 - theta2c_4 + zeroError2_4;
    
    c1_4 = PID(theta1_4, theta1c_4, zeroError1_4 , (Kp1+2.1) , (Kd1+0.6) , prev_error1_4);
    c2_4 = PID(theta2_4, theta2c_4, zeroError2_4 , Kp2 , Kd2 , prev_error2_4);

      ll1=0.0, lm1=30.0, ul1=20, um1=43;
    ll2=0.0, lm2=40.0, ul2=40.0, um2=80.0;

    correction1_4 = (um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1;
    correction2_4 = (um2-ul2)/(lm2-ll2)*abs(c2_4)+ul2;
    
//    correction1_4 = map(abs(c1_4), 0, 30, 0, 50);       //TODO
//    correction2_4 = map(abs(c2_4), 0, 40, 0, 375);      //TODO

    if(correction1_4 > um1)
    correction1_4=um1;

    if(correction2_4 > 255)
    correction2_4 = 255;
   
    Serial.print("theta1_4=");
    Serial.println(theta1_4);
    Serial.print("theta1c_4=");
    Serial.println(theta1c_4 - zeroError1_4);
    Serial.print("theta2c_4=");
    Serial.println(theta2c_4 - zeroError2_4);
    Serial.print("theta2_4=");
    Serial.println(theta2_4);
    Serial.print("c1_4=");
    Serial.println(c1_4);
    Serial.print("c2_4=");
    Serial.println(c2_4);
    Serial.print("pwm1=");
    Serial.println(correction1_4);
    Serial.print("pwm2=");
    Serial.println(correction2_4);
    Serial.println("------------------------");

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

//     do{
//      if ( counter1_4 != temp1_4 ) {
//        temp1_4 = counter1_4;
//
//        if (counter1_4 > 1200)
//        {
//          counter1_4 = 0;
//        }
//        theta1c_4 = (counter1_4 * 0.3);
//      }
//    }while(theta1_4 - theta1c_4 + zeroError1_4>0.1);
  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  {
    Serial.println("line-3");
    double xe_4 = 4.66 + u;
    double ye_4 = -45 ;

    if ( counter1_4 != temp1_4 ) {
      temp1_4 = counter1_4;

      if (counter1_4 > 1200)
      {
        counter1_4 = 0;
      }
      theta1c_4 = (counter1_4 * 0.3);
    }
    if ( counter2_4 != temp2_4 ) {
      temp2_4 = counter2_4;

      if (counter2_4 > 1200)
      {
        counter2_4 = 0;
      }
      theta2c_4 = -(counter2_4 * 0.3);
    }

    if (atan(ye_4 / xe_4) > 0)
      alpha_4 = atan(ye_4 / xe_4) - 3.14159;

    else
      alpha_4 = atan(ye_4 / xe_4);

    theta1_4 = 57.2958 * (cosine_rule(l1, l2, sqrt(xe_4 * xe_4 + ye_4 * ye_4)) + alpha_4);
    theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

    error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
    error2_4 = theta2_4 - theta2c_4 + zeroError2_4;
    
    c1_4 = PID(theta1_4, theta1c_4, zeroError1_4 , (Kp1+2.2) , (Kd1+0.6) , prev_error1_4);
    c2_4 = PID(theta2_4, theta2c_4, zeroError2_4 , Kp2 , Kd2 , prev_error2_4);

    ll1=0.0, lm1=30.0, ul1=26.8, um1=49.5;
    ll2=0.0, lm2=40.0, ul2=20.0, um2=100.0;

    correction1_4 = (um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1;
    correction2_4 = (um2-ul2)/(lm2-ll2)*abs(c2_4)+ul2;

//    correction1_4 = map(abs(c1_4), 0, 30, 0, 50);       //TODO
//    correction2_4 = map(abs(c2_4), 0, 40, 125, 260);      //TODO

    if(correction1_4 > um1)
    correction1_4=um1;

    if(correction2_4 > 255)
      correction2_4 = 255;
    
    Serial.print("theta1_4=");
    Serial.println(theta1_4);
    Serial.print("theta1c_4=");
    Serial.println(theta1c_4 - zeroError1_4);
    Serial.print("theta2c_4=");
    Serial.println(theta2c_4 - zeroError2_4);
    Serial.print("theta2_4=");
    Serial.println(theta2_4);
    Serial.print("c1_4=");
    Serial.println(c1_4);
    Serial.print("c2_4=");
    Serial.println(c2_4);
    Serial.print("pwm1=");
    Serial.println(correction1_4);
    Serial.print("pwm2=");
    Serial.println(correction2_4);
    Serial.println("------------------------");

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

//      do{
//      if ( counter1_4 != temp1_4 ) {
//        temp1_4 = counter1_4;
//
//        if (counter1_4 > 1200)
//        {
//          counter1_4 = 0;
//        }
//        theta1c_4 = (counter1_4 * 0.3);
//      }
//    }while(theta1_4 - theta1c_4 + zeroError1_4>0.1);
  }

  for (double t = 0.1666, u = 0.887; t < 1, u < 5.333; t = t + 0.1666, u = u + 0.887)
  { 

    if ( counter1_4 != temp1_4 ) {
      temp1_4 = counter1_4;

      if (counter1_4 > 1200)
      {
        counter1_4 = 0;
      }
      theta1c_4 = (counter1_4 * 0.3);
    }
    if ( counter2_4 != temp2_4 ) {
      temp2_4 = counter2_4;

      if (counter2_4 > 1200)
      {
        counter2_4 = 0;
      }
      theta2c_4 = -(counter2_4 * 0.3);
    }

    if (t < 0.5) {
      theta1_4 = (-1) * theta(theta1AT1_4, theta1_AT1_4, theta1__AT1_4, theta1AT2_4, theta1_AT2_4, theta1__AT2_4, t);
      theta2_4 = (-1) * theta(theta2AT1_4, theta2_AT1_4, theta2__AT1_4, theta2AT2_4, theta2_AT2_4, theta2__AT2_4, t);
    }
    else {
      theta1_4 = (-1) * theta(theta1AT2_4, theta1_AT2_4, theta1__AT2_4, theta1AT3_4, theta1_AT3_4, theta1__AT3_4, t - 0.5);
      theta2_4 = (-1) * theta(theta2AT2_4, theta2_AT2_4, theta2__AT2_4, theta2AT3_4, theta2_AT3_4, theta2__AT3_4, t - 0.5);
    }

    error1_4 = theta1_4 - theta1c_4 + zeroError1_4;
    error2_4 = theta2_4 - theta2c_4 + zeroError2_4; 
    
    if(t<0.5)
    c1_4 = PID(theta1_4, theta1c_4, zeroError1_4 , Kp1+1.5 , Kd1+2.8 , prev_error1_4);
    else
    c1_4 = PID(theta1_4, theta1c_4, zeroError1_4 , Kp1+2.1 , (Kd1+2.2) , prev_error1_4);
    c2_4 = PID(theta2_4, theta2c_4, zeroError2_4 , Kp2+2 , Kd2 , prev_error2_4);
    ll1=0.0, lm1=70.0, ul1=40.0, um1=65.0;

//    correction1_4 = (um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1;
    
    if(t<0.5)
    correction1_4 = (um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1;
    else{
    ll1=0.0, lm1=70.0, ul1=35, um1=45.0;
    correction1_4 = (um1-ul1)/(lm1-ll1)*abs(c1_4)+ul1;      //TODO   0-60
    }
    
    if(t<0.5){
    ll2=0.0, lm2=70.0, ul2=100.0, um2=225.0;
    correction2_4 = (um2-ul2)/(lm2-ll2)*abs(c2_4) + ul2;      //TODO
    }
    else{
      ll2=0.0, lm2=70.0, ul2=100.0, um2=225.0;
    correction2_4 = (um2-ul2)/(lm2-ll2)*abs(c2_4) + ul2;
    }

    if(correction1_4 > um1)
    correction1_4=um1;

    if(correction2_4 > 255)
    correction2_4=255;
    if(t>0.85)
    correction1_4=10;
    
    Serial.print("theta1_4=");
    Serial.println(theta1_4);
    Serial.print("theta1c_4=");
    Serial.println(theta1c_4 - zeroError1_4);
    Serial.print("theta2c_4=");
    Serial.println(theta2c_4 - zeroError2_4);
    Serial.print("theta2_4=");
    Serial.println(theta2_4);
    Serial.print("c1_4=");
    Serial.println(c1_4);
    Serial.print("c2_4=");
    Serial.println(c2_4);
    Serial.print("pwm1=");
    Serial.println(correction1_4);
    Serial.print("pwm2=");
    Serial.println(correction2_4);
    Serial.println("------------------------");

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

//    do {
//        if ( counter1_4 != temp1_4 ) {
//        temp1_4 = counter1_4;
//
//        if (counter1_4 > 1200)
//        {
//          counter1_4 = 0;
//        }
//        theta1c_4 = (counter1_4 * 0.3);
//      }
////      Serial.println("theta1c_4-----------------");
////      Serial.println(theta1c_4);
//
//    }while(t<0.5 && (theta1_4 - theta1c_4 + zeroError1_4)>0.5);
//
//    do {
//        if ( counter1_4 != temp1_4 ) {
//        temp1_4 = counter1_4;
//
//        if (counter1_4 > 1200)
//        {
//          counter1_4 = 0;
//        }
//        theta1c_4 = (counter1_4 * 0.3);
//      }
//
//    }while(t>0.5 && (theta1_4 - theta1c_4 + zeroError1_4)<-0.2);
//    
//    delay(150);
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

void upr_mtr_fwd_4() {
  digitalWrite(motor1_4, HIGH);
}

void upr_mtr_bwd_4() {
  digitalWrite(motor1_4, LOW);
}

void lwr_mtr_fwd_4() {
  digitalWrite(motor2_4, HIGH);
}

void lwr_mtr_bwd_4() {
  digitalWrite(motor2_4, LOW);
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

