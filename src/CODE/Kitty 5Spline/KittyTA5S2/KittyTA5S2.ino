#define motor1_3  27                 
#define motor2_3  26
#define motor1pwm_3  3
#define motor2pwm_3  2

#define motor1_4  36
#define motor2_4  29
#define motor1pwm_4  5
#define motor2pwm_4  4

#include <SoftwareSerial.h>

SoftwareSerial mySerial1(10, 11); // RX, TX          

int bb=1;

float alpha_3, theta1c_3 = 0.0 , theta2c_3 = 0.0, theta1_3, theta2_3, error1_3, error2_3, correction1_3, correction2_3, c1_3, c2_3;
float dif_error1_3 , prev_error1_3 = 0.0 , dif_error2_3 , prev_error2_3 = 0.0;

float alpha_4, theta1c_4 = 0.0 , theta2c_4 = 0.0, theta1_4, theta2_4, error1_4, error2_4, correction1_4, correction2_4, c1_4, c2_4;
float dif_error1_4 , prev_error1_4 = 0.0 , dif_error2_4 , prev_error2_4 = 0.0;

float Kp1 = 1.5, Kp2 = 1.5, Kd1 = 2.0, Kd2 = 2.0 ;

int l1 = 25, l2 = 25, a = 20, b = 7;

volatile int temp1_3 , counter1_3 = 0;
volatile int temp2_3 , counter2_3 = 0;

volatile int temp1_4 , counter1_4 = 0;
volatile int temp2_4 , counter2_4 = 0;

double theta1AT1_3=53.728,theta1AT2_3=45.78,theta1AT3_3=69.71,theta1_AT1_3=0,theta1_AT2_3=0.1,theta1_AT3_3=0,theta1__AT1_3=0,theta1__AT2_3=0.1,theta1__AT3_3=0;
double theta2AT1_3=42.68,theta2AT2_3=76.73,theta2AT3_3=50.75,theta2_AT1_3=0,theta2_AT2_3=0.1,theta2_AT3_3=0,theta2__AT1_3=0,theta2__AT2_3=0.1,theta2__AT3_3=0;

double theta1AT1_4=54.68,theta1AT2_4=48.41,theta1AT3_4=72.815,theta1_AT1_4=0,theta1_AT2_4=0.1,theta1_AT3_4=0,theta1__AT1_4=0,theta1__AT2_4=0.1,theta1__AT3_4=0;
double theta2AT1_4=45.57,theta2AT2_4=77.29,theta2AT3_4=49.55,theta2_AT1_4=0,theta2_AT2_4=0.1,theta2_AT3_4=0,theta2__AT1_4=0,theta2__AT2_4=0.1,theta2__AT3_4=0;

void setup()
{
  Serial.begin(9600);
  mySerial1.begin(9600);

//  LEG-3

  pinMode(21, INPUT_PULLUP);                                           
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(2, ai2_3, RISING);
  
  pinMode(20, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(3, ai3_3, RISING);

  pinMode(motor1_3, OUTPUT);
  pinMode(motor1pwm_3, OUTPUT);
  pinMode(motor2_3, OUTPUT);
  pinMode(motor2pwm_3, OUTPUT);

  // LEG-4
  
  pinMode(19, INPUT_PULLUP);                                           
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(4, ai4_4, RISING);
  
  pinMode(18, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  attachInterrupt(5, ai5_4, RISING);

  pinMode(motor1_4, OUTPUT);
  pinMode(motor1pwm_4, OUTPUT);
  pinMode(motor2_4, OUTPUT);
  pinMode(motor2pwm_4, OUTPUT);

}

void loop()
{
    for (float u = 0.888; u < 10.666 ; u = u + 0.888)    
    {

      float xe_4 = -6 + u ;
      float ye_4 = -45;

      float xe_3 = 1.3333 + u ;
      float ye_3 = -45 ;

      if ( counter1_3 != temp1_3 ){
        temp1_3 = counter1_3;

        if (counter1_3 > 600)
          counter1_3 = 0;

        theta1c_3 = (counter1_3 * 0.6);
      }

      if ( counter2_3 != temp2_3 ){
        temp2_3 = counter2_3;

        if (counter2_3 > 600)
          counter2_3 = 0;

        theta2c_3 = -(counter2_3 * 0.6);
      }

      if ( counter1_4 != temp1_4 ){
        temp1_4 = counter1_4;

        if (counter1_4 > 600)
          counter1_4 = 0;

        theta1c_4 = (counter1_4 * 0.6);
      }

      if ( counter2_4 != temp2_4 ){
        temp2_4 = counter2_4;

        if (counter2_4 > 600)
          counter2_4 = 0;

        theta2c_4 = -(counter2_4 * 0.6);
      }

      if ( counter1_3 != temp1_3 ) {
      temp1_3 = counter1_3;

      if (counter1_3 > 600)
        counter1_3 = 0;

      theta1c_3 = (counter1_3 * 0.6);
    }

    if ( counter2_3 != temp2_3 ) {
      temp2_3 = counter2_3;

      if (counter2_3 > 600)
        counter2_3 = 0;

      theta2c_3 = -(counter2_3 * 0.6);
    }

    if ( counter1_4 != temp1_4 ) {
      temp1_4 = counter1_4;

      if (counter1_4 > 600)
        counter1_4 = 0;

      theta1c_4 = (counter1_4 * 0.6);
    }

    if ( counter2_4 != temp2_4 ) {
      temp2_4 = counter2_4;

      if (counter2_4 > 600)
        counter2_4 = 0;

      theta2c_4 = -(counter2_4 * 0.6);
    }

    
        if (atan(ye_3 / xe_3) > 0)
          alpha_3 = atan(ye_3 / xe_3) - PI;

        else
          alpha_3 = atan(ye_3 / xe_3);

        if (atan(ye_4 / xe_4) > 0)
          alpha_4 = atan(ye_4 / xe_4) - PI;

        else
          alpha_4 = atan(ye_4 / xe_4);
      
        theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
        theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

        
        error1_3 = theta1_3 - theta1c_3 + 62.513;        
        error2_3 = theta2_3 - theta2c_3/4 + 51.58;           

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;

        c1_3 = (Kp1-0.5) * error1_3 + (Kd1+0.5) * (dif_error1_3);
//        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

        correction1_3 = map(abs(c1_3), 0, 30, 0, 50); //50
        correction2_3 = map(abs(c2_3), 0, 35, 0, 150);//175
        
        theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
        theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

        error1_4 = theta1_4 - theta1c_4 + 72.815;         
        error2_4 = theta2_4 - theta2c_4/4 + 49.55;              

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;

            c1_4 = (Kp1-0.5) * error1_4 + (Kd1+0.5) * (dif_error1_4);

//        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

        correction1_4 = map(abs(c1_4), 0, 30, 0, 50);     //50
        correction2_4 = map(abs(c2_4), 0, 35, 0, 150);    //175
      
        Serial.print("x=");
        Serial.println(xe_4);
        Serial.print("y=");
        Serial.println(ye_4);
        Serial.print("theta1_4=");
        Serial.println(theta1_4);
        Serial.print("theta1c_4=");
        Serial.println(theta1c_4-72.815);
        Serial.print("theta2c_4=");
        Serial.println(theta2c_4-49.55);
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
   
        if (error1_3 < 0 )
        {
          upr_mtr_fwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }
        else if (error1_3 > 0)
        {
          upr_mtr_bwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }

        if (error2_3 < 0)
        {
          lwr_mtr_fwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }
        else if (error2_3 > 0)
        {
          lwr_mtr_bwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }

        if (error1_4 < 0 )
        {
          upr_mtr_fwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }
        else if (error1_4 > 0)
        {
          upr_mtr_bwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }

        if (error2_4 < 0)
        {
          lwr_mtr_fwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
        else if (error2_4 > 0)
        {
          lwr_mtr_bwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
    }
    
    for (float t = 0.1666, u = 0.887; t < 1, u < 5.333 ; t = t + 0.1666, u = u + 0.887)
    {
      float xe_4 = 4.66 + u ;
      float ye_4 = -45 ;

      if ( counter1_3 != temp1_3 )
      {
        temp1_3 = counter1_3;

        if (counter1_3 > 600)
          counter1_3 = 0;

        theta1c_3 = (counter1_3 * 0.6);
      }

      if ( counter2_3 != temp2_3 )
      {
        temp2_3 = counter2_3;

        if (counter2_3 > 600)
          counter2_3 = 0;

        theta2c_3 = -(counter2_3 * 0.6);
      }

      if ( counter1_4 != temp1_4 )
      {
        temp1_4 = counter1_4;

        if (counter1_4 > 600)
          counter1_4 = 0;

        theta1c_4 = (counter1_4 * 0.6);
      }

      if ( counter2_4 != temp2_4 )
      {
        temp2_4 = counter2_4;

        if (counter2_4 > 600)
          counter2_4 = 0;

        theta2c_4 = -(counter2_4 * 0.6);
      }


        if (atan(ye_4 / xe_4) > 0)
          alpha_4 = atan(ye_4 / xe_4) - PI;

        else
          alpha_4 = atan(ye_4 / xe_4);

        if(t<0.5){
            theta1_3=(-1)*theta(theta1AT1_3,theta1_AT1_3,theta1__AT1_3,theta1AT2_3,theta1_AT2_3,theta1__AT2_3,t);
            theta2_3=(-1)*theta(theta2AT1_3,theta2_AT1_3,theta2__AT1_3,theta2AT2_3,theta2_AT2_3,theta2__AT2_3,t);
        }
        else{
            theta1_3=(-1)*theta(theta1AT2_3,theta1_AT2_3,theta1__AT2_3,theta1AT3_3,theta1_AT3_3,theta1__AT3_3,t-0.5);
            theta2_3=(-1)*theta(theta2AT2_3,theta2_AT2_3,theta2__AT2_3,theta2AT3_3,theta2_AT3_3,theta2__AT3_3,t-0.5);
        }

        error1_3 = theta1_3 - theta1c_3 + 62.513;       
        error2_3 = theta2_3 - theta2c_3/4 + 51.58;        

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;

        if(t<0.5)
            c1_3 = (Kp1-0.5) * error1_3 + Kd1 * (dif_error1_3);
         else
            c1_3 = (Kp1-0.9) * error1_3 + Kd1 * (dif_error1_3);
//        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);


        if(t<0.5)
    {
    correction1_3 = map(abs(c1_3), 0, 70, 0, 80);
    correction2_3 = map(abs(c2_3), 0, 90, 0, 200);//150
    }
    else
    {
      correction1_3 = map(abs(c1_3), 0, 70, 0, 80);
    correction2_3 = map(abs(c2_3), 0, 90, 0, 210);//150
    }
//        correction1_3 = map(abs(c1_3), 0, 70, 0, 65);       //80
//        correction2_3 = map(abs(c2_3), 0, 90, 0, 180);       //175

        theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
        theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));
  
        error1_4 = theta1_4 - theta1c_4 + 72.815;        
        error2_4 = theta2_4 - theta2c_4/4 + 49.55;            

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;

            c1_4 = (Kp1-0.5) * error1_4 + (Kd1+0.5) * (dif_error1_4);

//        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

        correction1_4 = map(abs(c1_4), 0, 30, 0, 50);     //50
        correction2_4 = map(abs(c2_4), 0, 35, 0, 150);    //175

        Serial.print("x=");
        Serial.println(xe_4);
        Serial.print("y=");
        Serial.println(ye_4);
        Serial.print("theta1_4=");
        Serial.println(theta1_4);
        Serial.print("theta1c_4=");
        Serial.println(theta1c_4-72.815);
        Serial.print("theta2c_4=");
        Serial.println(theta2c_4-49.55);
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
   
        if (error1_3 < 0 ){
          upr_mtr_fwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }
        
        else if (error1_3 > 0)
        {
          upr_mtr_bwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }

        if (error2_3 < 0)
        {
          lwr_mtr_fwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }
        else if (error2_3 > 0)
        {
          lwr_mtr_bwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }

        if (error1_4 < 0 )
        {
          upr_mtr_fwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }
        else if (error1_4 > 0)
        {
          upr_mtr_bwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }

        if (error2_4 < 0)
        {
          lwr_mtr_fwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
        else if (error2_4 > 0)
        {
          lwr_mtr_bwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
    }

    for (float t = 0.1666, u = 0.887; t < 1, u < 5.333 ; t = t + 0.1666, u = u + 0.887)
    {
      float xe_3 = -4 + u ;
      float ye_3 = -45 ;

      if ( counter1_3 != temp1_3 )
      {
        temp1_3 = counter1_3;

        if (counter1_3 > 600)
          counter1_3 = 0;

        theta1c_3 = (counter1_3 * 0.6);
      }

      if ( counter2_3 != temp2_3 )
      {
        temp2_3 = counter2_3;

        if (counter2_3 > 600)
          counter2_3 = 0;

        theta2c_3 = -(counter2_3 * 0.6);
      }

      if ( counter1_4 != temp1_4 )
      {
        temp1_4 = counter1_4;

        if (counter1_4 > 600)
          counter1_4 = 0;

        theta1c_4 = (counter1_4 * 0.6);
      }

      if ( counter2_4 != temp2_4 )
      {
        temp2_4 = counter2_4;

        if (counter2_4 > 600)
          counter2_4 = 0;

        theta2c_4 = -(counter2_4 * 0.6);
      }

        if (atan(ye_3 / xe_3) > 0)
          alpha_3 = atan(ye_3 / xe_3) - PI;

        else
          alpha_3 = atan(ye_3 / xe_3);

      
        theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
        theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

        error1_3 = theta1_3 - theta1c_3 + 62.513;        
        error2_3 = theta2_3 - theta2c_3/4 + 51.58;            

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;


            c1_3 = (Kp1-0.5) * error1_3 + (Kd1+0.5) * (dif_error1_3);

//        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

        correction1_3 = map(abs(c1_3), 0, 30, 0, 40);     
        correction2_3 = map(abs(c2_3), 0, 35, 0, 150);     //175

        if(t<0.5){
            theta1_4=(-1)*theta(theta1AT1_4,theta1_AT1_4,theta1__AT1_4,theta1AT2_4,theta1_AT2_4,theta1__AT2_4,t);
            theta2_4=(-1)*theta(theta2AT1_4,theta2_AT1_4,theta2__AT1_4,theta2AT2_4,theta2_AT2_4,theta2__AT2_4,t);
        }
        else{
            theta1_4=(-1)*theta(theta1AT2_4,theta1_AT2_4,theta1__AT2_4,theta1AT3_4,theta1_AT3_4,theta1__AT3_4,t-0.5);
            theta2_4=(-1)*theta(theta2AT2_4,theta2_AT2_4,theta2__AT2_4,theta2AT3_4,theta2_AT3_4,theta2__AT3_4,t-0.5);
        }

        error1_4 = theta1_4 - theta1c_4 + 72.815;         
        error2_4 = theta2_4 - theta2c_4/4 + 49.55;              

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;


        if(t<0.5)
    c1_4 = (Kp1-0.5) * error1_4 + Kd1 * (dif_error1_4);
    else
    c1_4 = (Kp1-0.9) * error1_4 + Kd1 * (dif_error1_4);
//        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);


          if(t<0.5)
    {
    correction1_4 = map(abs(c1_4), 0, 70, 0, 80);
    correction2_4 = map(abs(c2_4), 0, 90, 0, 200);//150
    }
    else
    {
      correction1_4 = map(abs(c1_4), 0, 70, 0, 80);
    correction2_4 = map(abs(c2_4), 0, 90, 0, 210);//150
    }
//        correction1_4 = map(abs(c1_4), 0, 70, 0, 65);   //80
//        correction2_4 = map(abs(c2_4), 0, 90, 0, 180);   //175

//        Serial.print("x=");
//        Serial.println(xe_4);
//        Serial.print("y=");
//        Serial.println(ye_4);
        Serial.print("theta1_4=");
        Serial.println(theta1_4);
        Serial.print("theta1c_4=");
        Serial.println(theta1c_4-72.815);
        Serial.print("theta2c_4=");
        Serial.println(theta2c_4-49.55);
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
        
        if (error1_3 < 0 )
        {
          upr_mtr_fwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }
        else if (error1_3 > 0)
        {
          upr_mtr_bwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }

        if (error2_3 < 0)
        {
          lwr_mtr_fwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }
        else if (error2_3 > 0)
        {
          lwr_mtr_bwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }

        if (error1_4 < 0 )
        {
          upr_mtr_fwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }
        else if (error1_4 > 0)
        {
          upr_mtr_bwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }

        if (error2_4 < 0)
        {
          lwr_mtr_fwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
        else if (error2_4 > 0)
        {
          lwr_mtr_bwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
    }

}

  double theta(double thet1,double thet1_,double thet1__,double thet2,double thet2_,double thet2__,double t){
    double a,b,c,d,e,f;

    a=thet1;
    b=thet1_;
    c=thet1__/2;
    d=(20*(thet2-thet1)-(8*thet2_+12*thet1_)*0.5+(3*thet1__-thet2__)*0.5*0.5)/(2*0.5*0.5*0.5);
    e=(30*(thet1-thet2)+(14*thet2_+16*thet1_)*0.5+(3*thet1__-2*thet2__)*0.5*0.5)/(2*0.5*0.5*0.5*0.5);
    f=(12*(thet2-thet1)-(6*thet2_+6*thet1_)*0.5-(thet1__-thet2__)*0.5*0.5)/(2*0.5*0.5*0.5*0.5*0.5);
    
    return (a+b*t+c*t*t+d*t*t*t+e*t*t*t*t+f*t*t*t*t*t);
  }

  float cosine_rule(float c, float b, float a){
    float x = ( a * a + b * b - c * c ) / ( 2 * a * b );
    return acos(x);
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

  void ai2_3(){
    if (digitalRead(6) == LOW)
    {
      counter1_3++;
    } else {
      counter1_3--;
    }
  }

  void ai3_3() {
  if (digitalRead(7) == LOW) {
      counter2_3++;
    } else {
      counter2_3--;
    }
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

  void ai4_4(){
    if (digitalRead(8) == LOW)
    {
      counter1_4++;
    } else {
      counter1_4--;
    }
  }

  void ai5_4() {
  if (digitalRead(9) == LOW) {
      counter2_4++;
    } else {
      counter2_4--;
    }
  }

