#include <PinChangeInt.h>

#define motor1  43
#define motor1pwm  2
#define motor2  40
#define motor2pwm  11
int A_1 = 3, A_2 = 4;
  
#define a1 20
#define b1 A15
#define a2 19
#define b2 51
//bool state1 = true, state2 = true;

float pwmx1=100, pwmx2=55, pwmx4=150;
float theta1c=0, theta2c=0, zeroError1 = 53.728, zeroError2=42.68, minAngle1 = 21.6, minAngle2 = 28.5;  //53.728  42.68
int x; 
    
volatile int temp1, counter1 = 0; 
volatile int temp2, counter2 = 0;
  
void setup() {
  Serial.begin(9600);
  
  pinMode(a1, INPUT_PULLUP);
  pinMode(b1, INPUT_PULLUP);  
  attachInterrupt(A_1, ai1, CHANGE);
//  state1 = digitalRead(a1);
  
  pinMode(a2, INPUT_PULLUP);  
  pinMode(b2, INPUT_PULLUP);
  attachInterrupt(A_2, ai2, CHANGE);
//  state2 = digitalRead(a2);
  
  pinMode(motor1, OUTPUT);
  pinMode(motor1pwm, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor2pwm, OUTPUT);
}

void loop() 
{
  if (Serial.available())
  {
    x = Serial.parseInt();
  }
    if (x) {
      if (x < 3)
      {
        digitalWrite(motor1, x - 1 );
        if(x==1)
        analogWrite(motor1pwm, pwmx1);
        else
        analogWrite(motor1pwm, pwmx2);
      }
      if(x == 3 || (-theta1c+minAngle1)>zeroError1){                   //53.728
        analogWrite(motor1pwm, 0);
        x=3;
        }
      if(x>3 && x<6){
        digitalWrite(motor2, x - 4 );
        analogWrite(motor2pwm , pwmx4);
        } 
      if(x == 6 || (-theta2c + minAngle2)>zeroError2){               //42.68
        analogWrite(motor2pwm , 0);
        x=6;
        }
    }


  //Convert Encoder Output into angle
      if ( counter1 != temp1 ) {
        temp1 = counter1;
        if (counter1 > 1200) {
          counter1 = 0;
         }
        theta1c = (counter1 * 0.3);
        Serial.println ("theta1");
        Serial.println (theta1c);
     }
      if ( counter2 != temp2 ) {
        temp2 = counter2;
        
        if (counter2 > 1200) {
          counter2 = 0;
        }
        theta2c = (counter2 * 0.3);
        Serial.println ("theta2");
        Serial.println (theta2c);
      }
}

// Encoder1
void ai1() {
  if (digitalRead(b1) == !digitalRead(a1)) {
    counter1++; 
  } else {
    counter1--;
  }
//  state1 = !state1;
}

//  Encoder2
void ai2() {
  if (digitalRead(b2) == !digitalRead(a2)) {
    counter2++;
  } else {
    counter2--;
  }
//  state2 = !state2;
}

