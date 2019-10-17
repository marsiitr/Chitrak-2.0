#include <iostream>
#include <math.h>

using namespace std;

double a = 26.0, b = 23.4, c = 0;
double x, y, angle;
double theta1, theta2;

double cosine1(){
  return (acos((a*a + c*c - b*b)/(2*a*c)))*90/acos(0);
}

double cosine2(){
  return (acos((a*a + b*b - c*c)/(2*a*b)))*90/acos(0);
}


int main() {
	x = 16;
	y = -45;
	c = sqrt(x*x + y*y);
    angle = atan(abs(y/x))*90/acos(0);
    if(x==0){
    	theta1 = 90 - cosine1();
	}
	else{
	
    theta1 = angle - cosine1();
    if(x>0){
      theta1 = 180 - angle - cosine1();
    }
}
    theta2= 180 - cosine2();
    cout<<"theta1 "<<theta1<<endl;
    cout<<"theta2 "<<theta2<<endl;
	return 0;
}
