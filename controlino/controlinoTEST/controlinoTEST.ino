

/*
	controlino.cpp - Library for controling an Arduino using the USB
	Created by Joel Koenka, April 2014
	Released under GPLv3

	controlino let's a user control the Arduino pins by issuing simple serial commands such as "Read" "Write" etc.
	It was originally written to be used with Instrumentino, the open-source GUI platform for experimental settings,
	but can also be used for other purposes.
	For Instrumentino, see:
	http://www.sciencedirect.com/science/article/pii/S0010465514002112	- Release article
	https://pypi.python.org/pypi/instrumentino/1.0						- Package in PyPi
	https://github.com/yoelk/instrumentino								- Code in GitHub
 */

#include "Arduino.h"
#include "HardwareSerial.h"
#include "SoftwareSerial.h"
#include "string.h"
#include <PID_v1.h>
 int pinOne = 0;

// ------------------------------------------------------------
// Main functions
// ------------------------------------------------------------

/***
 * The setup function is called once at startup of the sketch
 */
void setup() {
	Serial.begin(115200);

	
}

/***
 * The loop function is called in an endless loop
 */
void loop() {
  double val;
  Serial.println(analogRead(pinOne));
  val = tempCalc(analogRead(pinOne), 2);
  Serial.println(val);
  delay(100);
  
}


double tempCalc(int anVal, int n){
  double result;
  double A[] = {0.004045442, 0.003903939};
  double B[] = {-1.4426e-06, 2.21645e-05};
  double C[] = {-8.47204E-07, -9.45665e-07};
  double Rfix[] = {9900, 9850};
  if (anVal > 1)  {
    result = Rfix[n-1] / ((1023/anVal)-1);
    result = log(result);
    result = 1/ ( A[n-1] +B[n-1]*result +C[n-1]*result*result*result);
    result = double(result-273.15);
    Serial.println(result);
    delay(100);
    return result;
  }
  else {
    return 0;
  }
}
