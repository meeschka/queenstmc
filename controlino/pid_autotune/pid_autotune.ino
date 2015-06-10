//based off of br3ttb example code
//adapted for relay style pid
//using thermistors

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

byte ATuneModeRemember=2;
double input=80;
double output=25;
double setpoint=35;
double kp=2,ki=0.5,kd=2;

//input will be temp in C, output will be relay time in ms

//double kpmodel=1.5, taup=100, theta[50];
//double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;

int thermPin = A0;
int relayPin = 9;
int thermConst = 0;
//thermConst relates the thermstor to the correct properties in the thermCalc function

unsigned int aTuneLookBack=20;

boolean tuning = true;
unsigned long  serialTime;
int WindowSize = 5000;
int minRelayTime = 500;
unsigned long windowStartTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);



void setup()
{  
  windowStartTime = millis();
  pinMode(relayPin, OUTPUT);
  //relay is active low, so urn off to start
  digitalWrite(relayPin, HIGH);
  
  //tell PID range between 0 and full window size
  myPID.SetOutputLimits(0, WindowSize);
  
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  changeAutoTune();

  
  serialTime = 0;

  
  Serial.begin(115200);

}

void loop()
{

  unsigned long now = millis();

  input = tempCalc(analogRead(0), 1);
  
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      Serial.println("All done!");
      Serial.println();
      Serial.println();
    }
  }
  else myPID.Compute();
  
  if (millis() - windowStartTime > WindowSize)
   { //time to shift relay window
     windowStartTime += WindowSize;
     Serial.println("Window shift");
   }
  if (output < millis() - windowStartTime) digitalWrite(relayPin, HIGH);
  else digitalWrite(relayPin, LOW);
 
  
  //send-receive with processing if it's time
  if (millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune()
{
  
  //Set the output to the desired starting frequency.
  output=aTuneStartValue;
  aTune.SetNoiseBand(1);
  aTune.SetOutputStep(10);
  aTune.SetLookbackSec(20);
 
}



void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");

  Serial.println("tuning mode");

  Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
  Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
  Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println(" ");
  
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush();
  }
}


double tempCalc(int anVal, int n){
  double result = 0;
  anVal = double(anVal);
  double Rfix[] = {9900.0, 9850.0};   //measured resistance of fixed resistor, here nominally 10k
  //steinhart  variables for each thermistor determined experimentally
  double A[] = {0.004045442, 0.003903939};
  double B[] = {-1.4426e-06, 2.21645e-05};
  double C[] = {-8.47204E-07, -9.45665e-07};

  result = Rfix[n] / ((1023.0/anVal)-1.0);
  result = log(result);
  //result = 1/(0.004045442 + (1.4426e-6)*result + (-8.47204e-7)*result*result*result);
  result = (1 / ( A[n] + B[n]*result + C[n]*result*result*result))-273.15;
  return result;
  
}
