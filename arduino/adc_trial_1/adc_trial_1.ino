#include <Wire.h>
#include <Adafruit_ADS1015.h>
 
Adafruit_ADS1115 ads1(0x48);
Adafruit_ADS1115 ads2(0x49);
Adafruit_ADS1115 *value[2] = {&ads1, &ads2};

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");
  
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
  ads1.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads1.begin();
  ads2.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads2.begin(); 
  
}
 
void loop(void)
{
  int16_t adc0, adc1, adc2, adc3;
 
  adc0 = (*value[0]).readADC_SingleEnded(0);
  adc1 = ads1.readADC_SingleEnded(1);
  adc2 = (*value[1]).readADC_SingleEnded(0);
  adc3 = ads1.readADC_SingleEnded(3);
  Serial.print("AIN0: "); Serial.println(adc0);
  Serial.print("AIN1: "); Serial.println(adc1);
  Serial.print("AIN2: "); Serial.println(adc2);
  Serial.print("AIN3: "); Serial.println(adc3);
  Serial.println(" ");
  

  
  delay(1000);
}

