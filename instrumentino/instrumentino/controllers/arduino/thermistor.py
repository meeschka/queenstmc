from __future__ import division
from instrumentino.controllers.arduino import SysCompArduino, SysVarAnalogArduinoUnipolar
__author__ = 'pitts'


class thermistor(SysCompArduino):
    def __init__(self, name, rangeT, pinInT, pinInVoltsMax, pinInVoltsMin):
        SysCompArduino.__init__(self, name, (SysVarAnalogArduinoUnipolar('T', rangeT, pinInT, None, name, 'Temperature', 'C', pinInVoltsMax=pinInVoltsMax, pinInVoltsMin=pinInVoltsMin, therm=True),),
                                'measure the temperature')
                                                  
class resistorADC(SysCompArduino):   
    def __init__(self, name, rangeT, pinInT, pinInVoltsMax, pinInVoltsMin):
        SysCompArduino.__init__(self, name, (SysVarAnalogArduinoUnipolar('T', rangeT, pinInT, None, name, 'Temperature', 'C', pinInVoltsMax=pinInVoltsMax, pinInVoltsMin=pinInVoltsMin, therm=False),),
                                'measure the temperature')                                               
    #need to enter Steinhart-Hart values and the value of fixed resistors used in voltage dividers into the Arduino.init file, thermistorValues function
                                                                         
