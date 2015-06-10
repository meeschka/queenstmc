from __future__ import division
from instrumentino.controllers.arduino import SysCompArduino, SysVarAnalogArduinoUnipolarThermistor
__author__ = 'pitts'
'''
class AnalogPinCallibration(SysCompArduino):
    def __init__(self, name, rangeT, pinInT, A, B, C, pinInVoltsMax, pinInVoltsMin):
        SysCompArduino.__init__(self, name,
                                (SysVarAnalogArduinoResistance('C', rangeT, pinInT, A, B, C, pinInVoltsMax, pinInVoltsMin, name, self.name, None, pinInVoltsMax=pinInVoltsMax, pinInVoltsMin=pinInVoltsMin),),
                                'measure the temperature')
'''

class twoThermistorsUnipolar(SysCompArduino):
    def __init__(self, name, rangeT, pinInT1, pinInT2, pinInVoltsMax, pinInVoltsMin):
        SysCompArduino.__init__(self, name, (SysVarAnalogArduinoUnipolarThermistor('T', rangeT, pinInT1, pinInT2, None, name, 'Temperature', 'C', pinInVoltsMax=pinInVoltsMax, pinInVoltsMin=pinInVoltsMin),),
                                'measure the temperature using two thermistors')
                                                  
