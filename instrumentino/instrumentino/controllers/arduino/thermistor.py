from __future__ import division
from instrumentino.controllers.arduino import SysCompArduino, SysVarAnalogArduinoUnipolar
__author__ = 'pitts'
'''
class AnalogPinCallibration(SysCompArduino):
    def __init__(self, name, rangeT, pinInT, A, B, C, pinInVoltsMax, pinInVoltsMin):
        SysCompArduino.__init__(self, name,
                                (SysVarAnalogArduinoResistance('C', rangeT, pinInT, A, B, C, pinInVoltsMax, pinInVoltsMin, name, self.name, None, pinInVoltsMax=pinInVoltsMax, pinInVoltsMin=pinInVoltsMin),),
                                'measure the temperature')
'''

class thermistorUnipolar(SysCompArduino):
    def __init__(self, name, rangeT, pinInT, pinInVoltsMax, pinInVoltsMin):
        SysCompArduino.__init__(self, name, (SysVarAnalogArduinoUnipolar('T', rangeT, pinInT, None, name, 'Temperature', 'C', pinInVoltsMax=pinInVoltsMax, pinInVoltsMin=pinInVoltsMin, therm=True),),
                                'measure the temperature')
                                                  
