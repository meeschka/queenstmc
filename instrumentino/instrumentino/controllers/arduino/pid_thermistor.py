from __future__ import division
from instrumentino.controllers.arduino import SysCompArduino, SysVarDigitalArduino,\
    SysVarAnalogArduinoUnipolar, SysVarPidRelayArduino, SysVarAnalogArduino
__author__ = 'yoelk'

from instrumentino import cfg

class PidControlledThermistor(SysCompArduino):
    def __init__(self, name, rangeT, pinInT, pinOutRelay, sensorVoltsMin, sensorVoltsMax, pidVar, windowSizeMs, kp, ki, kd):
        self.varEnable = SysVarDigitalArduino('enable', None, name, PreSetFunc=self.PreEditEnable)
        self.pidRelayVar = SysVarPidRelayArduino('T', rangeT, pidVar, windowSizeMs, kp, ki, kd, pinInT, pinOutRelay, name, 'Temperature',
                                                 'C', pinInVoltsMin=sensorVoltsMin, pinInVoltsMax=sensorVoltsMax, therm=True)
        #self.pulseTimeVar = SysVarAnalogArduinoUnipolar('ms', [0-5000], pinTime, None, name)
        
        SysCompArduino.__init__(self, name,
                                (self.pidRelayVar, self.varEnable), 
                                'control a heating element through a relay to keep the temperature set')
        
    def FirstTimeOnline(self):
        super(PidControlledThermistor, self).FirstTimeOnline()
        
    def PreEditEnable(self, value):
        self.pidRelayVar.Enable(value=='on')
        
'''       
        
class PidCalliThermistor(SysCompArduino):
    def __init__(self, name, rangeT, pinInT, pinOutRelay, sensorVoltsMin, sensorVoltsMax, pidVar, windowSizeMs, kp, ki, kd):
        self.varEnable = SysVarDigitalArduino('enable', None, name, PreSetFunc=self.PreEditEnable)
        self.pidRelayVar = SysVarPidRelayArduino('T', rangeT, pidVar, windowSizeMs, kp, ki, kd, pinInT, pinOutRelay, name, 'Temperature',
                                                 'C', pinInVoltsMin=sensorVoltsMin, pinInVoltsMax=sensorVoltsMax, therm=False)
        #self.pulseTimeVar = SysVarAnalogArduinoUnipolar('ms', [0-5000], pinTime, None, name)
        
        SysCompArduino.__init__(self, name,
                                (self.pidRelayVar, self.varEnable), 
                                'control a heating element through a relay to keep the temperature set')
        
    def FirstTimeOnline(self):
        super(PidControlledThermistor, self).FirstTimeOnline()
        
    def PreEditEnable(self, value):
        self.pidRelayVar.Enable(value=='on')
        
 '''       