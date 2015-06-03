from __future__ import division
from instrumentino.controllers.arduino import SysCompArduino, SysVarDigitalArduino,\
    SysVarAnalogArduinoUnipolar, SysVarPidRelayArduino
__author__ = 'yoelk'

from instrumentino import cfg

class PidControlledThermistor(SysCompArduino):
    def __init__(self, name, rangeT, pinInT, pinOutRelay, sensorVoltsMin, sensorVoltsMax, pidVar, windowSizeMs, kp, ki, kd):
        self.varEnable = SysVarDigitalArduino('enable', None, name, PreSetFunc=self.PreEditEnable)
        self.pidRelayVar = SysVarPidRelayArduino('T', rangeT, pidVar, windowSizeMs, kp, ki, kd, pinInT, pinOutRelay, name, 'Temperature',
                                                 'C', pinInVoltsMin=sensorVoltsMin, pinInVoltsMax=sensorVoltsMax)
        SysCompArduino.__init__(self, name,
                                (self.pidRelayVar, self.varEnable), 
                                'control a heating element through a relay to keep the temperature set')
        
    def FirstTimeOnline(self):
        super(PidControlledThermistor, self).FirstTimeOnline()
        
    def PreEditEnable(self, value):
        self.pidRelayVar.Enable(value=='on')
        
        
        
        
        
        
        #WESLEYYYYYY
        #We want to make this similar to the thermistor pid, but opposite: the peltier element turns on when the
        #temperature exceeds the setpoint
        #not sure if we should simply multiply temperature readings by -1, or, more likely, have negative PID vars, or something else entirely
        #consider some of these options, implement whichever one you think is best if possible
        
        #On the other hand... this element will be using the same thermistor as the second heater
        #should we make it a conditional pid within the pid thermistor class in arduino init?
        #If the heater isn't on, double check, maybe the peltier element should be?