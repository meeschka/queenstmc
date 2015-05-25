# coding=UTF-8

from __future__ import division
from instrumentino import Instrument
from instrumentino import cfg
import time
from instrumentino.action import SysAction, SysActionParamTime, SysActionParamFloat, SysActionParamInt

from instrumentino.controllers.arduino import SysVarDigitalArduino
from instrumentino.controllers.arduino.pins import DigitalPins, AnalogPins
from instrumentino.controllers.arduino.pid_thermistor import PidControlledThermistor
from instrumentino.controllers.arduino.thermistor import thermistorUnipolar


'''
*** System constants
'''
# Arduino pin assignments



pinAnalInThermometerHeat1 = int(0)



pinDigiOutHeater1Relay = 13


pinVoltMax = 5.12
pinVoltMin = 0 

valMax = 50
valMin = 00
'''
*** System components
'''

#heatThermistor1 = AnalogPinCallibration('Heater Temperature', [15, 1000], pinAnalInThermometerInHeat, pinVoltMax, pinVoltMin)


heatThermistor1 = PidControlledThermistor('Heater 1', [valMin, valMax], pinAnalInThermometerHeat1, pinDigiOutHeater1Relay, 0.25, 5.05, 1, 5000, 0.5, 0.5,2)


digiPins1 = DigitalPins('digital pins', (SysVarDigitalArduino('Heat Element 1', 13),))
 
'''
*** System actions
'''
class SetThermostat1(SysAction):
    def __init__(self):
        self.temp = SysActionParamFloat(heatThermistor1.vars['T'])
        SysAction.__init__(self, 'Set Thermostat 1', (self.temp,))

    def Command(self):
        heatThermistor1.vars['T'].Set(self.temp.Get())
        heatThermistor1.vars['enable'].Set('on')
        cfg.Sleep(3)


class TuneThermostat1(SysAction):
    def __init__(self):
        self.kp = SysActionParamFloat(range=[0,1000], name='kp')
        self.ki = SysActionParamFloat(range=[0,1000], name='ki')
        self.kd = SysActionParamFloat(range=[0,1000], name='kd')
        SysAction.__init__(self, 'Thermostat tuning 1', (self.kp, self.ki, self.kd))

    def Command(self):
#         boxThermostat.vars['T'].Tune(self.kp.Get(), self.ki.Get(), self.kd.Get())
#         cfg.Sleep(3)
        
        for p in range(1,10):
            for i in range(1,10):
                for d in range(1,10):
                    heatThermistor1.vars['T'].Tune(p, i, d)
                    heatThermistor1.vars['T'].Set(30)
                    heatThermistor1.vars['enable'].Set('on')
                    cfg.Sleep(5*60)
                    
                    
        
def closeAll():
    heatThermistor1.vars['enable'].Set('off')

    print("off")
class ActionCloseSystem(SysAction):
    def __init__(self):
        SysAction.__init__(self, 'Close system', ())

    def Command(self):
        closeAll()

'''
*** System
'''
class System(Instrument):
    def __init__(self):
        comps = (heatThermistor1, digiPins1)
        #, sample1Thermometer, sample2Thermometer, sample3Thermometer, sample4Thermometer, digiPins1, digiPins2)
        #comps = (sample1Thermometer, sample2Thermometer, sample3Thermometer, sample4Thermometer,       
        actions = (SetThermostat1(),
                   TuneThermostat1(), ActionCloseSystem()
                   )
        name = 'Lab Toaster'
        description = 'A portable split-bar apparatus'
        version = '1.0'
         
        Instrument.__init__(self, comps, actions, version, name, description)
 
'''
*** Run program
'''        
if __name__ == '__main__':
    # run the program
    System()# -*- coding: utf-8 -*-
