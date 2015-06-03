# coding=UTF-8

### !!!!!!!!!!!!!! 
# check how much volume I need to take to inject samples.
# check what is the maximal speed in which I don't move liquids in the capillary
# check which side is for anions and which for cations, and note it on the box
# check that +/- 30kV works

from __future__ import division
from instrumentino import Instrument
from instrumentino import cfg
from instrumentino.action import SysAction, SysActionParamTime, SysActionParamFloat, SysActionParamInt
from instrumentino.controllers.labsmith_eib.labsmith_comps import LabSmithValves4VM01, LabSmithSPS01SyringePump, LabSmithSensors4AM01
from instrumentino.controllers.arduino.parker import ParkerPressureController
from instrumentino.controllers.labsmith_eib import SysVarDigitalLabSmith_AV201Position, SysVarAnalogLabSmith_SensorValue
from instrumentino.controllers.arduino.hvm import HvmSMHV05100, HvmSMHV05100N
from instrumentino.controllers.arduino.edaq import EdaqEcorder
from instrumentino.controllers.arduino import SysVarDigitalArduino,\
    SysVarAnalogArduinoUnipolar
from instrumentino.controllers.arduino.pins import DigitalPins, AnalogPins
from instrumentino.controllers.arduino.spellman import SpellmanUM30_4_MINUS, SpellmanUM30_4_PLUS
from instrumentino.controllers.arduino.pid_thermostat import PidControlledThermostat
from instrumentino.controllers.arduino.thermometer import AnalogPinThermometer_AD22103
import numpy as np
 
'''
*** System constants
'''
# Arduino pin assignments

pinAnalInParkerP = 0
pinPwmOutParkerP = 5

pinDigiOutEdaqTrigger = 9
 
pinDigiOutHeaterRelay = 8

pinAnalInThermometerEnvironment = 2
pinAnalInThermometerInBox = 0

# 3-way LabSmith Valves can be set to 'A', 'B' and 'closed'

# Syringe pump with holding coil 
syringeVolume_ul = 100
pumpBottomPercent = 5
pumpTopPercent = 95
syringeUsableVolume_ul = syringeVolume_ul * (pumpTopPercent - pumpBottomPercent) / 100
 
'''
*** System components
'''
pressureController = ParkerPressureController('Pressure', (0,50), pinAnalInParkerP, pinPwmOutParkerP)
ecorder = EdaqEcorder('e-corder', pinDigiOutEdaqTrigger)
sensors = LabSmithSensors4AM01('sensors', (SysVarAnalogLabSmith_SensorValue('Pressure', 1, 'psi', [0,115]),))

boxThermostat = PidControlledThermostat('Thermostat', [0,100], pinAnalInThermometerInBox, pinDigiOutHeaterRelay, 0.25, 3.05, 1, 5000, 2, 5, 1)
envThermometer = AnalogPinThermometer_AD22103('Env. Temperature', pinAnalInThermometerEnvironment)

 
'''
*** System actions
'''


def setPressure(prs):
    for p in np.linspace(0.0, prs, num=10):
        pressureController.vars['P'].Set(p)
        print p


class SetThermostat(SysAction):
    def __init__(self):
        self.temp = SysActionParamFloat(boxThermostat.vars['T'])
        SysAction.__init__(self, 'Thermostat', (self.temp,))

    def Command(self):
        boxThermostat.vars['T'].Set(self.temp.Get())
        boxThermostat.vars['enable'].Set('on')



class ActionFlushCapillary(SysAction):
    def __init__(self):
        self.prs = SysActionParamFloat(pressureController.vars['P'])
        self.seconds = SysActionParamTime()
        self.trigger = SysActionParamInt('trigger?', [0,1])
        SysAction.__init__(self, 'Flush capillary', (self.seconds, self.prs, self.trigger))

    def Command(self):
        
        if self.trigger.Get(): ecorder.vars['trigger'].Set('on')
        setPressure(self.prs.Get())
                
        cfg.Sleep(self.seconds.Get())
        
        pressureController.vars['P'].Set(0)
        print 0
        if self.trigger.Get(): ecorder.vars['trigger'].Set('off')



'''
*** System
'''
class System(Instrument):
    def __init__(self):
        comps = (pressureController, sensors, envThermometer, boxThermostat, ecorder)
        actions = (ActionFlushCapillary(),
                   SetThermostat()
                   )
        name = 'portableCE-Argentina'
        description = 'A portable dual-channel Capillary Electrophoresis instrument'
        version = '1.0'
         
        Instrument.__init__(self, comps, actions, version, name, description)
 
'''
*** Run program
'''        
if __name__ == '__main__':
    # run the program
    System()