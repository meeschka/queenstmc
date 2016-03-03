
# coding=UTF-8

from __future__ import division
from instrumentino import Instrument
from instrumentino import cfg
import time
from instrumentino.action import SysAction, SysActionParamTime, SysActionParamFloat, SysActionParamInt
from instrumentino.controllers.arduino import Arduino
from instrumentino.controllers.arduino import SysVarDigitalArduino
from instrumentino.controllers.arduino.pins import DigitalPins, AnalogPins
from instrumentino.controllers.arduino.thermistor import resistorADC
from instrumentino.SteadyStateOven import SteadyStateClass


'''
*** System constants
'''
# Arduino pin assignments

pinAnalInThermometer1 = int(0)
pinAnalInThermometer2 = int(0)



pinVoltMax = 5.12
pinVoltMin = 0 

valMax = 70
valMin = 30
'''
*** System components
'''



sample1Thermometer = resistorADC('Sample Temperature 1', (valMin, valMax), pinAnalInThermometer1, pinVoltMax, pinVoltMin)
sample2Thermometer = resistorADC('Sample Temperature 2', (valMin, valMax), pinAnalInThermometer2, pinVoltMax, pinVoltMin)
#realTemp = sample1Thermometer - sample2Thermometer
#SteadyState = SteadyStateClass('State',['Varying','Steady'], Arduino)
 
'''
*** System actions
'''


'''
*** System
'''
class System(Instrument):
    def __init__(self):
        comps = (sample1Thermometer, sample2Thermometer)
        #, sample1Thermometer, sample2Thermometer, sample3Thermometer, sample4Thermometer, digiPins1, digiPins2)
        #comps = (sample1Thermometer, sample2Thermometer, sample3Thermometer, sample4Thermometer,       
        actions = ()
        name = 'Lab Toaster'
        description = 'A portable split-bar apparatus'
        version = '1.0'
         
        Instrument.__init__(self, comps, actions, version, name, description)
 
'''
*** Run program
'''        
if __name__ == '__main__':
    # run the program
    System()