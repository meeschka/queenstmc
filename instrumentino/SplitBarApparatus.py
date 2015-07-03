# coding=UTF-8

from __future__ import division
from instrumentino import Instrument
from instrumentino import cfg
import time
from instrumentino.action import SysAction, SysActionParamTime, SysActionParamFloat, SysActionParamInt
from instrumentino.controllers.arduino import Arduino
from instrumentino.controllers.arduino import SysVarDigitalArduino
from instrumentino.controllers.arduino.pins import DigitalPins, AnalogPins
from instrumentino.controllers.arduino.pid_thermistor import PidControlledThermistor
from instrumentino.controllers.arduino.thermistor import thermistor
from instrumentino.SteadyStateModule import SteadyStateClass


'''
*** System constants
'''
# Arduino pin assignments

pinAnalInThermometer1 = int(3)
pinAnalInThermometer2 = int(6)

pinAnalInThermometer3 = int(7)
pinAnalInThermometer4 = int(8)

pinAnalInThermometerHeat1 = int(0)
pinAnalInThermometerHeat2 = int(1)


pinDigiOutHeater1Relay = 9
pinDigiOutHeater2Relay = 8

pinVoltMax = 5.12
pinVoltMin = 0 

valMax = 100
valMin = 10
'''
*** System components
'''

#heatThermistor1 = AnalogPinCallibration('Heater Temperature', [15, 1000], pinAnalInThermometerInHeat, pinVoltMax, pinVoltMin)


heatThermistor1 = PidControlledThermistor('Heater 1', [valMin, valMax], pinAnalInThermometerHeat1, pinDigiOutHeater1Relay, 0.25, 5.05, 1, 5000, 150, 0, 0)
heatThermistor2 = PidControlledThermistor('Heater 2', [valMin, valMax], pinAnalInThermometerHeat2, pinDigiOutHeater2Relay, 0.25, 5.05, 2, 5000, 45.0, 4.2, 120)



sample1Thermometer = thermistor('Sample Temperature 1', (valMin, valMax), pinAnalInThermometer1, pinVoltMax, pinVoltMin)
sample2Thermometer = thermistor('Sample Temperature 2', (valMin, valMax), pinAnalInThermometer2, pinVoltMax, pinVoltMin)

sample3Thermometer = thermistor('Sample Temperature 3', (valMin, valMax), pinAnalInThermometer3,  pinVoltMax, pinVoltMin)
sample4Thermometer = thermistor('Sample Temperature 4', (valMin, valMax), pinAnalInThermometer4,  pinVoltMax, pinVoltMin)


digiPins1 = DigitalPins('digital pins', (SysVarDigitalArduino('Heat Element 1', pinDigiOutHeater1Relay),))
digiPins2 = DigitalPins('digital pins', (SysVarDigitalArduino('Heat Element 2', pinDigiOutHeater2Relay),))


SteadyState = SteadyStateClass('State',['Varying','Steady'], Arduino)
 
'''
*** System actions
'''
class SetThermostat1(SysAction):
    def __init__(self):
        self.temp = SysActionParamFloat(heatThermistor1.vars['T'])
        SysAction.__init__(self, 'Thermostat 1', (self.temp,))

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
                    
                   
class SetThermostat2(SysAction):
    def __init__(self):
        self.temp = SysActionParamFloat(heatThermistor2.vars['T'])
        SysAction.__init__(self, 'Thermostat 2', (self.temp,))

    def Command(self):
        heatThermistor2.vars['T'].Set(self.temp.Get())
        heatThermistor2.vars['enable'].Set('on')
        cfg.Sleep(3)


class TuneThermostat2(SysAction):
    def __init__(self):
        self.kp = SysActionParamFloat(range=[0,1000], name='kp')
        self.ki = SysActionParamFloat(range=[0,1000], name='ki')
        self.kd = SysActionParamFloat(range=[0,1000], name='kd')
        SysAction.__init__(self, 'Thermostat tuning 2', (self.kp, self.ki, self.kd))

    def Command(self):
#         boxThermostat.vars['T'].Tune(self.kp.Get(), self.ki.Get(), self.kd.Get())
#         cfg.Sleep(3)
        
        for p in range(1,10):
            for i in range(1,10):
                for d in range(1,10):
                    heatThermistor2.vars['T'].Tune(p, i, d)
                    heatThermistor2.vars['T'].Set(30)
                    heatThermistor2.vars['enable'].Set('on')
                    cfg.Sleep(5*60)
                    


class heatPulse(SysAction):
    def __init__(self):
        self.Seconds = SysActionParamTime()
        #self.heater = SysActionParamInt()
        self.trigger = SysActionParamInt('heater', [0,1])
        
        #self.pin = SysActionParamFloat(digiPins1.vars['Heat Element 1'])
        SysAction.__init__(self, 'Heat pulse', (self.Seconds, self.trigger))
        
    def Command(self):
        if self.trigger.Get():
            
            #set pin
            digiPins1.vars['Heat Element 1'].Set('on')
            #wait pulse time
            cfg.Sleep(self.Seconds.Get())
            #Turn back off
            digiPins1.vars['Heat Element 1'].Set('off')

class blinkHeat(SysAction):
    def __init__(self):
        self.milliseconds = SysActionParamInt('blink time ms', [0, 2000])
        #self.pulseTime = SysActionParamInt('pulse time, ms', [0, 5000])
        #self.milliseconds = SysActionParamFloat(name='blink time ms', range=[0, 2000])
        self.trigger = SysActionParamInt('heat blink on?', [0, 1])
        SysAction.__init__(self, 'Blink Heat', (self.milliseconds, self.trigger))
    def Command(self):
        if self.trigger.Get():
            digiPins1.vars['Heat Element 1'].BlinkFunc(self.milliseconds.Get())


        
def closeAll():
    heatThermistor1.vars['enable'].Set('off')
    heatThermistor2.vars['enable'].Set('off')
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
        comps = (heatThermistor1, heatThermistor2, sample1Thermometer, sample2Thermometer, sample3Thermometer, sample4Thermometer, digiPins1, digiPins2, SteadyState)
        #, sample1Thermometer, sample2Thermometer, sample3Thermometer, sample4Thermometer, digiPins1, digiPins2)
        #comps = (sample1Thermometer, sample2Thermometer, sample3Thermometer, sample4Thermometer,       
        actions = (SetThermostat1(), TuneThermostat1(), SetThermostat2(), TuneThermostat2(), heatPulse(), blinkHeat(), ActionCloseSystem())
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