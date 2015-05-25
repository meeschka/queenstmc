from __future__ import division
from instrumentino.comp import SysVarDigital, SysVarAnalog, SysComp
from instrumentino.controllers import InstrumentinoController
__author__ = 'yoelk'

from instrumentino import cfg
import wx
import serial
import time
from threading import Semaphore
import numpy as np

class Arduino(InstrumentinoController):
    '''
    This class implements an interface to a simple arduino program that allows control of pins and softSerial.
    The idea for this approach was taken from this code: https://github.com/vascop/Python-Arduino-Proto-API-v2 
    '''

    # Arduino constants
    HIGH    = 1
    LOW     = 0

    PIN_VOLT_MIN        = 0
    PIN_VOLT_MAX        = 5
    PIN_VOLT_RANGE      = (PIN_VOLT_MAX - PIN_VOLT_MIN)
    PIN_VOLT_MIDDLE     = (PIN_VOLT_MIN + PIN_VOLT_RANGE / 2)

    ANAL_OUT_VAL_MAX    = 255
    ANAL_OUT_VAL_MIN    = 0
    ANAL_OUT_VAL_RANGE  = (ANAL_OUT_VAL_MAX - ANAL_OUT_VAL_MIN)
    ANAL_OUT_VAL_MIDDLE = (ANAL_OUT_VAL_MIN + ANAL_OUT_VAL_RANGE / 2)

    ANAL_IN_VAL_MAX     = 1023
    ANAL_IN_VAL_MIN     = 0
    ANAL_IN_VAL_RANGE   = (ANAL_IN_VAL_MAX - ANAL_IN_VAL_MIN)
    
    PIN_LED = 13
    
    baudrate                = 115200
    serialTimeoutSec        = 0.05
    serialWriteTimeoutSec   = 0
    # the read delay should to at least twice the time for a single read, so there's also time to write
    cacheReadDelayMilisec = max(250, (serialTimeoutSec + serialWriteTimeoutSec)*1000*2)
    
    maxNonResponseAllowed = 10
    
    pinValuesCache = {}
    timer = None
    
    name = 'Arduino'
    
    def __init__(self):
        '''
        init 
        '''
        InstrumentinoController.__init__(self, self.name)
        self.accessSemaphore = Semaphore()
        self.serial = None
        self.nonResponsiveCounter = 0
    
    def Connect(self, port):
        '''
        port - the name of the serial port connected to the Arduino. This works on unix and windows systems, given the appropriate name
        '''
        try:
            self.serial = serial.Serial(port, self.baudrate, writeTimeout=self.serialWriteTimeoutSec, timeout=self.serialTimeoutSec)
        except:
            cfg.LogFromOtherThread('Arduino did not respond', True)
            return None

        time.sleep(2)
        
        if self.serial == None or self._sendData('Read A0') == None:
            return None
        
        # start reading timer
        self.timer = wx.Timer(cfg.app)
        cfg.app.Bind(wx.EVT_TIMER, self.CacheUpdate, self.timer)
        self.timer.Start(self.cacheReadDelayMilisec)
        
        return True

    def Close(self, reset=True):
        '''
        Close the connection to the Arduino
        '''
        if self.serial != None:
            if self.timer != None:
                self.timer.Stop()
            if reset:
                self.Reset()
            self.serial.close()
            self.serial = None

    def CacheUpdate(self, event):
        pins = ''
        # save the keys in case they change while we read
        keys = self.pinValuesCache.keys()
        for k in keys:
            pins += k + ' '
        
        valuesStr = self._sendData('Read %s'%(pins.strip()))
        if valuesStr == None:
            return
        
        values = valuesStr.split(' ')
        
        # values are received in the same order we asker for them
        for key, val in zip(keys, values):
            self.pinValuesCache[key] = int(val)
            
    def PinModeOut(self, pin):
        '''
        Set digital pin mode as output 
        '''
        self.PinMode(pin, True)

    def PinModeIn(self, pin):
        '''
        Set digital pin mode as input 
        '''
        self.PinMode(pin, False)

    def PinMode(self, pin, modeOut):
        '''
        Set digital pin mode
        modeOut - set to 'out' if True, else set to 'in'  
        '''
        self._sendData('Set %d %s'%(pin, 'out' if modeOut else 'in'), wait=True)

    def AnalogWriteVolts(self, pin, volts):
        '''
        Set the voltage level (using PWM) of a digital output pin directly
        volts - between 0-5 volts  
        '''
        self.AnalogWrite(pin, self.ANAL_OUT_VAL_MAX *  volts / self.PIN_VOLT_MAX)

    def AnalogWriteFraction(self, pin, fraction, maxV=5, minV=0):
        '''
        Set the voltage level (using PWM) of a digital output pin using a fraction
        fraction - between 0-1  
        minV - minimal voltage
        maxV - maximal voltage
        '''
        self.AnalogWriteVolts(pin, minV + (maxV - minV) * fraction)

    def AnalogWrite(self, pin, value):
        '''
        Set the voltage level (using PWM) of a digital output pin using an 8-bit value
        value - between 0-255  
        '''
        self._sendData('Write %d analog %d'%(pin, value), wait=True)
            
    def SetHighFreqPwm(self, pin):
        '''
        Set this PWM pin to work in high frequency mode   
        '''
        self._sendData('SetPwmFreq %d 1'%(pin), wait=True)

    def AnalogReadVolts(self, pin):
        '''
        Read the voltage level of an analog input pin directly
        returns - value between 0-5 (volts)  
        '''
        value = self.AnalogRead(pin)
        if value != None:
            return self.PIN_VOLT_MAX * value / self.ANAL_IN_VAL_MAX
    
    def AnalogReadFraction(self, pin, maxV=5, minV=0):
        '''
        Read the voltage level of an analog input pin using a fraction
        minV - minimal voltage
        maxV - maximal voltage
        returns - value between 0-1  
        '''
        value = self.AnalogReadVolts(pin)
        if value != None:
            return (value - minV) / (maxV - minV)

    def AnalogRead(self, pin):
        '''
        Read the voltage level of an analog input pin using a 10-bit value
        
        Returns: value between 0-1023  
        '''
        # Get value from cache if possible, and if not, add it to the wish-list
        try:
            return self.pinValuesCache['A' + str(pin)]
        except KeyError:
            self.pinValuesCache['A' + str(pin)] = 0
        
    def AnalogReadMultiRes(self, pin):
        '''
        Takes the average of 6 values, and converts the analog reading into resistance
        '''   
        R = [9900, 9850, 9920, 9850, 9980, 9940]        
        
        i = 0
        value = [0, 0, 0, 0, 0, 0]
        while i<6:
            value[i] = self.AnalogRead(pin)
            i +=1
            time.sleep(0.05)
            val= [value[1], value[2], value[3], value[4], value[5]]
        result = float(np.average(val))
        result = R[pin]/((1023/result)-1) if result != 0 else 0

        return result if result!= None else 0            

    def DigitalWriteHigh(self, pin):
        '''
        Set the voltage level of a digital output pin to the maximum (HIGH level)
        '''
        self.DigitalWrite(self, pin, 1)

    def DigitalWriteLow(self, pin):
        '''
        Set the voltage level of a digital output pin to the minimum (LOW level)
        '''
        self.DigitalWrite(self, pin, 0)

    def DigitalWrite(self, pin, value):
        '''
        Set the voltage level of a digital output pin to value (1 to HIGH and 0 to low)
        '''
        self._sendData('Write %d digi %d'%(pin, value), wait=True)

    def DigitalRead(self, pin):
        '''
        Read the voltage level of a digital pin
        
        Returns: 1 for HIGH and 0 for LOW
        '''
        # Get value from cache if possible, and if not, add it to the wish-list
        try:
            return self.pinValuesCache['D' + str(pin)]
        except KeyError:
            self.pinValuesCache['D' + str(pin)] = 0
    
    def PidRelayCreate(self, pidVar, pinAnalIn, pinDigiOut, windowSizeMs, kp, ki, kd):
        '''
        Create a new PID controlled relay variable
        pidVar - the variable number (see controlino sketch for max supported variables)
        pinAnalIn - the analog input pin which provides input to the PID algorithm
        pinDigiOut - the digital pin that controls the relay
        windowSizeMs - the window size in ms
        kp - the Kp parameter 
        ki - the Ki parameter
        kd - the Kd parameter
        '''
        self._sendData('PidRelayCreate %d %d %d %d %f %f %f'%(pidVar, pinAnalIn, pinDigiOut, windowSizeMs, kp, ki, kd), wait=True)
            
    def PidRelaySet(self, pidVar, setPoint):
        '''
        Set the setpoint of a PID controlled relay variable and start striving towards it
        pidVar - the variable number (see controlino sketch for max supported variables)
        setPoint - The value we're striving for
        '''
        self._sendData('PidRelaySet %d %d'%(pidVar, setPoint), wait=True)
            
    def PidRelayTune(self, pidVar, kp, ki, kd):
        '''
        Start an auto-tune session for the PID controller
        pidVar - the variable number (see controlino sketch for max supported variables)
        kp - Kp parameter
        ki - Ki parameter 
        kd - Kd parameter
        '''
        self._sendData('PidRelayTune %d %f %f %f'%(pidVar, kp, ki, kd), wait=True)
            
    def PidRelayEnable(self, pidVar, enable):
        '''
        Enable/disable the contorl loop of a PID controlled relay variable
        pidVar - the variable number (see controlino sketch for max supported variables)
        enable - True/False for enable/disable
        '''
        self._sendData('PidRelayEnable %d %d'%(pidVar, 1 if enable else 0), wait=True)
    
    def HardSerConnect(self, baudrate, port=1):
        '''
        Setup a hardware serial port for communication
        baudrate - the serial connection baudrate
        port - the hardware serial port number
        '''
        self._sendData('HardSerConnect %d %d'%(baudrate, port), wait=True)
        time.sleep(1)
    
    def SoftSerConnect(self, pinRx, pinTx, baudrate, port=1):
        '''
        Setup a software serial port using pinRx and pinTx for communication
        baudrate - the serial connection baudrate (should be much smaller than the baudrate used for the Arduino)
        port - a port number to identify future transactions
        '''
        self._sendData('SoftSerConnect %d %d %d %d'%(pinRx, pinTx, baudrate, port), wait=True)
        time.sleep(1)
    
    def SerSend(self, serialData, writeTimeoutSec=None, isSoftSerial=True, port=1):
        '''
        Send serial data over an open serial connection
        serialData - the data to send. a NULL ('\0') character is added at the end to signify the end of transmission
        writeTimeoutSec - time in seconds to wait for a software serial response
        isSoftSerial - True for software serial ports, False for hardware serial ports
        port - the port number to use
        
        Returns: the data received (if any)
        '''        
        self.accessSemaphore.acquire(True)
        if writeTimeoutSec != None:
            self._sendData('SerReceive %s %d'%('soft' if isSoftSerial else 'hard', port), lock=False)
        
        self._sendData('SerSend %s %d'%('soft' if isSoftSerial else 'hard', port), lock=False)
        self._sendData(serialData + '\0', False, lock=False)
        
        rxData = None
        if writeTimeoutSec != None:
            time.sleep(writeTimeoutSec)
            rxData = self._sendData('SerReceive %s %d'%('soft' if isSoftSerial else 'hard', port), lock=False)
            
        self.accessSemaphore.release()
        return rxData

    def Reset(self):
        '''
        Reset the mode of all digital pins to input
        '''
        self._sendData('Reset', wait=True)
        time.sleep(1)
        
    def BlinkPin(self, pin=PIN_LED, ms=500):
        '''
        Start blinking a pin on the Arduino.
        This may be used to blink the LED on the board (pin 13), which might serve as an indication that the Arduino is still running.
        pin - which pin to blink
        ms - how many milliseconds between each blink
        '''
        self._sendData('BlinkPin %d %d'%(pin, ms), wait=True)
        
    def I2cWrite(self, address, values):
        '''
        Send a list of values over the I2C bus
        '''
        self._sendData('I2cWrite %d %s'%(address, ' '.join(str(n) for n in values)), wait=True)
    
    def _sendData(self, txData, addLineBreak=True, lock=True, wait=False, log=False):
        '''
        This function should be called only in this class
        Send a command (txData) to the Arduino and wait for acknowledgment.
        Failure of the acknowledgment to arrive logs the event and raises an exception
         
        writeTimeoutSec - time in seconds to wait for a software serial response
        addLineBreak - add a line break after txData (to tell the Arduino to execute the command) 
        lock - lock access to Arduino during this function
        wait - wait until the Arduino is unlocked. This is used for critical commands such as write commands
        log - log the communication to the screen. mostly for debug purposes
        
        Returns: the data received (if any)
        '''
        if self.serial == None:
            return None
                
        if lock == True:
            if self.accessSemaphore.acquire(False) == False:
                if wait:
                    while self.accessSemaphore.acquire(False) == False:
                        pass
                else:
                    return None
        
        if log:
            print 'we say: ' + txData
        if addLineBreak == True:
            txData = txData + '\r'

        try:
            self.serial.write(txData)
            rxData = self._getData()
        except:
            rxData = ''
        
        if log:
            print 'Arduino says: ' + rxData
        
        answerEnd = rxData.rfind("done") 
        if answerEnd == -1:
            self.nonResponsiveCounter += 1
            if wait == True or self.nonResponsiveCounter > self.maxNonResponseAllowed:
                cfg.LogFromOtherThread('Arduino did not respond %d times'%(self.nonResponsiveCounter), True)
            return None

        if lock == True:
            self.accessSemaphore.release()

        self.nonResponsiveCounter = 0            
        return rxData[0:answerEnd].strip();

    def _getData(self):
        '''
        This function should be called only in the class
        Reads the Arduino response for the last command
        
        Returns: the response
        '''
        try:        
            return self.serial.read(300)
        except:
            return ''
            
    def thermistorVars(self, pin):
        '''
        This function calls the Steinhart-Hart variables for each thermistor. These variables are determined experimentally for each thermistor
        '''

        A = (0.004045442,0.003903939,0.004027618,0.004044341,0.004059005,0.00397408)
        B = (-1.4426E-06,2.21645E-05,5.79403E-07,-2.24057E-06,-4.08686E-06,9.56374E-06)
        C = (-8.47204E-07,-9.45665E-07,-8.52131E-07,-8.40103E-07,-8.29844E-07,-8.80338E-07)
        
        varis = (A[pin], B[pin], C[pin])
        return varis if varis!= None else 0     
    
# base class and variables
class SysVarDigitalArduino(SysVarDigital):
    def __init__(self, name, pin, compName='', stateToValue={'on': 1, 'off':0}, helpLine='', editable=True, PreSetFunc=None):
        self.stateToValue = stateToValue
        self.valueToState = {v: k for k, v in stateToValue.items()}
        SysVarDigital.__init__(self, name, self.stateToValue.keys(), Arduino, compName, helpLine, editable, PreSetFunc)
        self.pin = pin
        self.lastSetState = None

    def FirstTimeOnline(self):
        if self.pin != None:
            self.GetController().PinMode(self.pin, self.editable)
        
    def GetFunc(self):
        if self.pin != None:
            value = self.GetController().DigitalRead(self.pin)
            return self.valueToState[value] if value != None else None
        else:
            return self.lastSetState
    
    def SetFunc(self, state):
        self.lastSetState = state
        if self.pin != None:
            self.GetController().DigitalWrite(self.pin, self.stateToValue[state])

class ArduinoI2cDac():
    def __init__(self, dacBits, address):
        self.maxVal = 2**dacBits-1
        self.address = address

    def WriteFraction(self, fraction, controller):
        controller.I2cWrite(self.address, (0, self.maxVal * fraction,))

class SysVarAnalogArduino(SysVarAnalog):
    '''
    An Arduino analog variable
    '''
    def __init__(self, name, range, pinAnalIn, pinPwmOut=None, SetPolarityPositiveFunc=None, GetPolarityPositiveFunc=None, compName='', helpLine='', units='', PreSetFunc=None, highFreqPWM=False, pinOutVoltsMax=5, pinInVoltsMax=5, pinOutVoltsMin=0, pinInVoltsMin=0, PostGetFunc=None, I2cDac=None):
        showEditBox = (pinPwmOut != None) or (PreSetFunc != None) or (I2cDac != None)
        SysVarAnalog.__init__(self, name, range, Arduino, compName, helpLine, showEditBox , units, PreSetFunc, PostGetFunc)
        self.pinIn = pinAnalIn
        self.pinOut = pinPwmOut
        self.SetPolarityPositiveFunc = SetPolarityPositiveFunc
        self.GetPolarityPositiveFunc = GetPolarityPositiveFunc
        self.highFreqPWM = highFreqPWM
        self.pinOutVoltsMax = pinOutVoltsMax
        self.pinInVoltsMax = pinInVoltsMax
        self.pinOutVoltsMin = pinOutVoltsMin
        self.pinInVoltsMin = pinInVoltsMin
        self.I2cDac = I2cDac
        
    def FirstTimeOnline(self):
        if self.pinOut != None:
            self.GetController().PinModeOut(self.pinOut)
            if self.highFreqPWM:
                self.GetController().SetHighFreqPwm(self.pinOut)
    
    def GetUnipolarRange(self):
        return self.GetUnipolarMax() - self.GetUnipolarMin()
    
    def GetFunc(self):
        fraction = self.GetController().AnalogReadFraction(self.pinIn, self.pinInVoltsMax, self.pinInVoltsMin)
        sign = 1 if self.GetPolarityPositiveFunc() else -1
        return sign * (self.GetUnipolarMin() + (self.GetUnipolarRange() * fraction)) if fraction != None else None
    
    def SetFunc(self, value):
        fraction = (abs(value) - self.GetUnipolarMin()) / self.GetUnipolarRange()
        if self.pinOut != None:
            self.GetController().AnalogWriteFraction(self.pinOut, fraction, self.pinOutVoltsMax, self.pinOutVoltsMin)
        elif self.I2cDac != None:
            self.I2cDac.WriteFraction(fraction, self.GetController())

class SysVarAnalogArduinoThermistor(SysVarAnalog):
    '''
    An Arduino analog variable that is used to calculate the thermistor value.
    I know this is not the best way to do this, but the only way I could make it work was
    to have my thermistor function call SysVarAnalogArduinoUnipolarThermistor, which calls this function.
    '''
    def __init__(self, name, range, pinAnalIn, pinPwmOut=None, SetPolarityPositiveFunc=None, GetPolarityPositiveFunc=None, compName='', helpLine='', units='', PreSetFunc=None, highFreqPWM=False, pinOutVoltsMax=0, pinInVoltsMax=5, pinOutVoltsMin=0, pinInVoltsMin=0, PostGetFunc=None):
        showEditBox = (pinPwmOut != None) or (PreSetFunc != None)
        SysVarAnalog.__init__(self, name, range, Arduino, compName, helpLine, showEditBox , units, PreSetFunc, PostGetFunc)
        self.pinIn = pinAnalIn
        self.pinOut = pinPwmOut
        self.SetPolarityPositiveFunc = SetPolarityPositiveFunc
        self.GetPolarityPositiveFunc = GetPolarityPositiveFunc
        self.highFreqPWM = highFreqPWM
        self.pinOutVoltsMax = pinOutVoltsMax
        self.pinInVoltsMax = pinInVoltsMax
        self.pinOutVoltsMin = pinOutVoltsMin
        self.pinInVoltsMin = pinInVoltsMin

        
    def FirstTimeOnline(self):
        if self.pinOut != None:
            self.GetController().PinModeOut(self.pinOut)
            if self.highFreqPWM:
                self.GetController().SetHighFreqPwm(self.pinOut)
        self.GetVaris()
        
    def GetVaris(self):
        self.varis = self.GetController().thermistorVars(self.pinIn)
        self.A = self.varis[0]
        self.B = self.varis[1]
        self.C = self.varis[2]    
        
    def GetUnipolarRange(self):
        return self.GetUnipolarMax() - self.GetUnipolarMin()
    
    def GetFunc(self):
        res = self.GetController().AnalogReadMultiRes(self.pinIn)
        
        R1 = (np.log(res)) if res != None else 0
        R3 = (R1*R1*R1) if R1 != None else 0

        tmp = self.A+self.B*R1+self.C*R3 if R3 != None else 0
        tmp = 1/tmp if tmp != None and tmp != 0 else 0
        if np.isnan(tmp):
            return 0
        else:
            return float(tmp-273.15) if tmp !=None else 0 
           
       # return res if res != None else 0
    
    def SetFunc(self, value):
        if self.pinOut != None:
            self.GetController().AnalogWriteFraction(self.pinOut, (abs(value) - self.GetUnipolarMin()) / self.GetUnipolarRange(), self.pinOutVoltsMax, self.pinOutVoltsMin)    


class SysVarAnalogArduinoUnipolarThermistor(SysVarAnalogArduinoThermistor):
    '''
    A unipolar analog variable, for which the range has to be [X1,X2] or [-X1,-X2].
    The voltage on the pin (normally 0-5 V) corresponds percentage-wise to the variable's value between X1 and X2 (or -X1 and -X2).
    '''
    def __init__(self, name, range, pinAnalIn, pinPwmOut, compName='', helpLine='', units='', PreSetFunc=None, highFreqPWM=False, pinOutVoltsMax=5, pinInVoltsMax=5, pinOutVoltsMin=0, pinInVoltsMin=0, PostGetFunc=None):
        SysVarAnalogArduinoThermistor.__init__(self, name, range, pinAnalIn, pinPwmOut, self.SetPolarityPositiveFunc, self.GetPolarityPositiveFunc, compName, helpLine, units, PreSetFunc, highFreqPWM, pinOutVoltsMax, pinInVoltsMax, pinOutVoltsMin, pinInVoltsMin, PostGetFunc)
        self.sign = 1 if range[0] >=0 and range[1] >= 0 else -1
        

    def SetPolarityPositiveFunc(self):
        pass
    
    def GetPolarityPositiveFunc(self):
        return self.sign == 1

    def GetUnipolarMin(self):
        return min(abs(self.range[0]), abs(self.range[1]))
    
    def GetUnipolarMax(self):
        return max(abs(self.range[0]), abs(self.range[1]))

class SysVarAnalogArduinoUnipolar(SysVarAnalogArduino):
    '''
    A unipolar analog variable, for which the range has to be [X1,X2] or [-X1,-X2].
    The voltage on the pin (normally 0-5 V) corresponds percentage-wise to the variable's value between X1 and X2 (or -X1 and -X2).
    '''
    def __init__(self, name, range, pinAnalIn, pinPwmOut, compName='', helpLine='', units='', PreSetFunc=None, highFreqPWM=False, pinOutVoltsMax=5, pinInVoltsMax=5, pinOutVoltsMin=0, pinInVoltsMin=0, PostGetFunc=None, I2cDac=None):
        SysVarAnalogArduino.__init__(self, name, range, pinAnalIn, pinPwmOut, self.SetPolarityPositiveFunc, self.GetPolarityPositiveFunc, compName, helpLine, units, PreSetFunc, highFreqPWM, pinOutVoltsMax, pinInVoltsMax, pinOutVoltsMin, pinInVoltsMin, PostGetFunc, I2cDac)
        self.sign = 1 if range[0] >=0 and range[1] >= 0 else -1

    def SetPolarityPositiveFunc(self):
        pass
    
    def GetPolarityPositiveFunc(self):
        return self.sign == 1

    def GetUnipolarMin(self):
        return min(abs(self.range[0]), abs(self.range[1]))
    
    def GetUnipolarMax(self):
        return max(abs(self.range[0]), abs(self.range[1]))
    
class SysVarAnalogArduinoBipolarWithExternalPolarity(SysVarAnalogArduino):
    '''
    A bipolar symmetric analog variable, for which the range has to be [-X,X].
    The voltage on the pin (normally 0-5 V) corresponds percentage-wise to the variable's absolute value between 0 and X (or -X).
    Polarity is set and read by user specific functions
    '''
    def __init__(self, name, range, pinAnalIn, pinPwmOut, SetPolarityPositiveFunc, GetPolarityPositiveFunc, compName='', helpLine='', units='', PreSetFunc=None, highFreqPWM=False, pinOutVoltsMax=5, pinInVoltsMax=5, pinOutVoltsMin=0, pinInVoltsMin=0, PostGetFunc=None, I2cDac=None):
        SysVarAnalogArduino.__init__(self, name, range, pinAnalIn, pinPwmOut, SetPolarityPositiveFunc, GetPolarityPositiveFunc, compName, helpLine, units, PreSetFunc, highFreqPWM, pinOutVoltsMax, pinInVoltsMax, pinOutVoltsMin, pinInVoltsMin, PostGetFunc, I2cDac)

    def GetUnipolarMin(self):
        return 0
    
    def GetUnipolarMax(self):
        # it abs(range[0]) and abs(range[1]) should be equal
        return abs(self.range[1])


class SysCompArduino(SysComp):
    def __init__(self, name, vars, helpLine=''):
        SysComp.__init__(self, name, vars, Arduino, helpLine)
        
    def FirstTimeOnline(self):
        for var in self.vars.values():
            var.FirstTimeOnline()
            
'''
class SysVarPidRelayArduino(SysVarAnalog):
    
    ''
    An Arduino variable that uses the PID Arduino library to turn a relay for the PID control.
    Example: a heating element to be turned on/off to regulate the temperature
    
    It shows an analog variable to set and read the PID controlled quantity (e.g. the temperature) 
    ''
    
    def __init__(self, name, range, pidVar, windowSizeMs, kp, ki, kd, pinAnalIn, pinDigiOut, compName='', helpLine='', units='', PreSetFunc=None, pinInVoltsMax=5, pinInVoltsMin=0, PostGetFunc=None):
        SysVarAnalog.__init__(self, name, range, Arduino, compName, helpLine, True, units, PreSetFunc, PostGetFunc)
        self.pinAnalIn = pinAnalIn
        self.pinDigiOut = pinDigiOut
        self.pinInVoltsMax = pinInVoltsMax
        self.pinInVoltsMin = pinInVoltsMin
        self.range = range
        self.pidVar = pidVar
        self.windowSizeMs = windowSizeMs
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def FirstTimeOnline(self):
        self.GetController().PidRelayCreate(self.pidVar, self.pinAnalIn, self.pinDigiOut, self.windowSizeMs, self.kp, self.ki, self.kd)
    
    def GetFunc(self):
        fraction = self.GetController().AnalogReadFraction(self.pinAnalIn, self.pinInVoltsMax, self.pinInVoltsMin)
        return (self.range[0] + (self.range[1] - self.range[0]) * fraction) if fraction != None else None
    
    def SetFunc(self, value):
        # write the setpoint in the range of the analog input pin
        fraction = (value - self.range[0]) / (self.range[1] - self.range[0])
        voltage = self.pinInVoltsMin + fraction * (self.pinInVoltsMax - self.pinInVoltsMin)
        analInCompatibleValue = voltage / Arduino.PIN_VOLT_MAX * Arduino.ANAL_IN_VAL_MAX
        self.GetController().PidRelaySet(self.pidVar, analInCompatibleValue)
        
    def Enable(self, enable):
        self.GetController().PidRelayEnable(self.pidVar, enable)
        
    def Tune(self, kp, ki, kd):
        self.GetController().PidRelayTune(self.pidVar, kp, ki, kd)
  '''

class SysVarPidRelayArduino(SysVarAnalog):
    '''
    An Arduino variable that uses the PID Arduino library to turn a relay for the PID control.
    Example: a heating element to be turned on/off to regulate the temperature
    
    It shows an analog variable to set and read the PID controlled quantity (e.g. the temperature) 
    '''
    def __init__(self, name, range, pidVar, windowSizeMs, kp, ki, kd, pinAnalIn, pinDigiOut, compName='', helpLine='', units='', PreSetFunc=None, pinInVoltsMax=5, pinInVoltsMin=0, PostGetFunc=None):
        SysVarAnalog.__init__(self, name, range, Arduino, compName, helpLine, True, units, PreSetFunc, PostGetFunc)
        self.pinAnalIn = pinAnalIn
        self.pinDigiOut = pinDigiOut
        self.pinInVoltsMax = pinInVoltsMax
        self.pinInVoltsMin = pinInVoltsMin
        self.range = range
        self.pidVar = pidVar
        self.windowSizeMs = windowSizeMs
        self.kp = kp
        self.ki = ki
        self.kd = kd
        Rfix = [9900, 9850, 9920, 9850, 9980, 9940] 
        self.Rfix = Rfix[pinAnalIn]
        
        
    def FirstTimeOnline(self):
        self.GetController().PidRelayCreate(self.pidVar, self.pinAnalIn, self.pinDigiOut, self.windowSizeMs, self.kp, self.ki, self.kd)
        self.varis = self.GetController().thermistorVars(self.pinAnalIn)
        self.A = self.varis[0]
        self.B = self.varis[1]
        self.C = self.varis[2]
        print("First time online")
        
    def GetFunc(self):
        
        fraction = self.GetController().AnalogReadFraction(self.pinAnalIn, self.pinInVoltsMax, self.pinInVoltsMin)
        return (self.range[0] + (self.range[1] - self.range[0]) * fraction) if fraction != None else None
        
        #calculate temperature
        res = self.GetController().AnalogReadMultiRes(self.pinAnalIn)
        
        res = (res /(res+ self.Rfix)) * 1023 
        
        R1 = (np.log(res)) if res != None else 0
        R3 = (R1*R1*R1) if R1 != None else 0

        tmp = self.A+self.B*R1+self.C*R3 if R3 != None else 0
        tmp = (1/tmp)-273.15 if tmp != None and tmp != 0 else 0
        print("Temperature: ")        
        print(tmp)
        
        if np.isnan(tmp):
            return 0
        else:
            return float(tmp) if tmp !=None else 0
         
        return tmp if tmp != None else 0   



            
    def SetFunc(self, value):
        # write the setpoint in the range of the analog input pin
        '''Want to find what analog value would correspond to a given temperature'''
        '''
        Original equations:        
        fraction = (value - self.range[0]) / (self.range[1] - self.range[0])
        voltage = self.pinInVoltsMin + fraction * (self.pinInVoltsMax - self.pinInVoltsMin)
        analInCompatibleValue = voltage / Arduino.PIN_VOLT_MAX * Arduino.ANAL_IN_VAL_MAX
        self.GetController().PidRelaySet(self.pidVar, analInCompatibleValue)
        '''
        #if value is T
        #Calculate desired R from T - find approx R from curve fit
        #based on difference between T desired and T calculated, change R until within a given threshold
       
        R3 = 0
        R = 10000
        if np.isnan(R):
            R=0
        R1 = (np.log(R)) if R != None else 0
        R3 = R1*R1*R1 if R3 != None else 0
        tmp = self.A+self.B*R1+self.C*R3 if R3 != None else 0
        tmp = (1/tmp)-273.15 if tmp != None and tmp != 0 else 0
        
        while abs(tmp-value)>0.5:

            if abs(tmp-value) > 4:
                deltaR = 400
            else:
                deltaR = 100

            if tmp < value:
                R = R+deltaR
            elif tmp > value:
                R = R-deltaR
            if R==0:
                R=5

            R1 = float(np.log(np.float(R))) if R != None else 0
            R3 = R1*R1*R1 if R3 != None else 0

            tmp = self.A+self.B*R1+self.C*R3 if R3 != None else 0
            tmp = (1/tmp)-273.15 if tmp != None and tmp != 0 else 0  
            print("Loop")            
            print(value)
            print(tmp)
            print(R)

 
        analInCompatibleValue = (R /(R+ self.Rfix)) * 1023
        

        print analInCompatibleValue


        self.GetController().PidRelaySet(self.pidVar, analInCompatibleValue)
        
        #based on difference between setTemp and temp, find amount of time t to activate heater
        
    def Enable(self, enable):
        
        self.GetController().PidRelayEnable(self.pidVar, enable)
        
    def Tune(self, kp, ki, kd):
        self.GetController().PidRelayTune(self.pidVar, kp, ki, kd)
            
          