from __future__ import division
from instrumentino.comp import SysVarDigital
import csv
from instrumentino import cfg

class SteadyStateClass(SysVarDigital):
    def __init__(self, name, states, controllerClass):
        self.vars= {0:self}
        self.units=''
        SysVarDigital.__init__(self, name, states, controllerClass, 'Steady State')
        self.temp = 0                         #list for all temperatures, updated one row at a time
        self.allowance = float(0.5)                    # +/- acceptable temperature range from initial point  
        self.timeperiod = 15                    #time period of steady temperatures to be considered a steady state   
        self.index = -1                         #is updated every calculation
        self.Data = []                       #list for all imported data               
        
    def Enable(self,x):     #Errors happen when this function doesn't exist
        pass
    
    def SetFunc(self,state):
        pass
    
    def getSeconds(self, s):        #this function gets seconds from hh:mm:ss format
        return sum(int(float(x)) * 60 ** i for i,x in enumerate(reversed(s.split(":"))))
        
    def GetFunc(self):
        self.Data = list(csv.reader(open(cfg.LogPath(cfg.timeNow + '.csv'), 'rU'), csv.QUOTE_NONE))   #this updates in chunks     

        if len(self.Data) > 0:            #if there is data that has not been put into temps list

            self.index = len(self.Data) - 1     #update index to match length of data

            for idx in reversed(range(1,self.index)):          #from index back to beginning

                    timedelta = abs(self.getSeconds(self.Data[idx][0]) - self.getSeconds(self.Data[self.index][0]))
            if timedelta >= self.timeperiod:       # steady if time difference is big enough
                return 'Steady'
            else:         #otherwise, it was not long enough. 
                return 'Varying'