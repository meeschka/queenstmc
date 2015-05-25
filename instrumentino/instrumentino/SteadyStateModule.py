from __future__ import division
from instrumentino.comp import SysVarDigital
import csv
from instrumentino import cfg

class SteadyStateClass(SysVarDigital):
    def __init__(self, name, states, controllerClass):
        self.vars= {0:self}
        self.units=''
        SysVarDigital.__init__(self, name, states, controllerClass, 'Steady State')
        self.temps = []                         #list for all temperatures, updated one row at a time
        self.allowance = float(0.5)                    # +/- acceptable temperature range from initial point  
        self.timeperiod = 15                    #time period of steady temperatures to be considered a steady state         
        self.tindex = -1                         #increases to 1 by first iteration to avoid checking headers
        self.colswithtime = [0,1,4,7,8,9,10]    #list of columns of time and temperatures   
        self.columns = [1,4,7,8,9,10]           #list of columns of temperatures  
                
    def Enable(self,x):     #Errors happen when this function doesn't exist
        pass
    
    def SetFunc(self,state):
        pass
    
    def getSeconds(self, s):        #returns time in seconds given hh:mm:ss format
        return sum(int(float(x)) * 60 ** i for i,x in enumerate(reversed(s.split(":"))))
        
    def LoadOneRow(self):   #adds one row of temperatures to temps list
            self.row = []                           #empty list used for current row                
            for c in self.colswithtime:             #make list of temperatures at current index                    
                self.row.append(self.AllData[len(self.temps)][c])
                
            self.temps.append(self.row)      #add current row list to end of temperatures
                
    def GetFunc(self):
        self.AllData = list(csv.reader(open(cfg.LogPath(cfg.timeNow + '.csv'), 'rU'), csv.QUOTE_NONE))   #this updates in chunks     

        while (len(self.AllData) > len(self.temps)):            #if there is data that has not been put into temps list
            self.LoadOneRow()                                   #load one row at a time
            self.tindex = self.tindex + 1                       #increase index
            
            for idx in reversed(range(1,self.tindex)):          #from index back to beginning
                for c in range(1,7):                                #check all columns except time
                    diff = abs(float(self.temps[self.tindex][c])-float(self.temps[idx][c])) #difference between current row index and most recently added index
    
                    if diff <= self.allowance:                  #if difference is small enough, go to next column
                        continue
                    
                    else:       #otherwise chain was broken
                        timedelta = abs(self.getSeconds(self.temps[idx][0]) - self.getSeconds(self.temps[self.tindex][0]))
                        
                        if  timedelta >= self.timeperiod:       #if time difference is big enough
                            return 'Steady'                     #temperatures are steady
                        
                        else:       #otherwise, it was not long enough.
                            return 'Varying'    #temperatures vary too much
                      