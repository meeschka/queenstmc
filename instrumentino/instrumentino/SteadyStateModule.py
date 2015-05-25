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
        self.Broken = False                     #variable for if steady state conditions are broken
        self.tindex = 0                         #increases to 1 by first iteration to avoid checking headers
        self.colswithtime = [0,3,4,6,8,9,11]
        #self.colswithtime = [0,4,5,7,8,9,11]    #list of columns of time and temperatures
        self.columns = [3,4,6,8,9,11]        
        #self.columns = [4,5,7,8,9,11]           #list of columns of temperatures
        self.AllData = []                       #list for all imported data                        
        
    def Enable(self,x):     #Errors happen when this function doesn't exist
        pass
    
    def SetFunc(self,state):
        pass
    
    def getSeconds(self, s):
        return sum(int(float(x)) * 60 ** i for i,x in enumerate(reversed(s.split(":"))))
        
    def LoadOneRow(self):
            self.row = []                               #empty list used for current row                
            for c in self.colswithtime:                      #make list of temperatures at current index                    
                self.row.append(self.AllData[len(self.temps)][c])
                
            self.temps.append(self.row)      #add current row list to end of all data
                
    def GetFunc(self):
        self.AllData = list(csv.reader(open(cfg.LogPath(cfg.timeNow + '.csv'), 'rU'), csv.QUOTE_NONE))   #this updates in chunks     

        if len(self.AllData) == 1 and len(self.temps) == 0:     #special case to load headers
            self.LoadOneRow()
                             
        while (len(self.AllData) > len(self.temps)):            #if there is data that has not been put into temps list
            self.LoadOneRow()                                   #load one row at a time
            self.tindex = self.tindex + 1                       #increase index
            print "Tindex: %d" %self.tindex
            
            for idx in reversed(range(1,self.tindex)):          #from index back to beginning
                self.Broken = False                             #initially not broker
                for c in range(1,7):                            #check all columns
                    diff = abs(float(self.temps[self.tindex][c])-float(self.temps[idx][c])) #difference between current row index and most recently added index
    
                    if diff <= self.allowance:                  #if difference is small enough, go to next column
                        continue
                    
                    else:       #otherwise chain was broken
                                    #if the chain was held for long enough, it was a steady state
                        timedelta = abs(self.getSeconds(self.temps[idx][0]) - self.getSeconds(self.temps[self.tindex][0]))
                        if  timedelta >= self.timeperiod:        # if time difference is big enough
                            print "Steady from %d to %d" %(idx,self.tindex)
                            return 'Steady'
                        
                        else:       #otherwise, it was not long enough. exit out of both for loops to increase max index by one
                            print "Broken (%f) or too short (%d)" %(diff,timedelta)
                            print "Broken at Row: %d   for row: %d   Column: %d" %(idx,self.tindex,c)
                            self.Broken = True                  #this variable is used to break out of outer for loop
                            return 'Varying'
#                            break
                print "row %d good" %idx
#                    
#                if self.Broken == True:     #if chain broken, exit for loop
#                    break                           