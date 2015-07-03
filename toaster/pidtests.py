# -*- coding: utf-8 -*-
"""
Created on Wed Jun 24 16:02:06 2015

@author: Queens
"""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import datetime as dt
'''
data = np.genfromtxt('data1.csv', delimiter=',')

time = data[1287:8040, 0]
y = data[1287:8040, 1]

#plt.plot(time, y)
#plt.show()

'''
df = pd.read_csv('data1.csv', parse_dates=[0], usecols=[0, 1])
#time = df.time
#time = pd.to_datetime(df.time)
#df.index = pd.to_datetime(df.time,format='%m-%s')

start = dt.datetime(year=2015,month=6,day=29)
time=df['time'][1287:8040]
#floattime = map(float, time)
#time = time.str.split(':')
#time = time[0]*60+time[1]
def time_convert(x):
    year = 2015
    month = 06
    day = 29
    hour = 15
    times = x.split(':')
    a = times[1].split('.')
    times = (times[0], a[0], (a[1]*1000))
    print times
    time_string = '%4.4d %2.2d %2.2d %2.2d %2.2d %2.2d %2.2d' % (int(year), int(month), int(day), int(hour), int(times[0]), int(times[1]), int(times[2]))
    return dt.datetime.strptime(time_string, "%Y %m %d %H %M %S %f")
    #return (60*float(times[0])+float(times[1]))
#time = time.apply(time_convert)

temp = df.temp1[1287:8040]

#plt.plot(time, temp)
#plt.show

