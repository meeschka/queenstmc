# -*- coding: utf-8 -*-
"""
Created on Fri Jun 12 10:42:32 2015

@author: Queens
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
'''

def deriv(y, t):
    yprime = np.array([3.5*y[0]])
    return yprime
time = np.linspace(0, 1, 1000)
y0 = np.array([10])

y=integrate.odeint(deriv, y0, time)
plt.plot(time, y[:])
plt.show()
'''
'''
def a(p, t):
    pprime = np.array([(36-p[0])/5.])
    return pprime
    
times = np.linspace(0, 30, 1000)
p0 = np.array([20])

p = integrate.odeint(a, p0, times)

plt.plot(times, p[:])
plt.show()

'''
'''
t1 = np.arange(0.0, 5.0, 0.1)
c = 10
pset = 30
p = 5
error = pset-p



plt.figure(1)
plt.subplot(211)
plt.plot(t1, p)
'''

'''
double Input, Output, Setpoint, errSum, lastErr kp, ki, kd;
unsigned long lastTIme
'''
'''
t = 
setpoint = 50
output = 0 if t<20 else 50
'''