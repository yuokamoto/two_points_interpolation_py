#!/usr/bin/env python3
"""
Example script for constant jerk interpolation
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import matplotlib.pyplot as plt
from two_point_interpolation.constant_jerk import TwoPointInterpolationJerk


ps = 5.5
pe = 100.0

jmax = 0.98
amax = 1
vmax = 5

t0 = 0.5
dt = 0.001
max = [vmax,amax,jmax]

interp = TwoPointInterpolationJerk()
interp.set(ps,pe,max)
interp.set_initial_time(t0)

te = interp.calc_trajectory()

#for visualization
tref = np.arange(t0,t0+te,dt)
pos=np.zeros((tref.size,3))
vel=np.zeros((tref.size,3))
acc=np.zeros((tref.size,3))
jer=np.zeros((tref.size,3))

# for i in range(0,3,1):
i = 0
for j in range(0,tref.size):
    pos[j][i], vel[j][i], acc[j][i], jer[j][i]     = interp.get_point(tref[j])

plt.subplot(4,1,1)
plt.plot(tref,jer[:,0])
plt.ylabel('jerk[m/s^3]')

plt.subplot(4,1,2)
plt.plot(tref,acc[:,0],'r')
plt.plot(tref,np.cumsum(jer[:,0])*dt,"--")
plt.ylabel('acc[m/s^2]')

plt.subplot(4,1,3)
plt.plot(tref,vel[:,0],'r')
plt.plot(tref,np.cumsum(acc[:,0])*dt,"--")
plt.ylabel('vel[m/s]')

plt.subplot(4,1,4)
plt.plot(tref,pos[:,0],'r')
plt.plot(tref,np.cumsum(vel[:,0])*dt+ps,"--")
plt.ylabel('pos[m]')
plt.xlabel('t[s]')
plt.show()
