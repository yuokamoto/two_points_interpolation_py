import numpy as np
import matplotlib.pyplot as plt
import two_point_interpolation_constant_acc as tpi

# constraints
p0 = 10
pe = 90.0

v0 = -5
ve = 1

amax = 1
vmax = 10

t0 = 1.5

# sim condition
dt = 0.001

# calc trajectory
interp = tpi.TwoPointInterpolation()
interp.init(p0, pe, amax, vmax, t0, v0, ve)
te = interp.calc_trajectory()

#for visualization
tref = np.arange(t0,t0+te,dt)
pos=np.zeros((tref.size,3))
vel=np.zeros((tref.size,3))
acc=np.zeros((tref.size,3))

# for i in range(0,3,1):
i = 0
for j in range(0,tref.size):
	pos[j][i], vel[j][i], acc[j][i]	= interp.get_point(tref[j])

plt.subplot(4,1,2)
plt.plot(tref,acc[:,0],'r')
plt.ylabel('acc[m/s^2]')

plt.subplot(4,1,3)
plt.plot(tref,vel[:,0],'r')
plt.plot(tref,np.cumsum(acc[:,0])*dt+v0,"--")
plt.ylabel('vel[m/s]')

plt.subplot(4,1,4)
plt.plot(tref,pos[:,0],'r')
plt.plot(tref,np.cumsum(vel[:,0])*dt+p0,"--")
plt.ylabel('pos[m]')
plt.xlabel('t[s]')
plt.show()