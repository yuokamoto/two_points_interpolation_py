import numpy as np
'''

d^3x/dt^3

    ^
    |
    |
 max|--------            ----------
    |       |            |
    |       |            |
 --------------------------------->
            | t1         |3t1
            |            | 
 min        --------------


*assupiton v0 = 0
*jerk is constant

'''

class TwoPointInterpolation(object):
	def __init__(self):
		self.point_setted = False
		self.constraints_setted = False
		self.trajectory_calced = False

	def set_initial_time(self,time):
		'''set start time'''
		self.t0 = time

	def set_point(self,ps,pe):
		'''set start point and end point'''
		self.ps = ps
		self.pe = pe
		self.point_setted = True;

	def set_constraints(self,max):
		'''set max'''
		self.jmax = max[2]
		self.amax = max[1]
		self.vmax = max[0]
		self.constraints_setted = True;

	def set(self,ps,pe,max):
		'''set point and constraints'''
		self.set_point(ps,pe);
		self.set_constraints(max)

	def calc_trajectory(self):
		'''calc_trajectory'''
		self.t1 = np.power( np.fabs(self.pe-self.ps)/2.0/self.jmax, 1/3.0 )
		self.te = 0.0

		if self.t1*self.jmax < self.amax: #not hit acc limit
			if self.t1*self.jmax*self.t1 < self.vmax: #not hit v limit
				self.case = 0
				self.te = 4*self.t1
			else: #hit v limit
				self.case = 1
				self.t1 = np.sqrt(self.vmax/self.jmax)
				self.t2 = np.fabs(self.pe-self.ps)/self.vmax - 2.0*np.sqrt(self.vmax/self.jmax)
				self.te =4*self.t1+self.t2
		else: #hit acc limit
			self.t1 = self.amax/self.jmax
			self.t2 = - 1.5*self.t1 + np.sqrt( 4*np.fabs(self.pe-self.ps)/self.amax + 1.0/3.0*self.t1**2 )/2.0
			if (self.t1+self.t2)*self.amax < self.vmax: #not hit v limit
				self.case = 2
				self.te = 4*self.t1 + 2*self.t2
			else: #hit v limit
				self.case = 3
				self.t1 = self.amax/self.jmax
				self.t2 = self.vmax/self.amax - self.t1
				self.t3 = np.fabs(self.pe-self.ps)/self.vmax - 2*self.t1 - self.t2
				self.te = 4*self.t1+2*self.t2+self.t3
			
		print("case", self.case, self.te)

		self.trajectory_calced = True

		return self.te

		
	def get_point(self,t):
		'''get pos,vel,acc depends on time t'''
		j = 0
		a = 0
		v = 0
		p = 0

		tau = t-self.t0
		jmax = self.jmax
		amax = self.amax
		vmax = self.vmax
		t1 = self.t1

		if self.case == 0 :
			if tau < 0:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.ps
			elif tau < 4*t1:
				if tau<t1:
					j = jmax
					a = jmax*tau
					v = 0.5*jmax*tau**2
					p = 1.0/6.0*jmax*tau**3

				elif tau<3*t1:
					tau1 = tau - t1
					j = -jmax
					a = -jmax*tau1 + jmax*t1
					v = -0.5*jmax*tau1**2 + jmax*t1*tau1 + 0.5*jmax*t1**2
					p = -1.0/6.0*jmax*tau1**3 + 0.5*jmax*t1*tau1**2 + 0.5*jmax*t1**2*tau1 + 1.0/6.0*jmax*t1**3

				else: #tau<4*t1:
					tau3 = tau - 3*t1
					j = jmax
					a = jmax*tau3 - jmax*t1
					v = 0.5*jmax*tau3**2 - jmax*t1*tau3 + 0.5*jmax*t1**2
					p = 1.0/6.0*jmax*tau3**3 - 0.5*jmax*t1*tau3**2 + 0.5*jmax*t1**2*tau3 + 11.0/6.0*jmax*t1**3

				if self.pe < self.ps:
					j = -j
					a = -a
					v = -v
					p = -p

				p += self.ps

			else: #tau > 4*t1:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.pe

		elif self.case == 1 :
			t2 = self.t2
			if tau < 0:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.ps
			elif tau < 4*t1+t2:
				if tau<t1:
					j = jmax
					a = jmax*tau
					v = 0.5*jmax*tau**2
					p = 1.0/6.0*jmax*tau**3

				elif tau<2*t1:
					tau1 = tau - t1
					j = -jmax
					a = -jmax*tau1+jmax*t1
					v = -0.5*jmax*tau1**2 + jmax*t1*tau1 + 0.5*jmax*t1**2
					p = -1.0/6.0*jmax*tau1**3 + 0.5*jmax*t1*tau1**2 + 0.5*jmax*t1**2*tau1 + 1.0/6.0*jmax*t1**3

				elif tau < 2*t1+t2:
					tau2 = tau - 2*t1
					j = 0
					a = 0
					v = vmax
					p = vmax*tau2+jmax*t1**3

				elif tau < 3*t1+t2:
					tau3 = tau - 2*t1 - t2
					j = -jmax
					a = -jmax*tau3
					v = -0.5*jmax*tau3**2+vmax
					p = -1.0/6.0*jmax*tau3**3 + vmax*tau3 + vmax*t2 + jmax*t1**3
 
				else: 
					tau4 = tau - 3*t1 - t2
					j = jmax
					a = jmax*tau4 - jmax*t1
					v = 0.5*jmax*tau4**2 - jmax*t1*tau4 - 0.5*jmax*t1**2+vmax
					p = 1.0/6.0*jmax*tau4**3 - 0.5*jmax*t1*tau4**2 + 0.5*jmax*t1**2*tau4 + vmax*t2 + vmax*t1 + 5.0/6.0*jmax*t1**3

				if self.pe < self.ps:
					j = -j
					a = -a
					v = -v
					p = -p

				p += self.ps

			else: #tau > 4*t1:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.pe


		elif self.case == 2 :
			t2 = self.t2
			if tau < 0:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.ps
			elif tau < 4*t1+2*t2:
				if tau<t1:
					j = jmax
					a = jmax*tau
					v = 0.5*jmax*tau**2
					p = 1.0/6.0*jmax*tau**3

				elif tau<t1+t2:
					tau1 = tau - t1
					j = 0
					a = amax
					v = amax*tau1 + 0.5*amax**2/jmax
					p = 0.5*amax*tau1**2 + 0.5*amax**2/jmax*tau1 + 1.0/6.0*amax**3/jmax**2

				elif tau < 3*t1+t2:
					tau2 = tau - t1 - t2
					j = -jmax
					a = -jmax*tau2 + amax
					v = -0.5*jmax*tau2**2 + amax*tau2 + amax*t2 + 0.5*amax**2/jmax
					p = -1.0/6.0*jmax*tau2**3 + 0.5*amax*tau2**2 + ( amax*t2+0.5*amax**2/jmax )*tau2 + 0.5*amax*t2**2 + 0.5*amax**2/jmax*t2 + 1.0/6.0*amax**3/jmax**2

				elif tau < 3*t1+2*t2:
					tau3 = tau - 3*t1 - t2
					j = 0
					a = -amax
					v = -amax*tau3 + amax*t2 + 0.5*amax**2/jmax
					p = -0.5*amax*tau3**2 + (amax*t2+0.5*amax**2/jmax)*tau3 + 0.5*amax*t2**2 + 2.5*amax**2/jmax*t2 + 2.0*amax**3/jmax**2
 
				else: 
					tau4 = tau - 3*t1 - 2*t2
					j = jmax
					a = jmax*tau4 - amax
					v = 0.5*jmax*tau4**2 - amax*tau4 + 0.5*amax**2/jmax
					p = 1.0/6.0*jmax*tau4**3 - 0.5*amax*tau4**2 + 0.5*amax**2/jmax*tau4 + amax*t2**2 + 3*amax**2/jmax*t2 + 2.0*amax**3/jmax**2

				if self.pe < self.ps:
					j = -j
					a = -a
					v = -v
					p = -p

				p += self.ps

			else: #tau > 4*t1:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.pe

		elif self.case == 3 :
			t2 = self.t2
			t3 = self.t3
			if tau < 0:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.ps
			elif tau < 4*t1+2*2*t2+t3:
				if tau<t1:
					j = jmax
					a = jmax*tau
					v = 0.5*jmax*tau**2
					p = 1.0/6.0*jmax*tau**3

				elif tau<t1+t2:
					tau1 = tau - t1
					j = 0
					a = amax
					v = amax*tau1 + 0.5*amax**2/jmax
					p = 0.5*amax*tau1**2 + 0.5*amax**2/jmax*tau1 + 1.0/6.0*amax**3/jmax**2

				elif tau < 2*t1+t2:
					tau2 = tau - t1 - t2
					j = -jmax
					a = -jmax*tau2 + amax
					v = -0.5*jmax*tau2**2 + amax*tau2 + amax*t2 + 0.5*amax**2/jmax
					p = -1.0/6.0*jmax*tau2**3 + 0.5*amax*tau2**2 + ( amax*t2+0.5*amax**2/jmax )*tau2 + 0.5*amax*t2**2 + 0.5*amax**2/jmax*t2 + 1.0/6.0*amax**3/jmax**2

				elif tau < 2*t1+t2+t3:
					tau3 = tau - 2*t1 - t2
					j = 0
					a = 0
					v = vmax
					p = vmax*tau3 + vmax*t1 + 0.5*vmax*t2

				elif tau < 3*t1+t2+t3:
					tau4 = tau - 2*t1 - t2 - t3
					j = -jmax
					a = -jmax*tau4
					v = -0.5*jmax*tau4**2 + vmax
					p = -1.0/6.0*jmax*tau4**3 + vmax*tau4 + vmax*(t1+0.5*t2+t3)

				elif tau< 3*t1+2*t2+t3:
					tau5 = tau - 3*t1 - t2 - t3
					j = 0
					a = -amax
					v = -amax*tau5 - 0.5*amax**2/jmax + vmax
					p = -0.5*amax*tau5**2 + (vmax-0.5*amax**2/jmax)*tau5 + vmax*(2.0*t1+0.5*t2+t3) - 1.0/6.0*amax**3/jmax**2

				else: 
					tau6 = tau - 3*t1 - 2*t2 - t3
					j = jmax
					a = jmax*tau6 - amax
					v = 0.5*jmax*tau6**2 - amax*tau6 + 0.5*amax**2/jmax
					p = 1.0/6.0*jmax*tau6**3 - 0.5*amax*tau6**2 + 0.5*amax**2/jmax*tau6 + vmax*(2.0*t1+t2+t3) - 1.0/6.0*amax**3/jmax**2

				if self.pe < self.ps:
					j = -j
					a = -a
					v = -v
					p = -p

				p += self.ps

			else: #tau > 4*t1:
				j = jmax
				if self.pe < self.ps:
					j = -j
				a = 0.0
				v = 0.0
				p = self.pe

		else:
			pass

		return p,v,a,j
