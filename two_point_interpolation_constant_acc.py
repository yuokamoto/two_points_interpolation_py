import numpy as np
'''
case==0 (not hit the vmax)
d^2x/dt^2

    ^
    |
    |
 max|--------------            
    |             |            
    |             |            
 0--------------------------------->
	 t0			  | t1         te
				  |             
 min              -----------------


case==1 (hit vmax)
d^2x/dt^2

    ^
    |
    |
 max|--------            
    |       |            
    |       |            
 0--------------------------------->
	 t0		 t1       | t2         te
				      |             
 min                  -------------

'''
def v_integ(v0, a, dt):
	return v0 + a * dt

def p_integ(p0, v0, a, dt):
	return p0 + v0 * dt + 0.5 * a * dt**2


class TwoPointInterpolation(object):
	def __init__(self):
		self.point_setted = False
		self.constraints_setted = False
		self.initial_state_setted = False
		self.trajectory_calced = False

	def set_initial(self,t0, p0, v0=0):
		'''set initial state'''
		self.t0 = t0
		self.p0 = p0
		self.v0 = v0
		self.initial_state_setted = True

	def set_point(self, pe, ve=0):
		'''set end state'''
		self.pe = pe
		self.ve = ve
		self.point_setted = True

	def set_constraints(self, amax, vmax):
		'''set constraints'''
		self.amax = amax
		self.vmax = vmax
		self.constraints_setted = True

	def init(self, p0, pe, amax, vmax, t0=0, v0=0, ve=0):
		'''set point and constraints'''
		self.set_initial(t0, p0, v0)
		self.set_point(pe, ve)
		self.set_constraints(amax, vmax)

	def calc_trajectory(self):
		'''calc_trajectory'''
		vmax = self.vmax
		v0 = self.v0
		p0 = self.p0
		t0 = self.t0
		ve = self.ve
		pe = self.pe
		dp = pe - p0
		dv = ve - v0

		self.dt = []
		self.a = []
		self.v = [v0]
		self.p = [p0]

		a_signed = self.a_signed = self.amax * dp/np.fabs(dp)
		b = 2 * v0 / a_signed
		c = (-dv * (ve + v0) * 0.5 - dp) / a_signed
		if b*b - 4*c > 0: # not reach the v max
			dt01 = 0.5 * (-b + np.power(b*b - 4*c, 1/2.0))
			v1 = v_integ(v0, a_signed, dt01)

			if np.fabs(v1) < vmax:
				self.case = 0
				p1 = p_integ(p0, v0, a_signed, dt01)
				dt1e = dt01 - dv / a_signed
				self.dt.append(dt01)
				self.dt.append(dt1e)
				self.a.extend([a_signed, -a_signed])
				self.v.append(v1)
				self.p.append(p1)

			else:
				self.case = 1
				#t01
				v1 = vmax * dp/np.fabs(dp)
				dt01 = np.fabs((v1 - v0) / a_signed)
				p1 = p_integ(p0, v0, a_signed, dt01)
				self.dt.append(dt01)
				self.a.append(a_signed)
				self.v.append(v1)
				self.p.append(p1)
				#t2e
				v2 = v1
				dt2e = - (ve - v2) / a_signed
				dp2e = p_integ(0, v2, -a_signed, dt2e)
				dt12 = (pe - p1 - dp2e) / v1
				#t12
				p2 = pe - dp2e
				self.dt.append(dt12)
				self.dt.append(dt2e)
				self.a.append(0.0)
				self.a.append(-a_signed)
				self.v.append(v2)
				self.p.append(p2)
			
			# self.a.append(0.0)
			# self.v.append(ve)
			# self.p.append(pe)

		else: # any case?
			print("error")
			return -1
			
		print("case", self.case)
		print("dt", self.dt)
		print("a", self.a)
		print("v", self.v)
		print("p", self.p)

		self.trajectory_calced = True

		return sum(self.dt)

		
	def get_point(self,t):
		'''get pos,vel,acc depends on time t'''

		# output
		a = 0
		v = 0
		p = 0

		tau = t-self.t0

		if tau < 0:
			a = 0.0
			v = self.v0
			p = self.p0
		elif tau >= sum(self.dt):
			a = 0.0
			v = self.ve
			p = self.pe
		else:
			a_in = v_in = p_in = 0
			t_in = tau
			for i in range(len(self.dt)):
				dt = sum(self.dt[:i+1])
				if tau <= dt:
					t_in = tau - sum(self.dt[:i])
					a_in = self.a[i]
					v_in = self.v[i]
					p_in = self.p[i]
					break

			a = a_in
			v = v_integ(v_in, a_in, t_in)
			p = p_integ(p_in, v_in, a_in, t_in)

		return p,v,a