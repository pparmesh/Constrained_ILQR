import numpy as np 
import math
import pdb
from scipy.optimize import fmin_cobyla

from ilqr.obstacles import Obstacle

class Constraints:
	def __init__(self, args, obstacle_bb):
		self.args = args
		self.control_cost = np.array([[self.args.w_acc,                   0],
									  [              0, self.args.w_yawrate]])

		self.state_cost = np.array([[self.args.w_pos, 0, 0, 0],
									[0, self.args.w_pos, 0, 0],
									[0, 0, self.args.w_vel, 0],
									[0, 0, 0,               0]])
		self.coeffs = None

		self.number_of_npc = 1 # hardcode

		self.obs_constraints = {}
		for i in range(self.number_of_npc):
			self.obs_constraints[i] = Obstacle(args, i, obstacle_bb)

	def get_state_cost_derivatives(self, state, poly_coeffs, x_local_plan, npc_traj):
		"""
		Returns the first order and second order derivative of the value function wrt state
		"""
		l_x = np.zeros((self.args.num_states, self.args.horizon))
		l_xx = np.zeros((self.args.num_states, self.args.num_states, self.args.horizon))
		c_state = 0
		for i in range(self.args.horizon):
			# Offset in path derivative
			x_r, y_r = self.find_closest_point(state[:, i], poly_coeffs, x_local_plan)
			traj_cost = 2*self.state_cost@(np.array([state[0, i]-x_r, state[1, i]-y_r, state[2, i]-self.args.desired_speed, 0]))

			# Compute first order derivative
			l_x_i = traj_cost

			# Compute second order derivative
			l_xx_i = 2*self.state_cost

			# Obstacle derivative
			for j in range(self.number_of_npc):
				# pdb.set_trace()
				b_dot_obs, b_ddot_obs = self.obs_constraints[j].get_obstacle_cost_derivatives(npc_traj, i, state[:, i])
				# b_dot_obs = np.array([0, 0, 0, 0])
				# b_ddot_obs = np.zeros((4,4))

				l_x_i += b_dot_obs.squeeze()
				l_xx_i += b_ddot_obs
				# print(b_dot_obs)

			l_xx[:, :, i] = l_xx_i
			l_x[:, i] = l_x_i
			# Calulate total state cost
			# ref_state = np.array([x_r, y_r, self.args.desired_speed, 0]) # Theta does not matter
			# state_diff = state[:,i]-ref_state
			# c_state = c_state + state_diff.T @ self.state_cost @ state_diff
		# pdb.set_trace()
		return l_x, l_xx

	def get_control_cost_derivatives(self, state, control):
		"""
		Returns the control quadratic (R matrix) and linear cost term (r vector) for the trajectory
		"""
		P1 = np.array([[1],[0]])
		P2 = np.array([[0],[1]])

		l_u = np.zeros((self.args.num_ctrls, self.args.horizon))
		l_uu = np.zeros((self.args.num_ctrls, self.args.num_ctrls, self.args.horizon))
		# c_ctrl = 0
		for i in range(self.args.horizon):
			# Acceleration Barrier Max
			c = (np.matmul(control[:, i].T, P1) - self.args.acc_limits[1])
			b_1, b_dot_1, b_ddot_1 = self.barrier_function(self.args.q1_acc, self.args.q2_acc, c, P1)

			# Acceleration Barrier Min
			c = (self.args.acc_limits[0] - np.matmul(control[:, i].T, P1))
			b_2, b_dot_2, b_ddot_2 = self.barrier_function(self.args.q1_acc, self.args.q2_acc, c, -P1)

			velocity = state[2, i]

			# Yawrate Barrier Max
			c = (np.matmul(control[:, i].T, P2) - velocity*math.tan(self.args.steer_angle_limits[1])/self.args.wheelbase)
			b_3, b_dot_3, b_ddot_3 = self.barrier_function(self.args.q1_yawrate, self.args.q2_yawrate, c, P2)

			# Yawrate Barrier Min
			c = (velocity*math.tan(self.args.steer_angle_limits[0])/self.args.wheelbase - np.matmul(control[:, i].T, P2))
			b_4, b_dot_4, b_ddot_4 = self.barrier_function(self.args.q1_yawrate, self.args.q2_yawrate, c, -P2)

			l_u_i = b_dot_1 + b_dot_2 + b_dot_3 + b_dot_4 + (2*control[:, i].T @ self.control_cost).reshape(-1, 1)
			l_uu_i = b_ddot_1 + b_ddot_2 + b_ddot_3 + b_ddot_4 + 2*self.control_cost

			l_u[:, i] = l_u_i.squeeze()
			l_uu[:, :, i] = l_uu_i.squeeze()

			# Calulate total control cost
			# c_ctrl = c_ctrl + control[:,i].T @ self.control_cost @ control[:,i]

		return l_u, l_uu

	def barrier_function(self, q1, q2, c, c_dot):
		b = q1*np.exp(q2*c)
		b_dot = q1*q2*np.exp(q2*c)*c_dot
		b_ddot = q1*(q2**2)*np.exp(q2*c)*np.matmul(c_dot, c_dot.T)

		return b, b_dot, b_ddot

	def get_cost_derivatives(self, state, control, poly_coeffs, x_local_plan, npc_traj):
		"""
		Returns the different cost terms for the trajectory
		This is the main function which calls all the other functions 
		"""
		self.state = state
		# pdb.set_trace()
		l_u, l_uu = self.get_control_cost_derivatives(state, control)
		l_x, l_xx = self.get_state_cost_derivatives(state, poly_coeffs, x_local_plan, npc_traj)
		l_ux = np.zeros((self.args.num_ctrls, self.args.num_states, self.args.horizon))
		# l = c_state + c_ctrl

		return l_x, l_xx, l_u, l_uu, l_ux

	def get_total_cost(self, state, control_seq, poly_coeffs, x_local_plan, npc_traj):
		"""
		Returns cost of a sequence
		"""
		J = 0
		for i in range(self.args.horizon):
			x_r, y_r = self.find_closest_point(state[:, i], poly_coeffs, x_local_plan)
			ref_state = np.array([x_r, y_r, self.args.desired_speed, 0]) # Theta does not matter
			state_diff = state[:,i]-ref_state

			c_state = state_diff.T @ self.state_cost @ state_diff
			c_ctrl = control_seq[:,i].T @ self.control_cost @ control_seq[:,i]

			J = J + c_state + c_ctrl
		return J

	def find_closest_point(self, state, coeffs, x_local_plan):
		new_x = np.linspace(x_local_plan[0], x_local_plan[-1], num=10*self.args.number_of_local_wpts)
		new_y = np.polyval(np.poly1d(coeffs), new_x)
		local_plan = np.vstack((new_x, new_y)).T

		closest_ind = np.sum((local_plan - [state[0], state[1]])**2, axis=1)
		min_i = np.argmin(closest_ind)
		
		return local_plan[min_i, :]











	# def get_acceleration_cost(self): 
	# 	return np.matmul(np.matmul(self.args.w_acc*self.control.T*np.array([[1,0],[0,0]]))*self.control)

	# def get_yawrate_cost(self):
	# 	return np.mamtul(np.matmul(self.args.w_acc*self.control.T*np.array([[0,0],[0,1]]))*self.control)

	def desired_pose_function(self, x):
		return np.polyval(self.coeffs,x)

	def offset_obj(self, X):
		x,y = X
		return np.sqrt((x - self.state[0])**2 + (y - self.state[1])**2)

	def c1(self, X):
		x,y = X
		return self.desired_pose_function(x) - y

	def get_offset_cost(self):
		# Get closest point from the curve
		X = fmin_cobyla(offset_obj, x0=[self.state[0], self.state[1]], cons=[c1])
		x_r, y_r = X
		state_diff = np.array([state[0]-x_r, state[1]-y_r])
		Qk = np.array([[1,0,0],[0,1,0],[0,0,self.args.w_vel]])

		return np.matmul(np.matmul(state_diff.T*Q),state_diff)

	def get_velocity_cost(self, current_speed):
		return self.args.w_vel*(abs(current_speed - self.desired_speed))

