import numpy as np 
import math
import pdb

class Obstacle:
    def __init__(self, args, track_id, bb):
        self.args = args
        self.car_length = bb[0]
        self.car_width = bb[1]
        self.track_id = track_id

    def get_obstacle_cost_derivatives(self, npc_traj, i, ego_state):

        a = self.car_length + np.abs(npc_traj[2, i]*math.cos(npc_traj[3, i]))*self.args.t_safe + self.args.s_safe_a + self.args.ego_rad
        b = self.car_width + np.abs(npc_traj[2, i]*math.sin(npc_traj[3, i]))*self.args.t_safe + self.args.s_safe_b + self.args.ego_rad
        
        P1 = np.diag([1/a**2, 1/b**2, 0, 0])

        theta = npc_traj[3, i]
        theta_ego = ego_state[3]

        transformation_matrix = np.array([[ math.cos(theta), math.sin(theta), 0, 0],
                                          [-math.sin(theta), math.cos(theta), 0, 0],
                                          [               0,               0, 0, 0],
                                          [               0,               0, 0, 0]])
        
        ego_front = ego_state + np.array([math.cos(theta_ego)*self.args.ego_lf, math.sin(theta_ego)*self.args.ego_lf, 0, 0])
        diff = (transformation_matrix @ (ego_front - npc_traj[:, i])).reshape(-1, 1) # (x- xo)
        c = 1 - diff.T @ P1 @ diff # Transform into a constraint function
        c_dot = -2 * P1 @ diff
        b_f, b_dot_f, b_ddot_f = self.barrier_function(self.args.q1_front, self.args.q2_front, c, c_dot)

        ego_rear = ego_state - np.array([math.cos(theta_ego)*self.args.ego_lr, math.sin(theta_ego)*self.args.ego_lr, 0, 0])
        diff = (transformation_matrix @ (ego_rear - npc_traj[:, i])).reshape(-1, 1)
        c = 1 - diff.T @ P1 @ diff
        c_dot = -2 * P1 @ diff
        b_r, b_dot_r, b_ddot_r = self.barrier_function(self.args.q1_rear, self.args.q2_rear, c, c_dot)

        return b_dot_f + b_dot_r, b_ddot_f + b_ddot_r

    def get_obstacle_cost(self, npc_traj, i, ego_state_nominal, ego_state):
        a = self.car_length + np.abs(npc_traj[2, i]*math.cos(npc_traj[3, i]))*self.args.t_safe + self.args.s_safe_a + self.args.ego_rad
        b = self.car_width + np.abs(npc_traj[2, i]*math.sin(npc_traj[3, i]))*self.args.t_safe + self.args.s_safe_b + self.args.ego_rad
        
        P1 = np.diag([1/a**2, 1/b**2, 0, 0])

        theta = npc_traj[3, i]
        theta_ego = ego_state[3]
        theta_ego_nominal = ego_state_nominal[3]


        transformation_matrix = np.array([[ math.cos(theta), math.sin(theta), 0, 0],
                                          [-math.sin(theta), math.cos(theta), 0, 0],
                                          [               0,               0, 0, 0],
                                          [               0,               0, 0, 0]])
        
        # front circle
        ego_front_nominal = ego_state_nominal + np.array([math.cos(theta_ego)*self.args.ego_lf, math.sin(theta_ego)*self.args.ego_lf, 0, 0])
        ego_front = ego_state + np.array([math.cos(theta_ego_nominal)*self.args.ego_lf, math.sin(theta_ego_nominal)*self.args.ego_lf, 0, 0])

        x_del = ego_front - ego_front_nominal

        diff = (transformation_matrix @ (ego_front_nominal - npc_traj[:, i])).reshape(-1, 1)
        c = 1 - diff.T @ P1 @ diff
        c_dot = -2 * P1 @ diff
        b_f, b_dot_f, b_ddot_f = self.barrier_function(self.args.q1_front, self.args.q2_front, c, c_dot)

        cost = b_f + x_del.T @ b_dot_f + x_del.T @ b_ddot_f @ x_del  

        # rear circle
        ego_rear_nominal = ego_state_nominal - np.array([math.cos(theta_ego)*self.args.ego_lr, math.sin(theta_ego)*self.args.ego_lr, 0, 0])
        ego_rear = ego_state - np.array([math.cos(theta_ego_nominal)*self.args.ego_lr, math.sin(theta_ego_nominal)*self.args.ego_lr, 0, 0])

        x_del = ego_rear - ego_rear_nominal

        diff = (transformation_matrix @ (ego_rear_normalized - npc_traj[:, i])).reshape(-1, 1)
        c = 1 - diff.T @ P1 @ diff
        c_dot = -2 * P1 @ diff
        b_r, b_dot_r, b_ddot_r = self.barrier_function(self.args.q1_rear, self.args.q2_rear, c, c_dot)

        cost += b_r + x_del.T @ b_dot_r + x_del.T @ b_ddot_r @ x_del  

        return cost

    def barrier_function(self, q1, q2, c, c_dot):
        b = q1*np.exp(q2*c)
        b_dot = q1*q2*np.exp(q2*c)*c_dot
        b_ddot = q1*(q2**2)*np.exp(q2*c)*np.matmul(c_dot, c_dot.T)

        return b, b_dot, b_ddot
