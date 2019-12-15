import numpy as np
import pdb

# Add lambda functions
cos = lambda a : np.cos(a)
sin = lambda a : np.sin(a)
tan = lambda a : np.tan(a)

class Model:
    """
    A vehicle model with 4 dof. 
    State - [x, y, vel, theta]
    Control - [acc, yaw_rate]
    """
    def __init__(self, args):
        self.wheelbase = args.wheelbase
        self.steer_min = args.steer_angle_limits[0]
        self.steer_max = args.steer_angle_limits[1]
        self.accel_min = args.acc_limits[0]
        self.accel_max = args.acc_limits[1]
        self.max_speed = args.max_speed
        self.Ts = args.timestep
        self.N = args.horizon
        self.z = np.zeros((self.N))
        self.o = np.ones((self.N))
        
    def forward_simulate(self, state, control):
        """
        Find the next state of the vehicle given the current state and control input
        """
        # Clips the controller values between min and max accel and steer values
        control[0] = np.clip(control[0], self.accel_min, self.accel_max)
        control[1] = np.clip(control[1], state[2]*tan(self.steer_min)/self.wheelbase, state[2]*tan(self.steer_max)/self.wheelbase)
        
        next_state = np.array([state[0] + cos(state[3])*(state[2]*self.Ts + (control[0]*self.Ts**2)/2),
                               state[1] + sin(state[3])*(state[2]*self.Ts + (control[0]*self.Ts**2)/2),
                               np.clip(state[2] + control[0]*self.Ts, 0.0, self.max_speed),
                               state[3] + control[1]*self.Ts])  # wrap angles between 0 and 2*pi - Gave me error
        return next_state

    def get_A_matrix(self, velocity_vals, theta, acceleration_vals):
        """
        Returns the linearized 'A' matrix of the ego vehicle 
        model for all states in backward pass. 
        """
        v = velocity_vals
        v_dot = acceleration_vals
        A = np.array([[self.o, self.z, cos(theta)*self.Ts, -(v*self.Ts + (v_dot*self.Ts**2)/2)*sin(theta)],
                      [self.z, self.o, sin(theta)*self.Ts,  (v*self.Ts + (v_dot*self.Ts**2)/2)*cos(theta)],
                      [self.z, self.z,             self.o,                                         self.z],
                      [self.z, self.z,             self.z,                                         self.o]])
        return A

    def get_B_matrix(self, theta):
        """
        Returns the linearized 'B' matrix of the ego vehicle 
        model for all states in backward pass. 
        """
        B = np.array([[self.Ts**2*cos(theta)/2,         self.z],
                      [self.Ts**2*sin(theta)/2,         self.z],
                      [         self.Ts*self.o,         self.z],
                      [                 self.z, self.Ts*self.o]])
        return B
