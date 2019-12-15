import argparse

def add_arguments(parser):
    # ___________________ Carla Parameters ___________________ #
    parser.add_argument('--add_npc_agents', action="store_true", default=False, help='Should there be NPC agents in the simulator')
    parser.add_argument('--verbose', action="store_true", default=False, help='Show debugging data')
    parser.add_argument('--fps', type= int, default=20, help='Frames per second')

    parser.add_argument('--number_of_npc', type= int, default=10, help='Number of NPC vehicles')
    parser.add_argument('--camera_width', type= int, default=800, help='Width of the image rendered by the cameras')
    parser.add_argument('--camera_height', type= int, default=800, help='Height of the image rendered by the cameras')

    parser.add_argument('--debug_simulator', action="store_true", default=False, help='Use visualizations in simulator for debugging')


    # ___________________ Planning Parameters ___________________ #
    parser.add_argument('--number_of_local_wpts', type= int, default=20, help='Number of local waypoints')
    parser.add_argument('--poly_order', type= int, default=5, help='Order of the polynomial to fit on')
    parser.add_argument('--use_pid', action="store_true", default=False, help='If we want to use PID instead of iLQR')
    parser.add_argument('--desired_speed', type= float, default=5.0, help='Desired Speed')
    parser.add_argument('--use_mpc', action="store_true", default=False, help='To use or not to use (MPC)')
    parser.add_argument('--mpc_horizon', type=int, default=5, help='For how many timesteps to use MPC')


    # ___________________ iLQR Parameters ___________________ #
    parser.add_argument('--timestep', type=float, default=0.1, help='Timestep at which forward and backward pass are done by iLQR')
    parser.add_argument('--horizon', type=int, default=40, help='Planning horizon for iLQR in num of steps (T=horizon*timesteps)')
    parser.add_argument('--tol', type=float, default=1e-4, help='iLQR tolerance parameter for convergence')
    parser.add_argument('--max_iters', type=int, default=20, help='Total number of iterations for iLQR')
    parser.add_argument('--num_states', type=int, default=4, help='Number of states in the model')
    parser.add_argument('--num_ctrls', type=int, default=2, help='Number of control inputs in the model')

    # ___________________ Cost Parameters ___________________ #

    parser.add_argument('--w_acc', type=float, default=1.00, help="Acceleration cost")
    parser.add_argument('--w_yawrate', type=float, default=3.00, help="Yaw rate cost")

    parser.add_argument('--w_pos', type=float, default=2.0, help="Path deviation cost")
    parser.add_argument('--w_vel', type=float, default=0.50, help="Velocity cost")

    parser.add_argument('--q1_acc', type=float, default=1.0, help="Barrier function q1, acc")
    parser.add_argument('--q2_acc', type=float, default=1.0, help="Barrier function q2, acc")

    parser.add_argument('--q1_yawrate', type=float, default=1.00, help="Barrier function q1, yawrate")
    parser.add_argument('--q2_yawrate', type=float, default=1.00, help="Barrier function q2, yawrate")

    parser.add_argument('--q1_front', type=float, default=2.75, help="Barrier function q1, obs with ego front")
    parser.add_argument('--q2_front', type=float, default=2.75, help="Barrier function q2, obs with ego front")

    parser.add_argument('--q1_rear', type=float, default=2.5, help="Barrier function q1, obs with ego rear")
    parser.add_argument('--q2_rear', type=float, default=2.5, help="Barrier function q2, obs with ego rear")

    # ___________________ Constraint Parameters ___________________ #
    parser.add_argument('--acc_limits', nargs="*", type=float, default=[-5.5, 2.0], help="Acceleration limits for the ego vehicle (min,max)")
    parser.add_argument('--steer_angle_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Steering Angle limits (rads) for the ego vehicle (min,max)")


    # ___________________ Ego Vehicle Parameters ___________________ #
    parser.add_argument('--wheelbase', type=float, default=2.94, help="Ego Vehicle's wheelbase")
    parser.add_argument('--max_speed', type=float, default=30.0, help="Ego Vehicle's max speed")
    parser.add_argument('--steering_control_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Steering control input limits (min,max)")
    parser.add_argument('--throttle_control_limits', nargs="*", type=float, default=[-1.0, 1.0], help="Throttle control input limits (min,max)")

    # ___________________ Obstacle Parameters ___________________ #
    parser.add_argument('--t_safe', type=float, default=0.1, help="Time safety headway")
    parser.add_argument('--s_safe_a', type=float, default=10.0, help="safety margin longitudinal")
    parser.add_argument('--s_safe_b', type=float, default=4.0, help="safety margin lateral")
    parser.add_argument('--ego_rad', type=float, default=2, help="Ego Vehicle's radius")
    parser.add_argument('--ego_lf', type=float, default=1.47, help="Distance to front tire")
    parser.add_argument('--ego_lr', type=float, default=1.47, help="Distance to rear tire")


    
