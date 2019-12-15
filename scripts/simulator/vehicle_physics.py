#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Tool functions to calculate vehicle physics
"""

import math

# from tf.transformations import euler_from_quaternion


class VehiclePhysicsInfo():
    def __init__(self, vehicle_info):
        self.vehicle_info = vehicle_info

        self.mass = self.get_vehicle_mass()
        self.max_steering_angle = self.get_vehicle_max_steering_angle()
        self.max_speed = self.get_vehicle_max_speed()
        self.max_accel = self.get_vehicle_max_acceleration()
        self.max_decel = self.get_vehicle_max_deceleration()
        self.engine_impedance = self.get_vehicle_lay_off_engine_acceleration()

        self.gravity = 9.81
        self.weight = self.mass * self.gravity

    def get_vehicle_lay_off_engine_acceleration(self):
        """
        Calculate the acceleration a carla vehicle faces by the engine on lay off

        This respects the following forces:
        - engine brake force

        :return: acceleration the vehicle [m/s^2 < 0]
        :rtype: float64
        """
        return -self.get_engine_brake_force() / self.mass


    def get_engine_brake_force(self):
        """
        Calculate the engine brake force of a carla vehicle if the gas pedal would be layed off

        As this heavily depends on the engine, the current gear and velocity, this is not
        trivial to calculate. Maybe one can get this from within Unreal to the outside,
        to enable better vehicle control.
        For the moment we just put a constant force.

        :return: engine braking force [N]
        :rtype: float64
        """
        return 500.0


    def get_vehicle_mass(self):
        """
        Get the mass of a carla vehicle (defaults to 1500kg)

        :return: mass of a carla vehicle [kg]
        :rtype: float64
        """
        mass = 1500.0
        if self.vehicle_info.mass:
            mass = self.vehicle_info.mass

        return mass


    def get_vehicle_driving_impedance_acceleration(self, vehicle_status, reverse):
        """
        Calculate the acceleration a carla vehicle faces by the driving impedance

        This respects the following forces:
        - rolling resistance force
        - aerodynamic drag force
        - slope force

        :param reverse: `True' if the vehicle is driving in reverse direction
        :type reverse: boolean
        :return: acceleration the vehicle [m/s^2 <= 0 on flat surface]
        :rtype: float64
        """
        # taking the following forumlar as basis
        #
        # mass * acceleration = rolling_resitance_force + aerodynamic_drag_force
        #
        # deceleration = -(rolling_resitance_force + aerodynamic_drag_force)/mass
        #
        # future enhancements: incorporate also the expected motor braking force
        #
        rolling_resistance_force = self.get_rolling_resistance_force()
        aerodynamic_drag_force = self.get_aerodynamic_drag_force(vehicle_status)
        slope_force = self.get_slope_force(vehicle_status)

        if reverse:
            slope_force = -slope_force

        impedence_force = -(rolling_resistance_force + aerodynamic_drag_force + slope_force)
        
        deceleration = impedence_force / self.mass

        return deceleration


    def get_rolling_resistance_force(self):
        """
        Calculate the rolling resistance force of a carla vehicle

        :return: rolling resistance force [N]
        :rtype: float64
        """
        # usually somewhere between 0.007 to 0.014 for car tyres
        # and between 0.0025 to 0.005 for bycicle tyres
        # see also https://en.wikipedia.org/wiki/Rolling_resistance
        # @todo: currently not within vehicle_info
        rolling_resistance_coefficient = 0.01
        normal_force = self.weight

        rolling_resistance_force = rolling_resistance_coefficient * normal_force

        # deceleration due to rolling resistance 0.098
        return rolling_resistance_force




    def get_aerodynamic_drag_force(self, vehicle_status):
        """
        Calculate the aerodynamic drag force of a carla vehicle

        :return: aerodynamic drag force [N]
        :rtype: float64
        """
        # see also https://en.wikipedia.org/wiki/Automobile_drag_coefficient
        default_aerodynamic_drag_coefficient = 0.3
        default_drag_reference_area = 2.37
        rho_air_25 = 1.184

        if self.vehicle_info.drag_coefficient:
            default_aerodynamic_drag_coefficient = self.vehicle_info.drag_coefficient

        # @todo currently not provided in vehicle_info
        drag_area = default_aerodynamic_drag_coefficient * default_drag_reference_area

        speed_squared = vehicle_status[1][0] * vehicle_status[1][0]

        # default value 0.4209 * vel^2
        aerodynamic_drag_force = 0.5 * drag_area * rho_air_25 * speed_squared
        return aerodynamic_drag_force


    def get_slope_force(self, vehicle_status):
        """
        Calculate the force of a carla vehicle faces when driving on a slope.

        :param vehicle_status: the ego vehicle status
        :return: slope force [N, >0 uphill, <0 downhill]
        :rtype: float64
        """
        pitch = vehicle_status[2][1]
        slope_force = self.weight * math.sin(-pitch)
        return slope_force


    def get_vehicle_max_steering_angle(self):
        """
        Get the maximum steering angle of a carla vehicle

        :return: maximum steering angle [radians]
        :rtype: float64
        """
        # 70 degrees is the default max steering angle of a car
        max_steering_angle = math.radians(70)
        # get max steering angle (use smallest non-zero value of all wheels)
        for wheel in self.vehicle_info.wheels:
            if wheel.max_steer_angle:
                if wheel.max_steer_angle and wheel.max_steer_angle < max_steering_angle:
                    max_steering_angle = wheel.max_steer_angle
        # print("Max Steering Angle: {}".format(max_steering_angle))
        return max_steering_angle


    def get_vehicle_max_speed(self):
        """
        Get the maximum speed of a carla vehicle

        :return: maximum speed [m/s]
        :rtype: float64
        """
        # 180 km/h is the default max speed of a car
        max_speed = 180.0 / 3.6

        return max_speed


    def get_vehicle_max_acceleration(self):
        """
        Get the maximum acceleration of a carla vehicle

        default: 3.0 m/s^2: 0-100 km/h in 9.2 seconds

        :return: maximum acceleration [m/s^2 > 0]
        :rtype: float64
        """
        max_acceleration = 3.0

        return max_acceleration


    def get_vehicle_max_deceleration(self):
        """
        Get the maximum deceleration of a carla vehicle

        default: 8 m/s^2

        :return: maximum deceleration [m/s^2 > 0]
        :rtype: float64
        """
        max_deceleration = 8.0

        return max_deceleration