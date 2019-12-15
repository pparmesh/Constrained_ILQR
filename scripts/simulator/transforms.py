#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Tool functions to convert transforms from carla to normal coordinate system
"""

import math
import numpy

import transformations

def carla_location_to_numpy_vector(carla_location):
    """
    Convert a carla location to a Numpy vector

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_location.x,
        -carla_location.y,
        carla_location.z
    ])

def carla_velocity_to_numpy_vector(carla_velocity):
    """
    Convert a carla velocity to a numpy array

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """

    return numpy.array([
        carla_velocity.x,
        -carla_velocity.y,
        carla_velocity.z
    ])



def carla_acceleration_to_numpy_vector(carla_acceleration):
    """
    Convert a carla acceleration to a numpy array

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)
    The angular accelerations remain zero.

    :param carla_acceleration: the carla acceleration
    :type carla_acceleration: carla.Vector3D
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_acceleration.x,
        -carla_acceleration.y,
        carla_acceleration.z
    ])

def carla_rotation_to_RPY(carla_rotation):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a tuple with 3 elements (roll, pitch, yaw)
    :rtype: tuple
    """
    roll = -math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return (roll, pitch, yaw)

def carla_angular_velocity_to_numpy_vector(carla_angular_velocity):
    """
    Convert a carla angular velocity to a numpy array

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_acceleration: the carla acceleration
    :type carla_acceleration: carla.Vector3D
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([math.radians(carla_angular_velocity.x), 
        -math.radians(carla_angular_velocity.y), 
        -math.radians(carla_angular_velocity.z)])

def carla_rotation_to_numpy_rotation_matrix(carla_rotation):
    """
    Convert a carla rotation to a ROS quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3x3 elements
    :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    numpy_array = transformations.euler_matrix(roll, pitch, yaw)
    rotation_matrix = numpy_array[:3, :3]
    return rotation_matrix


def carla_rotation_to_directional_numpy_vector(numpy_vector, carla_rotation):
    """
    Convert a carla rotation (as orientation) into a numpy directional vector

    ros_quaternion = np_quaternion_to_ros_quaternion(quat)
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3 elements as directional vector
        representation of the orientation
    :rtype: numpy.array
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    # directional_vector = numpy.array([1, 0, 0])
    rotated_directional_vector = rotation_matrix.dot(numpy_vector)
    return rotated_directional_vector
    


def carla_velocity_to_numpy_local_velocity(carla_linear_velocity, carla_rotation):
    """
    Convert a carla velocity to a numpy velocity
    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a ROS twist (with rotation)
    :rtype: geometry_msgs.msg.Twist
    """
    numpy_velocity = carla_velocity_to_numpy_vector(carla_linear_velocity)
    numpy_rotated_velocity = carla_rotation_to_directional_numpy_vector(numpy_velocity, carla_rotation)

    #convert to m/s from km/hr
    return numpy_rotated_velocity*5/18

def carla_acceleration_to_numpy_local_velocity(carla_linear_acceleration, carla_rotation):
    """
    Convert a carla acceleration to a numpy acceleration
    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    :param carla_acceleration: the carla acceleration
    :type carla_acceleration: carla.Vector3D
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a ROS twist (with rotation)
    :rtype: geometry_msgs.msg.Twist
    """
    numpy_acceleration = carla_acceleration_to_numpy_vector(carla_linear_acceleration)
    numpy_rotated_acceleration = carla_rotation_to_directional_numpy_vector(numpy_acceleration, carla_rotation)

    return numpy_rotated_acceleration