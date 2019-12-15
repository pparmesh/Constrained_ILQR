#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
# Link: https://github.com/carla-simulator/carla/blob/master/PythonAPI/util/lane_explorer.py

import carla
import math

red = carla.Color(255, 0, 0)
green = carla.Color(0, 255, 0)
blue = carla.Color(47, 210, 231)
cyan = carla.Color(0, 255, 255)
yellow = carla.Color(255, 255, 0)
orange = carla.Color(255, 162, 0)
white = carla.Color(255, 255, 255)

trail_life_time = 10

def draw_transform(debug, trans, col=carla.Color(255, 0, 0), lt=-1):
    yaw_in_rad = math.radians(trans.rotation.yaw)
    pitch_in_rad = math.radians(trans.rotation.pitch)
    p1 = carla.Location(
        x=trans.location.x + math.cos(pitch_in_rad) * math.cos(yaw_in_rad),
        y=trans.location.y + math.cos(pitch_in_rad) * math.sin(yaw_in_rad),
        z=trans.location.z + math.sin(pitch_in_rad))
    debug.draw_arrow(trans.location, p1, thickness=0.05, arrow_size=1.0, color=col, life_time=lt)

def draw_path(debug, path, color=carla.Color(255, 0, 0), lt=1, thickness=0.1):
    for i in range(len(path)-1):
        w0 = carla.Location(x=path[i, 0], y=path[i, 1], z=0.5)
        w1 = carla.Location(x=path[i+1, 0], y=path[i+1, 1], z=0.5)

        debug.draw_line(w0, w1, thickness=thickness, color=color, life_time=lt, persistent_lines=False)
        debug.draw_point(w1 - carla.Location(z=0.25), size=0.05, color=color, life_time=lt, persistent_lines=False)

def draw_waypoint_union(debug, w0, w1, color=carla.Color(255, 0, 0), lt=5):
    debug.draw_line(
        w0.transform.location + carla.Location(z=0.25),
        w1.transform.location + carla.Location(z=0.25),
        thickness=0.1, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(w1.transform.location + carla.Location(z=0.25), 0.1, color, lt, False)


def draw_waypoint_info(debug, w, lt=5):
    w_loc = w.transform.location
    debug.draw_string(w_loc + carla.Location(z=0.5), "lane: " + str(w.lane_id), False, yellow, lt)
    debug.draw_string(w_loc + carla.Location(z=1.0), "road: " + str(w.road_id), False, blue, lt)
    debug.draw_string(w_loc + carla.Location(z=-.5), str(w.lane_change), False, red, lt)
