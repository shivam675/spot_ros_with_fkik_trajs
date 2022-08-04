#!/usr/bin/env python3

from spot_ros.Kinematics.LegKinematics import LegIK
import math

front_left_keg_solver = LegIK(legtype='RIGHT', shoulder_length=0.055,
elbow_length=0.1065, wrist_length=0.145)


a, b, c = math.radians(-20), math.radians(20), math.radians(20)

d = front_left_keg_solver.solve(0, 0, 0.1)