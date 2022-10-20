#!/usr/bin/env python3

import math

def convert_pi_string_to_float(s: str) -> float:
    """Takes a string that contains pi and evaluates the expression. Returns a float
    Returns 0.0 if the expression cannot be evaluated"""
    value = 0.0
    negative = False

    if s.isdigit():
        return float(s)

    if s.find('pi') == -1:
        # Return 0 if string does not contain pi
        return value

    if not s.find('-') == -1:
        negative = True
        s = s.replace('-', '')

    split = s.split('pi')
    if not len(split) == 2:
        # Can't evaluate strings with multiple pi's, return 0
        return value

    before, after = split
    if before and after:
        before = before.replace('*', '')
        if before.isdigit():
            value = float(before) * math.pi
        after = after.replace('/', '')
        if after.isdigit():
            value /= float(after)
    elif before:
        before = before.replace('*', '')
        if before.isdigit():
            value = float(before) * math.pi
    elif after:
        after = after.replace('/', '')
        if after.isdigit():
            value = math.pi / float(after)
    else:
        value = math.pi

    if negative:
        return -value
    else:
        return value

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q
