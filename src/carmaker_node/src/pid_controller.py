#!/usr/bin/env python
import rospy
import numpy as np


class Pid_Controller:
    def __init__(self):
        self.Kp = 0.6
        self.Ki = 0.5
        self.Kd = 0.2
        self.I = 0
        self.last_input = 0

    def pd_control(self, dt, cur_vel_, last_input):
        d_input = cur_vel_ - last_input
        try:
            D = -self.Kd * (d_input / dt)
        except ZeroDivisionError:
            D = 0
        last_input_ = cur_vel_

        return D, last_input_

    def accel_control(self, tar_vel_, cur_vel_, dt, pcon=0, icon=0, dcon=0):
        cur_err = (tar_vel_ - cur_vel_)
        p_control = 0
        d_control = 0
        if pcon == 1:
            p_control = self.Kp * cur_err
        if dcon == 1:
            (d_control, self.last_input) = self.pd_control(dt, cur_vel_, self.last_input)

        if icon == 1:
            self.I = self.I + (self.Ki * cur_err * dt)

        else:
            self.I = 0
        acc_cmd = p_control + d_control + self.I
        return acc_cmd


def rate_limiter(value, prev_value, max_change):
    delta = value - prev_value
    delta = np.clip(delta, -max_change, max_change)
    return prev_value + delta


"""
Error = setPoint - Input
PTerm = Kp * Error
ITerm += Ki * Error * dt
dinput = input - previnput
DTerm = -Kd *(kinput / dt)
Output = PTerm + ITerm + DTerm
"""
