# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    path_tracking_helper.py                            :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/03/08 18:12:31 by saeed             #+#    #+#              #
#    Updated: 2021/03/09 11:55:42 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #


import numpy as np
import math


class PURE_PURSUIT:

    def __init__(self, x, y, yaw, v, w, course):
        # Parameters
        self.k = 0.1  # look forward gain
        self.Lfc = 1.5  # [m] look-ahead distance
        self.Kp = 1  # speed proportional gain
        self.dt = 0.1  # [s] time tick
        self.WB = 1.5  # [m] wheel base of vehicle
        self.v = 0  # linear_speed
        self.w = 0  # angular_speed
        self.max_linear = 1  # max_linear_speed
        self.max_angular = 5  # max_angular_speed
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w
        self.cx = [i[0] for i in course]
        self.cy = [i[1] for i in course]
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
        self.old_nearest_point_index = None
        self.target_ind = 0
        self.ind = None

    def search_target_index(self):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [self.rear_x - icx for icx in self.cx]
            dy = [self.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            self.ind = np.argmin(d)
            self.old_nearest_point_index = self.ind

        else:
            self.ind = self.old_nearest_point_index
            distance_this_index = self.calc_distance(self.cx[self.ind],
                                                     self.cy[self.ind])
            while True:
                distance_next_index = self.calc_distance(self.cx[self.ind + 1],
                                                         self.cy[self.ind + 1])
                if distance_this_index < distance_next_index:
                    break
                self.ind = self.ind + \
                    1 if (self.ind + 1) < len(self.cx) else self.ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = self.ind

        Lf = self.k * self.v + self.Lfc  # update look ahead distance
        # search look ahead target point index
        while Lf > self.calc_distance(self.cx[self.ind], self.cy[self.ind]):
            if (self.ind + 1) >= len(self.cx):
                break  # not exceed goal
            self.ind += 1
        return self.ind, Lf

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

    def pure_pursuit_steer_control(self):
        self.ind, Lf = self.search_target_index()
        if self.target_ind >= self.ind:
            self.ind = self.target_ind
        if self.ind < len(self.cx):
            tx = self.cx[self.ind]
            ty = self.cy[self.ind]
        else:  # toward goal
            tx = self.cx[-1]
            ty = self.cy[-1]
            self.ind = len(self.cx) - 1

        if self.yaw < 0:
            self.yaw = self.yaw + 2*np.pi
        omega = math.atan2(ty - self.rear_y, tx - self.rear_x)
        if omega < 0:
            omega = omega + 2*np.pi
        alpha = omega - self.yaw
        curvature = 2.0 * math.sin(alpha) / Lf
        return curvature, self.ind

    def proportional_control(self, target, current):
        # a = self.Kp * (target - current)
        # linear_speed = current + a
        return self.max_linear

    def get_velocity_comands(self):
        cmd_v = self.proportional_control(self.max_linear, self.v)
        curvature, self.target_ind = self.pure_pursuit_steer_control()
        cmd_w = curvature * self.max_linear
        cmd_v = np.clip(cmd_v, 0, self.max_linear)
        cmd_w = np.clip(cmd_w, -self.max_angular, self.max_angular)
        if self.target_ind+1 == len(self.cx):
            cmd_v, cmd_w = 0., 0.
            print " Reached end of path!"
        return cmd_v, cmd_w
