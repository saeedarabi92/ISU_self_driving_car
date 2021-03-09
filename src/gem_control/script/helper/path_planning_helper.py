# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    path_planning_helper.py                            :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: saeed <arabi@iastate.edu>                  +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2021/03/08 18:24:56 by saeed             #+#    #+#              #
#    Updated: 2021/03/09 11:40:50 by saeed            ###   ########.fr        #
#                                                                              #
# **************************************************************************** #


import numpy as np
import math
import dubins


class DUBINS_PATH_PLANNER:

    def __init__(self, x_current, y_current, yaw_current, x_goal, y_goal, yaw_goal):
        self.x_current = x_current
        self.y_current = y_current
        self.yaw_current = yaw_current
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.yaw_goal = yaw_goal
        self.turning_radius = 5.
        self.step_size = 0.05

    def get_path(self):
        q0 = (self.x_current, self.y_current, self.yaw_current)
        q1 = (self.x_goal, self.y_goal, self.yaw_goal)
        path = dubins.shortest_path(q0, q1, self.turning_radius)
        configurations, _ = path.sample_many(self.step_size)
        return configurations

    def get_path_info(self):
        return [(i, j, k, None) for i, j, k in self.get_path()]


class SIMPLE_PATH_PLANNER:

    def __init__(self, x_current, y_current, yaw_current, x_goal, y_goal, yaw_goal):
        # parameteres
        self.x_current = x_current
        self.y_current = y_current
        self.yaw_current = yaw_current
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.yaw_goal = yaw_goal

    def get_path(self, step=0.05):
        """
        bearing         value
            ^              0
            >            -pi/2
            <             pi/2
        downward arrow  -pi, pi
        ----------------------------------------
            yaw           value

            ^             -pi, pi
            >              pi/2
            <             -pi/2
        downward arrow      0 
        """
        dx = self.x_goal - self.x_current
        dy = self.y_goal - self.y_current
        bearing = math.atan2(dy, dx)
        distance_between_points = math.sqrt(dx ** 2 + dy ** 2)
        if (self.yaw_current > 0 and self.yaw_current < np.pi / 4) or (self.yaw_current < 0 and self.yaw_current > -np.pi / 4):
            if bearing < -np.pi / 4 and bearing > -3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [self.x_current + i - cx[-1] for i in cx]
                y_list = [self.y_current + i - cy[-1] for i in cy]
                x_list = list(reversed(x_list))
                y_list = list(reversed(y_list))

            elif bearing > np.pi / 4 and bearing < 3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [self.x_current + i - cx[0] for i in cx]
                y_list = [self.y_current + i - cy[0] for i in cy]

            else:
                x_list = []
                y_list = []
                for p in np.arange(0, distance_between_points, step):
                    x_list.append(self.x_current + p * math.cos(bearing))
                    y_list.append(self.y_current + p * math.sin(bearing))

        if (self.yaw_current > 3*np.pi/4 and self.yaw_current < np.pi) or (self.yaw_current > -np.pi and self.yaw_current < -3*np.pi/4):

            if bearing < -np.pi / 4 and bearing > -3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [-(-self.x_current + i - cx[0]) for i in cx]
                y_list = [-(-self.y_current + i - cy[0]) for i in cy]

            elif bearing > np.pi / 4 and bearing < 3 * np.pi / 4:
                theta = np.linspace(-np.pi/2, np.pi/2, 1000)
                r = distance_between_points/2
                cx = r * np.cos(theta)
                cy = r * np.sin(theta)
                x_list = [-(-self.x_current + i - cx[-1]) for i in cx]
                y_list = [-(-self.y_current + i - cy[-1]) for i in cy]
                x_list = list(reversed(x_list))
                y_list = list(reversed(y_list))

            else:
                x_list = []
                y_list = []
                for p in np.arange(0, distance_between_points, step):
                    x_list.append(self.x_current + p * math.cos(bearing))
                    y_list.append(self.y_current + p * math.sin(bearing))

        return zip(x_list, y_list)

    def get_path_info(self):
        return [(i, j, None, None) for i, j in self.get_path()]
