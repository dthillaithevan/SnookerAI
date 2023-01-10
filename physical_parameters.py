#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 27 20:18:29 2022

@author: dee
"""
import numpy as np
ball_mass = 0.14 # kg
ball_radius = 52.5e-3 # m
ball_ball_coef_friction = 0.05
ball_ball_coef_restitution = 0.94
ball_table_rolling_resistance = 0.01
ball_table_coef_sliding_friction = 0.2
ball_table_spin_deceleration_rate = 10 #rad/sec^2
ball_cushion_coef_restitution = 0.75
ball_mass_moment_of_inertia = 2/5 * ball_mass *  ball_radius**2
table_width = 1778e-3 #m
table_length = 3569e-3 #m
cue_tip_collision_time = 0.0015
g = 9.81
tol = 1e-5 #tolerance value to avoid numerical instability
decimals = 8 #rouding tolerance


x_min = 0 + ball_radius
x_max = table_width - ball_radius
y_min = 0 + ball_radius
y_max = table_length - ball_radius

table_c1 = [x_min, y_min] # Bottom left 
table_c2 = [x_min, y_max] # Top left 
table_c3 = [x_max, y_max] # Top right
table_c4 = [x_max, y_min] # Bottom right
# Table corners accounting for ball radius
table_corners_adjusted = np.array([table_c1, table_c2, table_c3, table_c4])

# Real Table corners 
table_corners = np.array([[0, 0],
                          [0, table_length],
                          [table_width, table_width],
                          [table_width, 0]])

table_sides = ['left', 'top', 'right', 'bottom']
ball_names = ['cue', 'red', 'yellow', 'green', 'blue', 'brown', 'pink', 'black']