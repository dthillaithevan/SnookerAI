#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 27 20:21:26 2022

@author: dee
"""
import numpy as np
import logging
logger = logging.getLogger(__name__)


def ball_ball_collision_aoa(ball_1_position, ball_1_velocity, ball_2_position):
    # Calculate the angle of attack of two balls colliding
    # Uses the velocity vector of the incoming ball to determine the angle 
    # between line joining the centre of mass and velocity vector of incoming ball
    # Uses law of cosines
    p3 = ball_1_position + ball_1_velocity
    d_12 = np.sum((ball_1_position - ball_2_position)**2)**0.5
    d_13 = np.sum((ball_1_position - p3)**2)**0.5
    d_23 = np.sum((ball_2_position - p3)**2)**0.5
    val = (d_12**2 + d_13**2 - d_23**2) / (2 * d_12 * d_13)
    aoa = np.arccos((val))
    print (f'Collision angle: {aoa}')
    return aoa
    
def convert_velocity_to_speed(velocity):
    # Calculates speed and direction (radian, angle) of travel
    speed = (velocity[0]**2 + velocity[1]**2)**0.5
    # direction = np.arctan2(velocity[1]/(velocity[0]+tol))
    direction = np.arctan2(np.array(velocity[1]), np.array(velocity[0]))
    if direction < 0:
        direction += 2*np.pi
    return speed, direction

def convert_speed_to_velocity(speed, direction):
    velocity = speed * np.array([np.cos(direction), np.sin(direction)])
    return velocity

def deg2rad(deg):
    return deg*np.pi/180

def rad2deg(rad):
    return rad*180/np.pi

def line_equation(point1: list, point2: list):
    m = (point2[1] - point1[1])/(point2[0] - point1[0])
    c = point1[1] - m*point1[0]
    return m, c

def table_intersection_point(current_ball_position: list, prev_ball_position: list, table_side_point1: list, table_side_point2: list):
    # Calculates the intersection point given 2 pair of points.     
    a1 = (prev_ball_position[1] - current_ball_position[1])
    b1 = (current_ball_position[0] - prev_ball_position[0])
    c1 = (current_ball_position[0] * prev_ball_position[1]) - (prev_ball_position[0] * current_ball_position[1])
    
    a2 = (table_side_point1[1] - table_side_point2[1])
    b2 = (table_side_point2[0] - table_side_point1[0])
    c2 = (table_side_point2[0] * table_side_point1[1]) - (table_side_point1[0] * table_side_point2[1])
    
    y =  (c1*(a1 - a2) - a1*(c1 - c2))/((a1-a2)*b1 - a1*(b1 - b2))
    x = (c1 - c2 - y*(b1 - b2))/(a1 - a2)
    intersection_point = [x,y]
    
    return intersection_point

def distance(positionA: list, positionB: list):
    return np.sum((positionA - positionB)**2)**0.5
    
    