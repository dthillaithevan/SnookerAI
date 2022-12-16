#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 29 20:21:51 2022

@author: dee
"""
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm

#TODO ball-ball interactions
#TODO spin modelling
#TODO multi-ball modelling

ball_mass = 0.14 # kg
ball_radius = 52.5e-3 # m
ball_ball_coef_friction = 0.05
ball_ball_coef_restitution = 0.94
ball_table_rolling_resistance = 0.01
ball_table_coef_sliding_friction = 0.2
ball_table_spin_deceleration_rate = 10 #rad/sec^2
ball_cushion_coef_restitution = 0.75
ball_mass_moment_of_inertia = 2/5 * ball_mass *  ball_radius**2
table_width = 1778e-3
table_length = 3569e-3

table_c1 = [0,0]
table_c2 = [0, table_length]
table_c3 = [table_width, table_length]
table_c4 = [table_width, 0]
table_corners = np.array([table_c1, table_c2, table_c3, table_c4])
table_sides = ['left', 'top', 'right', 'bottom']

cue_tip_collision_time = 0.0015
g = 9.81
tol = 1e-8 #tolerance value to avoid numerical instability
decimals = 5 #rouding tolerance
class ball:
    def __init__(self, x, y, z=-1):
        self.x = x
        self.y = y
        self.z = z
        self.position = np.array([x,y])
        self.velocity = np.array([0,0])
        self.speed = 0. # Velocity magnitude (ball reference frame)
        self.angle = 0. # Angle (in radians) relative to x axis of ball direction
    
    def convertSpeedToVelocity(self,speed,angle):
        u_x, u_y = speed*np.cos(angle), speed*np.sin(angle)
        return u_x, u_y
    
def convertVelocityToSpeed(velocity):
    # Calculates speed and direction (radian, angle) of travel
    speed = (velocity[0]**2 + velocity[1]**2)**0.5
    # direction = np.arctan2(velocity[1]/(velocity[0]+tol))
    direction = np.arctan2(np.array(velocity[1]), np.array(velocity[0]))
    if direction < 0:
        direction += 2*np.pi
    return speed, direction

def convertSpeedToVelocity(speed, direction):
    velocity = speed * np.array([np.cos(direction), np.sin(direction)])
    return velocity

def deg2rad(deg):
    return deg*np.pi/180

def rad2deg(rad):
    return rad*180/np.pi

def distance(point_1, point_2):
    # Euclidean distance between two points
    d = np.sqrt((point_2[0] -point_1[0])**2 + (point_2[1] -point_1[1])**2)
    return d

def direction(vector):
    horizontal = np.array([1,0])
    
    return np.dot(vector, horizontal) / np.sum(vector**2)**0.5

def detectTableCollision(ball_position):
    # Checks if ball position is inside bounds of table, if not also calcualte distance outside table
    # print (ball_position)
    left = ball_position[0] < 0
    right = ball_position[0] > table_width
    bottom = ball_position[1] < 0
    top = ball_position[1] > table_length
    # Determine which side it has gone over
    collision = np.argwhere([left, top, right, bottom])
    # print (left, top, right, bottom)
    if len(collision) > 0:
        if collision[0,0] == 0:
            side_length = table_corners[1]-table_corners[0]
            d = np.cross(side_length,ball_position-table_corners[0])/norm(side_length)
        elif collision[0,0] == 1:
            side_length = table_corners[2]-table_corners[1]
            d = np.cross(side_length,ball_position-table_corners[1])/norm(side_length)
        elif collision[0,0] == 2:
            side_length = table_corners[3]-table_corners[2]
            d = np.cross(side_length,ball_position-table_corners[2])/norm(side_length)
        elif collision[0,0] == 3:
            side_length = table_corners[0]-table_corners[3]
            d = np.cross(side_length,ball_position-table_corners[3])/norm(side_length)
            
        return True, table_sides[collision[0,0]], d
    else:
        return False, -1, -1 # -1 if no collison

def rollingVelocity(v0):
    # Velocity at which the ball begins to roll due to friction
    return v0 / (1 + (ball_mass_moment_of_inertia / (ball_mass * ball_radius**2)))

def slidingVelocityUpdate(s_t, dt):
    # Calculate velocity at t + dt when ball is sliding
    st_1 = s_t - (ball_table_coef_sliding_friction * g * dt)
    return st_1

def rollingVelocityUpdate(s_t, dt):
    # Calculate velocity at t + dt when ball is rolling
    st_1 = s_t - (ball_table_rolling_resistance * g * dt)
    return st_1
    
v_hist = [] # Velocity history
s_hist = [] # Speed history
theta_hist = [] # Direction history
t_hist = [] # Time history
position_hist = [] # Position history 
collision_bool = False #Indicates if collision has just happened
verbose = False # Print parameters for debugging
def forward_solver(cue_ball_velocity, cue_ball_position, t = 0, dt = 0.1):
    iteration = 0
    motion = 'sliding' # Ball is initally sliding
    v_hist.append(cue_ball_velocity) 
    cue_ball_speed, direction = convertVelocityToSpeed(cue_ball_velocity)
    s_hist.append(cue_ball_speed) 
    t_hist.append(t)
    position_hist.append(cue_ball_position)
    assert cue_ball_position[0] <= table_width
    assert cue_ball_position[1] <= table_length
    collision_bool = False
    #TODO add code to catch case where ball starts outside table - raise error of some kind
    
    # Loop until speed = 0
    while cue_ball_speed > tol:
        if verbose:
            print (f'Iteration: {iteration}')
            print (f'\tSpeed: {cue_ball_speed}')
            print (f'\tVelocity: {cue_ball_velocity}')
            print (f'\tDirection: {rad2deg(direction)}')
        
        s_t = cue_ball_speed 
        position_t = cue_ball_position
        # @T = 0 ball is hit so intially slides
        if t == 0:
            motion = 'sliding'
            rolling_speed = rollingVelocity(s_t)
            s_t1 = slidingVelocityUpdate(s_t, dt)
        else:
            if cue_ball_speed > rolling_speed:
                s_t1 = slidingVelocityUpdate(s_t, dt)
            else:
                motion = 'rolling'
                s_t1 = rollingVelocityUpdate(s_t, dt)
        
        # Determine new velocity based on new speed
        v_t1 = convertSpeedToVelocity(s_t1,direction)
        # Compute new position
        position_t1 = position_t + (v_t1 * dt)
        # Check if new position leads to collision with cushions
        collision, location, distance_to_cushion = detectTableCollision(position_t1)
        
        if collision and not collision_bool:
            print (f'Collided with {location} cushion')
            # Determine the exact time collision takes place 
            # (distance_to_cushion is how far over the table the ball is)
            time_since_collision = distance_to_cushion/s_t
            # Time step for when ball collides with wall
            dt_collision = dt - time_since_collision
            # Compute speed when ball hits wall
            if motion == 'rolling':
                s_t1 = rollingVelocityUpdate(s_t, dt_collision)
            elif motion == 'sliding':
                s_t1 = slidingVelocityUpdate(s_t, dt_collision)
            
            # Compute velocity when ball hits wall
            v_t1 = convertSpeedToVelocity(s_t1,direction)
            # Based on which cushion is hit, flip velocity vector and apply damping factor
            if location in ['left', 'right']:
                # print ('here')
                v_t1[0] = -(v_t1[0] * ball_cushion_coef_restitution)
            elif location in ['bottom', 'top']:
                v_t1[1] = -(v_t1[1] * ball_cushion_coef_restitution)
            # Compute new speed based on new velocity vector
            s_t1, direction = convertVelocityToSpeed(v_t1)
            # Determine position of ball (on wall)
            position_t1 = position_t + (v_t1 * dt_collision)
            # Set bool to True to avoid getting stuck in collision loop
            collision_bool = True
            t += dt_collision
        else:
            t += dt
            collision_bool = False
            
        # Update variables 
        cue_ball_speed = s_t1
        cue_ball_velocity = v_t1
        cue_ball_position = position_t1
        # Save history
        v_hist.append(v_t1)
        t_hist.append(t)
        position_hist.append(cue_ball_position)
        iteration += 1 
    
    
# Starting velocity
v0 = np.array([2,-1])
# Starting position
starting_position = np.array([1,1])

# Run forward solver
forward_solver(v0, starting_position)

# Plot results
position_hist = np.array(position_hist)
plt.figure(figsize = (5,10))
plt.plot(position_hist[:,0], position_hist[:,1])
plt.xlim([-0.2,table_width + 0.2])
plt.ylim([-0.2,table_length + 0.2])
plt.vlines(table_width, 0, table_length, color = 'k')
plt.hlines(table_length, 0, table_width, color = 'k')
plt.vlines(0, 0, table_length, color = 'k')
plt.hlines(0, 0, table_width, color = 'k')


