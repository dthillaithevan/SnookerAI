#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 27 20:18:00 2022

@author: dee
"""
import numpy as np
import itertools
import physical_parameters as pp
from generic_functions import convert_velocity_to_speed, convert_speed_to_velocity,\
    table_intersection_point
import logging
logger = logging.getLogger(__name__)


class Ball:
    """
    Ball class used to store and update single ball state
    """
    new_id = itertools.count().__next__
    def __init__(self, position: list, v0: list, name: str):
        assert name in pp.ball_names, f"Ball name \'{name}\' is not valid. Acceptable names are {pp.ball_names}"
        assert position[0] > 0 and position[0] < pp.table_width and position[1] > 0 and position[1] < pp.table_length, f"Ball position {position} is outside the bounds of the table"
        if name != "cue":
            assert v0[0] == 0 and v0[1] == 0, f"Starting velocity should be zero for {name} ball! Only cue ball is allowed to have non-zero starting velocity"
        self.name = name.lower()
        self.position = np.array(position)
        self.velocity = np.array(v0)
        self.speed, self.direction = convert_velocity_to_speed(self.velocity) # Speed and direction (angle in radians)
        self.speed_prev = 0 # Previous speed, set to 0
        self.velocity_prev = self.velocity * 0 # Previous velocity, set to 0
        self.position_prev = self.position # Previous position
        self.direction_prev = self.direction
        self.id = Ball.new_id() #Unique id to identify ball
        assert self.id <= 22, "Maximum number of balls on table reached"
        self.motion = '' # Describes type of motion ('sliding' or 'rolling')
        self.position_history = [] # Store history of ball position
        self.update_position_history()
        self.radius = pp.ball_radius
        self.mass = pp.ball_mass
        self.mass_moment_of_inertia = pp.ball_mass_moment_of_inertia
        self.cushion_coef_restitution = pp.ball_cushion_coef_restitution
        
        
    def __str__(self,):
        return f"Ball - id: {self.id}, name: {self.name}"
    
    def __repr__(self,):
        return f"Ball id: {self.id}"
    
    def update_position_history(self):
        # Update position history list using current position
        self.position_history.append(list(self.position))
    
    def move(self, dt: float):
        self.position_prev = self.position
        self.position = self.position + self.velocity*dt
    
    def slide(self, dt: float, update_position = True):
        # Compute new speed of ball (speed_t1) after one time step (dt) subjected to  sliding motion
        logger.debug(f'Sliding {self}')
        logger.debug(f'Current speed {np.round(self.speed, 3)}')
        logger.debug(f'Current velocity {self.velocity}')
        logger.debug(f'Current position {self.position}')
        speed_t1 = self.speed - (pp.ball_table_coef_sliding_friction * pp.g * dt)
        
        if speed_t1 < 0:
            logger.debug('Reaching 0 speed, time step too large, reducing current timestep')
            dt = self.speed / (pp.ball_table_coef_sliding_friction * pp.g)
            speed_t1 = 0

        if update_position:
            self.move(dt)
            
        self.speed_prev = self.speed
        self.speed = speed_t1
        self.velocity_prev = self.velocity
        self.velocity = convert_speed_to_velocity(self.speed, self.direction)
        
        logger.debug(f'New speed: {self.speed}')
        logger.debug(f'New velocity: {self.velocity}')
        logger.debug(f'New position: {self.position}')
        return self.speed
    
    def roll(self,dt: float, update_position = True):
        # Compute new speed of ball (speed_t1) after one time step (dt) subjected to rolling motion
        logger.debug(f'Rolling {self}')
        logger.debug(f'Current speed: {self.speed}')
        logger.debug(f'Current velocity: {self.velocity}')
        logger.debug(f'Current position: {self.position}')
        speed_t1 = self.speed - (pp.ball_table_rolling_resistance * pp.g * dt)
        if speed_t1 < 0:
            logger.debug('Reaching 0 speed, time step too large, reducing current timestep')
            dt = self.speed / (pp.ball_table_coef_sliding_friction * pp.g)
            speed_t1 = 0

        if update_position:
            self.move(dt)
            
        self.speed_prev = self.speed
        self.speed = speed_t1
        self.velocity_prev = self.velocity
        self.velocity = convert_speed_to_velocity(self.speed, self.direction)
        logger.debug(f'New speed: {self.speed}')
        logger.debug(f'New velocity: {self.velocity}')
        logger.debug(f'New position: {self.position}')
        return self.speed
    
    def reset_state_vars_previous_timestep(self,):
        # Resets state variables to previous timestep
        self.speed = self.speed_prev
        self.velocity = self.velocity_prev
        self.position = self.position_prev
    
    def calc_rolling_speed(self):
        # Velocity at which the ball begins to roll due to friction
        s_r = self.speed / (1 + (self.mass_moment_of_inertia / (self.mass * self.radius**2)))
        logger.debug(f'Calculating rolling speed: {s_r}')
        return s_r

    def detect_table_collision(self,):
        ball_position = self.position
        # Determine which edge collision occured
        # Ball needs to be travelling into edge for collision to be detected
        # E.g. if ball is on cushion but moving away from cushion then no collision is detected
        left = ball_position[0] < pp.x_min and (self.direction > np.pi/2 and self.direction < 1.5*np.pi)
        right = ball_position[0]  > pp.x_max and (self.direction < np.pi/2 or self.direction > 1.5*np.pi)
        bottom = ball_position[1]  < pp.y_min and self.direction > np.pi
        top = ball_position[1]  > pp.y_max and self.direction < np.pi
        
        # Determine which side it has gone over
        collision = np.argwhere([left, top, right, bottom])
        if len(collision) > 0:
            if collision[0,0] == 0:
                intersection_point = table_intersection_point(ball_position, self.position_prev, pp.table_c1, pp.table_c2)
            elif collision[0,0] == 1:
                intersection_point = table_intersection_point(ball_position, self.position_prev, pp.table_c2, pp.table_c3)
            elif collision[0,0] == 2:
                intersection_point = table_intersection_point(ball_position, self.position_prev, pp.table_c3, pp.table_c4)
            elif collision[0,0] == 3:
                intersection_point = table_intersection_point(ball_position, self.position_prev, pp.table_c4, pp.table_c1)
            return True, pp.table_sides[collision[0,0]], intersection_point
        else:
            return False, -1, -1 # -1 if no collison
    
    def table_collision_update(self, collision_point, dt, table_location):
        # Check how long has passed since ball crossed table boundary using speed from previous iteration
        # time_since_collision = distance_to_table/self.speed
        
        # print (f'Collision location {collision_point}')
        
        # Reset velocities to previous values (before collision)
        self.velocity = self.velocity_prev
        self.speed = self.speed_prev
        
        # Distance between ball and table before collision occured (t-dt)
        prev_distance_to_table_side = ((self.position_prev[0] - collision_point[0])**2 + (self.position_prev[1] - collision_point[1])**2)**0.5
        # print (f'Distance to table from previous position: {prev_distance_to_table_side}')
        t_collision = prev_distance_to_table_side/self.speed_prev
        # print (f'Time to collision: {t_collision}')
        self.position = self.position_prev
        # Set ball position to collision point on wall
        # self.position = np.array(collision_point)
        self.move(t_collision)
        # print (f'New location {self.position}')
        # Compute speed/velocity when ball hits wall
        if self.motion == 'rolling':
            self.roll(t_collision, update_position = False)
        elif self.motion == 'sliding':
            self.slide(t_collision, update_position = False)
        

        # Based on which cushion is hit, flip velocity vector and apply damping factor
        if table_location in ['left', 'right']:
            # print ('left or right')
            self.velocity[0] = -(self.velocity[0] * self.cushion_coef_restitution)
        elif table_location in ['bottom', 'top']:
            # print ('top or bottom')
            self.velocity[1] = -(self.velocity[1] * self.cushion_coef_restitution)
        
        # Compute new speed + direction based on new velocity vector
        self.speed, self.direction = convert_velocity_to_speed(self.velocity)
        
    def detect_update_table_collision(self, dt):
        # Check if ball collided with cushion
        table_collision_bool, table_location, collision_point = self.detect_table_collision()
        if table_collision_bool:
            # print (f'Table collision detect on {table_location} cushion')

            # Update ball state variables if table collisino occured
            self.table_collision_update(collision_point, dt, table_location)

        