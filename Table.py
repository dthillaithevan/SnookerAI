#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 27 20:27:37 2022

@author: dee
"""
import numpy as np
import physical_parameters as pp
from generic_functions import ball_ball_collision_aoa, convert_speed_to_velocity
import logging
logger = logging.getLogger(__name__)


class Table:
    """
    Table class used to store state of balls on table
    """
    def __init__(self, balls_in_play: list, ):
        self.balls_in_play = balls_in_play
        self.num_balls_in_play = len(balls_in_play)
        self.ball_positions = self.extract_ball_positions()
        self.check_balls_in_motion()
        self.cue_ball_speed, self.cue_ball_speed_bool = self.check_cue_ball_speed()
        self.cue_ball = self.extract_cue_ball()
    
    def extract_ball_positions(self,):
        # Extracts all the positions of balls on table
        ball_positions = []
        for ball in self.balls_in_play:
            ball_positions.append(ball.position)
        return np.array(ball_positions)
   
    def extract_cue_ball(self,):
        # Extract cue ball instance
        for ball in self.balls_in_play:
            if ball.name == 'cue':
                return ball

    def remove_ball_from_play(self,ball):
        # Removes ball from table when it is potted
        #TODO remove ball from self.balls_in_play
        pass
    
    def check_balls_in_motion(self,):
        # Check which balls are in motion
        balls_in_motion = []
        self.balls_in_motion_bool = False
        for ball in self.balls_in_play:
            if ball.speed > pp.tol:
                balls_in_motion.append(ball)
                self.balls_in_motion_bool = True
        self.balls_in_motion = balls_in_motion
    
    def check_cue_ball_speed(self):
        # Check if cue ball is in motion
        for ball in self.balls_in_play:
            if ball.name == 'cue':
                speed = ball.speed
        if speed > 0:
            return speed, True
        else:
            return 0, False
    
    def get_ball_positions(self, current_ball_id):
        # Returns positions and corresponding id of every ball, excluding current ball
        ball_positions = []
        ball_ids = []
        for ball in self.balls_in_play:
            if ball.id != current_ball_id:
                ball_positions.append(list(ball.position))
                ball_ids.append(int(ball.id))
        return np.array(ball_positions), ball_ids
    
    def calculate_ball_distances(self,ball_position, other_ball_positions):
        # Calculates the distance between ball and all other balls
        x_y_dist = (other_ball_positions - ball_position).T
        distance = np.sqrt(np.einsum('ij,ij->j', x_y_dist, x_y_dist))
        return distance

    def detect_update_ball_collision(self, dt, current_ball, other_ball_positions, other_ball_position_ids):
        # TODO: Detect ball-ball collision
        # TODO: Calculate new velocities following ball-ball collision, 2 balls
        # TODO: Calculate new velocities following ball-ball colliisons, multiple balls
        raise NotImplementedError('Ball ball collisions not implemented yet!')