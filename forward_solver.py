#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 29 20:21:51 2022

@author: dee
"""
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm
import itertools
from generic_functions import ball_ball_collision_aoa, convert_velocity_to_speed, \
    convert_speed_to_velocity, deg2rad, rad2deg, line_equation, table_intersection_point, \
        distance
from Ball import Ball
from Table import Table
import physical_parameters as pp
import logging
import sys
from plotting import plot_ball_history
logger = logging.getLogger(__name__)

#TODO ball-ball collisions
#TODO spin modelling
#TODO multi-ball modelling

def simulate_single_strike(table_state, dt = 0.1):
    """
    Computes the ball attributes following a single strike of the cue ball
    and updates table state (ball positions)

    Parameters
    ----------
    table_state : class
        Table state instance describing balls on table.

    Returns
    -------
    None.

    """
    
    table_state.check_balls_in_motion()
    cue_ball = table_state.cue_ball
    cue_ball_id = cue_ball.id
    cue_ball_speed = cue_ball.speed
    num_balls_on_table = len(table_state.balls_in_play)
    assert cue_ball_speed > 0, "Cue ball is not in motion!"
    
    # First iteration: strike cue ball
    # Cue ball slides as it is hit
    cue_ball.motion = 'sliding'
    cue_ball_speed = cue_ball.speed
    
    # Check speed at which ball will begin to roll 
    cue_ball.rolling_speed = cue_ball.calc_rolling_speed()
    
    # Slide ball - compute new velocity and position
    cue_ball.slide(dt, update_position = True)
    
    # Check table collision and update ball state variables
    cue_ball.detect_update_table_collision(dt)
    
    # Get positions of other balls (NOT INCLUDING CURRENT BALL)
    other_ball_positions, other_ball_positions_ids = table_state.get_ball_positions(cue_ball_id)
    
    # Detect and update state variables if ball-ball collision occured
    # INSERT CODE HERE
    
    # Update position history of cue ball
    cue_ball.update_position_history()
    
    iteration = 1
    while table_state.balls_in_motion_bool:
        logger.info(f'\n\nIteration: {iteration}')
        
        balls_in_motion = table_state.balls_in_motion
            # Iterate through balls on table
        for ball in table_state.balls_in_motion:
            logger.info(f'Ball name: {ball.name}')
            logger.info(f'Ball speed: {ball.speed}')
            logger.info(f'Ball motion: {ball.motion}')
            logger.info(f'Ball velocity: {ball.velocity}')
            logger.info(f'Ball position: {ball.position}')
            logger.info(f'Previous ball position: {ball.position_prev}')
            # Check if ball is rolling or sliding and update state parameters accordingly
            if ball.speed > ball.rolling_speed:
                ball.slide(dt, update_position = True)
            else:
                logger.info('rolling')
                ball.motion = 'rolling'
                ball.roll(dt, update_position = True)
            
            # Detect table collision and update ball state variables if collision
            ball.detect_update_table_collision(dt)
            
            # Detect and update state variables if ball-ball collision occured
            # INSERT CODE HERE
            
            ball.update_position_history()
                
        iteration += 1
        table_state.check_balls_in_motion()
        print ('-'*20)
    plot_ball_history(table_state)
    
