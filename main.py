#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 18:33:51 2023

@author: dee
"""

import numpy as np
from Ball import Ball
from Table import Table
import logging
import sys
from forward_solver import simulate_single_strike



if __name__ == "__main__":
    # Setup logger
    loglevel = 10 # 10 = debug, 20 = info
    logger = logging.getLogger(__name__)
    # Disable matplotlib logger output
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    logging.basicConfig(format='%(levelname)s | %(name)s | %(message)s', stream=sys.stdout, level=loglevel)
    

    # # Initial conditions, single ball
    # starting_point = np.array([0.8, 0.8])
    # starting_velocity = [1.0,2]
    # cue_ball = Ball(position = starting_point,v0 = starting_velocity, name = 'cue')
    # table_state = Table([cue_ball])
    
    # Initial conditions, multiple balls
    starting_point = np.array([0.8, 0.8])
    starting_velocity = [1.0,2]
    cue_ball = Ball(position = starting_point,v0 = starting_velocity, name = 'cue')
    p2 = np.array([0.1, 0.1])
    red_ball_1 = Ball(p2,[0,0], 'red')
    table_state = Table([cue_ball,red_ball_1])
    
    # Solve forward problem and compute new state for balls on table, updates table_state instance
    simulate_single_strike(table_state)
    
    # print new ball positions
    print (table_state.extract_ball_positions())
    
    # Turn off logging, dont need to reset ipython to rerun
    logging.shutdown()