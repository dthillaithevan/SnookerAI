#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 18:37:01 2023

@author: dee
"""
import physical_parameters as pp
import matplotlib.pyplot as plt
import numpy as np


def plot_ball_history(table_state):
    # # Plot results
    plt.figure(figsize = (pp.table_width*5,pp.table_length*5))
    
    for ball in table_state.balls_in_play:
        history = np.array(ball.position_history)
        alphas = np.linspace(0.1,1,len(history))
        plt.scatter(history[:,0], history[:,1], label = ball.name, alpha = alphas, s = 500)

    plt.xlim([-0.1,pp.table_width + 0.1])
    plt.ylim([-0.1,pp.table_length + 0.1])
    plt.vlines(pp.table_width, 0, pp.table_length, color = 'k')
    plt.hlines(pp.table_length, 0, pp.table_width, color = 'k')
    plt.vlines(0, 0, pp.table_length, color = 'k')
    plt.hlines(0, 0, pp.table_width, color = 'k')
    plt.legend()
    plt.grid()