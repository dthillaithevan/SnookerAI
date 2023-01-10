#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 18:37:01 2023

@author: dee
"""
import physical_parameters as pp
import matplotlib.pyplot as plt
import numpy as np
import physical_parameters as pp
from matplotlib import animation
from matplotlib import use 
use('Agg')

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
    
    
def animate_shot(ball_history):
    # Animates single ball movement
    # TODO: Update function to accept multiple ball histories
    fig = plt.figure()
    fig.set_dpi(50)
    fig.set_size_inches(pp.table_width*5, pp.table_length*5)
    
    ax = plt.axes(xlim=(0,pp.table_width), ylim=(0,pp.table_length))
    patch = plt.Circle((ball_history[0,0], ball_history[0,1]), pp.ball_radius, fc='white')
    patch = plt.Circle((0, 0), pp.ball_radius, fc='white')
    ax.grid()
    ax.set_facecolor("#0a6c03")
    def init():
        patch.center = (ball_history[0,0], ball_history[0,1])
        ax.add_patch(patch)
        return patch,
    
    def animate(i):
        patch.center = (ball_history[i,0], ball_history[i,1])
        return patch,
    
    anim = animation.FuncAnimation(fig, animate, 
                                   init_func=init, 
                                   frames=len(ball_history), 
                                   # interval=20,
                                   blit=True)
    
    writergif = animation.PillowWriter(fps=30)
    anim.save('ball_animation.gif',writer=writergif)


