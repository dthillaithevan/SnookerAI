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

#TODO ball-ball collisions
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

def ball_ball_collision_aoa(ball_1_position, ball_1_velocity, ball_2_position):
    # Calculate the angle of attack of two balls colliding
    # Uses the velocity vector of the incoming ball to determine the angle between the two angles
    p3 = ball_1_position + ball_1_velocity
    d_12 = np.sum((ball_1_position - ball_2_position)**2)**0.5
    d_13 = np.sum((ball_1_position - p3)**2)**0.5
    d_23 = np.sum((ball_2_position - p3)**2)**0.5
    print (f'{d_12}, {d_13}, {d_23}')
    aoa = np.arccos(np.round(np.abs((d_12**2 + d_13**2 - d_23**2) / (2 * d_12 * d_13)),10))
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
    

class Ball:
    """
    Ball class used to store and update single ball state
    """
    new_id = itertools.count().__next__
    def __init__(self, position: list, v0: list, name: str):
        assert name in ball_names, f"Ball name \'{name}\' is not valid. Acceptable names are {ball_names}"
        assert position[0] > 0 and position[0] < table_width and position[1] > 0 and position[1] < table_length, f"Ball position {position} is outside the bounds of the table"
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
        speed_t1 = self.speed - (ball_table_coef_sliding_friction * g * dt)
        self.speed_prev = self.speed
        self.speed = speed_t1
        self.velocity_prev = self.velocity
        self.velocity = np.round(convert_speed_to_velocity(self.speed, self.direction),decimals)
        if update_position:
            self.move(dt)
        return self.speed
    
    def roll(self,dt: float, update_position = True):
        # Compute new speed of ball (speed_t1) after one time step (dt) subjected to rolling motion
        speed_t1 = self.speed - (ball_table_rolling_resistance * g * dt)
        self.speed_prev = self.speed
        self.speed = speed_t1
        self.velocity_prev = self.velocity
        self.velocity = np.round(convert_speed_to_velocity(self.speed, self.direction), decimals)
        if update_position:
            self.move(dt)
        return self.speed
    
    def calc_rolling_speed(self):
        # Velocity at which the ball begins to roll due to friction
        return self.speed / (1 + (ball_mass_moment_of_inertia / (ball_mass * ball_radius**2)))

    def detect_table_collision(self,):
        ball_position = self.position
        # Determine which edge collision occured
        # Ball needs to be travelling into edge for collision to be detected
        # E.g. if ball is on cushion but moving away from cushion then no collision is detected
        left = ball_position[0] < x_min and (self.direction > np.pi/2 and self.direction < 1.5*np.pi)
        right = ball_position[0]  > x_max and (self.direction < np.pi/2 or self.direction > 1.5*np.pi)
        bottom = ball_position[1]  < y_min and self.direction > np.pi
        top = ball_position[1]  > y_max and self.direction < np.pi
        
        # Determine which side it has gone over
        collision = np.argwhere([left, top, right, bottom])
        if len(collision) > 0:
            if collision[0,0] == 0:
                intersection_point = table_intersection_point(ball_position, self.position_prev, table_c1, table_c2)
            elif collision[0,0] == 1:
                intersection_point = table_intersection_point(ball_position, self.position_prev, table_c2, table_c3)
            elif collision[0,0] == 2:
                intersection_point = table_intersection_point(ball_position, self.position_prev, table_c3, table_c4)
            elif collision[0,0] == 3:
                intersection_point = table_intersection_point(ball_position, self.position_prev, table_c4, table_c1)
            return True, table_sides[collision[0,0]], intersection_point
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
            self.velocity[0] = -(self.velocity[0] * ball_cushion_coef_restitution)
        elif table_location in ['bottom', 'top']:
            # print ('top or bottom')
            self.velocity[1] = -(self.velocity[1] * ball_cushion_coef_restitution)
        
        # Compute new speed + direction based on new velocity vector
        self.speed, self.direction = convert_velocity_to_speed(self.velocity)
        
    def detect_update_table_collision(self, dt):
        # Check if ball collided with cushion
        table_collision_bool, table_location, collision_point = self.detect_table_collision()
        if table_collision_bool:
            # print (f'Table collision detect on {table_location} cushion')

            # Update ball state variables if table collisino occured
            self.table_collision_update(collision_point, dt, table_location)

        
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
        # Removes ball from table
        #TODO remove ball from self.balls_in_play
        pass
    
    def check_balls_in_motion(self,):
        # Check which balls are in motion
        balls_in_motion = []
        self.balls_in_motion_bool = False
        for ball in self.balls_in_play:
            if ball.speed > tol:
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
        
    def detect_ball_collision(self, other_ball_positions, current_ball_position, current_ball_velocity, other_ball_position_ids):
        # TODO: Debug/reimplement ball-ball collision code
        # Determine if ball - ball collision occured based on ball distances
        
        # Find distance between current ball and all other balls in play
        distances = self.calculate_ball_distances(current_ball_position, other_ball_positions)
        # Update distances to account for radius of balls
        distances -=  (2*ball_radius)
        # Get list of balls that have collided with current ball
        collision_list = np.argwhere(distances <= tol)
        # Check if any balls are within collision distance and if the velocity vector is pointing into the balls
        if len(collision_list) > 0 and np.any(np.dot(current_ball_velocity, other_ball_positions[collision_list[0]].T) > 0):
            return True, [other_ball_position_ids[collision_ball_id] for collision_ball_id in collision_list[0]], distances[collision_list[0]]
        else:
            return False, [], []
        
        
    def detect_update_ball_collision(self, dt, current_ball, other_ball_positions, other_ball_position_ids):
        # TODO: Debug/reimplement ball-ball collision code
        # Detect if ball - ball collision(s) has occured
        collision_bool, collision_ball_ids, distances = self.detect_ball_collision(other_ball_positions, current_ball.position, current_ball.velocity, other_ball_position_ids)
        if collision_bool:
            print ('Ball - Ball collision detected')
            # Number of balls that ball collided with
            num_collision_balls = len(collision_ball_ids)
            if num_collision_balls == 1:
                # Get instance of ball that current ball has collided with
                collision_ball = [ball for ball in self.balls_in_play if ball.id == collision_ball_ids[0]][0]
                distance_to_collision_ball = np.sum(((collision_ball.position) - current_ball.position)**2)**0.5 - 1*ball_radius
                t_collision = distance_to_collision_ball/current_ball.speed_prev
                
                # Reset velocities to previous values (before collision)
                current_ball.velocity = current_ball.velocity_prev
                current_ball.speed = current_ball.speed_prev
                
                current_ball.position = current_ball.position_prev
                # Set ball position to collision location with other ball
                current_ball.move(t_collision)
                
                print (f'Incoming ball previous position: {current_ball.position_prev}')
                print (f'Incoming ball current position: {current_ball.position}')
                print (f'Collision ball current position: {collision_ball.position}')
                
                # Compute speed/velocity when ball hits other ball
                if current_ball.motion == 'rolling':
                    current_ball.roll(t_collision, update_position = False)
                elif current_ball.motion == 'sliding':
                    current_ball.slide(t_collision, update_position = False)
                
                collision_aoa = ball_ball_collision_aoa(current_ball.position, current_ball.velocity, collision_ball.position)
                if collision_aoa < tol and collision_aoa > -tol:
                    # Direct/head on collision so all momentum is transferred
                    print (f'Direct collision detected between {current_ball.name} ball and {collision_ball.name} ball')
                    collision_ball.speed = current_ball.speed * ball_ball_coef_restitution
                    collision_ball.direction = current_ball.direction
                    collision_ball.velocity = convert_speed_to_velocity(current_ball.speed, current_ball.direction)
                    collision_ball.rolling_speed = collision_ball.calc_rolling_speed()
                    
                    current_ball.velocity_prev = current_ball.velocity
                    current_ball.speed_prev = current_ball.speed
                    current_ball.velocity = np.array([0,0])
                    current_ball.speed = 0
                else:
                    # TODO: Account for collision at an angle, 
                    # Only some momentum is transferred based on to angle of attack
                    pass
            else:
                # TODO: ACCOUNT FOR MULTIBALL COLLISIONS
                pass
    
# PRINT PARAMETERS FOR ALL BALLS ON TABLE
# CHECK WHAT IS GOING ON WITH ANGLE
def simulate_single_strike(table_state, dt = 0.1):
    """
    Computes the ball attributes following a single strike of the cue ball

    Parameters
    ----------
    table_state : class
        Table state instance describing balls on table.

    Returns
    -------
    Updated ball positions
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
    # if num_balls_on_table > 1:
    #     table_state.detect_update_ball_collision(dt, cue_ball, other_ball_positions, other_ball_positions_ids)
    
    # Update position history of cue ball
    cue_ball.update_position_history()
    
    iteration = 1
    while table_state.balls_in_motion_bool:
        print (f'\n\nIteration: {iteration}')
        
        balls_in_motion = table_state.balls_in_motion
            # Iterate through balls on table
        for ball in table_state.balls_in_motion:
            print (f'Ball name: {ball.name}')
            print (f'Ball speed: {ball.speed}')
            print (f'Ball motion: {ball.motion}')
            print (f'Ball velocity: {ball.velocity}')
            print (f'Ball position: {ball.position}')
            print (f'Previous ball position: {ball.position_prev}')
            # Check if ball is rolling or sliding and update state parameters accordingly
            if ball.speed > ball.rolling_speed:
                ball.slide(dt, update_position = True)
            else:
                print ('rolling')
                ball.motion = 'rolling'
                ball.roll(dt, update_position = True)
            
            # Detect table collision and update ball state variables if collision
            ball.detect_update_table_collision(dt)
            
            # TODO: Implement ball-ball collisions
            # Detect ball collision and update ball state variables if collision
            # if num_balls_on_table > 1:
            #     # Get positions of other balls (NOT INCLUDING CURRENT BALL)
            #     other_ball_positions, other_ball_positions_ids = table_state.get_ball_positions(ball.id)
            #     table_state.detect_update_ball_collision(dt, ball, other_ball_positions, other_ball_positions_ids)

            ball.update_position_history()
                
        iteration += 1
        table_state.check_balls_in_motion()
        print ('-'*20)
    plot_ball_history(table_state)
    

def plot_ball_history(table_state):
    # # Plot results
    plt.figure(figsize = (table_width*5,table_length*5))
    
    for ball in table_state.balls_in_play:
        history = np.array(ball.position_history)
        alphas = np.linspace(0.1,1,len(history))
        plt.scatter(history[:,0], history[:,1], label = ball.name, alpha = alphas, s = 500)

    plt.xlim([-0.1,table_width + 0.1])
    plt.ylim([-0.1,table_length + 0.1])
    plt.vlines(table_width, 0, table_length, color = 'k')
    plt.hlines(table_length, 0, table_width, color = 'k')
    plt.vlines(0, 0, table_length, color = 'k')
    plt.hlines(0, 0, table_width, color = 'k')
    plt.legend()
    plt.grid()



verbose = False # Print parameters for debugging
cue_ball = Ball(position = [1.,1],v0 = [0,2], name = 'cue')
red_ball_1 = Ball([1,2],[0,0], 'red')
# red_ball_2 = Ball([1,3],[0,0], 'red')

table_state = Table([cue_ball,red_ball_1])#, red_ball_2])
# table_state = Table([cue_ball])

simulate_single_strike(table_state)
