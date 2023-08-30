# -*- coding: utf-8 -*-


import numpy as np
import matplotlib.pyplot as plt
"""

    Hw 2 Problem 1
    
    Writing funciton timps:
        - The way to think of writing function is
            - what are my inputs
            - what are my outputs
            
            - What will happen inside the blackbox/funciton
            
    For function to is_insdie obstacle:
        -What are inputs
            - Obstacle loction [x,y]
            - Obstacle size     r-radius
            - Where are we   [x,y]
        - What are outputs
            -True/false
                - If inside return true
                - If outside return false
        - What will happen inside the black box
            - Compute euclidian distance from current location to obstacle location
            - store this value in a variable : dist_from
            - If dist_from > obstacle size: 
                return false
            - Otherwise return true
            
"""




def is_inside_obstacle(obs_x:float,obs_y:float,obs_radius:float,curr_x:float,curr_y:float) -> bool:
    """
    Compute euclidian distance from current location -> obstalc location
    stor this value: dist_from
    if dist_form > obstacle size
        return false
    otherwise return true
    """
    
    dist_from = np.sqrt((curr_x - obs_x)**2+ (curr_y - obs_y)**2)
    if dist_from > obs_radius:
        return False
    return True

min_x = 0
max_x = 10
min_y = 0
max_y = 10


obs_x = 5
obs_y = 5
obs_radius = 1

agent_x =  4
agent_y = 5

obstacle_plot = plt.Circle((obs_x,obs_y), obs_radius, color='blue')
fig, ax = plt.subplots()
ax.scatter(agent_x,agent_y, c='r')
ax.add_patch(obstacle_plot)
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
plt.show()


if is_inside_obstacle(obs_x,obs_y,obs_radius,agent_x,agent_y):
    print ("Your're dead = (")
else:
    print("You're Okay =)")
    


"""
Make the obstacle method better
"""

class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
    def is_inside(self, curr_x:float, curr_y:float):
        """
        Compute euclidian distance from current location -> obstalc location
        stor this value: dist_from
        if dist_form > obstacle size
            return false
        otherwise return true
        """
        
        dist_from = np.sqrt((curr_x - self.x_pos)**2+ (curr_y - self.y_pos)**2)
        if dist_from > self.radius:
            return False
        return True

          
some_obs = Obstacle(obs_x, obs_y, obs_radius)
   
 
if some_obs.is_inside(agent_x,agent_y):
    print ("Your're dead = (")
else:
    print("You're Okay =)")
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    