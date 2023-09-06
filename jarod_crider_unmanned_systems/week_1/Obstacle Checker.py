
"""
Obstacle checker
"""
import numpy as np
import matplotlib.pyplot as plt


class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
    def is_inside(self, curr_x:float, curr_y:float, robot_radius:float=0):
        """
        Compute euclidian distance from current location -> obstalc location
        stor this value: dist_from
        if dist_form > obstacle size
            return false
        otherwise return true
        """
        
        dist_from = np.sqrt((curr_x - self.x_pos)**2+ (curr_y - self.y_pos)**2)
        if dist_from > self.radius+robot_radius:
            return False
        return True
"""
    Writing function tips
        - The way to think of writing funciton is
            - What are my inputs
            - What are my outputs
            
            - What will happen inside the blackbox
            
        - is_postion_valid()
        - What are our inputs
            - agent positon
            - agent radius
            - Obstacle
            - Obstacles radius
            - X limits
            - Y limits
            
        - What are the outputs
            - True / False
            - True if not valid
            - False if valid
            
        - What will happen inside the blackbox/function?
            - Check if insdie/near obstacle -> Done
            - Check if outside the boundary
                - Check x pos:
                    compare to x min and x max
                        - If x_min > x_current
                            Return True
                        - if x_max < x_current
                            Return True
                        - Else False
                    Compare to y min and y max
                        - If y_min > y_current
                            return True
                        - IF y_max < y_carrent
                            return True
                        - Else false


"""

def is_not_valid(obst_list:list,x_curr:float,y_curr:float,agent_radius:float=0.0):
    
    
    for obs in obst_list:
        if obs.is_inside(x_curr, y_curr,agent_radius):
            print("You're dead at ", obs.x_pos, obs.y_pos)
            return True    
    
    
    # if (x_min > x_curr) or (x_max < x_curr):
    #     return True

    return False





if __name__ == '__main__':
    obstacle_positions = [(1,1),(4,4),(3,4),(5,0),(5,1),(0,7),(1,7),(2,7),(3,7)]
    obstacle_list = [] # Store obstacle classes
    obstacle_radius = 0.5
    
    
    # loop through positon of obstacles 
    for obs_pos in obstacle_positions:
        print("obstacle_postions",obs_pos)
        obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_list.append(obstacle)
    
    agent_x = 1
    agent_y = 5
    agent_radius = 0.5
    
    for obs in obstacle_list:
        print("\n")
        print("This obstacle position is ", obs.x_pos, obs.y_pos)
        if obs.is_inside(agent_x, agent_y):
            print("Your're inside the obstacle", obs.x_pos, obs.y_pos)
        else:
            print("You're outside the position", obs.x_pos, obs.y_pos)
            
            


            
            
    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)        
            
    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color="blue")
        ax.add_patch(obs_plot)
        
    agent_plot = plt.Circle((agent_x,agent_y),agent_radius, color="red")
    ax.add_patch(agent_plot)
    plt.show()
            
            