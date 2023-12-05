#!/usr/bin/env python3
import rclpy
import math as m
import numpy as np
import time
from random import randint
import threading
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools
from unmanned_systems_ros2_pkg import ProNav


def get_mean_target(heading_list:list,distance_list:list) -> float:    # Output is rad   input is degree
    if len(heading_list) == 0:
        mean_target = 0
        target_dist = 0
    else:    
        mid = int(len(heading_list) // 2)
        mean_target = heading_list[mid]
        if mean_target > 180:
            mean_target -=360
        mean_target = np.deg2rad(mean_target)
        target_dist = distance_list[mid]

    return mean_target, target_dist

def get_global_heading(heading_target_rad:float, 
                        curent_yaw_rad:float):
    
    global_heading_rad = heading_target_rad + curent_yaw_rad
    
    if global_heading_rad > 2*np.pi:
        global_heading_rad = global_heading_rad - 2*np.pi
    elif global_heading_rad < 0:
        global_heading_rad = global_heading_rad + 2*np.pi
    
    #print("global heading deg", np.rad2deg(global_heading_rad))
    
    return global_heading_rad


def position_global(angle:float,distance:float,local_location:list) -> list:
    # angle is global, distance pursure to evader, local_location is pursuers xy locations in list
    position = [0.0, 0.0]
    position[0] = distance*np.cos(angle) + local_location[0]
    position[1] = distance*np.sin(angle) + local_location[1]
    return position

def velocity_math(new_position:list,old_position:list,dt:float) -> list:
    velocity = [0.0, 0.0]
    velocity[0] = (new_position[0]-old_position[0])/dt
    velocity[1] = (new_position[1]-old_position[1])/dt

    return velocity, new_position



def main() -> None:
    rclpy.init(args=None)
    turtlebot_pursuer = TurtleBotNode.TurtleBotNode('turtle', 'pursuer')
    turtlebot_pursuer.move_turtle(0.15,0.0)    

    lidar_freq = 4.62 #hz


    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_pursuer, ), 
                              daemon=True)
    thread.start()
    rate = turtlebot_pursuer.create_rate(lidar_freq*3)
    
    pro_nav = ProNav.ProNav(3)
    dt = 1/lidar_freq
    old_cord = [0.0,0.0]
    while rclpy.ok():
        rate.sleep()
        theta_local,distance = get_mean_target(turtlebot_pursuer.detected_heading_angle_list,turtlebot_pursuer.detected_range_list)

        theta_global = get_global_heading(theta_local,turtlebot_pursuer.orientation_euler[2])
        
        global_position = position_global(theta_global,distance,turtlebot_pursuer.current_position)

        global_velocity,old_cord = velocity_math(global_position,old_cord,dt)

        local_velocity = position_global(turtlebot_pursuer.orientation_euler[2],turtlebot_pursuer.current_velocity[0],[0.0,0.0,0.0])

        pursuer_position = [turtlebot_pursuer.current_position[0],turtlebot_pursuer.current_position[1],0.0]

        current_poston = [turtlebot_pursuer.current_position[0],turtlebot_pursuer.current_position[1]]

        flight_path_rate, v_command = pro_nav.true_pro_nav(
            current_poston,global_position,dt,global_velocity,local_velocity, False, theta_global)

        turtlebot_pursuer.move_turtle(v_command,flight_path_rate)


        print(time.process_time(),"t",turtlebot_pursuer.current_position[0],"t",turtlebot_pursuer.current_position[1])

if __name__ == "__main__":
    main()