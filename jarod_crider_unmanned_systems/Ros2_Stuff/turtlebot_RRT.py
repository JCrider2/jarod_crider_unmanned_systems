#!/usr/bin/env python3
from re import S
import rclpy
import math 
import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import some_python_module
from unmanned_systems_ros2_pkg import PIDTemplate
from unmanned_systems_ros2_pkg import RRTFun2


def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9
    
def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class TurtleBotNode(Node):
    def __init__(self, ns=''):
        super().__init__('minimial_turtlebot')
        
        if ns != '':
            self.ns = ns
        else:
            self.ns = ns
                
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  10) # this is the publisher
        
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 10)     # This is the subsciber
        
        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw
        
    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
def main()->None:
    rclpy.init(args=None)           # Initialized ros2 node
    print("starting")

    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)
    

    time_now = get_time_in_secs(turtlebot_node)
    print("time now is", time_now)
    
    kp_ang = 0.8
    ki_ang = 0.00001
    kd_ang = 0.7
    dt_ang = 1/20
    pid_ang = PIDTemplate.PID(
        kp = kp_ang,
        ki = ki_ang,
        kd = kd_ang,
        dt = dt_ang)
    
    MAX_ANGULAR_SPEED = 2.84 # rad/sec

    obs_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4,
              5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
    
    obs_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7,
              7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
    obj_radi = 0.25
    f = open("demo.txt","w")
    start = (1,1)
    finish = (7,13)
    map = [(0,15),(0,15)]    
    robot_radius = 0.5
    step_size = 0.5
    print("creating path")
    wp_list = RRTFun2.RRT(map,obs_x,obs_y,obj_radi,start,finish,robot_radius,step_size)
    print("path created")
    heading_error_tol = np.deg2rad(2)#rads
    dist_error_tol = 0.1#mst
    print("way point list ", wp_list)
    f.write(str(wp_list))
    f.close()
    try:
        rclpy.spin_once(turtlebot_node)
        while rclpy.ok():
            for current_wp in wp_list:    
                rclpy.spin_once(turtlebot_node)                         
                dy = current_wp[1] - turtlebot_node.current_position[1] # distance from me to way point
                dx = current_wp[0] - turtlebot_node.current_position[0]
                desired_heading_rad = np.arctan2(dy,dx)
                current_heading_error_rad = pid_ang.compute_error(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2])
                    
                curr_dist_error = np.sqrt(dx**2+dy**2)
                
                while curr_dist_error >= dist_error_tol:       # check if out of distance tolerance, change heading
                    
                    # based off angular erro
                    angular_gains = pid_ang.get_gains(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2])
                    
                    # spin the bot
                    turtlebot_node.move_turtle(0.0,angular_gains)
                    #print("desired heading", desired_heading_rad)

                    rclpy.spin_once(turtlebot_node)

                    # Recalculate error for next step
                    current_heading_error_rad = pid_ang.compute_error(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2])
                    
                    while  abs(current_heading_error_rad) <= heading_error_tol:    # check if heading correct then sends forward
                        
                        dy = current_wp[1] - turtlebot_node.current_position[1] # distance from me to way point
                        dx = current_wp[0] - turtlebot_node.current_position[0]

                        curr_dist_error = np.sqrt(dx**2+dy**2)

                        current_heading_error_rad = pid_ang.compute_error( # this is for course correction while driving forward
                            desired_heading_rad,
                            turtlebot_node.orientation_euler[2])
                        
                        angular_gains = pid_ang.get_gains(      # this is for course correction while driving forward
                            desired_heading_rad,
                            turtlebot_node.orientation_euler[2])
                        
                        turtlebot_node.move_turtle(0.15,angular_gains)

                        rclpy.spin_once(turtlebot_node)
                        if curr_dist_error < dist_error_tol:
                            break
                turtlebot_node.move_turtle(0.0,0.0)

                #print("wp", current_wp[0], " ", current_wp[1])

                rclpy.spin_once(turtlebot_node)
            break
    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0,0.0)
       

if __name__ == '__main__':
    """apply imported function"""
    main()