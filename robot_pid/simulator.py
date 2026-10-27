#MAP function is going build the map.
from .map import Map 

#AStarplanner class which will be used to design the shortest path in the map
from .planner import AStarPlanner 

#import the utility function angle_wrap to normalise angle values since they may get increased while robot moves
from .robot import Robot, angle_wrap 

#PID class to implement the PID control algorithm
from .pid import PID 
import math
import matplotlib.pyplot as plt

class Simulator:
   def __init__(self, grid_map, waypoints, heading_pid, speed_pid,dt=0.1, waypoint_threshold=0.15, goal_threshold=0.1, t_max=60.0, start_theta=0.0):
        """
            Args:
                 grid_map: the MAP object
                 waypoints: the path as a list of (x, y) in metres
                 heading_pid: The PID that controls robot's turning
                 speed_pid: The PID that controls robot's speed
                 dt: Time steps in seconds
                 waypoint_threshold: how close (in metres) counts as 'reached this waypoint'
                 goal_threshold: how close counts as ''reached the final goal'
                 t_max: maximum simulated time in seconds before the run stops
                 start_theta: the robot's starting direction in radians

        
        """

        self.grip_map = grid_map
        self.waypoints = waypoints
        self.heading_pid = heading_pid
        self.speed_pid = speed_pid
        self.dt = dt
        self.waypoint_threshold = waypoint_threshold
        self.goal_threshold = goal_threshold
        self.t_max = t_max
        self.start_theta = start_theta

         












