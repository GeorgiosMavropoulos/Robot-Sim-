"""
Simulation of a robot following a waypoint path with two PID controllers.

The Simulator receives a Map, a list of waypoints in metres and two
independent PID controllers: one turns the robot towards the next
waypoint (heading), the other sets its forward speed.

Design rationale
----------------
The simulator receives waypoints, not a planner. The path is computed once
(A* in planner.py) and handed to every run, so all gain sets are
compared on exactly the same path, and the control loop can be tested on
a hand-made path without any search.
Heading and speed use separate PID objects. A PID keeps an integral and a
previous error, so sharing one object between two signals would mix
radians with metres.
The constructor only validates and stores configuration. The robot and
the recorded results are created at the start of each run, so repeated
runs never leak state into each other.
run() only computes and records the data and opens no window. Everything
after it works on the dictionary that run() returns: the metrics
(metrics() and the methods it calls) and the visualisation (animate() with
pygame, plot() with matplotlib). main.py decides which of them to call, so
a run can be repeated without any window for experiments and tests.

Coordinate convention
---------------------
Positions are (x, y) in metres and angles are in radians, clockwise 
from the +x axis, wrapped to [-pi, pi) with angle_wrap.

Public API
----------
Simulator(grid_map, waypoints, heading_pid, speed_pid, dt=0.1,
          waypoint_threshold=0.15, goal_threshold=0.1, t_max=60.0,
          start_theta=0.0)
    Validate and store the configuration of one simulation.
Simulator.run() -> dict
    Run the simulation once and return the recorded data (keys are
    listed in the docstring of run()). Opens no window.

Metrics (all take the dict returned by run()):
Simulator.metrics(data) -> dict
    Collect all metrics of one run in a single dictionary: time,
    path_length, cte_mean, cte_max, heading_error_mean,
    heading_error_max, reached_goal, collisions.
   

Simulator.time_till_robot_reach_the_goal(data) -> float or None
    Time in seconds at which the goal was reached, None if it was not.
Simulator.calculate_path_distance(data) -> float
    Length in metres of the path travelled by the robot.
Simulator.cross_track_error(data) -> (float, float)
    Mean and maximum distance of the recorded trajectory from the
    planned path segments, in metres.

Visualisation (called from main.py after run()):
Simulator.animate(data, scale=150, fps=None) -> None
    Replay a recorded run in a pygame window.
Simulator.plot(data) -> None
    Show a matplotlib figure with the planned path and the robot
    trajectory; the title shows the gains of both PIDs.
Simulator.draw_map(screen, scale, ...) -> None
    Draw the occupancy grid on a pygame surface.
Simulator.world_to_screen(x, y, scale) -> (int, int)
    Convert a point in metres to pixel coordinates.

Geometry helpers (static methods):
Simulator.distance_between_two_waypoints(x, y) -> float
    Euclidean distance between two (x, y) points.
Simulator.compute_angle_between_start_and_goal(start, goal) -> float
    Direction from start to goal in radians, wrapped to [-pi, pi).
Simulator.heading_error(theta, desired_angle) -> float
    Wrapped difference desired_angle - theta. Positive means the target
    is to the left.
Simulator.distance_point_to_segment(point, a, b) -> float
    Shortest distance from a point to the segment a-b.

Internal helpers
----------------
Simulator._create_robot() -> Robot
    Build a new Robot at the first waypoint with heading start_theta.
Simulator._normalize_waypoints(waypoints) -> tuple
    Return the waypoints as a tuple of tuples (static method).

Simulator._validate_init_args(grid_map, waypoints, heading_pid, speed_pid,
          dt, waypoint_threshold, goal_threshold, t_max, start_theta)
    Validate the arguments of __init__ (static method).

Simulator._check_real(name, value, positive=False) -> None
    Check that a value is a finite real number, optionally > 0 (static).

Simulator._check_point(name, point) -> None
    Check that a point is a tuple of two finite real numbers (static).

Simulator._compute_heading_error_desired_angle_and_distance_between_two_waypoints(
          robot, target) -> (float, float)
    Heading error towards the target and distance from the robot to the
    final waypoint. The desired angle is computed inside the method.


Simulator.heading_error_stats(self, data) -> tuple[float, float]:
       Returns mean and maximum absolute heading error of one recorded run.
Control loop (see Simulator.run)
--------------------------------
Heading PID: wrapped heading error towards the next waypoint -> omega.
Speed PID: distance to the final waypoint -> v, scaled by
max(0, cos(heading error)). A waypoint is reached within
waypoint_threshold (goal_threshold for the last one). A run ends when the
goal is reached, the robot leaves the free space, or t_max is exceeded.
"""


from .map import Map
from numbers import Real
from collections.abc import Iterable
import pygame
# import the utility function angle_wrap to normalise angle values since they may get increased while robot moves
from .robot import Robot, angle_wrap

# PID class to implement the PID control algorithm
from .pid import PID
import math
import matplotlib.pyplot as plt


class Simulator:
    """
    Configuration of one waypoint-following simulation.

    Attributes:
        grid_map (Map): Occupancy grid used for collision and waypoint checks.
        waypoints (tuple[tuple[float, float]]): Path as (x, y) pairs in
            metres, at least two.
        heading_pid (PID): Controller that outputs the angular velocity.
        speed_pid (PID): Controller that outputs the forward velocity.
        dt (float): Time step in seconds.
        waypoint_threshold (float): Distance in metres at which an
            intermediate waypoint counts as reached.
        goal_threshold (float): Distance in metres at which the final
            waypoint counts as reached.
        t_max (float): Maximum simulated time in seconds.
        start_theta (float): Initial heading in radians, in [-pi, pi).
    """

    def __init__(self, grid_map, waypoints, heading_pid, speed_pid, dt=0.1,
                 waypoint_threshold=0.15, goal_threshold=0.1, t_max=60.0,
                 start_theta=0.0):
        """
        Validate and store the configuration of a simulation.

        All arguments are checked before anything is stored. The waypoints
        are copied into an immutable tuple of float pairs and the starting
        heading is wrapped to [-pi, pi).

        Args:
            grid_map (Map): Occupancy grid. Every waypoint must lie on a
                free cell of this map.
            waypoints (Iterable): The path as (x, y) pairs in metres. At
                least two are required, because the path is made of
                segments between consecutive points and the cross-track
                error is measured against those segments. Lists, tuples
                and one-shot iterables such as generators are accepted.
            heading_pid (PID): Controller for the heading error. Must not
                be the same object as speed_pid.
            speed_pid (PID): Controller for the forward speed. Must not be
                the same object as heading_pid.
            dt (float): Time step in seconds, finite and > 0.
                Defaults to 0.1.
            waypoint_threshold (float): Distance in metres at which an
                intermediate waypoint is reached, finite and > 0. Keep it
                larger than the distance travelled per step (v * dt).
                Defaults to 0.15.
            goal_threshold (float): Distance in metres at which the final
                waypoint is reached, finite and > 0. Defaults to 0.1.
            t_max (float): Maximum simulated time in seconds, finite and
                > 0. Defaults to 60.0.
            start_theta (float): Initial heading in radians, finite. It is
                wrapped with angle_wrap(). Defaults to 0.0.

        Raises:
            TypeError: If grid_map or a PID has the wrong type, waypoints
                is not an iterable of pairs, or a numeric argument is not a
                real number (booleans are rejected).
            ValueError: If both PIDs are the same object, a numeric
                argument is not finite or not in its allowed range, fewer
                than two waypoints are given, a waypoint does not have two
                coordinates, or a waypoint is outside the map or on an
                obstacle.

        Notes:
            Separate PID objects are required because each one stores its
            own integral and previous error.
        """

        # generalise waypoints
        normalized_waypoints = self._normalize_waypoints(waypoints)
        # validate that the input values on the class's instance are valid
        self._validate_init_args(grid_map, normalized_waypoints, heading_pid, speed_pid,
                                 dt, waypoint_threshold, goal_threshold, t_max, start_theta)

        self.grid_map = grid_map
        self.waypoints = tuple((float(x), float(y)) for x, y in normalized_waypoints)
        self.heading_pid = heading_pid
        self.speed_pid = speed_pid
        self.dt = float(dt)
        self.waypoint_threshold = float(waypoint_threshold)
        self.goal_threshold = float(goal_threshold)
        self.t_max = float(t_max)

        # match the heading convention used by Robot.
        self.start_theta = angle_wrap(start_theta)

    @staticmethod
    def _validate_init_args(grid_map, waypoints, heading_pid, speed_pid, dt,
                            waypoint_threshold, goal_threshold, t_max, start_theta):
        """Validate the arguments of __init__. Raises on the first problem."""
        if not isinstance(grid_map, Map):
            raise TypeError("grid_map must be an instance of Map.")
        for name, pid in (("heading_pid", heading_pid),
                          ("speed_pid", speed_pid)):
            if not isinstance(pid, PID):
                raise TypeError(f"{name} must be an instance of PID.")
        if heading_pid is speed_pid:
            raise ValueError("heading_pid and speed_pid must be different PID instances.")

        Simulator._check_real("dt", dt, positive=True)
        Simulator._check_real("t_max", t_max, positive=True)
        Simulator._check_real("waypoint_threshold", waypoint_threshold, positive=True)
        Simulator._check_real("goal_threshold", goal_threshold, positive=True)
        Simulator._check_real("start_theta", start_theta)

        if len(waypoints) < 2:
            raise ValueError(f"Waypoints must contain at least 2 points; received {len(waypoints)}.")

        for i, point in enumerate(waypoints):
            if len(point) != 2:
                raise ValueError(f"Waypoint {i} must contain exactly two coordinates.")
            x, y = point
            Simulator._check_real(f"waypoints[{i}].x", x)
            Simulator._check_real(f"waypoints[{i}].y", y)
            if not grid_map.is_free(float(x), float(y)):
                raise ValueError(f"Waypoint {i} ({x}, {y}) is outside the map or inside an obstacle.")

    # method to generalise the waypoints
    @staticmethod
    def _normalize_waypoints(waypoints):
        """Return waypoints as a tuple of tuples (consumes one-shot iterables)."""
        if isinstance(waypoints, (str, bytes)) or not isinstance(waypoints, Iterable):
            raise TypeError("waypoints must be an iterable of (x, y) pairs.")
        return tuple(tuple(p) if isinstance(p, Iterable) and not isinstance(p, (str, bytes)) else p
                     for p in waypoints)

    @staticmethod
    def distance_between_two_waypoints(x, y):
        """Euclidean distance between two (x, y) points."""
        Simulator._check_point("x", x)
        Simulator._check_point("y", y)
        return math.dist(x, y)

    @staticmethod
    def compute_angle_between_start_and_goal(start, goal):
        """Angle from start to goal in radians, wrapped to [-pi, pi)."""
        Simulator._check_point("start", start)
        Simulator._check_point("goal", goal)
        return angle_wrap(math.atan2(goal[1] - start[1], goal[0] - start[0]))

    @staticmethod
    def heading_error(theta, desired_angle):
        """Wrapped desired_angle - theta. Positive means the target is to the left."""
        Simulator._check_real("theta", theta)
        Simulator._check_real("desired_angle", desired_angle)
        return angle_wrap(desired_angle - theta)

    def _create_robot(self)-> Robot:
        """
        Create and initialise a new robot at the first waypoint.

        The robot's initial position is set to the coordinates of the first
        waypoint in self.waypoints, and its initial orientation is set to
        self.start_theta.

        This method creates a new Robot instance without advancing the
        simulation or modifying the recorded simulation data.

        Args:
            None

        Returns:
            Robot: A new robot instance initialised at the first waypoint with
            the configured starting orientation.
        """
        # take the first item from the waypoints list
        first_waypoint = self.waypoints[0]
        # unpack the item in 2 variables
        start_x, start_y = first_waypoint
        # initialise the robot
        robot = Robot(start_x, start_y, self.start_theta)

        return robot


    def _compute_heading_error_desired_angle_and_distance_between_two_waypoints(self, robot, target):
        """
        Compute the heading error towards the target and the distance to the goal.

        The desired angle is computed here from the robot's position to the
        target, so the caller does not need to compute it separately.

        Args:
            robot (Robot): Robot with the attributes x, y and theta.
            target (tuple): (x, y) in metres, the current target waypoint.

        Returns:
            tuple[float, float]: (error_theta, distance_to_goal), where
                error_theta is the wrapped difference in radians between the
                direction to the target and the robot's heading (positive means
                the target is to the left), and distance_to_goal is the distance
                in metres from the robot to the final waypoint.
        """
        position = (robot.x, robot.y)
        desired_angle = self.compute_angle_between_start_and_goal(position, target)
        error_theta = self.heading_error(robot.theta, desired_angle)
        distance_to_goal = self.distance_between_two_waypoints(position, self.waypoints[-1])
        return error_theta, distance_to_goal

    
    def run(self) ->dict:
        """
        Run one waypoint-following simulation and return the recorded data.

        A new Robot is created at the first waypoint with heading
        start_theta, and both PID controllers are reset. Repeated calls on
        the same Simulator therefore start from the same state and give the
        same result.

        At every time step the method:
            1. Takes the current target waypoint (waypoints[1] at the start).
            2. Checks whether the target is reached. The final waypoint uses
               goal_threshold and ends the run. An intermediate waypoint uses
               waypoint_threshold and the target moves to the next one
               without consuming simulated time.
            3. Computes the heading error towards the target (wrapped to
               [-pi, pi)) and the distance from the robot to the final
               waypoint.
            4. Passes the heading error to heading_pid to get the angular
               velocity command omega_com (rad/s), and the distance to the
               goal to speed_pid to get the forward velocity command
               v_com (m/s).
            5. Scales v_com by max(0, cos(heading_error)), so the robot
               slows down in sharp turns and does not drive forward when the
               target is behind it. v_com is never negative.
            6. Advances the robot with the unicycle model (Robot.step) and
               checks the new position against the map.
            7. Records the new state.

        The loop stops at the first of these events:
            - the final waypoint is reached within goal_threshold
              (reached_goal is True);
            - the new position is outside the map or inside an obstacle
              (collision_counter is 1; that position is not recorded);
            - the simulated time reaches t_max (reached_goal is False).

        Args:
              None

        Returns:
            dict: Recorded results. Every list has one entry per recorded
            state and starts with the initial state at t = 0.0, so the first
            entries of "v_com" and "omega_com" are 0.0 placeholders.

            - "t" (list[float]): Simulated time in seconds.
            - "x", "y" (list[float]): Robot position in metres.
            - "theta" (list[float]): Robot heading in radians, in [-pi, pi).
            - "v_com" (list[float]): Forward velocity command in m/s.
            - "omega_com" (list[float]): Angular velocity command in rad/s.
            - "reached_goal" (list[bool]): False everywhere except possibly
              the last entry, which is True if the goal was reached.
            - "index" (list[int]): Index of the target waypoint used at
              each step.
            - "collision_counter" (int): 1 if the run ended in a collision,
              otherwise 0.
            - "heading_error" (list[float]): Heading error towards the target in radians, wrapped to [-pi, pi).
  

        Raises:
            ValueError: If a controller or the robot produces a non-finite
                value (propagated from PID.update() or Robot.step()).
        """
        # create a fresh robot and reset both controllers.
        robot = self._create_robot()
        self.heading_pid.reset()
        self.speed_pid.reset()

         #calculate the heading error towards the target and the distance to the final waypoint
        start_error, distance_to_goal = self._compute_heading_error_desired_angle_and_distance_between_two_waypoints(robot, self.waypoints[1])
      
        # recorded data: every field is a list, with one value per time step.
        robot_data = {
            "t": [0.0],
            "x": [robot.x],
            "y": [robot.y],
            "theta": [robot.theta],
            "heading_error":[start_error],
            "v_com": [0.0],
            "omega_com": [0.0],
            "reached_goal": [False],
            "index": [1],
        }

        t = 0.0
        index = 1
        collision_counter = 0
        reached_goal = False

        # run this loop while time didn't reach the maximum allowed time and the robot didn't reach the goal
        while t < self.t_max and not reached_goal:
            # current target waypoint.
            goal = self.waypoints[index]

            # calculate the distance from the robot's current position to the target.
            distance = self.distance_between_two_waypoints(
                (robot.x, robot.y), goal
            )

            # check whether the robot has reached the final goal.
            if index == len(self.waypoints) - 1:
                if distance <= self.goal_threshold:
                    # set reached goal to True if robot reached the goal
                    reached_goal = True
                    robot_data["reached_goal"][-1] = True  # change the last value from the list to True
                    break
            elif distance <= self.waypoint_threshold:
                # move to the next waypoint.
                index += 1
                continue

            
            #calculate the heading error towards the target and the distance to the final waypoint and 
            # a second distance from the robot's position to the last waypoint of the path
            error_theta, distance_to_goal = self._compute_heading_error_desired_angle_and_distance_between_two_waypoints(robot, goal)

          
            # calculate the controller commands.
            omega_com = self.heading_pid.update(error_theta, self.dt)  # update the angular error
            v_com = self.speed_pid.update(distance_to_goal, self.dt)

            # the coefficient of max(0, cos(error)) slows down the robot on turns
            speed_factor = max(0.0, math.cos(error_theta))
            v_com *= speed_factor

            # prevent from commanding negative forward speed.
            v_com = max(0.0, v_com)

            # calculate the next robot pose using unicycle kinematics and check its new position.
            robot.step(v_com, omega_com, self.dt)

            # stop if the next position is outside the map or in an obstacle.
            if not self.grid_map.is_free(robot.x, robot.y):
                collision_counter += 1
                break

            # increment simulation time.
            t += self.dt

            # record this time step.
            robot_data["t"].append(t)
            robot_data["x"].append(robot.x)
            robot_data["y"].append(robot.y)
            robot_data["theta"].append(robot.theta)
            robot_data['heading_error'].append(error_theta)
            robot_data["v_com"].append(v_com)
            robot_data["omega_com"].append(omega_com)
            robot_data["reached_goal"].append(reached_goal)
            robot_data["index"].append(index)

        # return the recorded simulation results.
        robot_data["reached_goal"][-1] = reached_goal
        robot_data["collision_counter"] = collision_counter

        return robot_data

    @staticmethod
    def distance_point_to_segment(point, a, b)-> float:
        """
        Shortest distance from a point to the line segment a-b.

        The point is projected onto the infinite line through a and b. The
        projection parameter t is clamped to [0, 1], so a point beyond one
        end of the segment is measured to that end point and not to the
        extended line.

        Args:
            point (tuple): (x, y) in metres.
            a (tuple): (x, y) start of the segment.
            b (tuple): (x, y) end of the segment.

        Returns:
            float: Distance in metres.
        """
        px, py = point
        ax, ay = a
        bx, by = b

        abx, aby = bx - ax, by - ay
        length_sq = abx * abx + aby * aby

        # degenerate segment (a == b): distance to the single point
        if length_sq == 0:
            return math.dist(point, a)

        # t = 0 at a, t = 1 at b
        t = ((px - ax) * abx + (py - ay) * aby) / length_sq
        t = max(0.0, min(1.0, t))

        closest = (ax + t * abx, ay + t * aby)
        return math.dist(point, closest)


    def time_till_robot_reach_the_goal(self,data):
        """
           If the robot reached the goal, it returnes the time the robot made to reach the goal.

           Args:
                 Data: dictionary returned from run()

           Returns:
                   time in seconds or 'robot didn't reach the goal' if robot never reached the goal
                   
        
        """
        #validate that data is a dict returned by run method
        if not isinstance(data, dict) or "t" not in data or "reached_goal" not in data:
                    raise TypeError("Data must contain the 't' and the True/False value if robot reached the goal")


        #return None if the last value of reached_goal list's is False
        if data['reached_goal'][-1] is not True:
            return 'Failed to reach the goal point'
        else: 
            #if reached_goal == True return the last time value from the list
            return data['t'][-1]

        


    def calculate_path_distance(self,data)-> float:
        """
        Length of the path travelled by the robot, in metres.

        Args:
            data (dict): Result of run(); uses data["x"] and data["y"].

        Returns:
            float: Sum of the straight-line distances between consecutive
            recorded positions, in metres.

        Raises:
            TypeError: If data is not the dict returned by run().
            ValueError: If data["x"] and data["y"] are empty or differ in length.
        """
        #raise an error if data is not a dictionary
        if not isinstance(data, dict) or "x" not in data or "y" not in data:
                    raise TypeError("data must be the dict returned by run().")

        #raise a ValueError if lists are empty or not equally long
        if len(data["x"]) == 0 or len(data['y']) ==0 or len(data['x']) != len(data['y']):
                    raise ValueError("data['x'] and data['y'] must be non-empty and equally long.")

        #declare an empty value to update it later
        length = 0.0

        for i in range(len(data["x"]) - 1):
            current = data['x'][i],data['y'][i]
            next_position = data['x'][i + 1] , data['y'][i+ 1] 
            length += math.dist(current,next_position)

        return length


    def heading_error_stats(self, data) -> tuple[float, float]:
        """
        Mean and maximum absolute heading error of one recorded run.

        The first entry of data["heading_error"] is the initial error before
        the robot moves, so it is skipped: the metric measures the controller,
        not the choice of start_theta.

        Args:
            data (dict): Result of run(); uses data["heading_error"].

        Returns:
            tuple[float, float]: (mean_abs_error, max_abs_error) in radians.
            (0.0, 0.0) if the robot made no step.

        Raises:
            TypeError: If data is not the dict returned by run().
        """
        if not isinstance(data, dict) or "heading_error" not in data:
            raise TypeError("data must be the dict returned by run().")

        abs_errors = [abs(e) for e in data["heading_error"][1:]]
        if not abs_errors:
            return 0.0, 0.0
        return sum(abs_errors) / len(abs_errors), max(abs_errors)



    def metrics(self, data) -> dict:
        """
        Collect the performance metrics of one recorded run.

        Calls the existing metric methods and reads two values from the
        recorded data. 

        Args:
            data (dict): Result of run().

          Returns:
            dict: "time" (float or str), "path_length" (float, m),
            "cte_mean" (float, m), "cte_max" (float, m),
            "heading_error_mean" (float, rad),
            "heading_error_max" (float, rad),
            "reached_goal" (bool), "collisions" (int).
 

        Raises:
            TypeError: If data is not a dict (the other methods check the keys).
        """
        #TypeError if data is not a dict
        
        required = ("x", "y", "reached_goal", "collision_counter")
        if not isinstance(data, dict) or not all(key in data for key in required):
            raise TypeError("data must be the dict returned by run().")


        time_to_goal = self.time_till_robot_reach_the_goal(data) #call the time method
        path_length = self.calculate_path_distance(data)         #call the path method
        cte_mean, cte_max = self.cross_track_error(data)    #call cross_track_error
        reached = data['reached_goal'][-1]  # last item of data["reached_goal"]
        heading_mean, heading_max = self.heading_error_stats(data) #take the heading error mean and max value
        collisions = data["collision_counter"] #collisions computed number from data list

        return {                      
            "time": time_to_goal,
            "path_length": path_length,
            "cte_mean": cte_mean,
            "cte_max": cte_max,
            "heading_error_mean": heading_mean,
            "heading_error_max": heading_max,
            "reached_goal": reached,
            "collisions": collisions,
        }





    def cross_track_error(self, data) -> tuple[float, float]:
        """Mean and maximum distance (m) of the recorded trajectory from the planned path.

        For every recorded position, the error is the distance to the nearest
        segment of self.waypoints. Statistics are taken over all positions.

        Args:
            data (dict): Result of run(); uses data["x"] and data["y"].

        Returns:
            tuple[float, float]: (mean_error, max_error) in metres.

        Raises:
            TypeError: If data is not the dict returned by run(), or a value
                is not a real number.
            ValueError: If data["x"] and data["y"] are empty, differ in length,
                or contain non-finite values.
        """
        if not isinstance(data, dict) or "x" not in data or "y" not in data:
            raise TypeError("data must be the dict returned by run().")

        xs, ys = data["x"], data["y"]
        if len(xs) == 0 or len(xs) != len(ys):
            raise ValueError("data['x'] and data['y'] must be non-empty and equally long.")

        for i, (x, y) in enumerate(zip(xs, ys)):
            Simulator._check_real(f"data['x'][{i}]", x)
            Simulator._check_real(f"data['y'][{i}]", y)

        segments = list(zip(self.waypoints[:-1], self.waypoints[1:]))
        errors = [
            min(self.distance_point_to_segment((x, y), a, b) for a, b in segments)
            for x, y in zip(xs, ys)
        ]
        return sum(errors) / len(errors), max(errors)

    def world_to_screen(self, x, y, scale)-> tuple[int,int]:
        """
        Convert a point in metres to pixel coordinates.

        The map origin is the top-left corner of the window, x grows to the
        right and y grows downwards, the same as rows in the grid, so no
        flipping is needed.

        Args:
            x (float): Horizontal position in metres.
            y (float): Vertical position in metres.
            scale (float): Pixels per metre.

        Returns:
            tuple[int, int]: (px, py) pixel position.
        """
        return int(round(x * scale)), int(round(y * scale))

    def draw_map(self, screen, scale, free_color=(245, 245, 245),
                 obstacle_color=(40, 40, 40), grid_color=(210, 210, 210)):
        """
        Draw the occupancy grid on a pygame surface.

        Every cell is drawn as a filled square: obstacle cells in
        obstacle_color and free cells in free_color, with a thin outline in
        grid_color on free cells.

        Args:
            screen (pygame.Surface): Surface to draw on.
            scale (float): Pixels per metre. The window size should be
                cols * cell_size * scale by rows * cell_size * scale.
            free_color (tuple): RGB colour of free cells.
            obstacle_color (tuple): RGB colour of obstacle cells.
            grid_color (tuple): RGB colour of the cell outlines.

        Raises:
            TypeError: If screen is not a pygame.Surface, scale is not a
                real number, or a colour is not an RGB tuple.
            ValueError: If scale is not finite and greater than zero, or a
                colour channel is outside 0-255.
        """
        # raise Type error if screen is not a pygame surface
        if not isinstance(screen, pygame.Surface):
            raise TypeError("screen must be a pygame.Surface.")

        Simulator._check_real("scale", scale, positive=True)

        # raise an error if colours are not RGB tuples or channels are not integers in range of 0-255
        for name, color in (("free_color", free_color),
                            ("obstacle_color", obstacle_color),
                            ("grid_color", grid_color)):
            if not isinstance(color, tuple) or len(color) != 3:
                raise TypeError(f"{name} must be an (R, G, B) tuple.")
            for channel in color:
                if isinstance(channel, bool) or not isinstance(channel, int):
                    raise TypeError(f"{name} channels must be integers.")
                if not 0 <= channel <= 255:
                    raise ValueError(f"{name} channels must be in 0-255.")

        cell_px = self.grid_map.cell_size * scale

        for row in range(self.grid_map.rows):
            for col in range(self.grid_map.cols):
                # round the corner and use ceil for the size, so there are
                # no one-pixel gaps between neighbouring cells
                rect = pygame.Rect(
                    round(col * cell_px),
                    round(row * cell_px),
                    math.ceil(cell_px),
                    math.ceil(cell_px),
                )
                if self.grid_map.is_free_cell((row, col)):
                    pygame.draw.rect(screen, free_color, rect)
                    pygame.draw.rect(screen, grid_color, rect, 1)
                else:
                    pygame.draw.rect(screen, obstacle_color, rect)

    def animate(self, data, scale=150, fps=None):
        """
        Replay a recorded run in a pygame window.

        The simulation is not recomputed. The method steps through the
        positions stored in data, one recorded state per frame, and draws
        the map, the planned path, the trajectory so far and the robot.
        When the last state is reached the final frame stays on screen
        until the window is closed. It is called from main.py after run().

        Args:
            data (dict): Result of run().
            scale (float): Pixels per metre. Defaults to 150, which gives a
                600 x 600 px window for the 4 m default map.
            fps (float or None): Frames per second. None replays in real
                time (1 / dt). Defaults to None.

        Raises:
            TypeError: If data is not a dict with the keys of run(), or
                scale or fps is not a real number.
            ValueError: If scale or fps is not finite and greater than zero.
        """
        # raise TypeError if data is not a returned dictionary which occurs from run()
        if not isinstance(data, dict) or not all(
            key in data for key in ("x", "y", "theta", "t")
        ):
            raise TypeError("data must be the dict returned by run().")

        Simulator._check_real("scale", scale, positive=True)

        # if fps is set to None, swap its value to 1.0 / dt
        if fps is None:
            fps = 1.0 / self.dt
        Simulator._check_real("fps", fps, positive=True)

        # start pygame
        pygame.init()
        try:
            # calculate width/height and set up screen
            width = int(self.grid_map.cols * self.grid_map.cell_size * scale)
            height = int(self.grid_map.rows * self.grid_map.cell_size * scale)
            screen = pygame.display.set_mode((width, height))
            pygame.display.set_caption("Robot path tracking")
            clock = pygame.time.Clock()

            i = 0  # index of the state shown now
            last = len(data["t"]) - 1
            running = True

            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                screen.fill((255, 255, 255))
                self.draw_map(screen, scale)

                # draw the planned path (self.waypoints) with pygame.draw.lines and self.world_to_screen
                planned_points = [self.world_to_screen(x, y, scale) for x, y in self.waypoints]
                pygame.draw.lines(
                    screen,        # surface
                    (0, 0, 255),   # blue
                    False,         # do not close the path
                    planned_points,
                    3              # line width
                )

                # draw the trajectory up to step i (data["x"][:i + 1], data["y"][:i + 1])
                # +1 is used since python's slicing excludes the last item
                trajectory_points = [self.world_to_screen(x, y, scale)
                                     for x, y in zip(data["x"][:i + 1], data["y"][:i + 1])]
                # draw only when traj points are at least 2
                if len(trajectory_points) >= 2:
                    # 255,0,0 is red
                    pygame.draw.lines(screen, (255, 0, 0), False, trajectory_points, 2)

                # draw the robot at (data["x"][i], data["y"][i]) and a short line in the direction data["theta"][i]
                robot_pos = self.world_to_screen(data["x"][i], data["y"][i], scale)

                # robot body, 0,180,0 is green
                pygame.draw.circle(screen, (0, 180, 0), robot_pos, 7)

                # direction indicator
                theta = data["theta"][i]
                length = 20

                # compute end of x position and y
                end_x = data["x"][i] + length / scale * math.cos(theta)
                end_y = data["y"][i] + length / scale * math.sin(theta)

                direction_end = self.world_to_screen(end_x, end_y, scale)
                # 255, 165, 0 is orange
                pygame.draw.line(screen, (255, 165, 0), robot_pos, direction_end, 3)

                pygame.display.flip()
                clock.tick(fps)

                if i < last:
                    i += 1
        finally:
            pygame.quit()

    @staticmethod
    def _check_real(name, value, positive=False):
        """Check that value is a finite real number, optionally > 0.

        Booleans are rejected because bool is a subclass of int.

        Raises:
            TypeError: If value is a bool or not a real number.
            ValueError: If value is not finite, or not > 0 when positive=True.
        """
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f"{name} must be a real number.")
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite.")
        if positive and value <= 0:
            raise ValueError(f"{name} must be greater than zero.")

    @staticmethod
    def _check_point(name, point):
        """Check that point is a tuple of two finite real numbers."""
        if not isinstance(point, tuple):
            raise TypeError(f"{name} must be a tuple.")
        if len(point) != 2:
            raise ValueError(f"{name} must contain exactly two coordinates.")
        for coordinate in point:
            Simulator._check_real(f"Coordinates in {name}", coordinate)

    # helper method to plot robot's path, path PID and time
    def plot(self, data):
        """
        Plot the planned path against the robot trajectory.

        The title shows the gains of both controllers: heading_pid and
        speed_pid. It is called from main.py after run().
        """
        if not isinstance(data, dict) or "x" not in data or "y" not in data:
            raise TypeError("data must be the dict returned by run().")

        h = self.heading_pid
        s = self.speed_pid

        plt.plot([p[0] for p in self.waypoints],
                 [p[1] for p in self.waypoints],
                 "ro--", label="Planned path")
       
        plt.plot(data["x"], data["y"], "b-", label="Robot path")
        plt.axis("equal")
        plt.gca().invert_yaxis()  # y grows downwards, same as the pygame map
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.legend()
        plt.title(f"Heading PID (Kp={h.kp}, Ki={h.ki}, Kd={h.kd})\n"
                  f"Speed PID (Kp={s.kp}, Ki={s.ki}, Kd={s.kd})")
        plt.tight_layout() #tight layout so text is not overlapping
        plt.show()

