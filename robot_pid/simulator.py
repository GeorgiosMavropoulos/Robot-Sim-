"""
Simulation of a robot following a waypoint path with two PID controllers.

The ``Simulator`` receives a ``Map``, a list of waypoints in metres and two
independent ``PID`` controllers: one turns the robot towards the next
waypoint (heading), the other sets its forward speed.

Design rationale
----------------
The simulator receives waypoints, not a planner. The path is computed once
(A* in ``planner.py``) and handed to every run, so all gain sets are
compared on exactly the same path, and the control loop can be tested on
a hand-made path without any search.
Heading and speed use separate PID objects. A PID keeps an integral and a
previous error, so sharing one object between two signals would mix
radians with metres.
The constructor only validates and stores configuration. The robot and
the recorded results are created at the start of each run, so repeated
runs never leak state into each other.
Nothing here starts pygame, so runs can be headless.

Coordinate convention
---------------------
Positions are (x, y) in metres and angles are in radians, counter-clockwise
from the +x axis, wrapped to [-pi, pi) with ``angle_wrap``.

Public API
----------
Simulator(grid_map, waypoints, heading_pid, speed_pid, dt=0.1,
          waypoint_threshold=0.15, goal_threshold=0.1, t_max=60.0,
          start_theta=0.0)
    Validate and store the configuration of one simulation.

Internal helpers
----------------
Simulator._normalize_waypoints(waypoints)
Simulator._validate_init_args(grid_map, waypoints, heading_pid, speed_pid, dt=0.1,
          waypoint_threshold=0.15, goal_threshold=0.1, t_max=60.0,
          start_theta=0.0)

Simulator._validate_finite_number(name, value)
Simulator._validate_positive_number(name, value)


"""


from .map import Map 
from numbers import Real
from collections.abc import Iterable

#import the utility function angle_wrap to normalise angle values since they may get increased while robot moves
from .robot import Robot, angle_wrap 

#PID class to implement the PID control algorithm
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

    def __init__(self, grid_map, waypoints, heading_pid, speed_pid,dt=0.1, waypoint_threshold=0.15, goal_threshold=0.1, t_max=60.0, start_theta=0.0):
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

        #generalise waypoints
        normalized_waypoints = self._normalize_waypoints(waypoints)
        #validate that the input values on the class's instance are valid
        self._validate_init_args(grid_map,normalized_waypoints,heading_pid,speed_pid,dt,waypoint_threshold,goal_threshold,t_max,start_theta)

        self.grid_map = grid_map
        self.waypoints = tuple((float(x), float(y))for x, y in normalized_waypoints)
        self.heading_pid = heading_pid
        self.speed_pid = speed_pid
        self.dt = float(dt)
        self.waypoint_threshold = float(waypoint_threshold)
        self.goal_threshold = float(goal_threshold)
        self.t_max = float(t_max)

        #match the heading convention used by Robot.
        self.start_theta = angle_wrap(start_theta) 


    @staticmethod
    def _validate_finite_number(name, value):
          """Check that a value is a finite real number (booleans excluded).

        Booleans are rejected explicitly because bool is a subclass of int.

        Args:
            name (str): Name of the argument, used in the error message.
            value (Real): Value to check.

        Raises:
            TypeError: If value is a boolean or not a real number.
            ValueError: If value is NaN or infinite.
        """

          if isinstance(value, bool) or not isinstance(value, Real):
                raise TypeError(f"{name} must be a real number.")

          if not math.isfinite(value):
                raise ValueError(f"{name} must be finite.")


    @staticmethod
    def _validate_positive_number(name, value):
            """
            Check that a value is a finite real number greater than zero.

        Args:
            name (str): Name of the argument, used in the error message.
            value (Real): Value to check.

        Raises:
            TypeError: If value is a boolean or not a real number.
            ValueError: If value is not finite or is <= 0.
        """

            Simulator._validate_finite_number(name, value)

            if value <= 0:
                raise ValueError(f"{name} must be greater than zero.")




    @staticmethod
    def _validate_init_args(grid_map,waypoints,heading_pid,speed_pid,dt,waypoint_threshold,goal_threshold,t_max,start_theta):
        """    
            Validate the arguments of Simulator.__init__().

            Checks, in this order: the types of the map and the PIDs, that the
            two PIDs are different objects, the numeric settings, and the
            waypoints. It only validates and returns nothing.

            Args:
                grid_map (Map): Map the waypoints must lie on.
                waypoints (tuple): Already normalised by _normalize_waypoints(), a tuple of (x, y) pairs in metres.                   
                heading_pid (PID): Controller for the heading.
                speed_pid (PID): Controller for the speed.
                dt (Real): Time step in seconds, > 0.
                waypoint_threshold (Real): Intermediate waypoint tolerance in
                    metres, > 0.
                goal_threshold (Real): Final goal tolerance in metres, > 0.
                t_max (Real): Maximum simulated time in seconds, > 0.
                start_theta (Real): Initial heading in radians, finite.

            Raises:
                TypeError: If a type is wrong, a waypoint is not an iterable
                    pair, or a number is not a real number (booleans rejected).
                ValueError: If both PIDs are the same object, a number is not
                    finite or out of range, fewer than two waypoints are given,
                    a waypoint has other than two coordinates, or a waypoint is
                    outside the map or on an obstacle.
            """
       
        
            #validate object types.
        if not isinstance(grid_map, Map):
            raise TypeError(
                "grid_map must be an instance of Map."
            )

        if not isinstance(heading_pid, PID):
            raise TypeError(
                "heading_pid must be an instance of PID."
            )

        if not isinstance(speed_pid, PID):
            raise TypeError(
                "speed_pid must be an instance of PID."
            )

         #raise an error if speed_pid and heading_pid are the same objects
        if speed_pid is heading_pid:
              raise ValueError("speed_pid and heading_pid must be different PID instances")

        #validate time and distance parameters.
        Simulator._validate_positive_number("dt", dt)
        Simulator._validate_positive_number("t_max", t_max)

        Simulator._validate_positive_number(
            "waypoint_threshold", waypoint_threshold
        )
        Simulator._validate_positive_number(
            "goal_threshold", goal_threshold
        )

        Simulator._validate_finite_number(
            "start_theta", start_theta
        )

        # Validate waypoint container.
        if len(waypoints) < 2:
            raise ValueError(
                f"Waypoints must contain at least 2 points; "
                f"received {len(waypoints)}."
            )

        # Validate each waypoint and check map occupancy.
        for i, point in enumerate(waypoints):

            if isinstance(point, (str, bytes)):
                raise TypeError(
                    f"Waypoint {i} must contain two coordinates."
                )

            if not isinstance(point, Iterable):
                raise TypeError(
                    f"Waypoint {i} must be an (x, y) pair."
                )

            coordinates = tuple(point)

            if len(coordinates) != 2:
                raise ValueError(
                    f"Waypoint {i} must contain exactly two coordinates."
                )

            x, y = coordinates

            Simulator._validate_finite_number(
                f"waypoints[{i}].x", x
            )
            Simulator._validate_finite_number(
                f"waypoints[{i}].y", y
            )

            if not grid_map.is_free(float(x), float(y)):
                raise ValueError(
                    f"Waypoint {i} ({x}, {y}) is outside the map "
                    "or lies inside an obstacle."
                )
            


    #method to generalise the waypoints
    @staticmethod
    def _normalize_waypoints(waypoints):
        """
        Read the waypoints once and return them as a tuple of tuples.

        One-shot iterables such as generators are consumed here, so the
        result can be validated and stored without iterating the original
        argument a second time. Only the structure is converted; the
        values are checked later by _validate_init_args().

        Args:
            waypoints (Iterable): Waypoints as (x, y) pairs.

        Returns:
            tuple: One tuple per waypoint (an item that is not an iterable
            is left unchanged and rejected by the validator).

        Raises:
            TypeError: If waypoints is a string or bytes, is not an
                iterable, or a waypoint cannot be converted to a tuple.
        """

        if isinstance(waypoints, (str, bytes)):
            raise TypeError(
                "waypoints must be an iterable of (x, y) pairs."
            )

        if not isinstance(waypoints, Iterable):
            raise TypeError(
                "waypoints must be an iterable of (x, y) pairs."
            )

        try:
            return tuple(
                tuple(point) if not isinstance(point, (str, bytes))
                and isinstance(point, Iterable)
                else point
                for point in waypoints
            )
        except TypeError as exc:
            raise TypeError(
                "Each waypoint must be an iterable of coordinates."
            ) from exc
       


    @staticmethod     
    def distance_between_two_waypoints(x,y):
         """
            Calcuates the distance between two waypoints

            Args:
                 x: tuple containing the coordinates (x1, y1).
                 y: tuple containing the coordinates (x2, y2).

            Returns:
                 float: distance between 2 waypoints

            Raises:
                TypeError: If inputs are not tuples or coordinates are not numbers.
                ValueError: If tuples have invalid length
                            
         
         """
         #validate errors
         for name, point in (("x", x), ("y", y)):
          if not isinstance(point, tuple):
            raise TypeError(f"{name} must be a tuple.")

          if len(point) != 2:
            raise ValueError(f"{name} must contain exactly two coordinates.")

          for coordinate in point:
            if isinstance(coordinate, bool) or not isinstance(coordinate, Real):
                raise TypeError(
                    f"Coordinates in {name} must be real numbers."
                )

            if not math.isfinite(coordinate):
                raise ValueError(
                    f"Coordinates in {name} must be finite."
                )

         return math.dist(x, y)



    @staticmethod
    def compute_angle_between_start_and_goal(start,goal):
        """
        Computes the angle in radians of the line connecting the start
        waypoint to the goal waypoint using math.atan2().

        Args:
            start (tuple): Starting waypoint as (x, y).
            goal (tuple): Goal waypoint as (x, y).

        Returns:
            float: Angle in radians, measured from the positive x-axis.

        Raises:
            TypeError: If start or goal is not a tuple, or if a coordinate
                    is not a real number.
            ValueError: If either waypoint does not contain exactly two
                        coordinates, or if a coordinate is negative or
                        non-finite.
        """

        #raise TypeError if start and goal are not real numbers.
        for name, value in (("start", start), ("goal", goal)):
         if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f"{name} must be a real number.")

        #raise ValueError if start or goal are not finite numbers
         if not math.isfinite(value):
            raise ValueError(f"{name} must be finite.")

        
        #compute the coordinate differences
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        
        #compute the angle
        angle = math.atan2(dx, dy)

        return angle




    def heading_error(theta,desired_angle):

        """
         Computes the difference between current angle and the desired angle by substracting desired_angle - current_angle.
         Normalises the angle using the method angle_wrap(computed_difference).
         A positive error means the target is to the left.

         Args:
              theta: float 
              desired_angle: float

         Returns:
                 normalised_angle: float

         Raises:
               TypeError: 
                         theta or desired_angle is not a real number

                ValueError:
                            theta or desired_angle are not finite numbers
        
        """

        #raise an error if theta is not a Real number
        if isinstance(theta, bool) or not isinstance(theta, Real):
            raise TypeError("theta must be a real number")

        #raise an error if desired_angle is not a Real number
        if isinstance(desired_angle, bool) or not isinstance(desired_angle, Real):
            raise TypeError("desired_angle must be a real number")

        #raise an error if theta or desired_angle are not finite numbers
        if not math.isfinite(theta) or not math.isfinite(desired_angle):
            raise ValueError("Angles must be finite numbers")

        #computes the difference between current angle and the desired angle.
        difference = desired_angle - theta

        #normalise the difference 
        normalised_difference = angle_wrap(difference)

        return normalised_difference













