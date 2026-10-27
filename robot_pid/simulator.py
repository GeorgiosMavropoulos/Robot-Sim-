#MAP function is going build the map.
from .map import Map 
from numbers import Real
from collections.abc import Iterable
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
        Initialise a robot simulation with a map, a waypoint path,
        and separate PID controllers for heading and speed.

        The constructor validates all input arguments before storing
        them as instance attributes. It also converts the waypoint
        sequence into an immutable tuple and normalises the initial
        heading to the interval [-pi, pi).

        Args:
            grid_map (Map):
                The occupancy grid map used by the simulator. The map
                must be a valid Map instance and is used to check
                whether each waypoint lies in free space.

            waypoints (Iterable[tuple[float, float]]):
                An iterable containing the robot's target waypoints.
                Each waypoint must contain exactly two finite real
                numbers, (x, y), expressed in metres. The iterable
                must contain at least two waypoints otherwisethe cross-track error, one of the metrics the assignment asks for, 
                with one waypoint there is no segment, so there is nothing to measure against. 
                

            heading_pid (PID):
                The PID controller responsible for correcting the
                robot's heading error. This controller must be a PID
                instance and must not be the same object as speed_pid.

            speed_pid (PID):
                The PID controller responsible for controlling the
                robot's linear speed. This controller must be a PID
                instance and must not be the same object as heading_pid.

            dt (float, optional):
                The simulation time step in seconds. Must be a finite
                real number strictly greater than zero.
                Defaults to 0.1 seconds.

            waypoint_threshold (float, optional):
                The distance tolerance, in metres, used to determine
                whether an intermediate waypoint has been reached.
                Must be a finite real number strictly greater than
                zero. Defaults to 0.15 metres.

            goal_threshold (float, optional):
                The distance tolerance, in metres, used to determine
                whether the final goal has been reached.
                Must be a finite real number strictly greater than
                zero. Defaults to 0.1 metres.

            t_max (float, optional):
                The maximum simulated duration in seconds. Must be
                a finite real number strictly greater than zero.
                Defaults to 60.0 seconds.

            start_theta (float, optional):
                The robot's initial orientation in radians. Must be a
                finite real number. The value is normalised using
                angle_wrap() so that the stored heading lies in
                [-pi, pi). Defaults to 0.0 radians.

        Raises:
            TypeError:
                If grid_map or either PID controller has an invalid
                type; if waypoints is not an acceptable iterable; if
                a waypoint is not an iterable pair; or if a numeric
                argument is not a real number or is a boolean.

            ValueError:
                If heading_pid and speed_pid refer to the same object,
                if a numeric argument is non-finite or outside its
                permitted range, if waypoints are empty, if a waypoint
                does not contain exactly two coordinates or if a
                waypoint lies outside the map or inside an obstacle.

        Notes:
            Separate PID instances are required because each controller
            maintains its own internal state, including integral and
            previous-error values. Sharing one instance would cause
            the heading and speed control calculations to interfere
            with each other.
        """

        #validate that the input values on the class's instance are valid
        self._validate_init_args(grid_map,waypoints,heading_pid,speed_pid,dt,waypoint_threshold,goal_threshold,t_max,start_theta)

        self.grid_map = grid_map
        self.waypoints = tuple(
            (float(x), float(y)) for x, y in waypoints
        )
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
          """
        Validate that a value is a finite real number, excluding booleans.

        This helper provides consistent numeric validation for the
        simulator's configuration parameters and waypoint coordinates.
        Booleans are explicitly rejected even though bool is a subclass
        of int in Python.

        Args:
            name (str):
                The name of the argument being validated. Used to
                produce an informative error message.

            value (Real):
                The value that must be a finite real number.

        Returns:
            None:
                The method returns normally when validation succeeds.

        Raises:
            TypeError:
                If value is a boolean or is not an instance of Real.

            ValueError:
                If value is NaN, positive infinity, or negative infinity.
        """

          if isinstance(value, bool) or not isinstance(value, Real):
                raise TypeError(f"{name} must be a real number.")

          if not math.isfinite(value):
                raise ValueError(f"{name} must be finite.")


    @staticmethod
    def _validate_positive_number(name, value):
            """
        Validate that a value is a finite real number greater than zero.

        First checks the value's type and finiteness, then checks that
        it is strictly positive. Zero and negative values are rejected.

        Args:
            name (str):
                The name of the argument being validated, used in
                error messages.

            value (Real):
                The value to validate.

        Returns:
            None:
                The method returns normally when validation succeeds.

        Raises:
            TypeError:
                If value is a boolean or is not a real number.

            ValueError:
                If value is non-finite or is less than or equal to zero.
            """

            Simulator._validate_finite_number(name, value)

            if value <= 0:
                raise ValueError(f"{name} must be greater than zero.")




    @staticmethod
    def _validate_init_args(grid_map,waypoints,heading_pid,speed_pid,dt,waypoint_threshold,goal_threshold,t_max,start_theta):
        """
        Validate the arguments supplied to Simulator.__init__().

        The method checks the types of the map and PID controllers,
        ensures that the two controllers are distinct objects, validates
        the numeric configuration values, and checks that all waypoints
        are well-formed and located in free map space.

        The waypoint iterable is materialised into a tuple exactly once.
        This allows one-shot iterables, such as generators, to be
        validated and then reused by the constructor.

        Args:
            grid_map (Map):
                The occupancy grid map used for waypoint validation.

            waypoints (Iterable):
                An iterable of waypoint coordinate pairs in metres.

            heading_pid (PID):
                The PID controller for heading correction.

            speed_pid (PID):
                The PID controller for linear speed control.

            dt (Real):
                The simulation time step, which must be positive.

            waypoint_threshold (Real):
                The positive distance tolerance for intermediate
                waypoints, in metres.

            goal_threshold (Real):
                The positive distance tolerance for the final goal,
                in metres.

            t_max (Real):
                The positive maximum simulation duration, in seconds.

            start_theta (Real):
                The initial heading in radians; must be finite.

        Raises:
            TypeError:
                If grid_map is not a Map instance, either controller
                is not a PID instance, waypoints is not an acceptable
                iterable, a waypoint is not an iterable, or a
                coordinate is not a real number or is a boolean.

            ValueError:
                If the PID arguments refer to the same object, a
                numeric parameter is invalid, the waypoint collection
                contains less than 2 waypoints, a waypoint does not contain exactly two
                coordinates, or a waypoint is not in free map space.
                      
            
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

        #validate waypoint container.
        if isinstance(waypoints, (str, bytes)):
            raise TypeError(
                "waypoints must be an iterable of (x, y) pairs."
            )

        if not isinstance(waypoints, Iterable):
            raise TypeError(
                "waypoints must be an iterable of (x, y) pairs."
            )

        #materialize generators once so they can be checked and reused.
        points = tuple(waypoints)

        #verify that tuple contains at least 2 elements
        if len(points) < 2:
            raise ValueError(
                "waypoints must be at least 2 ."
            )

 

        #validate each waypoint and check map occupancy.
        for i, point in enumerate(points):

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
                    f"Waypoint  must contain exactly two coordinates."
                )

            x, y = coordinates

            Simulator._validate_finite_number(
                f"waypoints[{i}].x", x
            )
            Simulator._validate_finite_number(
                f"waypoints[{i}].y", y
            )

            #reject points outside the map or inside obstacles.
            if not grid_map.is_free(float(x), float(y)):
                raise ValueError(
                    f"Waypoint {i} ({x}, {y}) is outside the map "
                    "or lies inside an obstacle."
                )

       


         











