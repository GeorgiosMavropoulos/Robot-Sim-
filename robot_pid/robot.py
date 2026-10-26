
import math
"""
Kinematic Model in a 2D space

-------------------------------------------------------------
This module contains the Robot class in order to simulate a moving robot on space (x,y,theta).
Using Euler method (Forward Euler Integration), robot's state gets updated in discrete time steps (dt)
based on current linear (v) and angular (omega) speed. 
It contains strict error handling mechanisms to avoid non-finite values (NaN/Inf) and negative dt ensuinrg stability during the simulation.

"""
#utility function
def angle_wrap(a):
    """Wraps an angular value to the interval [-pi, pi].

    This utility ensures that orientation angles stay within a standardized 
    range, preventing arithmetic overflow and keeping trigonometric behaviors 
    consistent.

    Args:
        a (float): The input angle in radians.

    Returns:
        float: The wrapped angle bounded within [-pi, pi].
    """

    if not math.isfinite(a):
         raise ValueError('a value must be a finite number')
    
    return (a +math.pi) %(2 * math.pi) - math.pi
class Robot:

    """Represents a mobile robot moving in a 2D kinematic space.

    Tracks the robot's pose (x, y coordinates and heading angle) and simulates
    its motion using differential kinematics over discrete time steps.

    Attributes:
        x (float): The horizontal position of the robot in meters.
        y (float): The vertical position of the robot in meters.
        robot_size (float): The physical bounding radius or size of the robot.
        theta (float): The orientation heading of the robot in radians, 
            automatically wrapped to [-pi, pi].
    """
    def __init__(self, x, y,theta):

        """Initializes a new Robot instance with a specific 2D pose.

        Args:
            x (float): Initial horizontal position.
            y (float): Initial vertical position.
            theta (float): Initial heading angle in radians.
        """
        self.x =x #horizontal robot's position
        self.y = y #vertical robot's position        
        #wrap the angle
        self.theta  = angle_wrap(theta)

        #raise an error if x,y and theta are not valid values
        if  not (math.isfinite( self.x) and math.isfinite( self.y) and math.isfinite(self.theta)):
                    raise ValueError('Initial pose coordinates (x, y, theta) must be finite numbers.')


    def step(self, v, omega, dt):
        """
        Updates the robot's state over a time interval dt.
        
        :param v: Linear velocity (meters/second)
        :param omega: Angular velocity (radians/second)
        :param dt: Time step (seconds)
        Position is being updated with the old self.theta and the robot turns
        """

       
        if  not (math.isfinite(v) and math.isfinite(omega) and math.isfinite(dt)):
            raise ValueError('Invalid step values')

         #raise an error if dt is <= 0
        if dt <=0:
             raise ValueError('dt cannot be less or equal to zero')
        
        #calculate the change in orientation (omega * dt)
        delta_theta = omega * dt
        
        #update the robot's position based on its current heading
        #(Using standard Euler integration for simplicity)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        
        #update the global orientation angle
        self.theta += delta_theta
        self.theta= angle_wrap(self.theta)
        


