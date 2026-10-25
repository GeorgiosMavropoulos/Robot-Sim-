"""This file contains the PID algorithm which will control robot's forward velocity (speed) and heading (angular velocity).
   Design rationale: dual PID control
   The robot uses two independent PID controllers built from this PID class. A heading PID converts the wrapped heading error toward the next waypoint 
   into an angular velocity command, which is what allows a unicycle robot to follow the turns of A* path.
   The speed PID converts the distance to the final goal into a forward velocity command, in order to enable the robot to travel at speed alongside the
   path and decelerates smoothly on approach instead of overshooting the target.

   Controlling both signals gives smoother and more realistic path following than a fixed-speed robot, 
   since speed is also reduced during sharp turns (scaled by the cosine of the heading error), which limits cross-track error. 
   It also demonstrates that the PID class is general-purpose: 
   the same implementation is reused with different gains and output limits for two different control signals.
"""

class PID:
    def __init__(self,kp,ki,kd,output_limits):
        """
        P is responsible for the current error, I handles the accumulated errors (the steady-state errors) and D counts how fast the error changes.
        Output limits stop PID command in order not to exceed what the robot can do. PID computes P + I +D  and if the results is out of limits, output limits
        change the result in the closet limit. accumulated_error stores  the full error's value while previous error the previous errors.
        
        """

        self.kp= kp
       
        self.ki = ki
       
        self.kd = kd
        self.output_limits = output_limits
        self.previous_error = None
        self.accumulated_error = 0


   
    def reset(self):
       """Reset method resets the error count so each time a new expirement runs, the previous errors will get eraised"""
     
       self.accumulated_error= 0
       self.previous_error = None

