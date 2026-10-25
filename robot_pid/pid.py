"""PID controller used by the robot for path tracking.

Design rationale: dual PID control
----------------------------------
The robot uses two independent controllers built from the same ``PID``
class:

* Heading PID: converts the wrapped heading error toward the next waypoint
  into an angular velocity command (rad/s). This lets the unicycle robot
  follow the turns of the A* path.
* Speed PID: converts the distance to the final goal into a forward velocity
  command (m/s), so the robot travels at speed along the path and
  decelerates smoothly on approach instead of overshooting the target.

Controlling both signals gives smoother and more realistic path following
than a fixed-speed robot, since speed is also reduced during sharp turns
(scaled by the cosine of the heading error), which limits cross-track error.
Using the same class with different gains and output limits for two
different signals also shows that the implementation is general-purpose.

Public API
----------
PID(kp, ki, kd, output_limits=(None, None))
    Create a controller. Gains must be numbers; output_limits must be a
    (low, high) pair with low < high, where None means "no limit".
PID.update(error, dt) -> float
    Compute the control command for one time step. This is the method the
    simulator calls inside its loop.
PID.reset() -> None
    Clear the integral and the stored previous error. Call it before a new
    run when an existing controller object is reused.

Internal helpers (not part of the public API)
---------------------------------------------
PID._validate_init_args(kp, ki, kd, output_limits)
    Checks the constructor arguments and raises TypeError / ValueError.
PID._validate_inputs(error, dt)
    Checks the arguments of update() and raises ValueError.
PID._clamp(value) -> (float, bool)
    Cuts a value to output_limits and reports whether it was cut. The
    flag drives the anti-windup logic in update().
"""
import math
class PID:
    """Discrete PID controller with output limits and anti-windup.

    Converts an error value into a control command using proportional,
    integral and derivative terms. The class knows nothing about the robot:
    the caller computes the error and applies the returned command.

    Attributes:
        kp, ki, kd (float): Proportional, integral and derivative gains.
        output_limits (tuple): (low, high) bounds of the output; None means
            no limit on that side.
        accumulated_error (float): Running sum of error * dt (the integral).
        previous_error (float or None): Error of the previous call, used by
            the derivative term. None before the first call.

    Example:
        heading_pid = PID(2.0, 0.0, 0.5, output_limits=(-2, 2))
        omega_cmd = heading_pid.update(heading_error, dt)
    
    """


    def __init__(self,kp,ki,kd,output_limits=(None,None)):
       
        """Create a controller with a clean internal state.

        Args:
            kp (float): Proportional gain. Reacts to the current error.
            ki (float): Integral gain. Reacts to the error accumulated over time.
            kd (float): Derivative gain. Reacts to how fast the error changes.
            output_limits (tuple): (low, high) bounds for the output. Use None
                for a side with no limit, e.g. (0, None). Defaults to
                (None, None), i.e. no limits.

        Raises:
            TypeError: If kp, ki or kd is not a number.
            ValueError: If output_limits is not a pair, or if low >= high.
        """
        
        self.kp= kp 
        self.ki = ki      
        self.kd = kd
        self.output_limits = output_limits #default value is None,None
        self.previous_error = None
        self.accumulated_error = 0
        #run initialization validation first
        self._validate_init_args(self.kp,self.ki,self.kd,self.output_limits)


    #helper function to handle error during initialization
    @staticmethod
    def _validate_init_args(kp,ki,kd,output_limits,):
         """Validate the constructor arguments.

        Args:
            kp, ki, kd: Gains to check; each must be an int or float.
            output_limits: Must have exactly two elements. If both are
                not None, the first must be smaller than the second.

        Raises:
            ValueError: If output_limits is not a pair or low >= high.
            TypeError: If any gain is not a number.
        """
         #return a value error if output_limits is not a pair of two (None,None)
         if len(output_limits) != 2:
             raise ValueError('Output limits must be (value,value)')

         
         #check if kp, ki, kd are numbers
         for val, name in [(kp, 'Kp'), (ki, 'Ki'), (kd, 'Kd')]:
            if not isinstance(val, (int, float)):
                raise TypeError(f'{name} must be a float or integer number')
         
        #if user set values to output_limits (low,max) verify low < high and high > low
         low, high = output_limits
         if low is not None and high is not None:
            if low >= high:
                raise ValueError('Output limit pair should be (min, max) where min < max')
         
        
                 
         
   
    def reset(self):
       """Clear the controller's memory.

        Sets the accumulated integral to 0 and forgets the previous error.
        Call this before every new run on a reused controller, or when the
        target changes, so history from the previous run does not affect the
        next one. Gains and output limits are not changed.
        """
     
       self.accumulated_error= 0
       self.previous_error = None


    
    def update(self,error,dt):
        """Compute the control command for one time step.

        Calculates output = kp*error + ki*integral + kd*derivative, where
        integral is the running sum of error*dt and derivative is
        (error - previous_error)/dt. On the first call after construction or
        reset(), the derivative term is 0. The result is clamped to
        output_limits. If the output saturates, the integral increase of
        this step is undone (anti-windup).

        Args:
            error (float): Current error (setpoint minus measurement), in
                the units of the controlled quantity (e.g. rad for heading).
            dt (float): Time since the previous call, in seconds. Must be > 0.

        Returns:
            float: Control command (e.g. angular velocity in rad/s), within
                output_limits.

        Raises:
            ValueError: If error is None, NaN or infinite, or if dt <= 0.
        """
        #call this helper method to validate that error is not NaN or infinite and dt is > 0
        self._validate_inputs(error, dt)

        #store accumulated error's value into a variable in order to return to this value if _PID exceeds output_limit's values
        original_value_accumulated_error =self.accumulated_error 
        #compute P's error
        P = self.kp * error
        #compute I's error
        #add into accumalated_error variable the outcome of error *dt
        self.accumulated_error += error * dt
        I = self.ki * self.accumulated_error
        #calculate D's error

        #set D's value to zero if previous error is none
        if self.previous_error is None:
                    D = 0
        else:
            #if previous error is not none calculate D
            derivative = (error - self.previous_error)/dt
            D = self.kd * derivative 
      
        #add error's value into previous_error variable
        self.previous_error = error
                
        #calculate pid
        _PID = P + I +D

        #validate whether _PID is inside output_limits
              
        output, saturated = self._clamp(_PID)
        if saturated:
            #anti-windup: undo this step's integral increase
            self.accumulated_error = original_value_accumulated_error
        return output       


   #################### Helper Methods ########################################
        
    #helper method to validate update's inputs
    @staticmethod
    def _validate_inputs(error, dt):
        """Validate the arguments of update().

        Args:
            error (float): Must be a finite number (not None, NaN or inf).
            dt (float): Time step in seconds; must be greater than 0.

        Raises:
            ValueError: If either condition is violated. A NaN error is
                rejected because it would permanently corrupt the integral.
        """
    #validate that error is not NaN or infinite
        if error is None or not math.isfinite(error):
           raise ValueError('Error cannot be none or inf')
        
        if dt <= 0: #dt must be greater than zero, system will raise a value error if it's not
           raise ValueError('dt must be greater than 0')

        
     
    #helper method to clamp value to output_limits
    def _clamp(self, value):
        """Cut a value to output_limits.

        Args:
            value (float): Raw controller output (P + I + D).

        Returns:
            tuple: (clamped_value, saturated). saturated is True if the
            value was outside the limits and had to be cut; update() uses
            it to decide whether to apply anti-windup.
        """
        low, high = self.output_limits
        if low is not None and value < low:
            return low, True
        if high is not None and value > high:
            return high, True
        return value, False
        
        
