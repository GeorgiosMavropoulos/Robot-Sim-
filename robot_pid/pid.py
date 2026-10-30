
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
import math
class PID:
    def __init__(self,kp,ki,kd,output_limits):
        """Initialize the PID controller with gains and output constraints.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            output_limits (tuple or list, optional): A container with exactly two 
                elements (low, high) defining the controller bounds. Individual 
                elements can be None for single-sided constraints. Defaults to None.

        Raises:
            TypeError: If gains or limits are not finite numeric types (excluding bool) 
                or if output_limits is not a tuple/list.
            ValueError: If gains are infinite, output_limits does not contain two 
                values, or low is greater than or equal to high.
        """
        #validate initialization inputs
        self._validate_initialization(kp, ki, kd, output_limits)
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
    

    def _validate_initialization(self, kp, ki, kd, output_limits):
        """Perform full validation on constructor arguments.

        Args:
            kp, ki, kd (float): Controller tuning parameters.
            output_limits (any): Bounds passed during constructor instantiation.

        Raises:
            TypeError: For structural layout or type violations.
            ValueError: For numerical out-of-bound errors.
        """
        #gain parameters check
        for gain_name, gain_val in [("kp", kp), ("ki", ki), ("kd", kd)]:
            if isinstance(gain_val, bool) or not isinstance(gain_val, (int, float)):
                raise TypeError(f"Gain {gain_name} must be a float or int, got {type(gain_val).__name__}")
            if not math.isfinite(gain_val):
                raise ValueError(f"Gain {gain_name} must be a finite number.")

        #output limits validation checks
        if output_limits is not None:
            if isinstance(output_limits, bool) or not isinstance(output_limits, (tuple, list)):
                raise TypeError("output_limits must be a tuple or list")
            if len(output_limits) != 2:
                raise ValueError("output_limits must contain exactly two values (low, high)")
            
            low, high = output_limits
            if (isinstance(low, bool) or (low is not None and not isinstance(low, (int, float))) or
                isinstance(high, bool) or (high is not None and not isinstance(high, (int, float)))):
                raise TypeError("Limits must be numeric values or None")
                
            if low is not None and high is not None and low >= high:
                raise ValueError(f"Lower limit ({low}) must be strictly less than upper limit ({high})")

    
        
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
