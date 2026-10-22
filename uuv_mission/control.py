class PDController:
    """Represents a PD controller with stored previous error."""
    
    def __init__(self, Kp, Kd, prev_error = 0, time_period = 1):
        """Initialise proportional and derivative gains, as well as the first previous error (0 by default) 
        and time period between controller updates (1 by default)."""
        
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = prev_error
        self.time_period = time_period

    def control_action(self, curr_error):
        """Calculate control action and update stored previous error."""

        action = self.Kp * curr_error + self.Kd * (curr_error - self.prev_error)/self.time_period # Calculates derivative using backwards Euler

        self.prev_error = curr_error

        return action

    def reset_state(self):
        """Reset previous error to zero."""
         
        self.prev_error = 0

    # UNUSED
    def set_gains(self, Kp, Kd):
        """Change gains while preserving error history, useful for autotuning"""

        self.Kp = Kp
        self.Kd = Kd
    

# PID controller subclass that reuses the PD controller's error history and control calculation.
# Currently unused, and is present as an exercise/demonstration of class hierarchy
class PIDController(PDController):
    """Represents a PID controller with stored previous error and error integral."""
    
    def __init__(self, Kp, Kd, Ki, prev_error = 0, integral_error = 0, time_period = 1):
        """Initialise gains, as well as previous error and integrated error (both 0 by default)
        and time period between controller updates (1 by default)."""
        
        super().__init__(Kp, Kd, prev_error, time_period)
        
        self.Ki = Ki
        self.integral_error = integral_error

    def control_action(self, curr_error):
        """Calculate and add integral action term to the PD control action and update error integral."""
        
        self.integral_error += curr_error * self.time_period # Calculates integral according to ZOH
        
        # self.integral_error += (curr_error + self.prev_error) * self.time_period/2 # Calculates integral according to trapezium rule

        Int_action = self.Ki * self.integral_error
        
        PD_action = super().control_action(curr_error)

        action = PD_action + Int_action
            
        return action

    def reset_state(self):
        """Reset previous error and error integral to zero."""

        super().reset_state()

        self.integral_error = 0

    def set_gains(self, Kp, Kd, Ki):
        """Change gains while preserving error history, useful for autotuning."""
    
        super().set_gains(Kp, Kd)

        self.Ki = Ki










        