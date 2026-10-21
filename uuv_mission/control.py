# def PID_controller(curr_error, prev_error, Kp = 0.15, Kd = 0.6, Ki = 0, cumulative_error = 0):

#     control_action = Kp * curr_error + Kd * (curr_error - prev_error) + Ki * cumulative_error

#     return control_action


class PDController:
    """Represents a PD controller with stored previous error"""
    
    def __init__(self, Kp, Kd, prev_error = 0, time_period = 1):
        """Initialise proportional and deriviative gains, as well as the first previous error (0 by default)"""
        
        self.Kp = Kp
        self.Kd = Kd
        
        # The use of the following variables to store error history is analogous to using a static local variable in a C function
        self.prev_error = prev_error

        self.time_period = time_period

    def PD_control_action(self, curr_error):
        """Calculate control action"""

        control_action = self.Kp * curr_error + self.Kd * (curr_error - self.prev_error)/self.time_period

        self.prev_error = curr_error

        return control_action


# The following is an example of the use of subclasses and inheritance, and is unused.
# This implementation minimises rewriting code at the cost of readability
# Warning: Both PD and PID control action functions are exposed
class PIDController(PDController):
    """Represents a PID controller with previous error and integrated error history"""
    
    def __init__(self, Kp, Kd, prev_error, Ki, integral_error = 0):
        """Initialise integral gain, as well as integrated error (0 by default)"""
        super().__init__(Kp, Kd, prev_error)
        self.Ki = Ki

        self.integral_error = integral_error

    def PID_control_action(self, curr_error):
        """Calculate integral action term and add to the PD control action"""
        
        self.integral_error += curr_error * self.time_period # Calculates integral according to ZOH

        # self.integral_error += (curr_error + self.prev_error) * self.time_period/2 # Calculates integral according to trapezium rule

        Int_action = self.Ki * self.integral_error
        
        PD_action = self.PD_control_action(curr_error)

        control_action = PD_action + Int_action
            
        return control_action










        