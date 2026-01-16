#!/usr/bin/env python3
"""
Simple PID Controller for maintaining straight movement
"""

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0)):
        """
        Args:
            kp: Proportional gain (react to current error)
            ki: Integral gain (react to accumulated error)
            kd: Derivative gain (react to rate of change)
            output_limits: (min, max) output range
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        # State variables
        self.integral = 0.0
        self.previous_error = 0.0
        
    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.previous_error = 0.0
    
    def update(self, error, dt):
        """
        Calculate PID output based on error
        
        Args:
            error: Difference between target and current (e.g., yaw difference)
            dt: Time since last update (seconds)
            
        Returns:
            correction: PID output value
        """
        # P: Proportional term
        p_term = self.kp * error
        
        # I: Integral term (accumulated error over time)
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # D: Derivative term (rate of change)
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Clamp output to limits
        output = max(self.output_limits[0], min(output, self.output_limits[1]))
        
        # Save for next iteration
        self.previous_error = error
        
        return output


# Example usage in movement.py:
"""
# In __init__:
self.yaw_pid = PIDController(
    kp=0.5,    # Start with these, tune later
    ki=0.01,
    kd=0.1,
    output_limits=(-0.3, 0.3)  # Max ±30% speed adjustment
)
self.target_yaw = 0.0
self.last_pid_update = None

# In move_forward with correction:
def move_forward_straight(self):
    if self.last_pid_update is None:
        self.last_pid_update = self.get_clock().now()
        self.target_yaw = self.current_yaw  # Lock current heading
        return
    
    # Calculate dt (time since last update)
    current_time = self.get_clock().now()
    dt = (current_time - self.last_pid_update).nanoseconds * 1e-9
    self.last_pid_update = current_time
    
    # Calculate yaw error (how much we've drifted)
    yaw_error = self.target_yaw - self.current_yaw
    # Normalize to [-pi, pi]
    yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
    
    # Get PID correction
    correction = self.yaw_pid.update(yaw_error, dt)
    
    # Apply correction to motors
    base_speed = self.forward_speed
    left_speed = base_speed + correction   # If drifting right, speed up left
    right_speed = base_speed - correction  # and slow down right
    
    # Clamp speeds
    left_speed = max(0.0, min(1.0, left_speed))
    right_speed = max(0.0, min(1.0, right_speed))
    
    self.run_wheels("forward", left_speed, right_speed)
    
    self.get_logger().info(
        f"PID: error={yaw_error:.3f}, correction={correction:.3f}, "
        f"L={left_speed:.2f}, R={right_speed:.2f}"
    )
"""

