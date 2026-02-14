
import numpy as np

def solve_ackermann(ackermann_percent, delta, track_width, wheelbase):
    """
    Calculates the left and right wheel steering angles based on Ackermann geometry.
    
    Args:
        ackermann_percent (float): Ackermann percentage (0 to 1, where 1 is 100%).
        delta (float): Average steering angle in radians.
        track_width (float): Track width in consistent units (e.g., ft).
        wheelbase (float): Wheelbase in consistent units (e.g., ft).
        
    Returns:
        tuple: (delta_left, delta_right) in radians.
    """
    
    if delta == 0:
        return 0.0, 0.0
    
    if ackermann_percent == 0:
        return delta, delta

    effective_wheelbase = wheelbase / ackermann_percent

    turn_radius = effective_wheelbase / np.tan(abs(delta))
    
    angle_inner = np.arctan(effective_wheelbase / (turn_radius - track_width / 2))
    angle_outer = np.arctan(effective_wheelbase / (turn_radius + track_width / 2))
    
    if delta > 0:

        delta_left = angle_inner
        delta_right = angle_outer
    else:
        delta_left = -angle_outer
        delta_right = -angle_inner
        
    return delta_left, delta_right