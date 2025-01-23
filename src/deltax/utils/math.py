import numpy as np
from typing import Tuple, List, Optional

def calculate_intercept_point(obj_pos: Tuple[float, float],
                            obj_vel: Tuple[float, float],
                            robot_pos: Tuple[float, float],
                            robot_speed: float) -> Optional[Tuple[float, float]]:
    """Calculate interception point for moving object
    
    Uses quadratic equation to solve intersection of:
    - Object linear motion
    - Robot maximum speed circle
    
    Args:
        obj_pos: Current object position (x,y)
        obj_vel: Object velocity vector (vx,vy)
        robot_pos: Robot position (x,y)
        robot_speed: Maximum robot speed
        
    Returns:
        Interception point (x,y) or None if no solution
    """
    
    # Convert to numpy arrays
    p = np.array(obj_pos)
    v = np.array(obj_vel)
    r = np.array(robot_pos)
    
    # Calculate quadratic equation coefficients
    # at^2 + bt + c = 0
    a = np.dot(v, v)
    b = 2 * np.dot(v, p - r)
    c = np.dot(p - r, p - r) - robot_speed * robot_speed
    
    # Solve quadratic equation
    discriminant = b*b - 4*a*c
    if discriminant < 0:
        return None  # No real solution
        
    # Get smallest positive time
    t1 = (-b + np.sqrt(discriminant)) / (2*a)
    t2 = (-b - np.sqrt(discriminant)) / (2*a)
    t = min(t for t in [t1, t2] if t > 0)
    
    # Calculate interception point
    point = p + v*t
    return (float(point[0]), float(point[1]))

def calculate_pick_trajectory(start: Tuple[float, float],
                            target: Tuple[float, float],
                            z_up: float,
                            z_down: float,
                            points: int = 10) -> List[Tuple[float, float, float]]:
    """Calculate pick motion trajectory
    
    Generates trajectory points for pick motion:
    1. Move up from start
    2. Move to target
    3. Move down to pick/place
    4. Move up after pick/place
    
    Args:
        start: Start position (x,y)
        target: Target position (x,y) 
        z_up: Z coordinate for move above
        z_down: Z coordinate for pick/place
        points: Number of points per segment
        
    Returns:
        List of trajectory points (x,y,z)
    """
    
    trajectory = []
    
    # Move up from start
    for i in range(points):
        t = i / (points-1)
        z = start[2] + t*(z_up - start[2])
        trajectory.append((start[0], start[1], z))
        
    # Move to target position
    for i in range(points):
        t = i / (points-1)
        x = start[0] + t*(target[0] - start[0])
        y = start[1] + t*(target[1] - start[1])
        trajectory.append((x, y, z_up))
        
    # Move down to pick/place
    for i in range(points):
        t = i / (points-1)
        z = z_up + t*(z_down - z_up)
        trajectory.append((target[0], target[1], z))
        
    # Move up after pick/place
    for i in range(points):
        t = i / (points-1)
        z = z_down + t*(z_up - z_down)
        trajectory.append((target[0], target[1], z))
        
    return trajectory

def calculate_robot_time(trajectory: List[Tuple[float, float, float]],
                        speed: float,
                        acceleration: float) -> float:
    """Calculate time to execute trajectory
    
    Args:
        trajectory: List of trajectory points
        speed: Maximum speed
        acceleration: Maximum acceleration
        
    Returns:
        Estimated time in seconds
    """
    
    total_time = 0
    prev_point = None
    
    for point in trajectory:
        if prev_point is not None:
            # Calculate distance
            dx = point[0] - prev_point[0]
            dy = point[1] - prev_point[1]
            dz = point[2] - prev_point[2]
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Calculate time with acceleration
            if distance > 0:
                # Time to accelerate/decelerate
                t_acc = speed / acceleration
                
                # Distance covered during acceleration
                d_acc = 0.5 * acceleration * t_acc * t_acc
                
                if distance <= 2*d_acc:
                    # Short move - triangular velocity profile
                    t = 2 * np.sqrt(distance / acceleration)
                else:
                    # Long move - trapezoidal velocity profile
                    t = 2*t_acc + (distance - 2*d_acc) / speed
                    
                total_time += t
                
        prev_point = point
        
    return total_time 