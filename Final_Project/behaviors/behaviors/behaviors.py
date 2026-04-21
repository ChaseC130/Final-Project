from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np


def rotate_xy_vector(xy:np.ndarray, theta:float):
    # Rotate a vector by the specified radians
    cT=np.cos(theta)
    sT=np.sin(theta)
    return np.matmul([[cT,-sT],[sT, cT]],xy)

# Example Code 
#   If you have a known target location and a current robot location, then 
#   this function creates an attractive force towards that known target 
#   where the magnitude is proportional to the distance to the target
def move_to_goal_location(target_xy:np.ndarray, current_xyT:np.ndarray):
    # Get the target
    deltaP=target_xy-current_xyT[:2]

    # Rotate into the robot-local frame
    vec = rotate_xy_vector(deltaP, -current_xyT[2])
    d_vec=np.sqrt(np.power(vec,2).sum())

    # No vectors > 1.0 magnitude
    if d_vec>1.0:
        vec=vec/d_vec

    return vec

def move_to_visual_target(ctr_x, object_width):
    #Parameters
    #   ctr_x: the location of the most likely target in pixels. 
    #   object_width: the width of the most likely target in pixels. 
    #Purpose:
    #   Generate an attractive force towards the detected target.
    #   The direction of the force should be towards the target.
    #   The magnitude of the force should be related to the object_width
 
    vec = np.zeros((2),dtype=float)
    
    # 640px wide image; adjust if different
    if object_width is None or object_width <= 0:
        return vec

    # STOP threshold (pixel width when close enough)
    STOP_WIDTH = 500.0   # <-- tune for stop distance from cone

    if object_width > STOP_WIDTH:
        return np.zeros((2), dtype=float)

    norm_x = (ctr_x - 320.0) / 320.0
    norm_x = np.clip(norm_x, -1.0, 1.0)

    forward = np.clip((STOP_WIDTH - object_width) / STOP_WIDTH, 0.0, 1.0)
    lateral = -0.8 * norm_x

    vec[0] = forward
    vec[1] = lateral

    return vec

def avoid_obstacles(obstacle_xy:np.ndarray):
    #Parameters:
    #     obstacle_xy is a numpy array of all detected obstacle points
    #           The array is in robot local coordinates, ordered as [x,y].
    #Purpose:
    # Apply the potential fields based avoid obstacles with defined
    #   SPHERE_OF_INFLUENCE and SAFETY_MARGIN,
    # Return a robot_local vector of [x,y] to indicate both 
    # the direction and magnitude of motion

    vec = np.zeros((2),dtype=float)
    if obstacle_xy is None or len(obstacle_xy) == 0: 
        return vec 
        
    SPHERE_OF_INFLUENCE = 0.8 
    K_REP = 0.8 
    for (x, y) in obstacle_xy: 
        r = np.sqrt(x*x + y*y) 
        if r < 1e-6: 
            continue 
        if r < SPHERE_OF_INFLUENCE: 
            r_hat = np.array([x, y]) / r 
            mag = K_REP * (1.0/r - 1.0/SPHERE_OF_INFLUENCE) / (r*r) 
            vec -= mag * r_hat 
            
    mag = np.linalg.norm(vec) 
    if mag > 1.0: 
        vec = vec / mag
        
    return vec

def swirl_obstacles(obstacle_xy:np.ndarray):
    #Parameters:
    #     obstacle_xy is a numpy array of all detected obstacle points
    #           The array is in robot local coordinates, ordered as [x,y].
    #Purpose:
    # Apply the potential fields based swirl obstacles with defined
    #   SPHERE_OF_INFLUENCE 
    # Return a robot_local vector of [x,y] to indicate both 
    # the direction and magnitude of motion

    vec = np.zeros((2),dtype=float)

    if obstacle_xy is None or len(obstacle_xy) == 0:
        return vec

    SPHERE_OF_INFLUENCE = 0.8
    K_SWIRL = 1.2

    for (x, y) in obstacle_xy:
        r = np.sqrt(x*x + y*y)
        if r < 1e-6 or r > SPHERE_OF_INFLUENCE:
            continue

        r_hat = np.array([x, y]) / r
        t_hat = np.array([-r_hat[1], r_hat[0]])
        direction = 1.0 if abs(y) > 1e-3 else 1.0
        mag = K_SWIRL * np.exp(-2.0 * r)

        vec += direction * mag * t_hat

    mag = np.linalg.norm(vec)
    if mag > 1.0:
        vec = vec / mag

    return vec

def random_motion():
    # Purpose:
    #  generate random motion
    #  Return a robot_local vector of [x,y] to indicate both 
    #  the direction and magnitude of motion    
    vec = np.zeros((2),dtype=float)
    global _rand_state

    if '_rand_state' not in globals():
        _rand_state = np.zeros(2)

    # Parameters 
    alpha = 0.95     # memory (higher = smoother, slower change)
    noise_scale = 1.0

    # Generate new random direction
    noise = np.random.randn(2)
    noise = noise / (np.linalg.norm(noise) + 1e-6)

    _rand_state = alpha * _rand_state + (1 - alpha) * noise
    mag = np.linalg.norm(_rand_state)
    if mag > 1.0:
        _rand_state = _rand_state / mag

    return noise_scale * _rand_state

def behavioral_coordination(visual_target,
                            obstacle_xy:list,
                            option='all',
                            goal_xy=None,
                            current_xyT=None):

    # Run each of the behaviors and combine them together
    # Return a robot_local vector of [x,y] to indicate both 
    # the direction and magnitude of motion    

    
    if visual_target is not None:
        mtg=move_to_visual_target(visual_target['center_x'],visual_target['width'])
    else:
        mtg=np.zeros(2)
        
    avoid=avoid_obstacles(obstacle_xy) 
    swirl=swirl_obstacles(obstacle_xy) 
    rand_vec=random_motion()

    w_mtg   = 2.0
    w_avoid = 0.4
    w_swirl = 0.3
    w_rand  = 0.2

    vec = (w_mtg * mtg +
           w_avoid * avoid +
           w_swirl * swirl +
           w_rand * rand_vec)

    mag = np.linalg.norm(vec)
    if mag > 1.0:
        vec = vec / mag

    return vec
