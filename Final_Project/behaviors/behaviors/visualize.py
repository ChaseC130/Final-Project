import behaviors
import numpy as np
import matplotlib.pyplot as plt
import pdb
import argparse

DELTA_GRID=0.1
TARGET=[1.0,1.0]
TARGET_WIDTH=0.2 #meters
CAMERA_FX=1000
CAMERA_WIDTH=1080


def simulate_yolo_projection(object_location, object_width, camera_pose, focal_length=CAMERA_FX, img_width=CAMERA_WIDTH):
    """
    Simulates YOLO-like output for a single object given camera geometry.
    
    Parameters:
        object_location (tuple): (x, y) of object center in world coordinates.
        object_width (float): Real-world width of the object.
        focal_length (float): Camera focal length in pixels.
        image_width: width of the image in pixels.
        camera_pose (tuple): (x, y, theta) of camera in world coordinates.
    
    Returns:
        dict: {"x_centroid": float, "object_width_px": float}
    """

    # Unpack
    x_obj, y_obj = object_location
    x_cam, y_cam, theta = camera_pose
    
    # Relative position in world coordinates
    Xw = x_obj - x_cam
    Yw = y_obj - y_cam
    
    # Rotation matrix for yaw (theta)
    R = np.array([
        [np.cos(theta), np.sin(theta)],
        [-np.sin(theta), np.cos(theta)]
    ])
    
    # Transform into camera coordinates
    Xc, Yc = R @ np.array([Xw, Yw])
    
    # Identify column location of object centroid in image    
    col = (img_width/2) - (focal_length * Yc / Xc)
    
    # Project object width to image plane
    dist=np.sqrt(Xw*Xw+Yw*Yw)
    w_img = focal_length * object_width / dist
    
    return {"center_x": float(col), "width": float(w_img)}

def sample_mtg():
    target=np.array(TARGET,dtype=float)
    currP=np.zeros((3),dtype=float)
    all_vec=[]
    for x in np.arange(-1.5,1.5,DELTA_GRID):
        currP[0]=x
        for y in np.arange(-1.5,1.5,DELTA_GRID):
            currP[1]=y
            vec=behaviors.move_to_goal_location(target,currP)
            all_vec.append([x,y,vec[0],vec[1]])

    return all_vec

def sample_move_to_visual_target():
    all_vec=[]
    for x in np.arange(-1.5,1.5,DELTA_GRID):
        for y in np.arange(-1.5,1.5,DELTA_GRID):
            angle=np.arctan2(TARGET[1]-y,TARGET[0]-x)
            angle+=np.random.rand(1)[0]-0.5 # assign a random rotation for the robot            
            sim_yolo=simulate_yolo_projection(TARGET,TARGET_WIDTH,(x,y,angle))
            vec=behaviors.move_to_visual_target(sim_yolo['center_x'],sim_yolo['width'])
            vec2=behaviors.rotate_xy_vector(vec,angle)
            all_vec.append([x,y,vec2[0],vec2[1]])

    return all_vec

def sample_avoid():
    all_vec=[]
    obs=np.zeros((1,2),dtype=float)
    for x in np.arange(-1.5,1.5,DELTA_GRID):
        obs[0,0]=-x
        for y in np.arange(-1.5,1.5,DELTA_GRID):
            obs[0,1]=-y
            vec=behaviors.avoid_obstacles(obs)
            all_vec.append([x,y,vec[0],vec[1]])

    return all_vec

def sample_swirl():
    all_vec=[]
    obs=np.zeros((1,2),dtype=float)
    for x in np.arange(-1.5,1.5,DELTA_GRID):
        obs[0,0]=-x
        for y in np.arange(-1.5,1.5,DELTA_GRID):
            obs[0,1]=-y
            vec=behaviors.swirl_obstacles(obs)
            all_vec.append([x,y,vec[0],vec[1]])

    return all_vec

def sample_random():
    all_vec=[]
    for x in np.arange(-1.5,1.5,DELTA_GRID):
        for y in np.arange(-1.5,1.5,DELTA_GRID):
            vec=behaviors.random_motion()
            all_vec.append([x,y,vec[0],vec[1]])

    return all_vec

def sample_combo():
    all_vec=[]
    obs=np.zeros((2),dtype=float)
    for x in np.arange(-1.5,1.5,DELTA_GRID):
        obs[0]=-x
        for y in np.arange(-1.5,1.5,DELTA_GRID):
            obs[1]=-y
            angle=np.arctan2(TARGET[1]-y,TARGET[0]-x)
            angle+=np.random.rand(1)[0]-0.5 # assign a random rotation for the robot            
            sim_yolo=simulate_yolo_projection(TARGET,TARGET_WIDTH,(x,y,angle))
            obs_rotated=behaviors.rotate_xy_vector(obs,-angle).reshape((1,2))
            vec=behaviors.behavioral_coordination(sim_yolo,obs_rotated)
            vec2=behaviors.rotate_xy_vector(vec,angle)
            all_vec.append([x,y,vec2[0],vec2[1]])

    return all_vec

def plot_vector_field(all_vec):
    all_np=np.array(all_vec)
    d_vec=np.sqrt(np.power(all_np[:,2:],2).sum(1))
    plt.plot(all_np[:,0],all_np[:,1],'b.')
    for idx in range(all_np.shape[0]):
        if d_vec[idx]>10.0:
            vec=all_np[idx,2:]*DELTA_GRID/d_vec[idx]
            plt.plot([all_np[idx,0],all_np[idx,0]+vec[0]],
                    [all_np[idx,1],all_np[idx,1]+vec[1]],
                    'r')
        elif d_vec[idx]>0.01:
            vec=all_np[idx,2:]*DELTA_GRID            
            plt.plot([all_np[idx,0],all_np[idx,0]+vec[0]],
                    [all_np[idx,1],all_np[idx,1]+vec[1]],
                    'b')

    plt.plot(TARGET[0],TARGET[1],'c.',markersize=15)
    plt.plot(TARGET[0],TARGET[1],'k.',markersize=8)
    plt.plot(0,0,'r.',markersize=15)
    plt.plot(0,0,'k.',markersize=8)
    
if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('motion_type',type=str,help='Motion Type: [mtg,avoid,swirl,random,visual,combo]')
    args = parser.parse_args()    
    if args.motion_type=='mtg':
        all_vec=sample_mtg()
    elif args.motion_type=='visual':
        all_vec=sample_move_to_visual_target()
    elif args.motion_type=='avoid':
        all_vec=sample_avoid()
    elif args.motion_type=='swirl':
        all_vec=sample_swirl()
    elif args.motion_type=='random':
        all_vec=sample_random()
    elif args.motion_type=='combo':
        all_vec=sample_combo()
    else:
        print("Invalid motion type " + args.motion_type)
    plot_vector_field(all_vec)
    plt.title(args.motion_type)
    plt.show()