import numpy as np
import matplotlib.pyplot as plt
from behaviors import behavioral_coordination

# -----------------------
# Initial robot state
# -----------------------
x, y, theta = 0.0, 0.0, 0.0
dt = 0.1

trajectory = []

# -----------------------
# World objects
# -----------------------
cone_x, cone_y = 15.0, 15.5
obs_world = np.array([5.0, 5.0])   # obstacle directly in path

# -----------------------
# Simulation loop
# -----------------------
for i in range(300):

    # ---- Compute distance to cone ----
    dx = cone_x - x
    dy = cone_y - y
    dist_to_cone = np.sqrt(dx**2 + dy**2)

    # ---- Simulate YOLO width ----
    # Stop at 0.3 meters
    if dist_to_cone <= 0.3:
        sim_width = 400.0
    else:
        sim_width = 100.0 / (dist_to_cone + 0.1)

    # Convert cone from world → robot frame
    dx_cone = cone_x - x
    dy_cone = cone_y - y

    cone_robot_x = dx_cone * np.cos(-theta) - dy_cone * np.sin(-theta)
    cone_robot_y = dx_cone * np.sin(-theta) + dy_cone * np.cos(-theta)

    # Compute horizontal angle to cone
    angle_to_cone = np.arctan2(cone_robot_y, cone_robot_x)

    # Convert angle to pixel position (assume 60° FOV camera)
    FOV = np.deg2rad(60)
    pixel_offset = (angle_to_cone / (FOV / 2)) * 320.0

    center_x = 320.0 + pixel_offset
    center_x = np.clip(center_x, 0, 640)

    sim_yolo = {
        'center_x': center_x,
        'width': sim_width
}

    # ---- Convert obstacle from world → robot frame ----
    dx_obs = obs_world[0] - x
    dy_obs = obs_world[1] - y

    obs_robot = np.array([
        dx_obs * np.cos(-theta) - dy_obs * np.sin(-theta),
        dx_obs * np.sin(-theta) + dy_obs * np.cos(-theta)
    ])

    obstacles = np.array([obs_robot])

    # ---- Behavioral coordination ----
    vec = behavioral_coordination(sim_yolo, obstacles)

    v = vec[0]
    w = vec[1]

    # ---- Robot motion update (unicycle model) ----
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += w * dt

    trajectory.append([x, y])

trajectory = np.array(trajectory)

# -----------------------
# Plot results
# -----------------------
plt.figure()
plt.plot(trajectory[:, 0], trajectory[:, 1], label="Robot Path")
plt.scatter(cone_x, cone_y, c='blue', label="Cone")
plt.scatter(obs_world[0], obs_world[1], c='red', label="Obstacle")

plt.axis('equal')
plt.legend()
plt.title("Behavior-Based Navigation Simulation")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.show()
