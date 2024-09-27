import numpy as np
from utils.image_manager import ImageManager
from utils.path_generator import HybridAstarPushPlanner
from utils.utils import *

image = ImageManager("../video/Pushing_bead_two_swimmers 1.mp4")
pusher_n = 2
pusher_r = 3
safe_margin = 5

for i in range(10):
    for step in range(20):
        image.get_image()
    image.get_target()
    image.get_pushers(pusher_n)
    image.set_goal()
    image.set_obstacle()
    image.show_system()

    goal        = np.array([image.goal[0], image.goal[1], 0]).astype(float)
    target      = np.array([image.target[0], image.target[1], 0]).astype(float)
    target_r    = image.target_r
    pushers     = image.pushers
    obs         = image.obs
    map_corners = [0, image.map[0], 0, image.map[1]]

    pusher_distance = target_r * 0.8
    pusher_angle = np.arcsin((pusher_distance + pusher_r) / (target_r + pusher_r))

    # current object pose base contact point
    contact_point  = np.zeros(3)
    contact_vector = norm_v(goal - target)
    contact_angle  = np.arctan2(contact_vector[1], contact_vector[0])
    contact_point  = target - contact_vector * target_r

    target[2] = contact_angle
    goal[2]   = contact_angle

    planner = HybridAstarPushPlanner(
        grid_size = 8,
        dtheta    = 1,
        safe_dist = pusher_r + target_r + safe_margin
        )

    planner.update_map(
        map_corners   = map_corners,
        map_obstacles = obs,
        )

    best_path = planner.plan(
        goal            = tuple(goal),
        target          = tuple(target),
        )

    finger1 = []
    finger2 = []
    
    for point in best_path:
        v = np.asarray(point[0:2])
        finger1.append(v + (target_r + pusher_r) * unit(point[2] + np.pi + pusher_angle))
        finger2.append(v + (target_r + pusher_r) * unit(point[2] + np.pi - pusher_angle))

    image.show_path(best_path, finger1, finger2)
