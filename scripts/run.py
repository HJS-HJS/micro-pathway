import os
import numpy as np
import yaml
import csv
from utils.image_manager import ImageManager
from utils.path_generator import HybridAstarPushPlanner
from utils.utils import *


if not os.path.exists(os.path.abspath('.') + "/../path/"):
    os.makedirs(os.path.abspath('.') + "/../path/")

# Get config file
with open(os.path.abspath('.') + "/../config/config.yaml") as f:
    config = yaml.load(f,Loader=yaml.FullLoader)

image = ImageManager(os.path.abspath('.') + "/../video/" + config["video_name"])
pusher_n = config["pusher"]["number"]
pusher_r = config["pusher"]["radius"]
safe_margin = 5

pusher_angle = np.zeros(pusher_n)

for step in range(config["frame"]):
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

if pusher_n == 2:
    pusher_distance = target_r * 0.8
    pusher_angle[0] = np.arcsin((pusher_distance + pusher_r) / (target_r + pusher_r))
    pusher_angle[1] = -np.arcsin((pusher_distance + pusher_r) / (target_r + pusher_r))
else: 
    pusher_distance = target_r + pusher_r
    pusher_angle[0] = 0

# current object pose base contact point
contact_point  = np.zeros(3)
contact_vector = norm_v(goal - target)
contact_angle  = np.arctan2(contact_vector[1], contact_vector[0])
contact_point  = target - contact_vector * target_r

target[2] = contact_angle
goal[2]   = contact_angle

planner = HybridAstarPushPlanner(
    grid_size = 4,
    dtheta    = np.deg2rad(config["pusher"]["max_rot_angle"]),
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
if len(best_path) == 0:
    i = 1
    while i < 4:
        best_path = planner.plan(
            goal            = tuple(goal   + [0, 0, np.deg2rad(15) * i]),
            target          = tuple(target + [0, 0, -np.deg2rad(15) * i]),
            )
        if len(best_path) != 0: break
        
        best_path = planner.plan(
            goal            = tuple(goal   + [0, 0, np.deg2rad(15) * i]),
            target          = tuple(target + [0, 0, -np.deg2rad(15) * i]),
            )
        if len(best_path) != 0: break

        i += 1

if len(best_path) == 0:
    raise Exception("Path is not generated")

fingers      = []
if pusher_n == 2:
    fingers.append([])
    fingers.append([])
    for point in best_path:
        v = np.asarray(point[0:2])
        fingers[0].append(v + (target_r + pusher_r) * unit(point[2] + np.pi + pusher_angle[0]))
        fingers[1].append(v + (target_r + pusher_r) * unit(point[2] + np.pi + pusher_angle[1]))
else:
    fingers.append([])
    for point in best_path:
        v = np.asarray(point[0:2])
        fingers[0].append(v + (target_r + pusher_r) * unit(point[2] + np.pi + pusher_angle[0]))

for idx, finger in enumerate(fingers):
    with open(os.path.abspath('.') + "/../path/" + "finger_" + str(idx) + ".csv", "w") as f:
        csv.writer(f).writerows(finger)
        f.close()

with open(os.path.abspath('.') + "/../path/" + "target.csv", "w") as f:
    f.write("target position\n")
    f.write(str(target[0:2]) + "\n")
    f.write("goal position\n")
    f.write(str(goal[0:2]) + "\n")
    f.close()

image.show_path(best_path, fingers)

