import numpy as np
import collision

from corgipath.collision import BoundingVolumeHierarchy
from corgipath.search_space import DefaultHybridGrid, DefaultHybridNode
from corgipath.planning import HybridAstar

class HybridAstarPushPlanner(object):
    def __init__(self, grid_size, dtheta, safe_dist):
        """Initialize the HybridAstarPushPlanner.

        Args:
            stable_determinator (StageDeterminator): The stage determinator object.
            grid_size (float): The grid size in meters.
            dtheta (float): The direction resolution in rad.
        """
        
        # parameters
        self._grid_size = grid_size
        self._dtheta = dtheta
        self._safe_dist = safe_dist * 2
        self._corners = None
        self._obstacles = None

        # objects        
        self._planner = HybridAstar()

    def sample_grid(self, grid_size, theta_gap):
        grid = DefaultHybridGrid(dxy=grid_size, dtheta=theta_gap, node_type=DefaultHybridNode)
    
        # You can use the default successor template. Or you can customize it.
        grid.use_default_successor_template(max_heading_change=np.radians(30), allow_backward=False)
    
        return grid

    def _get_collision_system(self):
        """get the collision system.

        Args:
            contact_point (ContactPoint): Contact point.

        Returns:
            collision_system (BoundingVolumeHierarchy): Bounding volume hierarchy.
        """
        # Define static objects (obstacles) here.
        collision_system = BoundingVolumeHierarchy(bounds=self._corners)
        collision_system.add_obstacles(self._obstacles)
        collision_system.agent_collision = collision.Poly.from_box(collision.Vector(0, 0), self._safe_dist, self._safe_dist)
        
        return collision_system

    def update_map(self, map_corners, map_obstacles, **kwargs):
        """Update grid map.

        Args:
            map_corners (list): List of corners [xmin, xmax, ymin, ymax]
            map_obstacles (list): List of collision objects.
        """
        self._corners = map_corners
        self._obstacles = []
        for obs in map_obstacles:
            self._obstacles.append(collision.Circle(collision.Vector(obs[0], obs[1]), obs[2]))

    def plan(self, goal, target):
        """Get stable push path from a given contact point and goal.

        Args:
            contact_point (ContactPoint): ContactPoint object.
            goal (Tuple[float, float, float]): Goal X, Y, Theta.
            visualize (bool, optional): Whether to visualize the path. Defaults to False.

        Returns:
            path (List[Tuple[float, float, float]]): The path.
        """
        # Note: Planner plan with contact point frame 
        # but we want result with object frame
        # slider offset on the Node

        planner = HybridAstar()
        planner.collision_system = self._get_collision_system()
        planner.collision_system.build()
        planner.search_space = self.sample_grid(self._grid_size, np.radians(self._dtheta))
        planner.search_space.reset()
    
        waypoints = planner.solve(
            target, goal,
            )
        return waypoints
