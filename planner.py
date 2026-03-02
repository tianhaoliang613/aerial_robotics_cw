"""
Mission Planner Module

Core module for the structural inspection path planning challenge.
Handles scenario parsing, obstacle representation, collision detection,
TSP solving, and path planning.
"""

import yaml
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional


@dataclass
class Viewpoint:
    """A viewpoint the drone must visit."""
    id: int
    x: float
    y: float
    z: float
    yaw: float  # w in the scenario file (radians)

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


@dataclass
class Obstacle:
    """An axis-aligned cuboid obstacle."""
    id: int
    x: float      # center x
    y: float      # center y
    z: float      # base z (bottom face)
    width: float   # extent in x (w in yaml)
    depth: float   # extent in y (d in yaml)
    height: float  # extent in z (h in yaml)

    @property
    def min_corner(self) -> np.ndarray:
        """Lower corner of the AABB."""
        return np.array([
            self.x - self.width / 2,
            self.y - self.depth / 2,
            self.z
        ])

    @property
    def max_corner(self) -> np.ndarray:
        """Upper corner of the AABB."""
        return np.array([
            self.x + self.width / 2,
            self.y + self.depth / 2,
            self.z + self.height
        ])


@dataclass
class Scenario:
    """Parsed scenario data."""
    name: str
    drone_start: np.ndarray
    viewpoints: Dict[int, Viewpoint]
    obstacles: Dict[int, Obstacle]


def load_scenario(file_path: str) -> Scenario:
    """Load and parse a scenario YAML file."""
    with open(file_path, 'r') as f:
        raw = yaml.safe_load(f)

    start = raw['drone_start_pose']
    drone_start = np.array([start['x'], start['y'], start['z']])

    viewpoints = {}
    for vid, vp in raw.get('viewpoint_poses', {}).items():
        viewpoints[int(vid)] = Viewpoint(
            id=int(vid),
            x=vp['x'], y=vp['y'], z=vp['z'],
            yaw=vp['w']
        )

    obstacles = {}
    for oid, obs in raw.get('obstacles', {}).items():
        obstacles[int(oid)] = Obstacle(
            id=int(oid),
            x=obs['x'], y=obs['y'], z=obs['z'],
            width=obs['w'], depth=obs['d'], height=obs['h']
        )

    return Scenario(
        name=raw.get('name', 'unknown'),
        drone_start=drone_start,
        viewpoints=viewpoints,
        obstacles=obstacles,
    )


def segment_intersects_aabb(
    p0: np.ndarray,
    p1: np.ndarray,
    box_min: np.ndarray,
    box_max: np.ndarray,
    buffer: float = 0.3
) -> bool:
    """
    Test if the line segment p0->p1 intersects an AABB (with safety buffer).
    Uses the slab method for ray-AABB intersection.

    Args:
        p0: Start point (3D)
        p1: End point (3D)
        box_min: Lower corner of the AABB
        box_max: Upper corner of the AABB
        buffer: Safety margin around the box

    Returns:
        True if the segment intersects the buffered AABB
    """
    bmin = box_min - buffer
    bmax = box_max + buffer

    d = p1 - p0
    t_min = 0.0
    t_max = 1.0

    for i in range(3):
        if abs(d[i]) < 1e-10:
            if p0[i] < bmin[i] or p0[i] > bmax[i]:
                return False
        else:
            inv_d = 1.0 / d[i]
            t1 = (bmin[i] - p0[i]) * inv_d
            t2 = (bmax[i] - p0[i]) * inv_d
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
            if t_min > t_max:
                return False

    return True


def is_path_clear(
    p0: np.ndarray,
    p1: np.ndarray,
    obstacles: Dict[int, Obstacle],
    buffer: float = 0.3
) -> bool:
    """Check if a straight-line path between two points is free of obstacles."""
    for obs in obstacles.values():
        if segment_intersects_aabb(p0, p1, obs.min_corner, obs.max_corner, buffer):
            return False
    return True


def is_point_in_obstacle(
    point: np.ndarray,
    obstacles: Dict[int, Obstacle],
    buffer: float = 0.3
) -> bool:
    """Check if a point is inside any obstacle (with buffer)."""
    for obs in obstacles.values():
        bmin = obs.min_corner - buffer
        bmax = obs.max_corner + buffer
        if np.all(point >= bmin) and np.all(point <= bmax):
            return True
    return False


def compute_distance_matrices(
    scenario: Scenario,
    include_start: bool = True,
    penalty_factor: float = 2.0
) -> Tuple[np.ndarray, np.ndarray, List[int]]:
    """
    Compute both Euclidean and collision-aware distance matrices in one pass.

    Args:
        scenario: Parsed scenario
        include_start: Whether to include drone start as node 0
        penalty_factor: Multiplier for edges that pass through obstacles

    Returns:
        euclidean_matrix: NxN pure Euclidean distances
        collision_matrix: NxN distances with obstacle penalty applied
        node_ids: list mapping matrix index -> viewpoint id (0 = start)
    """
    vp_ids = sorted(scenario.viewpoints.keys())

    if include_start:
        positions = [scenario.drone_start]
        node_ids = [0]
    else:
        positions = []
        node_ids = []

    for vid in vp_ids:
        positions.append(scenario.viewpoints[vid].position)
        node_ids.append(vid)

    n = len(positions)
    euclidean_matrix = np.zeros((n, n))
    collision_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(i + 1, n):
            d = np.linalg.norm(positions[i] - positions[j])
            euclidean_matrix[i][j] = d
            euclidean_matrix[j][i] = d

            c = d
            if not is_path_clear(positions[i], positions[j], scenario.obstacles):
                c *= penalty_factor
            collision_matrix[i][j] = c
            collision_matrix[j][i] = c

    return euclidean_matrix, collision_matrix, node_ids


def solve_tsp(
    distance_matrix: np.ndarray,
    node_ids: List[int],
) -> Tuple[List[int], float]:
    """
    Args:
        distance_matrix: NxN distance matrix (index 0 = drone start)
        node_ids: mapping from matrix index to viewpoint id (0 = start)

    Returns:
        route: ordered list of viewpoint IDs to visit (not including start)
        total_distance: total route distance
    """
    n = len(distance_matrix)

    if n <= 15:
        from python_tsp.exact import solve_tsp_dynamic_programming
        permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
    else:
        from python_tsp.heuristics import solve_tsp_local_search
        permutation, distance = solve_tsp_local_search(
            distance_matrix,
            perturbation_scheme="two_opt",
            max_processing_time=10
        )

    route = [node_ids[i] for i in permutation if node_ids[i] != 0]
    return route, distance


def solve_tsp_nearest_neighbour(
    distance_matrix: np.ndarray,
    node_ids: List[int],
) -> Tuple[List[int], float]:
    """Compatibility wrapper. Implementation moved to planner_tools.py."""
    from planner_tools import solve_tsp_nearest_neighbour as _impl
    return _impl(distance_matrix, node_ids)


def compare_planning_methods(
    scenario_paths: List[str],
) -> List[Dict]:
    """Compatibility wrapper. Implementation moved to planner_tools.py."""
    from planner_tools import compare_planning_methods as _impl
    return _impl(scenario_paths)


def write_method_comparison_csv(results: List[Dict], out_path: str = "runs/method_comparison.csv") -> None:
    """Compatibility wrapper. Implementation moved to planner_tools.py."""
    from planner_tools import write_method_comparison_csv as _impl
    _impl(results, out_path)


def plan_path(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles: Dict[int, Obstacle],
    resolution: float = 1.0,
    buffer: float = 0.8
) -> List[np.ndarray]:
    """
    Plan a collision-free path from start to goal using A*.
    If the straight line is clear, returns [start, goal] directly.

    Args:
        start: 3D start position
        goal: 3D goal position
        obstacles: dict of Obstacle objects
        resolution: grid cell size in meters
        buffer: safety margin around obstacles

    Returns:
        List of 3D waypoints from start to goal
    """
    import heapq

    if is_path_clear(start, goal, obstacles, buffer):
        return [start, goal]

    all_coords = np.vstack([start, goal])
    for obs in obstacles.values():
        all_coords = np.vstack([all_coords, obs.min_corner, obs.max_corner])
    grid_min = np.floor(np.min(all_coords, axis=0)) - 2.0
    grid_max = np.ceil(np.max(all_coords, axis=0)) + 2.0

    def to_grid(pos):
        return tuple(np.round((pos - grid_min) / resolution).astype(int))

    def to_world(cell):
        return grid_min + np.array(cell) * resolution

    def heuristic(cell_a, cell_b):
        return np.linalg.norm(np.array(cell_a) - np.array(cell_b)) * resolution

    start_cell = to_grid(start)
    goal_cell = to_grid(goal)
    grid_size = tuple(((grid_max - grid_min) / resolution).astype(int))

    def _find_free_cell(cell):
        """If cell is in obstacle, search nearby cells for a free one."""
        if not is_point_in_obstacle(to_world(cell), obstacles, buffer):
            return cell
        for r in range(1, 4):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-r, r + 1):
                        if abs(dx) != r and abs(dy) != r and abs(dz) != r:
                            continue
                        c = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                        if not is_point_in_obstacle(to_world(c), obstacles, buffer):
                            return c
        return cell

    start_cell = _find_free_cell(start_cell)
    goal_cell = _find_free_cell(goal_cell)

    neighbors_offsets = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                neighbors_offsets.append((dx, dy, dz))

    open_set = [(0.0, start_cell)]
    g_score = {start_cell: 0.0}
    came_from = {}
    closed_set = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current in closed_set:
            continue
        closed_set.add(current)

        if current == goal_cell:
            path = [goal]
            cell = goal_cell
            while cell in came_from:
                cell = came_from[cell]
                path.append(to_world(cell))
            path[-1] = start
            path.reverse()
            return _simplify_path(path, obstacles, buffer)

        for offset in neighbors_offsets:
            neighbor = (current[0] + offset[0],
                        current[1] + offset[1],
                        current[2] + offset[2])

            if neighbor in closed_set:
                continue

            if (neighbor[0] < 0 or neighbor[1] < 0 or neighbor[2] < 0 or
                    neighbor[0] >= grid_size[0] or neighbor[1] >= grid_size[1] or
                    neighbor[2] >= grid_size[2]):
                continue

            neighbor_pos = to_world(neighbor)

            if is_point_in_obstacle(neighbor_pos, obstacles, buffer):
                continue

            move_cost = np.linalg.norm(np.array(offset)) * resolution
            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal_cell)
                came_from[neighbor] = current
                heapq.heappush(open_set, (f, neighbor))

    # A* failed to find a path — fall back to straight line
    return [start, goal]


def _simplify_path(
    path: List[np.ndarray],
    obstacles: Dict[int, Obstacle],
    buffer: float = 0.5
) -> List[np.ndarray]:
    """
    Remove unnecessary intermediate waypoints.
    If you can go directly from point i to point j without collision,
    skip all points between them.
    """
    if len(path) <= 2:
        return path

    simplified = [path[0]]
    i = 0

    while i < len(path) - 1:
        # Find the farthest point we can reach directly from path[i]
        farthest = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            if is_path_clear(path[i], path[j], obstacles, buffer):
                farthest = j
                break
        simplified.append(path[farthest])
        i = farthest

    return simplified


def plan_full_route(
    scenario: Scenario,
    visit_order: List[int],
) -> List[np.ndarray]:
    """
    Plan the complete flight path: from drone start through all viewpoints
    in the given order, with collision-free segments.

    Args:
        scenario: parsed scenario
        visit_order: list of viewpoint IDs in visit order (from solve_tsp)

    Returns:
        List of all waypoints the drone should fly through
    """
    takeoff_pos = scenario.drone_start.copy()
    takeoff_pos[2] = 1.0  # takeoff height

    all_waypoints = [takeoff_pos]
    current = takeoff_pos

    for vid in visit_order:
        vp = scenario.viewpoints[vid]
        segment = plan_path(current, vp.position, scenario.obstacles)
        # segment[0] is current position, skip it to avoid duplicates
        all_waypoints.extend(segment[1:])
        current = vp.position

    return all_waypoints


def visualize_scenario(
    scenario: Scenario,
    route_ids: Optional[List[int]] = None,
    title: str = "Scenario Visualization"
):
    """Compatibility wrapper. Implementation moved to planner_tools.py."""
    from planner_tools import visualize_scenario as _impl
    _impl(scenario, route_ids=route_ids, title=title)


if __name__ == '__main__':
    from planner_tools import main
    main()
