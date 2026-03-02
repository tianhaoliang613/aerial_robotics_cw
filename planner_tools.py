"""Optional tools for planner analysis and visualization.

Kept separate from planner core to keep planner.py minimal.
"""

import math
from typing import Dict, List, Optional, Tuple

import numpy as np


def solve_tsp_nearest_neighbour(
    distance_matrix: np.ndarray,
    node_ids: List[int],
) -> Tuple[List[int], float]:
    """Nearest-neighbour TSP heuristic (fast, not guaranteed optimal)."""
    n = len(distance_matrix)
    current = 0
    unvisited = set(range(1, n))
    route_indices = []
    total_dist = 0.0

    while unvisited:
        best_j = min(unvisited, key=lambda j: distance_matrix[current, j])
        total_dist += distance_matrix[current, best_j]
        route_indices.append(best_j)
        current = best_j
        unvisited.discard(best_j)

    route = [node_ids[i] for i in route_indices]
    return route, total_dist


def compare_planning_methods(scenario_paths: List[str]) -> List[Dict]:
    """Compare baseline, TSP, and nearest-neighbour distances/times."""
    import time
    from planner import compute_distance_matrices, load_scenario, solve_tsp

    results = []
    for path in scenario_paths:
        scenario = load_scenario(path)
        _, col_matrix, node_ids = compute_distance_matrices(scenario)
        id_to_idx = {nid: i for i, nid in enumerate(node_ids)}

        # Baseline: YAML order (1,2,3,...)
        baseline_dist = 0.0
        prev = id_to_idx[0]
        for vid in sorted(scenario.viewpoints.keys()):
            idx = id_to_idx[vid]
            baseline_dist += col_matrix[prev, idx]
            prev = idx

        t0 = time.perf_counter()
        _, tsp_dist = solve_tsp(col_matrix, node_ids)
        t_tsp = (time.perf_counter() - t0) * 1000

        t0 = time.perf_counter()
        _, nn_dist = solve_tsp_nearest_neighbour(col_matrix, node_ids)
        t_nn = (time.perf_counter() - t0) * 1000

        results.append(
            {
                "scenario": scenario.name,
                "baseline_dist": baseline_dist,
                "baseline_time_ms": 0.0,
                "tsp_dist": tsp_dist,
                "tsp_time_ms": t_tsp,
                "nn_dist": nn_dist,
                "nn_time_ms": t_nn,
            }
        )
    return results


def write_method_comparison_csv(results: List[Dict], out_path: str = "runs/method_comparison.csv") -> None:
    """Write comparison table to CSV."""
    import os

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w") as f:
        f.write("scenario,baseline_dist_m,tsp_dist_m,tsp_time_ms,nn_dist_m,nn_time_ms\n")
        for r in results:
            f.write(
                f"{r['scenario']},{r['baseline_dist']:.2f},{r['tsp_dist']:.2f},"
                f"{r['tsp_time_ms']:.2f},{r['nn_dist']:.2f},{r['nn_time_ms']:.2f}\n"
            )


def visualize_scenario(
    scenario,
    route_ids: Optional[List[int]] = None,
    title: str = "Scenario Visualization",
):
    """Visualize obstacles, viewpoints, start point, and optional route."""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    for obs in scenario.obstacles.values():
        _draw_box(ax, obs)

    for vp in scenario.viewpoints.values():
        ax.scatter(vp.x, vp.y, vp.z, c="blue", s=60, marker="^", zorder=5)
        ax.text(vp.x, vp.y, vp.z + 0.3, f"VP{vp.id}", fontsize=8, ha="center")
        arrow_len = 0.8
        dx = arrow_len * math.cos(vp.yaw)
        dy = arrow_len * math.sin(vp.yaw)
        ax.quiver(vp.x, vp.y, vp.z, dx, dy, 0, color="blue", alpha=0.5, arrow_length_ratio=0.3)

    s = scenario.drone_start
    ax.scatter(s[0], s[1], s[2], c="green", s=120, marker="o", zorder=5, label="Start")
    ax.text(s[0], s[1], s[2] + 0.3, "START", fontsize=9, ha="center", color="green")

    if route_ids is not None:
        route_points = [scenario.drone_start.copy()]
        route_points[0][2] = 1.0
        for vid in route_ids:
            route_points.append(scenario.viewpoints[vid].position)
        route_points = np.array(route_points)
        ax.plot(route_points[:, 0], route_points[:, 1], route_points[:, 2], "r-o", linewidth=1.5, markersize=4, label="Route", alpha=0.7)
        for idx, vid in enumerate(route_ids):
            vp = scenario.viewpoints[vid]
            ax.text(vp.x + 0.3, vp.y + 0.3, vp.z + 0.5, f"#{idx+1}", fontsize=7, color="red")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title)
    ax.legend()
    plt.show()


def _draw_box(ax, obs):
    """Draw a 3D AABB obstacle."""
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    x0, y0, z0 = obs.min_corner
    x1, y1, z1 = obs.max_corner

    vertices = [
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ]
    faces = [
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[0], vertices[3], vertices[7], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
    ]
    box = Poly3DCollection(faces, alpha=0.2, facecolor="red", edgecolor="darkred", linewidth=0.5)
    ax.add_collection3d(box)


def main():
    """CLI demo for planner core + visualization."""
    import argparse
    from planner import compute_distance_matrices, is_path_clear, load_scenario, plan_full_route, solve_tsp

    parser = argparse.ArgumentParser(description="Planner module test")
    parser.add_argument(
        "scenario",
        type=str,
        nargs="?",
        default="scenarios/scenario1.yaml",
        help="Path to scenario YAML file (default: scenarios/scenario1.yaml)",
    )
    parser.add_argument("--visualize", "-v", action="store_true", help="Show 3D visualization")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    print(f"=== Scenario: {scenario.name} ===")
    print(f"Viewpoints: {len(scenario.viewpoints)} | Obstacles: {len(scenario.obstacles)}")

    vp_ids = sorted(scenario.viewpoints.keys())
    total = len(vp_ids) * (len(vp_ids) - 1) // 2
    blocked = sum(
        1
        for i, vid_a in enumerate(vp_ids)
        for vid_b in vp_ids[i + 1:]
        if not is_path_clear(
            scenario.viewpoints[vid_a].position,
            scenario.viewpoints[vid_b].position,
            scenario.obstacles,
        )
    )
    print(f"Blocked direct edges: {blocked}/{total}")

    _, col_matrix, node_ids = compute_distance_matrices(scenario)
    route, distance = solve_tsp(col_matrix, node_ids)
    print(f"\n--- TSP Result ---")
    print(f"Visit order: {route}")
    print(f"Total distance (TSP estimate): {distance:.2f} m")

    waypoints = plan_full_route(scenario, route)
    actual_dist = sum(np.linalg.norm(waypoints[i + 1] - waypoints[i]) for i in range(len(waypoints) - 1))
    print(f"Waypoints: {len(waypoints)} | Actual distance: {actual_dist:.2f} m")

    if args.visualize:
        visualize_scenario(scenario, route_ids=route, title=f"Scenario: {scenario.name} (TSP + A*)")


if __name__ == "__main__":
    main()
