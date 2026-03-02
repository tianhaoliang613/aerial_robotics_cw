#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Simple mission for a single drone."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
from time import sleep
import time
import json
import csv
import threading

import numpy as np
from datetime import datetime
from pathlib import Path

from as2_python_api.drone_interface import DroneInterface
import rclpy

import planner

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

ARUCO_OK = True
ARUCO_IMPORT_ERROR = ""

# Flight and detection constants
TAKE_OFF_HEIGHT = 1.0  # meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
VIEWPOINT_SETTLE_S = 1.0  # Extra settle time after viewpoint (before next go_to) to reduce drift on large yaw changes
SPEED = 1.0  # Max speed in m/s
LAND_SPEED = 0.5  # Max speed in m/s
ARUCO_TIMEOUT_S = 3.0


def drone_start(drone_interface: DroneInterface) -> bool:
    """Arm, offboard, takeoff. Returns True if takeoff succeeded."""
    ok = drone_interface.arm() and drone_interface.offboard()
    if ok:
        ok = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    return ok


def drone_run(drone_interface: DroneInterface, scenario, waypoints, aruco_state) -> bool:
    """Fly waypoints; at viewpoints use yaw and run ArUco detection. All control via DroneInterface (no custom flight callbacks)."""
    # waypoints = list of N points (each point is [x,y,z]). In the loop, wp is ONE point (one row), not the whole list.
    vp_positions = {vid: vp.position for vid, vp in scenario.viewpoints.items()}

    for idx, wp in enumerate(waypoints[1:], start=1):
        # wp = single 3D point (length 3): wp[0]=x, wp[1]=y, wp[2]=z
        goal = [float(wp[0]), float(wp[1]), float(wp[2])]

        # Match current goal to a viewpoint by position (within 0.2 m)
        vp_id = None
        for vid, vpos in vp_positions.items():
            if np.linalg.norm(wp - vpos) < 0.2:
                vp_id = vid
                break

        if vp_id is not None:
            yaw = float(scenario.viewpoints[vp_id].yaw)
            print(f'Go to VP{vp_id} ({idx}/{len(waypoints)-1}) with yaw={yaw:.2f}')
            success = drone_interface.go_to.go_to_point_with_yaw(goal, angle=yaw, speed=SPEED)
        else:
            print(f'Go to detour ({idx}/{len(waypoints)-1})')
            success = drone_interface.go_to.go_to_point(goal, speed=SPEED)

        if not success:
            sleep(SLEEP_TIME)
            continue

        if vp_id is not None:
            aruco_state["visited"].add(vp_id)
            exp_id = int(f"{(vp_id % 8) + 1}4")
            if detect_marker(aruco_state, exp_id):
                aruco_state["verified"].add(vp_id)
                print(f'  ArUco marker {exp_id} verified at VP{vp_id}')
            else:
                print(f'  ArUco marker {exp_id} NOT detected at VP{vp_id}')
            sleep(SLEEP_TIME)
            sleep(VIEWPOINT_SETTLE_S)
            continue

        sleep(SLEEP_TIME)

    return True


def drone_end(drone_interface: DroneInterface) -> bool:
    """Land then switch to manual. Returns True if both succeeded."""
    ok = drone_interface.land(speed=LAND_SPEED)
    if ok:
        ok = drone_interface.manual()
    return ok

def polyline_dist(points):
    """Total path length along consecutive points."""
    return float(sum(np.linalg.norm(points[i+1] - points[i]) for i in range(len(points)-1)))


def setup_aruco(drone_namespace):
    """Separate ROS2 node + executor thread for ArUco so camera callbacks do not block flight; subscription created on demand in detect_marker.
    Namespace must match the drone's so we subscribe to that drone's camera topic (e.g. /drone0/sensor_measurements/...)."""
    # state: shared dict for node, executor, CvBridge, ArUco config, and result sets (detected/verified/visited)
    state = {"detected": set(), "verified": set(), "visited": set(), "node": None}
    if not ARUCO_OK:
        return state
    state["bridge"] = CvBridge()
    state["dictionary"] = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    state["params"] = getattr(aruco, "DetectorParameters", aruco.DetectorParameters_create)()
    node = rclpy.create_node("aruco_detector", namespace=drone_namespace)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    # Run executor.spin() in a daemon thread so callbacks run without blocking main thread
    threading.Thread(target=executor.spin, daemon=True).start()
    state["node"], state["executor"] = node, executor
    return state


def detect_marker(aruco_state, expected_id, timeout_s=ARUCO_TIMEOUT_S):
    """Temporarily subscribe on the aruco node, poll for expected_id within timeout_s, then destroy subscription. No publisher: we only read camera and return a bool."""
    if not ARUCO_OK or not aruco_state.get("node"):
        return False
    found = [False]  # list so _cb can mutate it (closure)
    n, br, dic, prm = aruco_state["node"], aruco_state["bridge"], aruco_state["dictionary"], aruco_state["params"]

    def _cb(msg):
        """Callback when a camera Image msg arrives; runs on the aruco node's executor thread."""
        if found[0]:
            return
        try:
            _, ids, _ = aruco.detectMarkers(br.imgmsg_to_cv2(msg, "bgr8"), dic, parameters=prm)
            if ids is not None:
                for mid in ids.flatten().tolist():
                    mid = int(mid)
                    aruco_state["detected"].add(mid)
                    if mid == expected_id:
                        found[0] = True
        except Exception:
            pass  # one bad frame or CvBridge error must not kill the callback

    sub = n.create_subscription(Image, "sensor_measurements/hd_camera/image_raw", _cb, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < timeout_s and not found[0]:
        sleep(0.05)
    n.destroy_subscription(sub)
    return found[0]


def _save_metrics(runs_dir, row, fly_order, aruco_state):
    """Append row to runs/metrics.csv and write timestamped JSON; backfill order_type if missing."""
    runs_dir = Path(runs_dir)
    runs_dir.mkdir(exist_ok=True)
    ts, sname = row["timestamp"], row["scenario"]
    csv_path = runs_dir / "metrics.csv"
    fieldnames = list(row.keys())
    if csv_path.exists():
        with csv_path.open("r", newline="") as f:
            r = csv.DictReader(f)
            old_hdr = r.fieldnames or []
            if "order_type" not in old_hdr:
                rows_old = list(r)
                for r_ in rows_old:
                    r_["order_type"] = "tsp"
                with csv_path.open("w", newline="") as fw:
                    csv.DictWriter(fw, fieldnames=fieldnames).writeheader()
                    csv.DictWriter(fw, fieldnames=fieldnames).writerows(rows_old)
        with csv_path.open("a", newline="") as f:
            csv.DictWriter(f, fieldnames=fieldnames).writerow(row)
    else:
        with csv_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerow(row)
    with (runs_dir / f"{ts}_{sname}.json").open("w") as f:
        json.dump({"summary": row, "visit_order": fly_order,
                   "visited": sorted(aruco_state["visited"]),
                   "aruco_verified": sorted(aruco_state["verified"]),
                   "aruco_detected_ids": sorted(aruco_state["detected"])}, f, indent=2)
    return csv_path


if __name__ == '__main__':
    # argparse: scenario=positional (required); -n/--namespace etc.=optional. store_true => flag, default when not given.
    p = argparse.ArgumentParser(description='Single drone mission')
    p.add_argument('scenario', type=str, help='Scenario YAML path')
    p.add_argument('-n', '--namespace', default='drone0', help='Drone namespace')
    p.add_argument('-v', '--verbose', action='store_true', help='Verbose DroneInterface logs')
    p.add_argument('-s', '--use_sim_time', action='store_true', default=True, help='Use sim time')
    p.add_argument('--baseline', action='store_true', help='Fly YAML order instead of TSP (for comparison)')
    args = p.parse_args()  # parse sys.argv; args.scenario, args.namespace, args.verbose, ...

    scenario = planner.load_scenario(args.scenario)
    plan_t0 = time.time()
    _, col_matrix, node_ids = planner.compute_distance_matrices(scenario)
    visit_order, _ = planner.solve_tsp(col_matrix, node_ids)
    waypoints = planner.plan_full_route(scenario, visit_order)
    baseline_order = sorted(scenario.viewpoints.keys())
    baseline_wps = planner.plan_full_route(scenario, baseline_order)
    opt_dist = polyline_dist(waypoints)
    base_dist = polyline_dist(baseline_wps)
    improve = (1.0 - opt_dist / base_dist) * 100.0 if base_dist > 0 else 0.0
    plan_time = time.time() - plan_t0

    if args.baseline:
        fly_order, waypoints_to_fly, flown_dist, order_type = baseline_order, baseline_wps, base_dist, 'baseline'
    else:
        fly_order, waypoints_to_fly, flown_dist, order_type = visit_order, waypoints, opt_dist, 'tsp'

    rclpy.init()
    uav = DroneInterface(drone_id=args.namespace, use_sim_time=args.use_sim_time, verbose=args.verbose)
    aruco_state = setup_aruco(args.namespace)  # same namespace => subscribe to this drone's camera

    success = drone_start(uav)
    try:
        t0 = time.time()
        if success:
            success = drone_run(uav, scenario, waypoints_to_fly, aruco_state)
        exec_time = time.time() - t0
        total_vps = len(scenario.viewpoints)
        collision_free = all(
            planner.is_path_clear(waypoints_to_fly[i], waypoints_to_fly[i + 1], scenario.obstacles, buffer=0.0)
            for i in range(len(waypoints_to_fly) - 1))
        avg_speed = flown_dist / exec_time if exec_time > 1e-3 else 0.0

        row = {
            "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"), "scenario": Path(args.scenario).stem,
            "order_type": order_type, "success": bool(success),
            "planning_s": round(plan_time, 2), "execution_s": round(exec_time, 2),
            "viewpoints_total": total_vps, "viewpoints_visited": len(aruco_state["visited"]),
            "aruco_verified": len(aruco_state["verified"]),
            "optimized_dist_m": round(opt_dist, 2), "baseline_dist_m": round(base_dist, 2),
            "improvement_pct": round(improve, 1), "avg_speed_mps": round(avg_speed, 2), "collision_free": collision_free,
        }
        csv_path = _save_metrics("runs", row, fly_order, aruco_state)
        print(f"Mission done: {exec_time:.1f}s | VPs {len(aruco_state['visited'])}/{total_vps} | ArUco {len(aruco_state['verified'])}/{total_vps} | {csv_path}")
    except KeyboardInterrupt:
        pass
    drone_end(uav)
    if aruco_state.get("executor"):
        aruco_state["executor"].shutdown()
    if aruco_state.get("node"):
        aruco_state["node"].destroy_node()
    uav.shutdown()
    rclpy.shutdown()
    exit(0)
