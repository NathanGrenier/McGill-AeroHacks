import argparse
import json
import math
import os

from aerohacks.core.models import (
    ActionStep,
    ActionType,
    ConstraintPhase,
    Observation,
    Plan,
    Position2D,
)
from aerohacks.policy.base import Policy


class MyPolicy(Policy):
    # DEFAULT KINEMATIC LIMITS (Fallbacks if JSON is missing)
    DEFAULT_MAX_HORIZONTAL_SPEED = 15.0
    DEFAULT_MAX_VERTICAL_RATE = 1
    DEFAULT_ENERGY_DECAY_RATE = 0.1
    DEFAULT_ENERGY_RESERVE = 5.0

    # BOILERPLATE CONSTANTS
    STEPS_PER_PLAN = 5
    TRAVEL_RANGE_SAFETY_FACTOR = 0.9  # Safety factor for energy-based diversion decisions

    # SEPARATION THRESHOLDS (From Guidelines)
    COLLISION_DIST_M = 20.0  # Collision if <= 20m AND same altitude layer
    CONFLICT_DIST_M = 50.0  # Conflict if <= 50m AND alt diff <= 1
    ADVISORY_DIST_M = 100.0  # Advisory if <= 100m AND alt diff <= 1
    SEPARATION_ALT_DIFF = 1  # Layer difference threshold for conflicts/advisories
    MAX_ALT_LAYER = 4  # Maximum altitude layer (0-4)
    GROUND_ALT_LAYER = 0  # Altitude layer for emergency landing

    # SCORING & PENALTIES (From Guidelines)
    SCORE_BASE_SUCCESS = 1000
    SCORE_TIME_BONUS_MULT = 10
    SCORE_SAFE_EMERGENCY = 300
    PENALTY_CONTROLLED_ZONE = -50  # Per step
    PENALTY_RESTRICTED_ZONE = -200  # Per step
    PENALTY_ADVISORY_SEP = -10  # Per event
    PENALTY_CONFLICT_SEP = -150  # Per event
    MAX_RESTRICTED_STEPS = 5  # 5+ consecutive steps in restricted = catastrophic (0 score)

    def __init__(self):
        super().__init__()
        self.emergency_sites = []

        self.max_horizontal_speed = self.DEFAULT_MAX_HORIZONTAL_SPEED
        self.max_vertical_rate = self.DEFAULT_MAX_VERTICAL_RATE
        self.energy_decay_rate = self.DEFAULT_ENERGY_DECAY_RATE
        self.energy_reserve_threshold = self.DEFAULT_ENERGY_RESERVE

        # Parse the command-line args to find out which scenario we are running
        parser = argparse.ArgumentParser(allow_abbrev=False)
        parser.add_argument("--scenario", type=str, default="example_training")
        parser.add_argument("--scenarios-dir", type=str, default="./scenarios")
        args, _ = parser.parse_known_args()

        scenario_path = os.path.join(args.scenarios_dir, "public", f"{args.scenario}.json")
        try:
            with open(scenario_path, "r") as f:
                data = json.load(f)
                self.emergency_sites = data.get("emergency_landing_sites", [])

                # Dynamically load vehicle limits from the scenario
                limits = data.get("vehicle_limits", {})
                self.max_horizontal_speed = float(
                    limits.get("max_horizontal_speed", self.DEFAULT_MAX_HORIZONTAL_SPEED)
                )
                self.max_vertical_rate = int(
                    limits.get("max_vertical_rate", self.DEFAULT_MAX_VERTICAL_RATE)
                )
                self.energy_decay_rate = float(
                    limits.get("energy_decay_rate", self.DEFAULT_ENERGY_DECAY_RATE)
                )
                self.energy_reserve_threshold = float(
                    limits.get("energy_reserve_threshold", self.DEFAULT_ENERGY_RESERVE)
                )

        except Exception as e:
            print(f"Policy Init Warning: Could not load scenario JSON: {e}")

    def _get_region_center_json(self, region):
        if region.get("type") == "CircleRegion":
            return Position2D(float(region["center_pos"]["x"]), float(region["center_pos"]["y"]))
        elif region.get("vertices"):
            xs = [float(v["x"]) for v in region["vertices"]]
            ys = [float(v["y"]) for v in region["vertices"]]
            return Position2D(sum(xs) / len(xs), sum(ys) / len(ys))
        return Position2D(0.0, 0.0)

    def _is_inside_json_region(self, px, py, region):
        if region.get("type") == "CircleRegion":
            cx, cy = float(region["center_pos"]["x"]), float(region["center_pos"]["y"])
            return math.hypot(px - cx, py - cy) <= float(region["radius"])
        elif region.get("vertices"):
            inside = False
            n = len(region["vertices"])
            if n == 0:
                return False
            j = n - 1
            for i in range(n):
                xi, yi = float(region["vertices"][i]["x"]), float(region["vertices"][i]["y"])
                xj, yj = float(region["vertices"][j]["x"]), float(region["vertices"][j]["y"])
                intersect = ((yi > py) != (yj > py)) and (
                    px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi
                )
                if intersect:
                    inside = not inside
                j = i
            return inside
        return False

    def _get_closest_point(self, px: float, py: float, region):
        if hasattr(region, "radius"):
            cx = region.center_pos.x if hasattr(region, "center_pos") else region.center().x
            cy = region.center_pos.y if hasattr(region, "center_pos") else region.center().y
            r = region.radius
            dist_to_center = math.hypot(px - cx, py - cy)

            if dist_to_center == 0:
                return cx + r, cy, -r

            cx_edge = cx + (px - cx) / dist_to_center * r
            cy_edge = cy + (py - cy) / dist_to_center * r
            return cx_edge, cy_edge, dist_to_center - r

        elif hasattr(region, "vertices"):
            vertices = region.vertices
            min_dist = float("inf")
            best_x, best_y = px, py
            inside = False

            n = len(vertices)
            j = n - 1
            for i in range(n):
                xi, yi = vertices[i].x, vertices[i].y
                xj, yj = vertices[j].x, vertices[j].y

                intersect = ((yi > py) != (yj > py)) and (
                    px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi
                )
                if intersect:
                    inside = not inside

                dx = xj - xi
                dy = yj - yi
                length_sq = dx * dx + dy * dy

                if length_sq == 0:
                    d = math.hypot(px - xi, py - yi)
                    closest_x, closest_y = xi, yi
                else:
                    t = max(0, min(1, ((px - xi) * dx + (py - yi) * dy) / length_sq))
                    closest_x = xi + t * dx
                    closest_y = yi + t * dy
                    d = math.hypot(px - closest_x, py - closest_y)

                if d < min_dist:
                    min_dist = d
                    best_x = closest_x
                    best_y = closest_y
                j = i

            return best_x, best_y, -min_dist if inside else min_dist
        else:
            return px, py, float("inf")

    def step(self, obs: Observation) -> Plan:
        steps = []
        ownship = obs.ownship_state
        current_pos = ownship.position

        # Extract Goal Center
        try:
            goal_pos = obs.mission_goal.region.center()
        except AttributeError:
            goal_pos = Position2D(x=0.0, y=0.0)
            if hasattr(obs.mission_goal.region, "center_pos"):
                goal_pos = obs.mission_goal.region.center_pos

        # --- DYNAMIC BATTERY DIVERSION LOGIC ---
        # Calculate theoretical max distance per 1 unit of energy
        # e.g., (15m/s / 0.1 decay) = 150 meters per unit
        theoretical_dist_per_energy = self.max_horizontal_speed / max(self.energy_decay_rate, 0.001)

        # Calculate usable energy (keep threshold in reserve)
        usable_energy = max(0.0, ownship.energy - self.energy_reserve_threshold)

        # Safe travel range applies a safety factor for dodging NOTAMs and traffic
        safe_travel_range = (
            usable_energy * theoretical_dist_per_energy * self.TRAVEL_RANGE_SAFETY_FACTOR
        )

        dist_to_goal = math.hypot(goal_pos.x - current_pos.x, goal_pos.y - current_pos.y)

        # Only divert if it is mathematically impossible to reach the primary goal safely
        if dist_to_goal > safe_travel_range and self.emergency_sites:
            best_site = None
            min_dist = float("inf")
            for site in self.emergency_sites:
                center = self._get_region_center_json(site["region"])
                d = math.hypot(current_pos.x - center.x, current_pos.y - center.y)
                if d < min_dist:
                    min_dist = d
                    best_site = site

            if best_site:
                if self._is_inside_json_region(current_pos.x, current_pos.y, best_site["region"]):
                    return Plan(
                        steps=[ActionStep(action_type=ActionType.EMERGENCY_LAND)]
                        * self.STEPS_PER_PLAN
                    )
                # Overwrite the goal with the emergency site
                goal_pos = self._get_region_center_json(best_site["region"])

        # Determine Target Altitude
        base_alt = (
            obs.mission_goal.target_alt_layer
            if obs.mission_goal.target_alt_layer is not None
            else ownship.alt_layer
        )
        target_alt = base_alt

        # Traffic Vertical Evasion
        for traffic in obs.traffic_tracks:
            dist_to_traffic = math.hypot(
                traffic.position.x - current_pos.x, traffic.position.y - current_pos.y
            )
            if (
                dist_to_traffic < (self.ADVISORY_DIST_M * 1.5)
                and abs(traffic.alt_layer - target_alt) <= self.SEPARATION_ALT_DIFF
            ):
                target_alt = target_alt + 2 if target_alt < 2 else target_alt - 2
                break

        # Constraint Vertical Evasion
        for constraint in obs.active_constraints:
            if target_alt in constraint.alt_layers and constraint.phase in [
                ConstraintPhase.RESTRICTED,
                ConstraintPhase.CONTROLLED,
            ]:
                _, _, dist_edge = self._get_closest_point(
                    current_pos.x, current_pos.y, constraint.region
                )
                if dist_edge < 150.0:
                    for test_alt in range(1, self.MAX_ALT_LAYER + 1):
                        if test_alt not in constraint.alt_layers:
                            target_alt = test_alt
                            break

        # Simulate Forward Kinematics
        sim_pos = Position2D(x=current_pos.x, y=current_pos.y)
        sim_alt = ownship.alt_layer

        for i in range(self.STEPS_PER_PLAN):
            # 1. Horizontal Calculations
            dx_goal = goal_pos.x - sim_pos.x
            dy_goal = goal_pos.y - sim_pos.y
            dist_to_goal = math.hypot(dx_goal, dy_goal)

            if dist_to_goal < 1.0:
                vx, vy = 0.0, 0.0
            else:
                vx = (dx_goal / dist_to_goal) * self.max_horizontal_speed
                vy = (dy_goal / dist_to_goal) * self.max_horizontal_speed

            repulse_x, repulse_y = 0.0, 0.0

            # Traffic Repulsion
            for traffic in obs.traffic_tracks:
                if abs(traffic.alt_layer - target_alt) > self.SEPARATION_ALT_DIFF:
                    continue
                tx = traffic.position.x
                ty = traffic.position.y
                if traffic.velocity:
                    tx += traffic.velocity.x * (i + 1)
                    ty += traffic.velocity.y * (i + 1)
                dist_t = math.hypot(tx - sim_pos.x, ty - sim_pos.y)
                if 0 < dist_t < (self.ADVISORY_DIST_M * 2.0):
                    force = 20000.0 / (dist_t**2)
                    repulse_x += ((sim_pos.x - tx) / dist_t) * force
                    repulse_y += ((sim_pos.y - ty) / dist_t) * force

            # Constraint Repulsion
            for constraint in obs.active_constraints:
                if target_alt not in constraint.alt_layers:
                    continue
                cx, cy, dist_edge = self._get_closest_point(sim_pos.x, sim_pos.y, constraint.region)
                weight = (
                    80000.0
                    if constraint.phase == ConstraintPhase.RESTRICTED
                    else 40000.0
                    if constraint.phase == ConstraintPhase.CONTROLLED
                    else 10000.0
                )
                if dist_edge < (self.ADVISORY_DIST_M * 1.5):
                    safe_dist = max(abs(dist_edge), 1.0)
                    force_mag = weight / (safe_dist**2)
                    if dist_edge >= 0:
                        dir_x, dir_y = sim_pos.x - cx, sim_pos.y - cy
                    else:
                        dir_x, dir_y = cx - sim_pos.x, cy - sim_pos.y
                        force_mag *= 5.0
                    mag = math.hypot(dir_x, dir_y)
                    if mag > 0:
                        repulse_x += (dir_x / mag) * force_mag
                        repulse_y += (dir_y / mag) * force_mag

            total_vx, total_vy = vx + repulse_x, vy + repulse_y
            total_speed = math.hypot(total_vx, total_vy)
            if total_speed > 0:
                step_dist = (
                    min(self.max_horizontal_speed, dist_to_goal)
                    if repulse_x == 0 and repulse_y == 0
                    else self.max_horizontal_speed
                )
                total_vx, total_vy = (
                    (total_vx / total_speed) * step_dist,
                    (total_vy / total_speed) * step_dist,
                )

            sim_pos = Position2D(x=sim_pos.x + total_vx, y=sim_pos.y + total_vy)

            # 2. Vertical Rate Clamping (using dynamic max_vertical_rate)
            if target_alt > sim_alt:
                sim_alt = min(target_alt, sim_alt + self.max_vertical_rate)
            elif target_alt < sim_alt:
                sim_alt = max(target_alt, sim_alt - self.max_vertical_rate)

            steps.append(
                ActionStep(
                    action_type=ActionType.WAYPOINT,
                    target_position=Position2D(x=sim_pos.x, y=sim_pos.y),
                    target_alt_layer=sim_alt,
                )
            )

        return Plan(steps=steps)
