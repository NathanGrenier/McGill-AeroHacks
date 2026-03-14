import argparse
import json
import math
import os


def load_json(path):
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def get_notam_phase(notam, time_value):
    advisory = int(notam.get("advisory_start_time", 10**9))
    controlled = int(notam.get("controlled_start_time", 10**9))
    restricted = int(notam.get("restricted_start_time", 10**9))
    if time_value < advisory:
        return "inactive"
    if time_value < controlled:
        return "advisory"
    if time_value < restricted:
        return "controlled"
    return "restricted"


def get_traffic_state(segments, time_value):
    for seg in segments:
        start_time = int(seg.get("start_time", 0))
        end_time = int(seg.get("end_time", -1))
        if start_time <= time_value < end_time:
            sx, sy = float(seg["start_pos"]["x"]), float(seg["start_pos"]["y"])
            vx, vy = float(seg["velocity"]["x"]), float(seg["velocity"]["y"])
            dt = time_value - start_time
            return sx + vx * dt, sy + vy * dt, int(seg.get("alt_layer", 0))
    return None


def is_point_in_polygon(px, py, vertices):
    inside = False
    n = len(vertices)
    if n == 0:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = float(vertices[i]["x"]), float(vertices[i]["y"])
        xj, yj = float(vertices[j]["x"]), float(vertices[j]["y"])
        intersect = ((yi > py) != (yj > py)) and (
            px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi
        )
        if intersect:
            inside = not inside
        j = i
    return inside


def is_point_in_region(px, py, region):
    if not region:
        return False
    if region.get("type") == "CircleRegion":
        cx, cy = float(region["center_pos"]["x"]), float(region["center_pos"]["y"])
        r = float(region["radius"])
        return math.hypot(px - cx, py - cy) <= r
    elif region.get("vertices"):
        return is_point_in_polygon(px, py, region["vertices"])
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--playback", default="playback.json")
    parser.add_argument("--scenario", default="scenarios/public/example_training.json")
    parser.add_argument("--hidden", default="scenarios/hidden/example_training.json")
    args = parser.parse_args()

    if not os.path.exists(args.playback):
        print("Run the simulation first to generate playback.json!")
        return

    playback_data = load_json(args.playback)
    scenario_data = load_json(args.scenario)
    hidden_data = load_json(args.hidden)

    # --- Handle raw lists vs wrapped dictionary outputs ---
    if isinstance(playback_data, list):
        history = playback_data
    elif isinstance(playback_data, dict):
        history = playback_data.get("participants", [{}])[0].get("history", [])
    else:
        print("Error: Unknown playback.json format.")
        return

    if not history:
        print("Playback history is empty.")
        return

    # Trackers
    controlled_steps = 0
    restricted_steps = 0
    advisory_sep_events = 0
    conflict_sep_events = 0
    consecutive_restricted = 0
    catastrophic = False

    print("\n--- 🚁 AeroHacks Score Analyzer ---")

    map_boundaries = scenario_data.get("map_boundaries", {})

    for tick in history:
        t = int(tick["time"])
        px, py = float(tick["x"]), float(tick["y"])
        alt = int(tick["alt_layer"])
        energy = float(tick["energy"])

        # 1. Check Out of Bounds
        if map_boundaries.get("vertices") and not is_point_in_polygon(
            px, py, map_boundaries["vertices"]
        ):
            print(f"\n💀 CATASTROPHIC FAILURE at time {t}: Out of Map Boundaries!")
            catastrophic = True
            break

        # 2. Check Static Obstacles
        static_collision = False
        for obs in scenario_data.get("static_obstacles", []):
            if is_point_in_region(px, py, obs):
                print(f"\n💀 CATASTROPHIC FAILURE at time {t}: Collided with Static Obstacle!")
                catastrophic = True
                static_collision = True
                break
        if static_collision:
            break

        # 3. Check Permanent Constraints
        for constraint in scenario_data.get("permanent_constraints", []):
            if alt in constraint.get("alt_layers", []):
                if is_point_in_region(px, py, constraint["region"]):
                    restricted_steps += 1
                    consecutive_restricted += 1
                    print(f"[Time {t}] ❌ Entered Permanent Restricted Airspace!")

        # 4. Check Dynamic NOTAMs
        for notam in hidden_data.get("shrinking_notams", []):
            if alt in notam.get("alt_layers", []):
                phase = get_notam_phase(notam, t)
                if phase in ["controlled", "restricted"] and is_point_in_region(
                    px, py, notam["region"]
                ):
                    if phase == "controlled":
                        controlled_steps += 1
                        consecutive_restricted = 0
                        print(f"[Time {t}] ⚠️ Entered Controlled Airspace")
                    else:
                        restricted_steps += 1
                        consecutive_restricted += 1
                        print(f"[Time {t}] ❌ Entered Restricted Airspace")

        # Catastrophic Check: 5+ consecutive restricted steps
        if consecutive_restricted >= 5:
            print(f"\n💀 CATASTROPHIC FAILURE at time {t}: 5+ consecutive restricted steps!")
            catastrophic = True
            break

        # 5. Check Traffic Separation
        for trace in hidden_data.get("traffic_traces", []):
            state = get_traffic_state(trace.get("segments", []), t)
            if state:
                tx, ty, t_alt = state
                dist = math.hypot(px - tx, py - ty)
                alt_diff = abs(alt - t_alt)

                if alt_diff <= 1:
                    if dist <= 20.0 and alt_diff == 0:
                        print(f"\n💀 CATASTROPHIC FAILURE at time {t}: Collision with NPC!")
                        catastrophic = True
                        break
                    elif dist <= 50.0:
                        conflict_sep_events += 1
                        print(f"[Time {t}] 💥 Conflict Separation Loss ({dist:.1f}m)")
                    elif dist <= 100.0:
                        advisory_sep_events += 1
                        print(f"[Time {t}] ⚠️ Advisory Separation Loss ({dist:.1f}m)")

        if catastrophic:
            break

        # Reset consecutive restricted if safe this tick
        if not any(
            is_point_in_region(px, py, n["region"]) and get_notam_phase(n, t) == "restricted"
            for n in hidden_data.get("shrinking_notams", [])
        ):
            consecutive_restricted = 0

    # --- TERMINAL EVALUATION & SCORE CALCULATION ---
    last_tick = history[-1]
    last_t = int(last_tick["time"])
    last_x, last_y = float(last_tick["x"]), float(last_tick["y"])
    last_alt = int(last_tick["alt_layer"])
    last_energy = float(last_tick["energy"])

    scoring_config = scenario_data.get("scoring_config", {})
    mission_goal = scenario_data.get("mission_goal", {})

    is_goal_reached = False
    is_safe_emergency = False
    time_bonus = 0

    if not catastrophic:
        # Check if Goal Reached
        target_alt = mission_goal.get("target_alt_layer")
        if is_point_in_region(last_x, last_y, mission_goal.get("region", {})):
            if target_alt is None or last_alt == target_alt:
                is_goal_reached = True

        # Check if Safe Emergency Landing
        if last_alt == 0:
            for site in scenario_data.get("emergency_landing_sites", []):
                if is_point_in_region(last_x, last_y, site.get("region", {})):
                    is_safe_emergency = True
                    break

        # Check Battery Depletion rules
        if last_energy <= 0 and not is_goal_reached and not is_safe_emergency:
            print(
                f"\n💀 CATASTROPHIC FAILURE at time {last_t}: Battery depleted outside safe site!"
            )
            catastrophic = True

    # Assemble Final Score
    base_score = 1000
    penalty_controlled = controlled_steps * -50
    penalty_restricted = restricted_steps * -200
    penalty_advisory = advisory_sep_events * -10
    penalty_conflict = conflict_sep_events * -150

    print("\n--- 📊 Final Score Breakdown ---")
    if catastrophic:
        print("STATUS: 💀 CATASTROPHIC (Run Terminated)")
        print("FINAL SCORE: 0.0")
    else:
        final_score = (
            base_score
            + penalty_controlled
            + penalty_restricted
            + penalty_advisory
            + penalty_conflict
        )
        print(f"Base Score: +{base_score}")

        if is_goal_reached:
            # Defaults assumed from typical simulator settings if missing in JSON
            max_time = scoring_config.get("max_time", 4000)
            mult = scoring_config.get("time_bonus_multiplier", 10)
            time_bonus = mult * max(0, max_time - last_t)
            final_score += time_bonus
            print(f"Status: 🏁 GOAL REACHED")
            print(
                f"Time Bonus: +{time_bonus} (Finished at t={last_t}, Max: {max_time}, Mult: x{mult})"
            )

        elif is_safe_emergency:
            safe_landing_score = scoring_config.get("safe_emergency_landing_score", 300)
            # Replaces the base 1000 score with the partial credit score
            final_score = final_score - base_score + safe_landing_score
            print(f"Status: 🛬 SAFE EMERGENCY LANDING")
            print(f"Emergency Credit Adjust: Base score replaced with +{safe_landing_score}")

        else:
            print(f"Status: ⏱️ TIME/ENERGY EXPIRED (Goal not reached)")

        # Print Penalties
        if controlled_steps > 0:
            print(f"Controlled Zone Penalties ({controlled_steps} steps): {penalty_controlled}")
        if restricted_steps > 0:
            print(f"Restricted Zone Penalties ({restricted_steps} steps): {penalty_restricted}")
        if advisory_sep_events > 0:
            print(f"Advisory Traffic Losses ({advisory_sep_events} events): {penalty_advisory}")
        if conflict_sep_events > 0:
            print(f"Conflict Traffic Losses ({conflict_sep_events} events): {penalty_conflict}")

        print("---------------------------------------")
        print(f"FINAL SCORE: {max(0, final_score)}")


if __name__ == "__main__":
    main()
