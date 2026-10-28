"""Entry point: runs the three PID experiments (under, over, tuned).

Run from the project root with:  python main.py
"""
from robot_pid.map import Map
from robot_pid.pid import PID
from robot_pid.planner import AStarPlanner
from robot_pid.simulator import Simulator


def main():
    #build the map and plan the path once, so every gain set  is compared on exactly the same path
    grid_map = Map.build_default_map()
    waypoints = AStarPlanner(grid_map).get_waypoints((0, 0), (19, 19))  #(row, col)


    #(label, heading (kp, ki, kd), speed (kp, ki, kd))
    params = [
        ("under", (0.1, 0.1, 0.2), (0.1, 0.2, 0.1)),
        ("over",  (0.5, 0.2, 0.2), (0.5, 0.2, 0.2)),
        ("tuned", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ]

    for label, (hkp, hki, hkd), (skp, ski, skd) in params:
        #new PID objects for every experiment
        heading_pid = PID(hkp, hki, hkd, output_limits=(-2, 2))
        speed_pid = PID(skp, ski, skd, output_limits=(0, 0.5))

        sim = Simulator(grid_map, waypoints, heading_pid, speed_pid)

        #compute (no window is opened here)
        data = sim.run()

        #metrics
        mean_cte, max_cte = sim.cross_track_error(data)
        print(f"[{label}] reached goal: {data['reached_goal'][-1]}, "
              f"collisions: {data['collision_counter']}, "
              f"CTE mean/max: {mean_cte:.3f} / {max_cte:.3f} m")
 

        #visualisation: close each window to continue
        sim.animate(data)        # pygame
        sim.plot(data)   # matplotlib




if __name__ == "__main__":
    main()