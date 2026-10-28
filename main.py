"""Main file to run the experements and GUI Visualization"""

from robot_pid.map import Map
from robot_pid.pid import PID
from robot_pid.planner import AStarPlanner
from robot_pid.simulator import Simulator


if __name__ == "__main__":
    print("start")

    grid_map = Map.build_default_map()
    waypoints = AStarPlanner(grid_map).get_waypoints((0, 0), (19, 19))
    print("waypoints:", len(waypoints))

    sim = Simulator(grid_map, waypoints,
                    PID(2.0, 0.0, 0.5, output_limits=(-2, 2)),
                    PID(1.0, 0.1, 0.2, output_limits=(0, 0.5)))
    data = sim.run()

    print("reached goal:", data["reached_goal"][-1])
    print("collisions:", data["collision_counter"])
    print("cross-track (mean, max):", sim.cross_track_error(data))