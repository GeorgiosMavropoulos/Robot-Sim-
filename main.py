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
        ("under", (1, 0.1, 0.1), (0.2, 0.1, 0.1)),
        ("over",  (10,  1.0, 0.0), (0.6, 0.0, 0.0)),
        ("tuned", (6,   0.0, 0.2), (0.6, 0.0, 0.0)),
    ]



        #empty dictionary to store metrics (cross-track-error, traversed distance and time to reach goal if did)
    metrics_collection = {}
    for label, (hkp, hki, hkd), (skp, ski, skd) in params:
        #new PID objects for every experiment
        heading_pid = PID(hkp, hki, hkd, output_limits=(-2.5, 2.5))
        speed_pid = PID(skp, ski, skd, output_limits=(0, 0.3))

        sim = Simulator(grid_map, waypoints, heading_pid, speed_pid)

        #compute (no window is opened here)
        data = sim.run()

        #call metrics method from simulator calss to calculate cross-track-error, the distance the robot traversed and the time to reach goal if reached.
        metrics = sim.metrics(data)

        #append calculated metrics into metrics_collection
        metrics_collection[label] = metrics
        #print values
        print(f"{label}:Total time till reach the goal: {metrics['time']}") #print time to reach goal if reached
        print(f"{label}:Length of traversed path: {metrics['path_length']}") #print traversed distance
        print(f"{label}:Mean cross track error: {metrics['cte_mean']}") #print mean cross track error
        print(f"{label}:Maximum cross track error: {metrics['cte_max']}") #print maximum cross track error
        print(f"{label}:Heading error mean value:{metrics['heading_error']}") #print heading error's mean value
        print(f"{label}:Heading error max value:{metrics['heading_error']}") #print heading error's max value
        print(f"{label}:Did robot reach the goal? {metrics['reached_goal']}") #print True if reached goal or false if didn't
        print(f"{label}:Total collisions: {metrics['collisions']}") #print number of collisions if any
        #seperate each experiment results with a seperation line
        print("-" * 40)
 

        #visualisation: close each window to continue
        sim.animate(data)        # pygame
        sim.plot(data)   # plot results

   




if __name__ == "__main__":
    main()