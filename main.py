"""Entry point: runs the three PID experiments (under, over, tuned).

Run from the project root with:  python main.py
"""
import csv
import os
from robot_pid.map import Map
from robot_pid.pid import PID
from robot_pid.planner import AStarPlanner
from robot_pid.simulator import Simulator


def main():
    #build the map and plan the path once, so every gain set is compared on exactly the same path
    grid_map = Map.build_default_map()
    waypoints = AStarPlanner(grid_map).get_waypoints((0, 0), (19, 19))  # (row, col)

    #initial experiment parameters: (label, heading (kp, ki, kd), speed (kp, ki, kd))
    params = [
        ("under", (4, 0.01, 0.05), (0.5, 0.1, 0.1)),
        ("over", (7, 0.01, 0.1), (0.5, 0.1, 0.1)),
        ("tuned", (4, 0.1, 0.1), (0.5, 0.1, 0.1)),
    ]

    #empty dictionary to store metrics (cross-track-error, traversed distance, and time to reach goal)
    metrics_collection = {}

    #define standard column headers for the CSV file
    fieldnames = [
        "Experiment",  #row ID: under, over, or tuned
        "time",
        "path_length",
        "cte_mean",
        "cte_max",
        "heading_error_mean",
        "heading_error_max",
        "reached_goal",
        "collisions",
    ]

    #declare the output CSV file name
    file_name = "robot_metrics.csv"

    #start the experiment loop
    for label, (hkp, hki, hkd), (skp, ski, skd) in params:
        # Define new PID objects for every experiment with custom limits
        heading_pid = PID(hkp, hki, hkd, output_limits=(-2.5, 2.5))
        speed_pid = PID(skp, ski, skd, output_limits=(0, 0.3))

        sim = Simulator(grid_map, waypoints, heading_pid, speed_pid)

        #run the simulation (no window is opened during calculation)
        data = sim.run()

        #call metrics method from simulator class to calculate results
        metrics = sim.metrics(data)

        #append calculated metrics into metrics_collection dictionary
        metrics_collection[label] = metrics

        #convert metrics_collection into a flat list of dictionaries for the CSV writer
        all_results = []
        for exp_label, exp_metrics in metrics_collection.items():
            row = {
                "Experiment": exp_label
            }  # Store the label name under the 'Experiment' column
            row.update(exp_metrics)  # Copy all the remaining calculated metrics
            all_results.append(row)

        #write current accumulated results into the CSV file
        #we use mode="w" because 'all_results' always contains ALL experiments completed so far
        with open(file_name, mode="w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()  # Write the header columns at the top
            writer.writerows(all_results)  # Write all current rows

        #display results on terminal
        print(f"{label}:Total time till reach the goal: {metrics['time']}")
        print(f"{label}:Length of traversed path: {metrics['path_length']}")
        print(f"{label}:Mean cross track error: {metrics['cte_mean']}")
        print(f"{label}:Maximum cross track error: {metrics['cte_max']}")
        print(
            f"{label}:Heading error mean value: {metrics['heading_error_mean']}"
        )
        print(
            f"{label}:Heading error max value: {metrics['heading_error_max']}"
        )
        print(f"{label}:Did robot reach the goal? {metrics['reached_goal']}")
        print(f"{label}:Total collisions: {metrics['collisions']}")

        #separate each experiment results with a visual line
        print("-" * 40)

        #visualization: Close each window to continue to the next experiment
        sim.animate(data)  #renders the Pygame animation
        sim.plot(data)  #plots performance charts using Matplotlib


if __name__ == "__main__":
    main()