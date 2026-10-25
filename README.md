Robot Path Tracking with PID Control (CET3004 – Assignment 1)

A 2D robot simulator in Python in which a unicycle robot follows a path planned by A* using two PID controllers (heading and speed). Visualisation uses pygame (live animation) and matplotlib (final plots).

Status: work in progress. See "Project status" below for what is finished and what is still to do.

Project structure
project/
├── main.py              # entry point (runs the experiments)
└── robot_pid/
    ├── __init__.py      # empty, marks the folder as a package
    ├── pid.py           # PID controller class            [done, tested]
    ├── robot.py         # robot dynamics (unicycle model) [to do]
    ├── world.py         # map / grid / obstacles          [created, to be reviewed]
    ├── planner.py       # A* path planner                 [to do]
    └── simulator.py     # simulation loop, metrics, plots [to do]

This layout deliberately differs from the suggested template: the map is a separate class (world.py) instead of being part of the robot, and each component is a class with a single responsibility.

How to run (VS Code)
Install Python 3.10+ and open the project folder in VS Code.
Install dependencies: pip install pygame matplotlib
Run from the project root: python main.py