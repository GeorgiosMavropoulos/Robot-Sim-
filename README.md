# Robot Path Tracking with PID Control (CET3004 – Assignment 1)

A 2D robot simulator in Python in which a unicycle robot follows a path planned by A* using two PID controllers (heading and speed). Visualisation uses pygame (live animation) and matplotlib (final plots).


## Project structure

```
project/
├── main.py              # entry point (runs the experiments)          [to do]
├── requirements.txt     # third-party dependencies
├── README.md
├── .gitignore
└── robot_pid/
    ├── __init__.py      # empty, marks the folder as a package
    ├── pid.py           # PID controller class                        [done]
    ├── robot.py         # robot kinematics (unicycle model)           [done]
    ├── map.py           # occupancy grid, obstacles, conversions      [done]
    ├── planner.py       # A* path planner                             [done]
    └── simulator.py     # simulation loop, metrics, plots, pygame     [to do]
```

This layout deliberately differs from the suggested template: the map is a separate class (`map.py`) instead of being part of the robot, and each component is a class with a single responsibility.

## How to run (VS Code)

The program does not install its own dependencies: they are installed once from `requirements.txt` into a virtual environment.

1. Install Python 3.10+ and open the project folder in VS Code.
2. Create and activate a virtual environment (project root):
   - Windows (PowerShell): `python -m venv .venv` then `.venv\Scripts\Activate.ps1`
   - macOS / Linux: `python3 -m venv .venv` then `source .venv/bin/activate`
3. In VS Code select the interpreter of `.venv` (Ctrl+Shift+P -> "Python: Select Interpreter").
4. Install the dependencies: `pip install -r requirements.txt`
5. Run from the project root: `python main.py`.


## Design overview

| Component | Responsibility | Must not know about |
|---|---|---|
| `PID` | error + dt -> control command | robot, map, pygame |
| `Map` | grid, obstacles, collision check, cell <-> metre conversion | robot, PID |
| `AStarPlanner` | map, start, goal -> list of (x, y) waypoints | PID, robot |
| `Robot` | state (x, y, theta), `step(v, omega, dt)`, drawing | PID, planner |
| `Simulator` | connects everything, runs the loop, records metrics, plots | internals of A* or dynamics |

Control loop (one step of length `dt`):

1. Compute distance and bearing to the current waypoint.
2. `heading_error = angle_wrap(desired_heading - robot.theta)`
3. `omega_cmd = heading_pid.update(heading_error, dt)`
4. `v_cmd = speed_pid.update(distance_to_goal, dt)`, scaled by `max(0, cos(heading_error))` to slow down in sharp turns.
5. `robot.step(v_cmd, omega_cmd, dt)`

Design rationale for two controllers: the heading PID lets the unicycle follow the turns of the A* path; the speed PID makes the robot decelerate smoothly towards the final goal instead of overshooting it. Both are instances of the same `PID` class with different gains and output limits.

## `pid.py` – the PID class

```python
pid = PID(kp, ki, kd, output_limits=(None, None))
command = pid.update(error, dt)   # one control step, returns the command
pid.reset()                       # clear integral and previous error
```

- P, I, D terms with `integral += error * dt` and `derivative = (error - previous_error) / dt`; derivative is 0 on the first call.
- Output clamped to `(low, high)` (`None` = no limit on that side).
- Anti-windup: if the output saturates, the integral increase of that step is undone.
- Input validation: `ValueError` for non-finite `error`, `dt <= 0` or inconsistent limits; `TypeError` for non-numeric gains.

Tests (P only, I only, D only, limits, reset, anti-windup) passed; anti-windup check: with `ki=1`, error=10, limits (-1, 1) the integral stayed at 1.0 after 100 steps instead of growing to 100. After the final refactor (`_clamp`, `_validate_inputs`) the tests and the three exception cases (`PID(1,0,0,(2,1))`, `update(1, 0)`, `update(nan, 0.1)`) should be re-run.

## `robot.py` – unicycle kinematics

State: `x`, `y` (m) and `theta` (rad, wrapped to [-pi, pi)). `step(v, omega, dt)` uses forward Euler integration: the position is updated first with the current heading, then the heading is rotated by `omega * dt` and wrapped. The model is deterministic (no random motion), so every run with the same gains gives the same result.

Inputs are validated (`v`, `omega`, `dt` finite, `dt > 0`).

Tests: straight motion, motion along +y, rotation in place and wrap-around passed. Circle test (`v=1`, `omega=1`, `dt=0.01`, 628 steps) returned the robot to within about 0.003 m of the start, which confirms the model; the remaining deviation comes from the Euler step. A coarser `dt=0.1` run is planned to justify the choice of `dt` in the report.

## `map.py` – occupancy grid

A grid of `rows x cols` cells; `0` = free, `1` = obstacle. A cell is always a tuple `(row, col)`; positions in metres are `(x, y)` with `x` <-> column and `y` <-> row. Each cell is `cell_size` metres wide (default 0.2).

| Method | Purpose |
|---|---|
| `Map(rows, cols, cell_size=0.2)` | create an empty map (validated arguments) |
| `inbound_cell(cell)` | is the cell inside the grid? |
| `is_free_cell(cell)` | inside the grid and not an obstacle? (out of bounds -> `False`) |
| `neighbors(cell)` | free 4-connected neighbours, as a list of `(row, col)` tuples |
| `cell_to_world(cell)` | centre of the cell in metres `(x, y)` |
| `world_to_cell(x, y)` | cell containing a point in metres (uses `math.floor`) |
| `is_free(x, y)` | collision check in metres (used by the simulator) |
| `add_obstacle_rect(r0, c0, r1, c1)` | fill a rectangle of cells (corners may be given in any order) |

Design decisions:

- Cells are plain tuples (hashable), which is what the A* dictionaries need.
- Out-of-bounds queries return `False` (normal answer, the planner asks for neighbours at the map border); malformed input (wrong type, NaN) raises an exception.
- The grid is built inside the constructor, so its size always matches `rows` and `cols`.
- The map is built once (fixed layout) and reused by all experiment runs, so gain sets are compared on the same path.

Tests: `inbound_cell` (6 cells) and `neighbors` on a 5x5 map were checked. Still to confirm and record: `is_free_cell`, round trip `world_to_cell(*cell_to_world(c)) == c` for every cell, `is_free`, and `add_obstacle_rect` (normal order, reversed corners giving the same grid, out-of-bounds and non-integer errors).

## Experiment plan

- Same map and same planned path for every run (fixed layout built by one function).
- Three heading-PID gain sets: **under-damped**, **over-damped**, **well-tuned**; the speed PID is held fixed so the effect of the heading gains can be isolated.
- One extra comparison: heading PID only vs. heading + speed PID, to support the claim that dual control gives smoother tracking.
- Metrics: time to reach goal, path length, cross-track error (average and maximum), number of collisions (via `Map.is_free`).
- Tuning strategy: start with `ki = kd = 0`, raise `kp` until the robot follows the path but begins to oscillate, add `kd` to damp the oscillation, add a very small `ki` only if a steady-state error remains.
- Experiments run headless (no window); pygame is used for the live demo, matplotlib plots are produced after the simulation ends.

## Project status

- [x] Architecture and class responsibilities decided
- [x] `pid.py` written, documented and tested
- [x] `robot.py` written, documented and tested (circle test)
- [x] `map.py` written and documented (last tests to record)
- [x] `build_default_map()` (fixed obstacle layout for the experiments)
- [x] `planner.py` (A*: heuristic, open set, `came_from`, path reconstruction, cells -> metres)
- [ ] `simulator.py` (loop, headless mode, metrics, plots, pygame view, `draw` methods)
- [ ] `main.py` (three gain sets, results table)
- [ ] Tuning of the three gain sets
- [ ] Report (max 1,500 words, including a short ethics / safety reflection)



## References

- Wikipedia, "PID controller" (discrete PID pseudocode)
- `simple-pid` library (m-lundberg/simple-pid), read as a design reference only
- GeekForGeeks A*search algorithm