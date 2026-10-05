"""Bisection harness: finds the largest lean a controller recovers from, and despins after.

Usage: ./isaac/run_harness.sh --label lqr_0p025 [--gui]
The controller under test runs separately as a ROS 2 node; this script only plays the plant,
sets each initial lean, and judges the outcome.
"""
import argparse
import csv
import os

parser = argparse.ArgumentParser()
parser.add_argument("--label", required=True, help="names the CSV, e.g. lqr_0p025")
parser.add_argument("--lo", type=float, default=3.0, help="lean that must recover (deg)")
parser.add_argument("--hi", type=float, default=15.0, help="lean that must fall (deg)")
parser.add_argument("--tol", type=float, default=0.05, help="stop when hi - lo is below this (deg)")
parser.add_argument("--gui", action="store_true", help="render a window; slower")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

# The app must exist before any other isaacsim/omni import; those modules load with it.
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": not args.gui})

from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.prims import SingleArticulation
import numpy as np

# Resolve the stage relative to this script, not the shell's working directory.
STAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "first_pendulum.usd")

# ROS 2 bridge: the stage's OmniGraph publishes /joint_states and /clock, subscribes /joint_command.
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

open_stage(STAGE)

import omni.timeline
from isaacsim.core.api import SimulationContext

# One app update = one 240 Hz physics step, as in the GUI runs. The standalone app's default
# frame is 1/60 s, which feeds the controller state at 60 Hz and makes LQR fail from 3 deg.
sim = SimulationContext(physics_dt=1 / 240, rendering_dt=1 / 240, stage_units_in_meters=1.0)

# Equivalent of pressing Play: without it the stage loads but physics never steps.
sim.play()

pendulum = SingleArticulation(prim_path="/World/pendulum")
simulation_app.update()        # physics must have stepped once before initialize
pendulum.initialize()

BASE = pendulum.get_dof_index("base_body")
ARM = pendulum.get_dof_index("body_arm")


def teleport(theta0_deg):
    """Set the body lean, and zero both joint velocities so no motion carries into the trial."""
    pendulum.set_joint_positions(np.array([np.deg2rad(theta0_deg)]), joint_indices=np.array([BASE]))
    pendulum.set_joint_velocities(np.zeros(2), joint_indices=np.array([BASE, ARM]))


# The harness listens to /joint_command only to confirm a controller is running.
import time

import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("harness")
last_command_wall = None  # wall time of the latest /joint_command, None until one arrives


def on_command(msg):
    global last_command_wall
    last_command_wall = time.monotonic()


node.create_subscription(JointState, "/joint_command", on_command, 10)

timeline = omni.timeline.get_timeline_interface()

TRIAL_S = 10.0       # sim seconds per trial, long enough for despin
FELL_DEG = 45.0      # same "fell" definition as analysis/bag_reader.py
UPRIGHT_DEG = 0.5    # recovered: body within this of upright at the end...
DESPUN_RAD_S = 1.0   # ...and arm slower than this
ALIVE_S = 0.5        # controller counts as alive if it published within this much wall time


def step():
    """Advance the app one update and let rclpy deliver any waiting messages."""
    simulation_app.update()
    rclpy.spin_once(node, timeout_sec=0.0)


def wait_for_controller(timeout_s=30.0):
    start = time.monotonic()
    while last_command_wall is None or time.monotonic() - last_command_wall > ALIVE_S:
        if time.monotonic() - start > timeout_s:
            raise RuntimeError("no /joint_command: is the controller running?")
        step()


def trial(theta0_deg):
    """Teleport to theta0 and run TRIAL_S of sim time. Returns a dict of the outcome."""
    wait_for_controller()
    teleport(theta0_deg)
    t0 = timeline.get_current_time()
    fell = False
    while timeline.get_current_time() - t0 < TRIAL_S:
        step()
        theta = pendulum.get_joint_positions()[BASE]
        if abs(np.rad2deg(theta)) > FELL_DEG:
            fell = True
            break
    if time.monotonic() - last_command_wall > ALIVE_S:
        raise RuntimeError("controller went silent during the trial; result discarded")
    theta_end = np.rad2deg(pendulum.get_joint_positions()[BASE])
    w_arm_end = pendulum.get_joint_velocities()[ARM]
    recovered = (not fell) and abs(theta_end) < UPRIGHT_DEG and abs(w_arm_end) < DESPUN_RAD_S
    result = {"theta0": theta0_deg, "recovered": recovered, "fell": fell,
              "theta_end_deg": theta_end, "w_arm_end": w_arm_end}
    print(f"trial {theta0_deg:6.2f} deg: recovered={recovered} fell={fell} "
          f"theta_end={theta_end:+.3f} deg w_arm_end={w_arm_end:+.2f} rad/s", flush=True)
    return result


RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
os.makedirs(RESULTS, exist_ok=True)
csv_path = os.path.join(RESULTS, f"bisect_{args.label}.csv")
rows = []


def run(theta0, note):
    r = trial(theta0)
    r["note"] = note
    rows.append(r)
    return r["recovered"]


try:
    # Sanity pair: a controller that fails these is broken or misconfigured, not near a boundary.
    if not run(args.lo, "sanity_lo"):
        raise RuntimeError(f"failed to recover from {args.lo} deg; check controller and rates")
    if run(args.hi, "sanity_hi"):
        raise RuntimeError(f"recovered from {args.hi} deg; raise --hi")

    # Invariant: lo always recovered, hi always failed.
    lo, hi = args.lo, args.hi
    while hi - lo > args.tol:
        mid = 0.5 * (lo + hi)
        if run(mid, "bisect"):
            lo = mid
        else:
            hi = mid

    # Determinism check: the two bracket ends again. A changed answer means the boundary is noisy.
    repeat_lo = run(lo, "repeat_lo")
    repeat_hi = run(hi, "repeat_hi")
    print(f"BOUNDARY {args.label}: recovers {lo:.3f} deg, fails {hi:.3f} deg; "
          f"repeat agrees: {repeat_lo and not repeat_hi}", flush=True)
finally:
    # Written even if a trial raised, so a partial run is not lost.
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["theta0", "recovered", "fell",
                                               "theta_end_deg", "w_arm_end", "note"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} trials to {csv_path}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
