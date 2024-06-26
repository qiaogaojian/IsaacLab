"""
This script demonstrates different legged robots.
.. code-block:: bash
    # Usage
    ./isaaclab.sh -p source/standalone/demos/quadrupeds.py
"""
from pycore.utils.file_utils import FileUtils

"""Launch Isaac Sim Simulator first."""

import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import numpy as np
import torch
import numpy as np
import pandas as pd
import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation

##
# Pre-defined configs
##
from omni.isaac.lab_assets.pai import PAI_CFG

from pycore.base import Core
from pycore.logger import Logger
from pycore.utils.tools import Tools


def design_scene() -> tuple[dict, list[list[float]]]:
    """Designs the scene."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Origin 1 with Anymal B
    prim_utils.create_prim("/World/Origin1", "Xform")
    # -- Robot
    robot = Articulation(PAI_CFG.replace(prim_path="/World/Origin1/Robot"))

    return robot


def run_simulator(sim: sim_utils.SimulationContext, robot: dict[str, Articulation]):
    """Runs the simulation loop."""
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # ********************* action queue *********************
    # 使用pandas读取CSV文件
    csv_file_path = FileUtils.get_project_path("resources/csv/action_Jun12_20-01-14_pos_v13.csv")
    df = pd.read_csv(csv_file_path)
    # 将DataFrame转换为numpy数组
    np_array = df.values

    action_queue = []
    for row in np_array:
        ten = torch.Tensor(row)
        action = ten.unsqueeze(0)
        action_queue.append(action)
    print(f"load csv: {csv_file_path} success!")

    # Simulate physics
    while simulation_app.is_running():
        # reset
        sim_time = 0.0

        # root state
        root_state = robot.data.default_root_state.clone()
        # root_state[:, 2] += 1
        robot.write_root_state_to_sim(root_state)
        # joint state
        joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
        robot.write_joint_state_to_sim(joint_pos, joint_vel)
        # reset the internal state
        robot.reset()
        print("[INFO]: Resetting robots state...")

        for action in action_queue:
            # generate random joint positions
            joint_pos_target = action[0] * 0.25
            # apply action to the robot
            robot.set_joint_position_target(joint_pos_target)
            # write data to sim
            robot.write_data_to_sim()

            # perform step
            sim.step()
            # update sim-time
            sim_time += sim_dt

            # update buffers
            robot.update(sim_dt)


def main():
    """Main function."""
    # Initialize the simulation context
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01))
    # Set main camera
    sim.set_camera_view(eye=[1.5, 1.5, 1.5], target=[0.0, 0.0, 0.0])
    # design scene
    robot = design_scene()
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, robot)


if __name__ == '__main__':
    args = Tools.parse_args()
    core = Core()
    core.init(env=args.env)
    Logger.instance().info('********************************* Test *********************************')


    # run the main function
    main()
    # close sim app
    simulation_app.close()
