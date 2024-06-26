import os
import sys

ISAACLAB_DIR = "D:/Git/Work/Robot/IsaacLab"
if os.path.exists(f"{ISAACLAB_DIR}/source/extensions/omni.isaac.lab"):
    sys.path.append(f"{ISAACLAB_DIR}/source/extensions/omni.isaac.lab")
if os.path.exists(f"{ISAACLAB_DIR}/source/extensions/omni.isaac.lab_assets"):
    sys.path.append(f"{ISAACLAB_DIR}/source/extensions/omni.isaac.lab_assets")
if os.path.exists(f"{ISAACLAB_DIR}/source/extensions/omni.isaac.lab_tasks"):
    sys.path.append(f"{ISAACLAB_DIR}/source/extensions/omni.isaac.lab_tasks")
