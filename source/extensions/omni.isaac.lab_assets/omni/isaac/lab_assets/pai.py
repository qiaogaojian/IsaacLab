# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Agility robots.

The following configurations are available:

* :obj:`CASSIE_CFG`: Agility Cassie robot with simple PD controller for the legs

Reference: https://github.com/UMich-BipedLab/Cassie_Model/blob/master/urdf/cassie.urdf
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

PAI_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"D:/Git/Work/Robot/IsaacLab/resources/robots/pai_12dof/pai_12dof.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4),
        joint_pos={
            "left_hip_yaw_joint":      0.0,
            "left_hip_roll_joint":     0.0,
            "left_hip_pitch_joint":    0.0,
            "left_knee_joint":         0.0,
            "left_ankle_pitch_joint":  0.0,
            "left_ankle_roll_joint":   0.0,

            "right_hip_yaw_joint":     0.0,
            "right_hip_roll_joint":    0.0,
            "right_hip_pitch_joint":   0.0,
            "right_knee_joint":        0.0,
            "right_ankle_pitch_joint": 0.0,
            "right_ankle_roll_joint":  0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_joint",
            ],
            effort_limit=200.0,
            velocity_limit=10.0,
            stiffness={
                ".*_hip_yaw_joint":     15.0,  # 40.0,
                ".*_hip_roll_joint":    15.0,  # 80.0,
                ".*_hip_pitch_joint":   15.0,  # 15.0,
                ".*_knee_joint":        15.0,  # 40.0,
                ".*_ankle_pitch_joint": 15.0,  # 80.0,
                ".*_ankle_roll_joint":  15.0,  # 15.0,
            },
            damping={
                ".*_hip_yaw_joint":     0.05,  # 0.125,
                ".*_hip_roll_joint":    0.05,  # 0.125,
                ".*_hip_pitch_joint":   0.05,  # 0.125,
                ".*_knee_joint":        0.05,  # 0.125,
                ".*_ankle_pitch_joint": 0.05,  # 0.125,
                ".*_ankle_roll_joint":  0.05,  # 0.125,
            },
        ),
    },
)
