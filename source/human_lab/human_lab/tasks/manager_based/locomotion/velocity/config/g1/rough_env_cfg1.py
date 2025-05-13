# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg

import human_lab.tasks.manager_based.locomotion.velocity.mdp as mdp
##
# Pre-defined configs
##
from isaaclab_assets import G1_MINIMAL_CFG  # isort: skip
from human_lab.humanlab_assets.robots.unitree import G1_LEG_CFG 

@configclass
class G1Rewards(RewardsCfg):
    """Reward terms for the MDP."""
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp1, weight=1.5, params={"command_name": "base_velocity", "std": 0.5})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp1, weight=1.5, params={"command_name": "base_velocity", "std": 0.5})
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l21, weight=-1.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l21, weight=-0.05)
    energy = RewTerm(func=mdp.energy1, weight=-1e-3)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l21, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l21, weight=-0.01)
    undesired_contacts = RewTerm(func=mdp.undesired_contacts1, weight=-1.0, params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="(?!.*ankle.*).*"), "threshold": 1.0})
    fly = RewTerm(func=mdp.fly1, weight=-1.0, params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_roll.*"), "threshold": 1.0})
    body_orientation_l2 = RewTerm(func=mdp.body_orientation_l21, params={"asset_cfg": SceneEntityCfg("robot", body_names=".*torso.*")}, weight=-2.0)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l21, weight=-1.0)
    termination_penalty = RewTerm(func=mdp.is_terminated1, weight=-200.0)
    # feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped1, 
    #                         weight=0.15, 
    #                         params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_roll.*"), 
    #                                 "threshold": 0.4})
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped1,
        weight=0.25,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle_roll_link"),
            "threshold": 0.4,
        },
    )
    feet_slide = RewTerm(func=mdp.feet_slide1, weight=-0.25, params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_roll.*"), "asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll.*")})
    feet_force = RewTerm(func=mdp.body_force1, weight=-3e-3, params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_roll.*"), "threshold": 500, "max_reward": 400})
    feet_too_near = RewTerm(func=mdp.feet_too_near_humanoid1, weight=-2.0, params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll.*"]), "threshold": 0.2})
    feet_stumble = RewTerm(func=mdp.feet_stumble1, weight=-2.0, params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*ankle_roll.*"])})
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-2.0)
    joint_deviation_hip = RewTerm(func=mdp.joint_deviation_l11, weight=-0.15, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw.*", ".*_hip_roll.*", ".*_shoulder_pitch_joint", ".*_elbow.*"])})
    joint_deviation_arms = RewTerm(func=mdp.joint_deviation_l11, weight=-0.2, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*waist.*", ".*_shoulder_roll_joint", ".*_shoulder_yaw_joint", ".*_wrist.*"])})
    joint_deviation_legs = RewTerm(func=mdp.joint_deviation_l11, weight=-0.02, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_pitch.*", ".*_knee.*", ".*_ankle.*"])})





@configclass
class G1RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: G1Rewards = G1Rewards()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # Scene
        self.scene.robot = G1_LEG_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/torso_link"
        self.scene.num_envs = 4096
        # Randomization
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["torso_link"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        # Rewards


        # Commands
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = "torso_link"


@configclass
class G1RoughEnvCfg_PLAY(G1RoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (1.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.base_external_force_torque = None
        self.events.push_robot = None
