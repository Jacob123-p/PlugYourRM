# custom/schemes/tjurm_scheme/pose_solver.py
"""tjurm 帧级位姿解算：遍历检测 -> 单板 PnP -> C' -> G/W -> 打包 Observation。"""
from __future__ import annotations

import math

import numpy as np

from config import cfg
from core.algorithms.math.transform import (abs_camera_to_gimbal,
                                            gimbal_to_world,
                                            rpy_to_R)
from core.bus.dataflow import ArmorObservation, Observation
from core.entities.property.entity_types import EntityType
from core.interfaces.pose_solver import PoseSolver, register_pose_solver

from custom.components.armor_solvers import get_solver


@register_pose_solver("yours")
class YourPoseSolver(PoseSolver):

    description = ""

    def __init__(self, armor_solver_name=None, *, frame=None):
        self.frame = str(cfg.sim.frame if frame is None else frame).upper()
        if self.frame not in ("G", "W"):
            raise ValueError(f"Sim.frame 必须是 'G' 或 'W'，实得 {self.frame!r}")
        self.armor_solver = get_solver(
            armor_solver_name or str(cfg.sim.pnp_method))

    def reset(self) -> None:
        pass

    def solve(self, frame, self_state):
        cam = self_state.camera
        R_camera_to_abs_camera = rpy_to_R(cam.rpy)
        gimbal_pos_world = np.asarray(self_state.gimbal.pos, float)
        camera_pos_gimbal = np.asarray(cam.pos, float) - gimbal_pos_world

        armors = []
        return Observation(armors, float(frame.timestamp))

    @staticmethod
    def _mount_pitch(entity_type):
        tilt = float(cfg.outpost.armor_tilt if entity_type is EntityType.Outpost
                     else cfg.robot.armor_tilt)
        return math.radians(tilt)