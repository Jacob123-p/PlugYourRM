# custom/schemes/tjurm_scheme/tracker.py
"""多目标跟踪器：马氏门控分配 + 装甲板 EKF + 一发解。

I/O 协议（core 契约）：
    输入   Observation + SelfState
    输出   List[Prediction]，第一条是"该打谁"
    失败   不产出 Prediction

一发解（model→select→traj 迭代 + fire/mode 判定 + 前馈）封装在 ShotSolver。
tracker.step 只调 ShotSolver.solve 一次，拿回 solution + 瞄点。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from config import cfg
from core.algorithms.math.angle import limit_rad
from core.bus import public_bus
from core.bus.dataflow import GimbalTarget, Prediction
from core.bus.messages import PublicMessages
from core.bus.public_bus import debug_plot, debug_text
from core.entities.property.entity_types import EntityType, TargetType
from core.interfaces.tracker import Tracker, register_tracker

from custom.components.armor_selectors.tjurm_armor_selector import ArmorSelector
from custom.components.shooters.tjurm_shooter import ShootDecider
from custom.components.traj_solvers.tjurm_traj_solver import TjurmTrajSolver




class _GimbalView:
    def __init__(self, gimbal_state):
        self.world_rpy = np.asarray(gimbal_state.rpy, float)
        self.world_omg = np.asarray(gimbal_state.omg, float)


class _SelfView:
    """把 core 的 SelfState 适配成自瞄认的自车口径。"""

    def __init__(self, self_state, frame):
        self.state = self_state
        self.frame = frame
        self.gimbal = _GimbalView(self_state.gimbal)

    def _rel(self, pos):
        pos = np.asarray(pos, float)
        if self.frame == "W":
            return pos
        return pos - np.asarray(self.state.gimbal.pos, float)

    def get_muzzle_rel_pos(self) -> np.ndarray:
        return self._rel(self.state.muzzle.pos)

    def get_gimbal_rel_pos(self) -> np.ndarray:
        return self._rel(self.state.gimbal.pos)

    def get_camera_rel_pos(self) -> np.ndarray:
        return self._rel(self.state.camera.pos)


@register_tracker("yours", aliases=["yours", "default"])
class YourTrackerNode(Tracker):
    description = ""

    def __init__(self, params: Optional[MultiTargetParams] = None):
        self.params = params or MultiTargetParams()
        self.frame = str(cfg.sim.frame).upper()
        if self.frame not in ("G", "W"):
            raise ValueError(f"Sim.frame 必须是 'G' 或 'W'，实得 {self.frame!r}")

        self.obs_armors = None
        self.timestamp = 0.

    def _new_selector(self) -> ArmorSelector:
        return ArmorSelector(spin_branch_thresh=self.params.spin_thresh,
                             lock_bias=self.params.lock_bias)

    @staticmethod
    def _clear_selector(sel: ArmorSelector) -> None:
        sel.locked_armor_id = None
        sel.selected_armor_id = None

    def reset(self) -> None:
        pass

    # ------------------------------------------------------------------ 主流程
    def step(self, obs, self_state) -> List[Prediction]:
        if obs is None:
            obs_list = []
        else:
            obs_list = list(obs.obs_armors or [])
            self.timestamp = float(obs.timestamp)

        self.robot = _SelfView(self_state, self.frame)
        self.ref_gen.robot = self.robot
        self.obs_armors = obs_list
        timestamp = self.timestamp

    def _build_predictions(self, bundle, target, timestamp) -> List[Prediction]:
        if bundle.pred_armor is None:
            return []
        sol = bundle.solution
        pred = Prediction(
            target_type=self._target_type(target),
            center=bundle.pred_center,
            armors=[bundle.pred_armor],
            timestamp=timestamp,
            is_tracking=True,
            aim_rate=np.array([sol.yaw_rate, sol.pitch_rate]),
        )
        return [pred]