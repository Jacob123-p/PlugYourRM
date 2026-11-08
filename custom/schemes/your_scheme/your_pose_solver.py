# custom/schemes/your_scheme/your_pose_solver.py
"""你的位姿解算：一帧检测进，一帧观测出。

契约（core/interfaces/pose_solver.py）：
    输入     DetectionFrame + SelfState
    输出     Observation，每块看到的板一条 ArmorObservation

单板 PnP 在 custom/components/armor_solvers/，用它、换掉它、不用它都行。
坐标系约定：docs/coordinate_systems.md
"""
from __future__ import annotations

from core.bus.dataflow import Observation
from core.interfaces.pose_solver import PoseSolver, register_pose_solver


@register_pose_solver("yours")
class YourPoseSolver(PoseSolver):
    description = "我的方案：位姿解算（还没写）"

    def solve(self, frame, self_state) -> Observation:
        # TODO: 遍历 frame.detections，解出每块板的位置，打包成 Observation。
        return Observation([], float(frame.timestamp))
