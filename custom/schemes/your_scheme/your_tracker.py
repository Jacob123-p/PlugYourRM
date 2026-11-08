# custom/schemes/your_scheme/your_tracker.py
"""你的跟踪器：一帧观测进，一帧预测出。

契约（core/interfaces/tracker.py）：
    输入     Observation + SelfState
    输出     List[Prediction]，第一条是"该打谁"
    没目标   返回空列表

云台只认 ShotSolution：要让云台动，得在 step() 里自己
public_bus().publish(PublicMessages.CONTROL, solution)。

写作指南：docs/writing_your_tracker.md
"""
from __future__ import annotations

from typing import List

from core.bus.dataflow import Prediction
from core.interfaces.tracker import Tracker, register_tracker


@register_tracker("yours")
class YourTracker(Tracker):
    description = "我的方案：跟踪器（还没写）"

    def step(self, obs, self_state) -> List[Prediction]:
        # TODO: 写你的跟踪器。obs.obs_armors 是本帧观测到的板（可能是空列表）。
        return []
