# custom/components/armor_solvers/__init__.py
"""单板 PnP 求解器：像素四角点 -> C' 系下的板位姿。

当前只有一种实现（ippe.py）。没有抽象基类——新的求解器只要用同一套关键字签名，
再在下面的 _SOLVERS 里登记一行就接上了。

公共契约::

    estimate(*, camera_k, armor_size, image_points_2d,
             rot_camera_to_abs_camera, mount_pitch=0.0) -> (pos, rpy) | (None, None)

    image_points_2d        (4,2) 原始像素（RDF 轴序，主点在图像中心，顺序 LT/RT/RB/LB）
    rot_camera_to_abs_camera  R^{C'}_{C1}：相机系 -> 绝对方向系（= 相机在 world 下的姿态）
    mount_pitch            theta_0：板的安装俯仰（rad）
    返回                    pos 在 C' 系（光心为原点），rpy = [0, theta_0, psi]
"""
from __future__ import annotations

import importlib

#: 名字 -> (模块, 类名)；config 里的 sim.pnp_method 就是这个键。
_SOLVERS = {
    "ippe": ("custom.components.armor_solvers.ippe", "IppeSolver"),
}


def plate_size(armor_size):
    """板几何的唯一来源：config（与 core/entities/rigids/armor.py 同源）。

    large = hero 车的大板，其余 = 小板。
    """
    from config import cfg
    veh = cfg.robot.hero if str(armor_size) == "large" else cfg.robot.infantry
    return float(veh.light_bar_interval), float(veh.light_bar_length)


def get_solver(name):
    """按 config 的 sim.pnp_method 取实现；延迟 import，避免包内循环。"""
    key = str(name)
    if key not in _SOLVERS:
        raise KeyError(f"未注册的单板求解器 {key!r}；可用：{sorted(_SOLVERS)}")
    module_name, class_name = _SOLVERS[key]
    return getattr(importlib.import_module(module_name), class_name)()
