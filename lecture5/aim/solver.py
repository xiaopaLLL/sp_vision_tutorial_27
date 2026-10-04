"""四角点 → 装甲板位姿 → 机器人位置/朝向量测。"""

import cv2 as cv
import numpy as np
from scipy.optimize import least_squares

from sim.scene import ARMOR_POINTS, ROT_CAMERA_TO_CV, Armor, Camera, wrap_angle
from utils.ekf import FloatArray


class Solver:
    def __init__(self, camera: Camera) -> None:
        self.camera = camera

    def solve(self, image_points: FloatArray) -> tuple[float, FloatArray]:
        """先用平面 PnP 初始化位置，再固定倾角优化 yaw/x/y/z。"""
        success, _, translation = cv.solvePnP(
            ARMOR_POINTS,
            image_points,
            self.camera.matrix,
            self.camera.distortion,
            flags=cv.SOLVEPNP_IPPE,
        )
        if not success:
            raise RuntimeError("装甲板 PnP 解算失败")
        position = ROT_CAMERA_TO_CV.T @ translation.reshape(3) + self.camera.position

        def residual(state: FloatArray) -> FloatArray:
            armor = Armor(position=state[1:], yaw=float(state[0]), index=0)
            return (self.camera.project(armor) - image_points).reshape(-1)

        result = least_squares(residual, np.array([0.0, *position]))
        if not result.success or not np.all(np.isfinite(result.x)):
            raise RuntimeError(f"装甲板位姿优化失败: {result.message}")
        return wrap_angle(float(result.x[0])), result.x[1:].copy()

    def robot_measurement(
        self, image_points: FloatArray, armor_index: int, radius: float
    ) -> tuple[float, FloatArray]:
        """教学中已知装甲板编号和半径，避免引入数据关联与半径估计。"""
        armor_yaw, armor_position = self.solve(image_points)
        robot_yaw = wrap_angle(armor_yaw - armor_index * np.pi / 2)
        center = armor_position + radius * np.array(
            [np.cos(armor_yaw), np.sin(armor_yaw), 0.0]
        )
        return robot_yaw, center
