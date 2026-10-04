"""通用 EKF：模型由主程序传入，滤波器只负责预测和更新。"""

from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
Model = Callable[[FloatArray], FloatArray]
Residual = Callable[[FloatArray, FloatArray], FloatArray]


class ExtendedKalmanFilter:
    def __init__(self, x0: FloatArray, p0: FloatArray) -> None:
        self.x = np.asarray(x0, dtype=float).copy()
        self.p = np.asarray(p0, dtype=float).copy()
        if self.x.ndim != 1 or self.p.shape != (self.x.size, self.x.size):
            raise ValueError("x0 必须是一维状态，P0 的形状必须是 (状态维度, 状态维度)")
        if not np.all(np.isfinite(self.x)) or not np.all(np.isfinite(self.p)):
            raise ValueError("x0 和 P0 必须是有限数值")
        if not np.allclose(self.p, self.p.T) or np.linalg.eigvalsh(self.p).min() < 0:
            raise ValueError("P0 必须是对称半正定矩阵")

    def predict(self, f: Model, jacobian_f: Model, q: FloatArray) -> None:
        """x⁻ = f(x)，P⁻ = F P Fᵀ + Q。"""
        transition = jacobian_f(self.x)
        self.x = f(self.x)
        self.p = transition @ self.p @ transition.T + q

    def update(
        self,
        z: FloatArray,
        h: Model,
        jacobian_h: Model,
        r: FloatArray,
        residual: Residual = np.subtract,
    ) -> None:
        """用量测修正预测；Joseph 形式保持协方差的数值稳定性。"""
        observation_matrix = jacobian_h(self.x)
        innovation = residual(z, h(self.x))
        innovation_covariance = observation_matrix @ self.p @ observation_matrix.T + r
        gain = np.linalg.solve(
            innovation_covariance, (self.p @ observation_matrix.T).T
        ).T
        self.x += gain @ innovation
        correction = np.eye(self.x.size) - gain @ observation_matrix
        self.p = correction @ self.p @ correction.T + gain @ r @ gain.T
        self.p = 0.5 * (self.p + self.p.T)
