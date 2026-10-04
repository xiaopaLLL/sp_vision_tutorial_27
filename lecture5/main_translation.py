"""入口一：固定 x，沿 y 轴匀速往返，比较 x/y 真值、预测与估计。"""

import numpy as np

from aim.solver import Solver
from sim.scene import Camera, Robot
from utils.ekf import ExtendedKalmanFilter, FloatArray
from utils.visualizer import RerunVisualizer


# 状态顺序：[x, vx, y, vy]，单位分别为 m、m/s、m、m/s。

# 1. 初始状态 X0
X0: list[float] = [
    2.5,  # 初始 x 位置 (m)
    0.0,  # 初始 x 速度 (m/s)
    -0.2,  # 初始 y 位置 (m)
    0.0,  # 初始 y 速度 (m/s)
]

# 2. 初始协方差 P0 
# 数值越大，表示越不相信对应的初始状态。
P0_DIAG: list[float] = [0.25, 1.0, 0.25, 1.0]

# 3. 过程噪声 Q
# 数值越大，通常对运动变化响应越快；越小，通常更平滑但换向滞后更明显。
SIGMA_VA: float = 1 # 加速度标准差 (m/s²)

# 4. 量测协方差 R 
# 数值越大，越不相信本帧量测；越小，越跟随量测。
R_DIAG: list[float] = [0.03**2, 0.01**2]


PIXEL_NOISE_STD: float = 1.0
DURATION: float = 10.0
DT: float = 1.0 / 60.0
SEED: int = 42
INITIAL_POSITION: tuple[float, float, float] = (3.0, -0.8, 0.1)
Y_LIMITS: tuple[float, float] = (-0.8, 0.8)
TRANSLATION_SPEED: float = 1.2
ROBOT_YAW: float = 0.0
ROBOT_RADIUS: float = 0.26


def translation_truth(timestamp: float) -> FloatArray:
    """三角波位置，端点立即反向；即使 dt 跨过端点也不越界。"""
    lower, upper = Y_LIMITS
    span = upper - lower
    phase = (INITIAL_POSITION[1] - lower + TRANSLATION_SPEED * timestamp) % (2 * span)
    if phase < span:
        y, vy = lower + phase, TRANSLATION_SPEED
    else:
        y, vy = upper - (phase - span), -TRANSLATION_SPEED
    return np.array([INITIAL_POSITION[0], 0.0, y, vy])


def run_demo() -> None:
    x0 = np.array(X0, dtype=float)
    p0 = np.diag(P0_DIAG)
    r = np.diag(R_DIAG)
    if DURATION <= 0 or DT <= 0 or ROBOT_RADIUS <= 0:
        raise ValueError("duration、dt 和 radius 必须为正")
    lower, upper = Y_LIMITS
    if lower >= upper or not lower <= INITIAL_POSITION[1] <= upper:
        raise ValueError("y_limits 必须递增，初始 y 必须在往返区间内")
    if TRANSLATION_SPEED <= 0:
        raise ValueError("translation_speed 必须为正")
    if PIXEL_NOISE_STD < 0 or SIGMA_VA < 0:
        raise ValueError("噪声标准差必须非负")
    if x0.shape != (4,) or p0.shape != (4, 4) or r.shape != (2, 2):
        raise ValueError("平移场景要求 x0=(4,)、P0=(4,4)、R=(2,2)")
    rng = np.random.default_rng(SEED)
    camera = Camera()
    solver = Solver(camera)
    ekf = ExtendedKalmanFilter(x0, p0)
    visualizer = RerunVisualizer(
        "lecture5_translation",
        ("x", "y"),
    )
    visualizer.log_camera(camera)

    dt = DT
    # f(x) = F x：匀速模型。h(x) = H x：位姿解算后只观测位置。
    transition = np.array(
        [
            [1.0, dt, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, dt],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    observation_matrix = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
    noise_mapping = np.array(
        [[0.5 * dt**2, 0.0], [dt, 0.0], [0.0, 0.5 * dt**2], [0.0, dt]]
    )
    q = noise_mapping @ np.diag(np.square((SIGMA_VA, SIGMA_VA))) @ noise_mapping.T

    def f(state: FloatArray) -> FloatArray:
        return transition @ state

    def jacobian_f(state: FloatArray) -> FloatArray:
        return transition

    def h(state: FloatArray) -> FloatArray:
        return observation_matrix @ state

    def jacobian_h(state: FloatArray) -> FloatArray:
        return observation_matrix

    timestamps = np.arange(0.0, DURATION, dt)
    for index, timestamp in enumerate(timestamps):
        truth = translation_truth(float(timestamp))
        robot = Robot(
            np.array([truth[0], truth[2], INITIAL_POSITION[2]]),
            ROBOT_YAW,
            ROBOT_RADIUS,
        )
        armor = robot.observe(camera)
        points = camera.project(armor) + rng.normal(0.0, PIXEL_NOISE_STD, (4, 2))
        _, measured_position = solver.robot_measurement(
            points, armor.index, robot.radius
        )
        z = measured_position[:2]
        if index > 0:
            ekf.predict(f, jacobian_f, q)
        prediction = ekf.x.copy()  # x⁻：看到本帧量测之前的预测。
        ekf.update(z, h, jacobian_h, r)
        visualizer.set_time(float(timestamp))
        visualizer.log_scalars(
            {
                "x/truth": float(truth[0]),
                "x/prediction": float(prediction[0]),
                "x/estimate": float(ekf.x[0]),
                "x/measurement": float(z[0]),
                "y/truth": float(truth[2]),
                "y/prediction": float(prediction[2]),
                "y/estimate": float(ekf.x[2]),
                "y/measurement": float(z[1]),
            }
        )
        visualizer.log_robot(robot)
        visualizer.log_robot(
            Robot(
                np.array([ekf.x[0], ekf.x[2], robot.position[2]]),
                robot.yaw,
                robot.radius,
            ),
            estimated=True,
        )
        visualizer.log_observation(camera, robot, points)
    visualizer.close()


def main() -> None:
    run_demo()


if __name__ == "__main__":
    main()
