"""入口二：纯匀速旋转，比较 angle/角速度真值、预测和估计。"""

import numpy as np

from aim.solver import Solver
from sim.scene import Camera, Robot, limit_rad, wrap_angle
from utils.ekf import ExtendedKalmanFilter, FloatArray
from utils.visualizer import RerunVisualizer


# 状态顺序：[angle, angular_velocity]，单位分别为 rad、rad/s。

# 1. 初始状态 X0
X0: list[float] = [
    0.4,  # 初始角度 (rad)
    0.0,  # 初始角速度 (rad/s)
]

# 2. 初始协方差 P0
# 数值越大，表示越不相信对应的初始状态。
P0_DIAG: list[float] = [0.25, 4.0]

# 3. 过程噪声 Q
# 数值越大，通常对运动变化响应越快；越小，通常更平滑但换向滞后更明显。
SIGMA_W: float = 4 # 角加速度标准差 (rad/s²)

# 4. 量测协方差 R 
# 数值越大，越不相信本帧量测；越小，越跟随量测。
R_DIAG: list[float] = [0.08**2]


PIXEL_NOISE_STD: float = 1.0
DURATION: float = 20.0
DT: float = 1.0 / 60.0
SEED: int = 42
POSITION: tuple[float, float, float] = (3.0, 0.0, 0.1)
INITIAL_ANGLE: float = 0.0
ANGULAR_VELOCITY: float = 6.0
ROBOT_RADIUS: float = 0.26


def angle_residual(observed: FloatArray, predicted: FloatArray) -> FloatArray:
    return np.array([wrap_angle(float(observed[0] - predicted[0]))])


def run_demo() -> None:
    x0 = np.array(X0, dtype=float)
    p0 = np.diag(P0_DIAG)
    r = np.diag(R_DIAG)
    if DURATION <= 0 or DT <= 0 or ROBOT_RADIUS <= 0:
        raise ValueError("duration、dt 和 radius 必须为正")
    if PIXEL_NOISE_STD < 0 or SIGMA_W < 0:
        raise ValueError("噪声标准差必须非负")
    if x0.shape != (2,) or p0.shape != (2, 2) or r.shape != (1, 1):
        raise ValueError("旋转场景要求 x0=(2,)、P0=(2,2)、R=(1,1)")
    rng = np.random.default_rng(SEED)
    camera = Camera()
    solver = Solver(camera)
    ekf = ExtendedKalmanFilter(x0, p0)
    visualizer = RerunVisualizer(
        "lecture5_rotation",
        ("angle", "angular_velocity"),
    )
    visualizer.log_camera(camera)

    dt = DT
    transition = np.array([[1.0, dt], [0.0, 1.0]])
    observation_matrix = np.array([[1.0, 0.0]])
    noise_mapping = np.array([[0.5 * dt**2], [dt]])
    q = noise_mapping @ noise_mapping.T * SIGMA_W**2

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
        angle = INITIAL_ANGLE + ANGULAR_VELOCITY * timestamp
        robot = Robot(np.array(POSITION), float(angle), ROBOT_RADIUS)
        armor = robot.observe(camera)
        points = camera.project(armor) + rng.normal(0.0, PIXEL_NOISE_STD, (4, 2))
        measured_angle, _ = solver.robot_measurement(points, armor.index, robot.radius)
        z = np.array([measured_angle])
        if index > 0:
            ekf.predict(f, jacobian_f, q)
        prediction = ekf.x.copy()
        ekf.update(z, h, jacobian_h, r, residual=angle_residual)
        # 滤波内部保留连续角度；展示时限制到 [0, 2π)，每圈回到零。
        visualizer.set_time(float(timestamp))
        visualizer.log_scalars(
            {
                "angle/truth": limit_rad(float(angle)),
                "angle/prediction": limit_rad(float(prediction[0])),
                "angle/estimate": limit_rad(float(ekf.x[0])),
                "angle/measurement": limit_rad(measured_angle),
                "angular_velocity/truth": ANGULAR_VELOCITY,
                "angular_velocity/prediction": float(prediction[1]),
                "angular_velocity/estimate": float(ekf.x[1]),
            }
        )
        visualizer.log_robot(robot)
        visualizer.log_robot(
            Robot(robot.position, float(ekf.x[0]), robot.radius), estimated=True
        )
        visualizer.log_observation(camera, robot, points)
    visualizer.close()


def main() -> None:
    run_demo()


if __name__ == "__main__":
    main()
