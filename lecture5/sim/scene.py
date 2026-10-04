"""仅保留教学所需的机器人几何、相机投影和图像生成。"""

from dataclasses import dataclass, field

import cv2 as cv
import numpy as np
from numpy.typing import NDArray

from utils.ekf import FloatArray

NDImage = NDArray[np.uint8]

ARMOR_WIDTH = 0.135
LIGHT_HEIGHT = 0.056
ARMOR_PITCH = np.deg2rad(15.0)
ARMOR_POINTS = np.array(
    [
        [0.0, ARMOR_WIDTH / 2, LIGHT_HEIGHT / 2],
        [0.0, ARMOR_WIDTH / 2, -LIGHT_HEIGHT / 2],
        [0.0, -ARMOR_WIDTH / 2, -LIGHT_HEIGHT / 2],
        [0.0, -ARMOR_WIDTH / 2, LIGHT_HEIGHT / 2],
    ]
)
ROT_CAMERA_TO_CV = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def limit_rad(angle: float) -> float:
    """将角度限制到 [0, 2π)，用于周期角度展示。"""
    return float(angle % (2.0 * np.pi))


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def rot_z(angle: float) -> FloatArray:
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def armor_rotation(yaw: float) -> FloatArray:
    cosine, sine = np.cos(ARMOR_PITCH), np.sin(ARMOR_PITCH)
    pitch_rotation = np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
    )
    return rot_z(yaw) @ pitch_rotation


@dataclass
class Armor:
    position: FloatArray
    yaw: float
    index: int

    @property
    def rotation(self) -> FloatArray:
        return armor_rotation(self.yaw)


@dataclass
class Robot:
    position: FloatArray
    yaw: float = 0.0
    radius: float = 0.26

    @property
    def armors(self) -> list[Armor]:
        result: list[Armor] = []
        for index in range(4):
            yaw = self.yaw + index * np.pi / 2
            offset = -self.radius * np.array([np.cos(yaw), np.sin(yaw), 0.0])
            result.append(Armor(self.position + offset, yaw, index))
        return result

    def observe(self, camera: "Camera") -> Armor:
        def score(armor: Armor) -> float:
            direction = camera.position - armor.position
            direction /= np.linalg.norm(direction)
            return float(-armor.rotation[:, 0] @ direction)

        return max(self.armors, key=score)


@dataclass
class Camera:
    width: int = 1440
    height: int = 1080
    horizontal_fov: float = 45.0
    position: FloatArray = field(default_factory=lambda: np.array([0.0, 0.0, 0.4]))
    distortion: FloatArray = field(default_factory=lambda: np.zeros(5))

    @property
    def matrix(self) -> FloatArray:
        focal_length = self.width / (2 * np.tan(np.deg2rad(self.horizontal_fov / 2)))
        return np.array(
            [
                [focal_length, 0.0, self.width / 2],
                [0.0, focal_length, self.height / 2],
                [0.0, 0.0, 1.0],
            ]
        )

    def project(self, armor: Armor) -> FloatArray:
        rotation = ROT_CAMERA_TO_CV @ armor.rotation
        position = ROT_CAMERA_TO_CV @ (armor.position - self.position)
        projected = cv.projectPoints(
            ARMOR_POINTS, rotation, position, self.matrix, self.distortion
        )[0].reshape(4, 2)
        return np.asarray(projected, dtype=np.float64)

    def render(self, robot: Robot) -> NDImage:
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for armor in robot.armors:
            points = np.rint(self.project(armor)).astype(np.int32)
            color = (0, 200, 0) if armor.index == 0 else (160, 160, 160)
            cv.polylines(image, [points], True, color, thickness=2)
        return image
