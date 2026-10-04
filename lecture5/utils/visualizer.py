"""所有 Rerun 调用集中在这里，仿真模块无需依赖可视化。"""

from collections.abc import Mapping
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from sim.scene import ARMOR_POINTS, ROT_CAMERA_TO_CV, Camera, Robot, rot_z
from utils.ekf import FloatArray

ARMOR_MODEL_PATH = Path(__file__).resolve().parents[1] / "assets" / "armor.glb"


class RerunVisualizer:
    def __init__(
        self,
        application_id: str,
        metric_groups: tuple[str, str],
    ) -> None:
        if not ARMOR_MODEL_PATH.is_file():
            raise FileNotFoundError(f"装甲板模型不存在: {ARMOR_MODEL_PATH}")
        self.recording = rr.RecordingStream(application_id, default_enabled=True)
        self._static_asset_paths: set[str] = set()
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.Horizontal(
                    rrb.Spatial3DView(
                        origin="world", name="Armor model / EKF estimate (orange)"
                    ),
                    rrb.Spatial2DView(
                        origin="world/camera", name="Camera / noisy corners (red)"
                    ),
                ),
                rrb.Horizontal(
                    *(
                        rrb.TimeSeriesView(origin=f"metrics/{group}", name=group)
                        for group in metric_groups
                    )
                ),
            ),
            auto_layout=False,
            auto_views=False,
        )
        self.recording.spawn(default_blueprint=blueprint)
        self.recording.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        colors = {
            "truth": [0, 200, 0],
            "prediction": [0, 150, 255],
            "estimate": [255, 165, 0],
            "measurement": [220, 70, 70],
        }
        for group in metric_groups:
            for name, color in colors.items():
                self.recording.log(
                    f"metrics/{group}/{name}",
                    rr.SeriesLines(colors=color, names=name),
                    static=True,
                )

    def set_time(self, seconds: float) -> None:
        self.recording.set_time("sim_time", duration=seconds)

    def log_robot(self, robot: Robot, *, estimated: bool = False) -> None:
        path = "world/estimate" if estimated else "world/truth"
        color = [255, 165, 0] if estimated else [0, 200, 0]
        if estimated:
            strips: list[FloatArray] = []
            for armor in robot.armors:
                points = (armor.rotation @ ARMOR_POINTS.T).T + armor.position
                strips.append(np.vstack([points, points[0]]))
            self.recording.log(path, rr.LineStrips3D(strips, colors=color, radii=0.003))
        else:
            for armor in robot.armors:
                armor_path = f"{path}/armors/{armor.index}"
                self.recording.log(
                    armor_path,
                    rr.Transform3D(translation=armor.position, mat3x3=armor.rotation),
                )
                model_path = f"{armor_path}/model"
                if model_path not in self._static_asset_paths:
                    self.recording.log(
                        model_path, rr.Asset3D(path=ARMOR_MODEL_PATH), static=True
                    )
                    self._static_asset_paths.add(model_path)
        self.recording.log(
            f"{path}/heading",
            rr.Arrows3D(
                origins=[robot.position],
                vectors=[rot_z(robot.yaw) @ np.array([-robot.radius, 0.0, 0.0])],
                colors=color,
            ),
        )

    def log_camera(self, camera: Camera) -> None:
        self.recording.log(
            "world/camera",
            rr.Transform3D(translation=camera.position, mat3x3=ROT_CAMERA_TO_CV.T),
            static=True,
        )
        self.recording.log(
            "world/camera",
            rr.Pinhole(
                image_from_camera=camera.matrix,
                resolution=[camera.width, camera.height],
                camera_xyz=rr.ViewCoordinates.RDF,
            ),
            static=True,
        )

    def log_observation(
        self, camera: Camera, robot: Robot, image_points: FloatArray
    ) -> None:
        self.recording.log(
            "world/camera/image",
            rr.Image(camera.render(robot), color_model=rr.ColorModel.BGR).compress(
                jpeg_quality=85
            ),
        )
        self.recording.log(
            "world/camera/image/corners",
            rr.Points2D(image_points, colors=[255, 0, 0], radii=2.0),
        )

    def log_scalars(self, values: Mapping[str, float]) -> None:
        for name, value in values.items():
            self.recording.log(f"metrics/{name}", rr.Scalars(value))

    def close(self) -> None:
        self.recording.flush()
