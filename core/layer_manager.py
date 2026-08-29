import numpy as np
from core.base.base_engine import BaseRegistrationEngine

class ScanStationLayer:
    def __init__(self, station_id: int, name: str, filepath: str, raw_points: np.ndarray, is_3d: bool = False):
        self.station_id = station_id
        self.name = name
        self.filepath = filepath
        self.raw_points = raw_points
        self.is_3d = is_3d

        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        self.rot_yaw_deg = 0.0
        self.rot_pitch_deg = 0.0
        self.rot_roll_deg = 0.0

    @property
    def sensor_center_2d(self) -> np.ndarray:
        """Restituisce la coordinata (X, Y) reale in cui si trovava il LiDAR nella stanza."""
        return np.array([self.tx, self.ty], dtype=np.float32)

    def get_transform_dict(self) -> dict:
        return {
            'tx': self.tx, 'ty': self.ty, 'tz': self.tz,
            'yaw': self.rot_yaw_deg,
            'pitch': self.rot_pitch_deg,
            'roll': self.rot_roll_deg
        }

    def get_transformed_points(self, engine: BaseRegistrationEngine) -> np.ndarray:
        return engine.transform_points(self.raw_points, self.get_transform_dict())