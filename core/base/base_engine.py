from abc import ABC, abstractmethod
import numpy as np

class BaseRegistrationEngine(ABC):
    @abstractmethod
    def transform_points(self, points: np.ndarray, transform_params: dict) -> np.ndarray:
        """Applica la rototraslazione rigida ai punti."""
        pass

    @abstractmethod
    def align_icp(self, source_pts: np.ndarray, target_pts: np.ndarray, **kwargs):
        """Esegue l'aggancio ICP tra due scansioni."""
        pass

    @abstractmethod
    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        """Applica il campionamento a griglia per uniformare la densità."""
        pass