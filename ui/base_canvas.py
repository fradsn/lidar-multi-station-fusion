from PyQt6.QtWidgets import QWidget
import numpy as np

class BaseStitchCanvas(QWidget):
    """Interfaccia base per viewport di visualizzazione multi-stazione."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.point_size = 3.5

    def update_layer_view(self, layer_idx: int, points: np.ndarray):
        raise NotImplementedError

    def show_merged_preview(self, points: np.ndarray, visible: bool):
        raise NotImplementedError

    def set_point_size(self, size: float):
        """Aggiorna dinamicamente la dimensione di rendering dei punti."""
        raise NotImplementedError

    def remove_layer(self, layer_idx: int):
        raise NotImplementedError

    def clear_all(self):
        raise NotImplementedError

    def reset_camera(self):
        raise NotImplementedError