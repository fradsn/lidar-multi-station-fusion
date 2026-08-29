from PyQt6.QtWidgets import QWidget
import numpy as np

class BaseStitchCanvas(QWidget):
    """Interfaccia base per viewport di visualizzazione multi-stazione."""

    def __init__(self, parent=None):
        super().__init__(parent)

    def update_layer_view(self, layer_idx: int, points: np.ndarray):
        """Aggiorna o crea il rendering per una specifica stazione."""
        raise NotImplementedError

    def remove_layer(self, layer_idx: int):
        """Rimuove un layer dal canvas."""
        raise NotImplementedError

    def clear_all(self):
        """Pulisce tutti i punti e reimposta la vista."""
        raise NotImplementedError

    def reset_camera(self):
        """Reimposta l'inquadratura iniziale."""
        raise NotImplementedError