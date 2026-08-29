import numpy as np
from PyQt6.QtWidgets import QVBoxLayout, QLabel
from PyQt6.QtCore import Qt
from ui.base_canvas import BaseStitchCanvas

class Canvas3D(BaseStitchCanvas):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Placeholder visivo: verrà sostituito con pyqtgraph.opengl.GLViewWidget
        self.placeholder_label = QLabel("🧊 Viewport 3D OpenGL (Predisposto per Modulo 3D)")
        self.placeholder_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder_label.setStyleSheet("""
            background-color: #0d1117; 
            color: #8b949e; 
            font-size: 14px; 
            font-weight: bold;
            border: 1px dashed #30363d;
        """)
        layout.addWidget(self.placeholder_label)
        self.scatter_items = {}

    def update_layer_view(self, layer_idx: int, points: np.ndarray):
        # TODO: Implementare aggiornamento gl.GLScatterPlotItem per la nuvola 3D
        pass

    def remove_layer(self, layer_idx: int):
        pass

    def clear_all(self):
        self.scatter_items.clear()

    def reset_camera(self):
        pass