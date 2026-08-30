import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6.QtWidgets import QVBoxLayout
from ui.base_canvas import BaseStitchCanvas
from config import LAYER_COLORS

class Canvas3D(BaseStitchCanvas):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.gl_view = gl.GLViewWidget()
        self.gl_view.setBackgroundColor('#0d1117')
        self.gl_view.setCameraPosition(distance=14, elevation=35, azimuth=45)

        grid = gl.GLGridItem()
        grid.setSize(20, 20)
        grid.setSpacing(1, 1)
        grid.setColor((100, 100, 100, 80))
        self.gl_view.addItem(grid)

        axes = gl.GLAxisItem()
        axes.setSize(1.5, 1.5, 1.5)
        self.gl_view.addItem(axes)

        layout.addWidget(self.gl_view)
        self.scatter_items = {}

        self.merged_scatter = gl.GLScatterPlotItem(
            pos=np.empty((0, 3)),
            color=(0.0, 1.0, 1.0, 0.9),
            size=self.point_size,
            pxMode=True
        )
        self.gl_view.addItem(self.merged_scatter)
        self.merged_scatter.setVisible(False)

    def _hex_to_rgba(self, hex_code: str, alpha: float = 0.85):
        hex_code = hex_code.lstrip('#')
        r = int(hex_code[0:2], 16) / 255.0
        g = int(hex_code[2:4], 16) / 255.0
        b = int(hex_code[4:6], 16) / 255.0
        return (r, g, b, alpha)

    def set_point_size(self, size: float):
        self.point_size = float(size)
        for scatter in self.scatter_items.values():
            scatter.setData(size=self.point_size)
        self.merged_scatter.setData(size=self.point_size)

    def update_layer_view(self, layer_idx: int, points: np.ndarray):
        color_rgba = self._hex_to_rgba(LAYER_COLORS[layer_idx % len(LAYER_COLORS)])

        if layer_idx not in self.scatter_items:
            scatter = gl.GLScatterPlotItem(
                pos=np.empty((0, 3)),
                color=color_rgba,
                size=self.point_size,
                pxMode=True
            )
            self.gl_view.addItem(scatter)
            self.scatter_items[layer_idx] = scatter

        if len(points) > 0:
            self.scatter_items[layer_idx].setData(pos=points[:, :3], size=self.point_size)
        else:
            self.scatter_items[layer_idx].setData(pos=np.empty((0, 3)))

    def show_merged_preview(self, points: np.ndarray, visible: bool):
        if visible and len(points) > 0:
            for sc in self.scatter_items.values():
                sc.setVisible(False)
            self.merged_scatter.setData(pos=points[:, :3], size=self.point_size)
            self.merged_scatter.setVisible(True)
        else:
            self.merged_scatter.setVisible(False)
            for sc in self.scatter_items.values():
                sc.setVisible(True)

    def remove_layer(self, layer_idx: int):
        if layer_idx in self.scatter_items:
            self.gl_view.removeItem(self.scatter_items[layer_idx])
            del self.scatter_items[layer_idx]

    def clear_all(self):
        for item in self.scatter_items.values():
            self.gl_view.removeItem(item)
        self.scatter_items.clear()
        self.merged_scatter.setData(pos=np.empty((0, 3)))
        self.merged_scatter.setVisible(False)

    def reset_camera(self):
        self.gl_view.setCameraPosition(distance=14, elevation=35, azimuth=45)