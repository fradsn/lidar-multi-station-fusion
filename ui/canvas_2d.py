import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QVBoxLayout
from ui.base_canvas import BaseStitchCanvas
from config import LAYER_COLORS

class Canvas2D(BaseStitchCanvas):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plot_widget = pg.PlotWidget(title="LiDAR Stitcher 2D — Vista Planimetrica")
        self.plot_widget.setBackground('#0d1117')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setRange(xRange=[-10, 10], yRange=[-10, 10])
        self.plot_widget.setLabel('bottom', "X", units='m')
        self.plot_widget.setLabel('left', "Y", units='m')

        layout.addWidget(self.plot_widget)
        self.scatter_items = {}

        self.merged_scatter = pg.ScatterPlotItem(
            size=self.point_size,
            pen=pg.mkPen(None),
            brush=pg.mkBrush('#00ffff'),
            symbol='o'
        )
        self.merged_scatter.setZValue(10)
        self.plot_widget.addItem(self.merged_scatter)
        self.merged_scatter.hide()

    def set_point_size(self, size: float):
        self.point_size = float(size)
        for scatter in self.scatter_items.values():
            scatter.setSize(self.point_size)
        self.merged_scatter.setSize(self.point_size)

    def update_layer_view(self, layer_idx: int, points: np.ndarray):
        color_hex = LAYER_COLORS[layer_idx % len(LAYER_COLORS)]

        if layer_idx not in self.scatter_items:
            scatter = pg.ScatterPlotItem(
                size=self.point_size,
                pen=pg.mkPen(None),
                brush=pg.mkBrush(color_hex),
                symbol='o'
            )
            self.plot_widget.addItem(scatter)
            self.scatter_items[layer_idx] = scatter

        if len(points) > 0:
            self.scatter_items[layer_idx].setData(pos=points[:, :2], size=self.point_size)
        else:
            self.scatter_items[layer_idx].setData(pos=np.empty((0, 2)))

    def show_merged_preview(self, points: np.ndarray, visible: bool):
        if visible and len(points) > 0:
            for sc in self.scatter_items.values():
                sc.hide()
            self.merged_scatter.setData(pos=points[:, :2], size=self.point_size)
            self.merged_scatter.show()
        else:
            self.merged_scatter.hide()
            for sc in self.scatter_items.values():
                sc.show()

    def remove_layer(self, layer_idx: int):
        if layer_idx in self.scatter_items:
            self.plot_widget.removeItem(self.scatter_items[layer_idx])
            del self.scatter_items[layer_idx]

    def clear_all(self):
        for item in self.scatter_items.values():
            self.plot_widget.removeItem(item)
        self.scatter_items.clear()
        self.merged_scatter.setData(pos=np.empty((0, 2)))
        self.merged_scatter.hide()

    def reset_camera(self):
        self.plot_widget.setRange(xRange=[-10, 10], yRange=[-10, 10])