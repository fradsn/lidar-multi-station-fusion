import os
import numpy as np
import pandas as pd
import pyqtgraph as pg

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QPushButton,
    QFileDialog, QListWidget, QLabel, QDoubleSpinBox, QGroupBox,
    QMessageBox, QListWidgetItem, QComboBox, QStackedWidget
)
from PyQt6.QtCore import Qt

from config import LAYER_COLORS, DEFAULT_VOXEL_SIZE_2D, DEFAULT_VOXEL_SIZE_3D
from core.layer_manager import ScanStationLayer
from core.modules_2d.icp_engine_2d import ICPEngine2D
from core.modules_2d.exporter_2d import Exporter2D
from core.modules_3d.icp_engine_3d import ICPEngine3D
from core.modules_3d.exporter_3d import Exporter3D
from ui.base_canvas import BaseStitchCanvas
from ui.canvas_2d import Canvas2D
from ui.canvas_3d import Canvas3D

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR Universal Stitcher — Multi-Station Alignment Suite")
        self.resize(1300, 800)

        self.is_3d_mode = False
        self.engine_2d = ICPEngine2D()
        self.exporter_2d = Exporter2D()
        self.engine_3d = ICPEngine3D()
        self.exporter_3d = Exporter3D()

        self.current_engine = self.engine_2d
        self.current_exporter = self.exporter_2d

        self.layers = []
        self._setup_ui()

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        side_panel = QVBoxLayout()
        side_panel.setSpacing(8)

        # 1. Selettore Suite
        gb_mode = QGroupBox("⚙️ Modalità")
        v_mode = QVBoxLayout(gb_mode)
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["🗺️ Planimetria 2D (Piante CAD)", "🧊 Scansione 3D Volumetrica (Nuvole)"])
        self.combo_mode.currentIndexChanged.connect(self._on_mode_changed)
        v_mode.addWidget(self.combo_mode)
        side_panel.addWidget(gb_mode)

        # 2. File Scansioni
        gb_files = QGroupBox("📁 Scansioni Caricate (Layer)")
        v_files = QVBoxLayout(gb_files)
        self.list_layers = QListWidget()
        self.list_layers.currentRowChanged.connect(self._on_layer_selected)
        
        btn_add = QPushButton("➕ Aggiungi Scansione CSV")
        btn_add.clicked.connect(self._load_csv_dialog)
        btn_clear = QPushButton("🗑️ Rimuovi Tutto")
        btn_clear.clicked.connect(self._clear_all)

        v_files.addWidget(self.list_layers)
        v_files.addWidget(btn_add)
        v_files.addWidget(btn_clear)
        side_panel.addWidget(gb_files)

        # 3. Allineamento Automatico e Manuale
        gb_align = QGroupBox("⚡ Allineamento Scansioni")
        v_align = QVBoxLayout(gb_align)

        btn_auto_all = QPushButton("🚀 AUTO-ALLINEA TUTTO IN 1-CLICK")
        btn_auto_all.setStyleSheet("background-color: #238636; color: white; font-weight: bold; padding: 8px;")
        btn_auto_all.clicked.connect(self._run_global_auto_alignment)
        v_align.addWidget(btn_auto_all)

        btn_run_single_icp = QPushButton("🎯 Allinea Solo Selezionato (ICP)")
        btn_run_single_icp.setStyleSheet("background-color: #1f6feb; color: white; padding: 5px;")
        btn_run_single_icp.clicked.connect(self._run_icp_alignment)
        v_align.addWidget(btn_run_single_icp)

        self.lbl_icp_status = QLabel("Stato: Pronto")
        self.lbl_icp_status.setStyleSheet("color: #8b949e; font-size: 11px;")
        v_align.addWidget(self.lbl_icp_status)

        # Gizmo manuale
        v_align.addWidget(QLabel("Offset Manuale X / Y / Yaw:"))
        h_gizmo = QHBoxLayout()
        self.spin_tx = QDoubleSpinBox()
        self.spin_tx.setRange(-30.0, 30.0)
        self.spin_tx.setSingleStep(0.05)
        self.spin_tx.setPrefix("X: ")
        self.spin_tx.valueChanged.connect(self._on_transform_changed)

        self.spin_ty = QDoubleSpinBox()
        self.spin_ty.setRange(-30.0, 30.0)
        self.spin_ty.setSingleStep(0.05)
        self.spin_ty.setPrefix("Y: ")
        self.spin_ty.valueChanged.connect(self._on_transform_changed)

        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-360.0, 360.0)
        self.spin_yaw.setSingleStep(1.0)
        self.spin_yaw.setPrefix("θ: ")
        self.spin_yaw.valueChanged.connect(self._on_transform_changed)

        h_gizmo.addWidget(self.spin_tx)
        h_gizmo.addWidget(self.spin_ty)
        h_gizmo.addWidget(self.spin_yaw)
        v_align.addLayout(h_gizmo)

        side_panel.addWidget(gb_align)

        # 4. Fusione Fisica e Generazione Master Map
        gb_fusion = QGroupBox("🔥 Generazione Mappa Fusa")
        v_fusion = QVBoxLayout(gb_fusion)

        btn_fuse = QPushButton("✨ GENERA MAPPA FUSA DEI PUNTI")
        btn_fuse.setStyleSheet("background-color: #8957e5; color: white; font-weight: bold; padding: 8px;")
        btn_fuse.clicked.connect(self._execute_fusion)
        v_fusion.addWidget(btn_fuse)

        self.btn_preview_merged = QPushButton("👁️ Mostra/Nascondi Anteprima Fusa")
        self.btn_preview_merged.setCheckable(True)
        self.btn_preview_merged.clicked.connect(self._toggle_merged_preview)
        v_fusion.addWidget(self.btn_preview_merged)

        side_panel.addWidget(gb_fusion)

        # 5. Esportazione
        gb_export = QGroupBox("💾 Esporta Mappa Fusa")
        v_export = QVBoxLayout(gb_export)
        btn_export_cad = QPushButton("📐 Esporta DXF (AutoCAD)")
        btn_export_cad.clicked.connect(self._export_cad)
        btn_export_table = QPushButton("📊 Esporta CSV Finale")
        btn_export_table.clicked.connect(self._export_table)

        v_export.addWidget(btn_export_cad)
        v_export.addWidget(btn_export_table)
        side_panel.addWidget(gb_export)

        side_panel.addStretch()

        # Canvas Stack
        self.canvas_stack = QStackedWidget()
        self.canvas_2d = Canvas2D()
        self.canvas_3d = Canvas3D()
        
        self.canvas_stack.addWidget(self.canvas_2d)
        self.canvas_stack.addWidget(self.canvas_3d)

        main_layout.addLayout(side_panel, 1)
        main_layout.addWidget(self.canvas_stack, 3)

    def _on_mode_changed(self, index: int):
        self.is_3d_mode = (index == 1)
        self.canvas_stack.setCurrentIndex(index)
        self.current_engine = self.engine_3d if self.is_3d_mode else self.engine_2d
        self.current_exporter = self.exporter_3d if self.is_3d_mode else self.exporter_2d
        self._refresh_all_canvas()

    def _load_csv_dialog(self):
        paths, _ = QFileDialog.getOpenFileNames(self, "Carica Scansioni CSV", "", "File CSV (*.csv)")
        for p in paths:
            try:
                df = pd.read_csv(p)
                if 'X' in df.columns and 'Y' in df.columns:
                    cols = ['X', 'Y', 'Z'] if ('Z' in df.columns and self.is_3d_mode) else ['X', 'Y']
                    pts = df[cols].to_numpy(dtype=np.float32)
                elif 'x' in df.columns and 'y' in df.columns:
                    cols = ['x', 'y', 'z'] if ('z' in df.columns and self.is_3d_mode) else ['x', 'y']
                    pts = df[cols].to_numpy(dtype=np.float32)
                else:
                    pts = df.iloc[:, : (3 if self.is_3d_mode else 2)].to_numpy(dtype=np.float32)

                idx = len(self.layers)
                layer = ScanStationLayer(idx, f"Stazione {idx + 1}: {os.path.basename(p)}", p, pts, self.is_3d_mode)
                self.layers.append(layer)

                item = QListWidgetItem(f"● {layer.name}")
                item.setForeground(pg.mkColor(LAYER_COLORS[idx % len(LAYER_COLORS)]))
                self.list_layers.addItem(item)

                self._get_active_canvas().update_layer_view(idx, layer.get_transformed_points(self.current_engine))
            except Exception as e:
                QMessageBox.warning(self, "Errore", f"Impossibile leggere {p}:\n{e}")

        if self.layers and self.list_layers.currentRow() < 0:
            self.list_layers.setCurrentRow(0)

    def _on_layer_selected(self, row: int):
        if 0 <= row < len(self.layers):
            layer = self.layers[row]
            self.spin_tx.blockSignals(True)
            self.spin_ty.blockSignals(True)
            self.spin_yaw.blockSignals(True)

            self.spin_tx.setValue(layer.tx)
            self.spin_ty.setValue(layer.ty)
            self.spin_yaw.setValue(layer.rot_yaw_deg)

            self.spin_tx.blockSignals(False)
            self.spin_ty.blockSignals(False)
            self.spin_yaw.blockSignals(False)

            is_base = (row == 0)
            self.spin_tx.setEnabled(not is_base)
            self.spin_ty.setEnabled(not is_base)
            self.spin_yaw.setEnabled(not is_base)

    def _on_transform_changed(self):
        row = self.list_layers.currentRow()
        if 0 <= row < len(self.layers) and row != 0:
            layer = self.layers[row]
            layer.tx = self.spin_tx.value()
            layer.ty = self.spin_ty.value()
            layer.rot_yaw_deg = self.spin_yaw.value()
            self._get_active_canvas().update_layer_view(row, layer.get_transformed_points(self.current_engine))

    def _run_icp_alignment(self):
        """Allinea il layer selezionato alla mappa di riferimento accumulata."""
        row = self.list_layers.currentRow()
        if row <= 0 or len(self.layers) < 2:
            QMessageBox.information(self, "Avviso", "Seleziona la Stazione 2 o successiva per allinearla.")
            return

        # Target = unione di tutte le stazioni precedenti
        target_pts = np.vstack([self.layers[i].get_transformed_points(self.current_engine) for i in range(row)])
        active_layer = self.layers[row]
        current_pts = active_layer.get_transformed_points(self.current_engine)

        self.lbl_icp_status.setText("Allineamento ICP in corso...")
        self.repaint()

        _, delta_params, res_error = self.current_engine.align_icp(
            current_pts, target_pts, max_iterations=60, max_distance_m=0.65
        )

        active_layer.tx += delta_params.get('tx', 0.0)
        active_layer.ty += delta_params.get('ty', 0.0)
        active_layer.rot_yaw_deg = (active_layer.rot_yaw_deg + delta_params.get('yaw', 0.0)) % 360.0

        self.spin_tx.setValue(active_layer.tx)
        self.spin_ty.setValue(active_layer.ty)
        self.spin_yaw.setValue(active_layer.rot_yaw_deg)

        self._get_active_canvas().update_layer_view(row, active_layer.get_transformed_points(self.current_engine))
        self.lbl_icp_status.setText(f"✓ Agganciato | Residuo: {res_error * 100:.1f} cm")

    def _run_global_auto_alignment(self):
        """Allinea automaticamente TUTTE le stazioni in cascata progressiva (1-Click)."""
        if len(self.layers) < 2:
            QMessageBox.information(self, "Avviso", "Carica almeno 2 scansioni CSV per allinearle.")
            return

        self.lbl_icp_status.setText("⚡ Auto-Allineamento Globale in corso...")
        self.repaint()

        accumulated_target = self.layers[0].get_transformed_points(self.current_engine)

        for i in range(1, len(self.layers)):
            layer = self.layers[i]
            src_pts = layer.get_transformed_points(self.current_engine)

            _, delta_params, _ = self.current_engine.align_icp(
                src_pts, accumulated_target, max_iterations=60, max_distance_m=0.65
            )

            layer.tx += delta_params.get('tx', 0.0)
            layer.ty += delta_params.get('ty', 0.0)
            layer.rot_yaw_deg = (layer.rot_yaw_deg + delta_params.get('yaw', 0.0)) % 360.0

            # Aggiorna il riferimento progressivo includendo la stazione appena allineata
            accumulated_target = np.vstack([accumulated_target, layer.get_transformed_points(self.current_engine)])
            self._get_active_canvas().update_layer_view(i, layer.get_transformed_points(self.current_engine))

        self._on_layer_selected(self.list_layers.currentRow())
        self.lbl_icp_status.setText("✓ Tutte le stazioni allineate con successo!")
        QMessageBox.information(self, "Allineamento Completato", "Tutte le scansioni sono state allineate automaticamente sulla mappa base.")

    def _execute_fusion(self):
        """Fonde fisicamente tutti i punti coincidenti nei loro centroidi medi."""
        pts = self._get_merged_points()
        if len(pts) == 0:
            QMessageBox.warning(self, "Avviso", "Nessun punto da fondere.")
            return

        self.btn_preview_merged.setChecked(True)
        if isinstance(self._get_active_canvas(), Canvas2D):
            self.canvas_2d.show_merged_preview(pts, visible=True)
            self.lbl_icp_status.setText(f"🔥 Mappa Fusa Generata: {len(pts)} punti totali")
            QMessageBox.information(self, "Fusione Eseguita", 
                f"I muri sovrapposti sono stati fusi nei rispettivi centroidi geometrici!\nTotale punti unificati: {len(pts)}")

    def _toggle_merged_preview(self):
        is_preview = self.btn_preview_merged.isChecked()
        if is_preview:
            pts = self._get_merged_points()
            if isinstance(self._get_active_canvas(), Canvas2D):
                self.canvas_2d.show_merged_preview(pts, visible=True)
        else:
            if isinstance(self._get_active_canvas(), Canvas2D):
                self.canvas_2d.show_merged_preview(np.empty((0, 2)), visible=False)

    def _get_merged_points(self) -> np.ndarray:
        if not self.layers:
            return np.empty((0, 3 if self.is_3d_mode else 2))

        # Associa i punti trasformati alla posizione reale (X, Y) del sensore nel sistema globale
        tagged_stations = []
        for l in self.layers:
            pts = l.get_transformed_points(self.current_engine)
            if len(pts) > 0:
                sensor_pos = np.array([l.tx, l.ty], dtype=np.float32)
                tagged_stations.append((pts, sensor_pos))

        if not tagged_stations:
            return np.empty((0, 2))

        voxel_sz = DEFAULT_VOXEL_SIZE_3D if self.is_3d_mode else DEFAULT_VOXEL_SIZE_2D
        return self.engine_2d.weighted_voxel_fusion(tagged_stations, voxel_size=voxel_sz)

    def _export_cad(self):
        pts = self._get_merged_points()
        if len(pts) == 0:
            QMessageBox.warning(self, "Avviso", "Nessun punto da esportare.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Esporta DXF CAD", "pianta_fusa.dxf", "File DXF (*.dxf)")
        if path:
            if self.current_exporter.export_cad(path, pts):
                QMessageBox.information(self, "Completato", f"File CAD DXF esportato con {len(pts)} punti fusi!")

    def _export_table(self):
        pts = self._get_merged_points()
        if len(pts) == 0:
            QMessageBox.warning(self, "Avviso", "Nessun punto da esportare.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Esporta CSV", "pianta_fusa.csv", "File CSV (*.csv)")
        if path:
            if self.current_exporter.export_table(path, pts):
                QMessageBox.information(self, "Completato", f"File CSV esportato con {len(pts)} punti fusi!")

    def _get_active_canvas(self) -> BaseStitchCanvas:
        return self.canvas_3d if self.is_3d_mode else self.canvas_2d

    def _refresh_all_canvas(self):
        active_cv = self._get_active_canvas()
        active_cv.clear_all()
        for idx, layer in enumerate(self.layers):
            active_cv.update_layer_view(idx, layer.get_transformed_points(self.current_engine))

    def _clear_all(self):
        self.layers.clear()
        self.list_layers.clear()
        self.canvas_2d.clear_all()
        self.canvas_3d.clear_all()
        self.btn_preview_merged.setChecked(False)
        self.lbl_icp_status.setText("Stato: Pronto")