import os
import numpy as np
import pandas as pd
import pyqtgraph as pg

from PyQt6.QtWidgets import (
    QMainWindow, QSlider, QWidget, QHBoxLayout, QVBoxLayout, QPushButton,
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

        # 3. Rendering & Visualizzazione
        gb_view = QGroupBox("🔍 Rendering & Visualizzazione")
        v_view = QVBoxLayout(gb_view)
        
        self.lbl_pt_size = QLabel("Dimensione Punti: 4 px")
        self.slider_pt_size = QSlider(Qt.Orientation.Horizontal)
        self.slider_pt_size.setRange(1, 10)
        self.slider_pt_size.setValue(4)
        self.slider_pt_size.valueChanged.connect(self._on_point_size_changed)
        
        v_view.addWidget(self.lbl_pt_size)
        v_view.addWidget(self.slider_pt_size)
        side_panel.addWidget(gb_view)

        # 4. Allineamento Automatico e Manuale
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
        self.spin_tx.setRange(-50.0, 50.0)
        self.spin_tx.setSingleStep(0.05)
        self.spin_tx.setPrefix("X: ")
        self.spin_tx.valueChanged.connect(self._on_transform_changed)

        self.spin_ty = QDoubleSpinBox()
        self.spin_ty.setRange(-50.0, 50.0)
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

        # 5. Fusione Fisica e Generazione Master Map
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

        # 6. Esportazione
        gb_export = QGroupBox("💾 Esporta Mappa Fusa")
        v_export = QVBoxLayout(gb_export)
        btn_export_cad = QPushButton("📐 Esporta DXF/PLY (CAD/3D)")
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
        paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Carica Scansioni (2D o 3D)",
            "",
            "File Compatibili (*.csv *.xyz *.txt *.dat);;Tutti i file (*.*)"
        )
        if not paths:
            return

        for p in paths:
            try:
                pts = self._parse_point_file(p, is_3d=self.is_3d_mode)
                if len(pts) == 0:
                    QMessageBox.warning(self, "Attenzione", f"Nessun punto valido trovato in:\n{p}")
                    continue

                idx = len(self.layers)
                layer_name = f"Stazione {idx + 1}: {os.path.basename(p)}"
                layer = ScanStationLayer(idx, layer_name, p, pts, self.is_3d_mode)
                self.layers.append(layer)

                item = QListWidgetItem(f"● {layer.name}")
                item.setForeground(pg.mkColor(LAYER_COLORS[idx % len(LAYER_COLORS)]))
                self.list_layers.addItem(item)

                self._get_active_canvas().update_layer_view(idx, layer.get_transformed_points(self.current_engine))

            except Exception as e:
                QMessageBox.warning(self, "Errore", f"Impossibile leggere {p}:\n{e}")

        if self.layers and self.list_layers.currentRow() < 0:
            self.list_layers.setCurrentRow(0)

    def _parse_point_file(self, file_path: str, is_3d: bool) -> np.ndarray:
        """
        Parser Universale per nuvole 2D e 3D:
        - Supporta file con header testuali o raw numerici.
        - Supporta delimitatori a virgola, spazi, tab o punto e virgola.
        - Normalizza la scala in metri se i dati sono espressi in centimetri.
        """
        first_line = ""
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                l_str = line.strip()
                if l_str and not l_str.startswith('#'):
                    first_line = l_str
                    break

        tokens = first_line.replace(',', ' ').replace(';', ' ').split()
        is_numeric_header = False
        try:
            [float(t) for t in tokens]
            is_numeric_header = True
        except ValueError:
            is_numeric_header = False

        header_opt = None if is_numeric_header else 'infer'

        try:
            df = pd.read_csv(file_path, sep=None, engine='python', header=header_opt, comment='#')
        except Exception:
            df = pd.read_csv(file_path, sep=r'\s+', engine='python', header=header_opt, comment='#')

        req_dim = 3 if is_3d else 2

        # 1. File con Header Testuale
        if not is_numeric_header:
            col_map = {str(c).strip().upper(): c for c in df.columns}
            
            x_col = next((col_map[k] for k in ['X', 'X_M', 'X_CM', 'X(M)', 'X(CM)'] if k in col_map), None)
            y_col = next((col_map[k] for k in ['Y', 'Y_M', 'Y_CM', 'Y(M)', 'Y(CM)'] if k in col_map), None)
            z_col = next((col_map[k] for k in ['Z', 'Z_M', 'Z_CM', 'Z(M)', 'Z(CM)'] if k in col_map), None)

            if x_col is not None and y_col is not None:
                x_vals = df[x_col].to_numpy(dtype=np.float32)
                y_vals = df[y_col].to_numpy(dtype=np.float32)

                if 'CM' in str(x_col).upper():
                    x_vals /= 100.0
                    y_vals /= 100.0

                if is_3d:
                    if z_col is not None:
                        z_vals = df[z_col].to_numpy(dtype=np.float32)
                        if 'CM' in str(z_col).upper():
                            z_vals /= 100.0
                    else:
                        z_vals = np.zeros_like(x_vals)
                    pts = np.column_stack([x_vals, y_vals, z_vals])
                else:
                    pts = np.column_stack([x_vals, y_vals])
                return pts

            ang_col = next((col_map[k] for k in ['ANGLE', 'ANGLE_DEG', 'THETA'] if k in col_map), None)
            dist_col = next((col_map[k] for k in ['DISTANCE', 'DISTANCE_CM', 'DIST', 'R'] if k in col_map), None)
            if ang_col is not None and dist_col is not None:
                angles = df[ang_col].to_numpy(dtype=np.float32)
                distances = df[dist_col].to_numpy(dtype=np.float32)
                rad = np.deg2rad(angles)
                r_m = distances / 100.0 if np.max(distances) > 15.0 else distances
                x = -r_m * np.sin(rad)
                y = r_m * np.cos(rad)
                if is_3d:
                    return np.column_stack([x, y, np.zeros_like(x)])
                return np.column_stack([x, y])

        # 2. File numerico raw
        numeric_df = df.select_dtypes(include=[np.number])
        if numeric_df.shape[1] >= req_dim:
            pts = numeric_df.iloc[:, :req_dim].to_numpy(dtype=np.float32)
        elif numeric_df.shape[1] == 2 and is_3d:
            pts_2d = numeric_df.iloc[:, :2].to_numpy(dtype=np.float32)
            pts = np.hstack([pts_2d, np.zeros((len(pts_2d), 1), dtype=np.float32)])
        else:
            pts = np.empty((0, req_dim), dtype=np.float32)

        # 3. Normalizzazione cm -> m
        if len(pts) > 0 and np.max(np.abs(pts)) > 30.0:
            pts = pts / 100.0

        return pts

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

            # Abilita sempre i controlli: permette di riallineare agli assi anche la Stazione 1 (Base)
            self.spin_tx.setEnabled(True)
            self.spin_ty.setEnabled(True)
            self.spin_yaw.setEnabled(True)

    def _on_transform_changed(self):
        row = self.list_layers.currentRow()
        if 0 <= row < len(self.layers):
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

            accumulated_target = np.vstack([accumulated_target, layer.get_transformed_points(self.current_engine)])
            self._get_active_canvas().update_layer_view(i, layer.get_transformed_points(self.current_engine))

        self._on_layer_selected(self.list_layers.currentRow())
        self.lbl_icp_status.setText("✓ Tutte le stazioni allineate con successo!")
        QMessageBox.information(self, "Allineamento Completato", "Tutte le scansioni sono state allineate automaticamente sulla mappa base.")

    def _execute_fusion(self):
        """Fonde fisicamente tutti i punti coincidenti e aggiorna l'anteprima del canvas corrente."""
        pts = self._get_merged_points()
        if len(pts) == 0:
            QMessageBox.warning(self, "Avviso", "Nessun punto da fondere.")
            return

        self.btn_preview_merged.setChecked(True)
        active_canvas = self._get_active_canvas()
        active_canvas.show_merged_preview(pts, visible=True)

        self.lbl_icp_status.setText(f"🔥 Mappa Fusa Generata: {len(pts)} punti totali")
        QMessageBox.information(
            self,
            "Fusione Eseguita",
            f"Fusione geometrica completata!\nTotale punti unificati: {len(pts)}"
        )

    def _toggle_merged_preview(self):
        """Attiva o disattiva la visualizzazione della mappa fusa sul canvas attivo."""
        is_preview = self.btn_preview_merged.isChecked()
        active_canvas = self._get_active_canvas()

        if is_preview:
            pts = self._get_merged_points()
            active_canvas.show_merged_preview(pts, visible=True)
        else:
            empty_pts = np.empty((0, 3 if self.is_3d_mode else 2), dtype=np.float32)
            active_canvas.show_merged_preview(empty_pts, visible=False)

    def _get_merged_points(self) -> np.ndarray:
        """Calcola la nuvola unificata delegando all'engine corretto (2D o 3D)."""
        if not self.layers:
            return np.empty((0, 3 if self.is_3d_mode else 2), dtype=np.float32)

        tagged_stations = []
        for l in self.layers:
            pts = l.get_transformed_points(self.current_engine)
            if len(pts) > 0:
                # Il centro del sensore corrisponde alle coordinate di traslazione globale (tx, ty, tz)
                if self.is_3d_mode:
                    sensor_pos = np.array([l.tx, l.ty, getattr(l, 'tz', 0.0)], dtype=np.float32)
                else:
                    sensor_pos = np.array([l.tx, l.ty], dtype=np.float32)
                tagged_stations.append((pts, sensor_pos))

        if not tagged_stations:
            return np.empty((0, 3 if self.is_3d_mode else 2), dtype=np.float32)

        voxel_sz = DEFAULT_VOXEL_SIZE_3D if self.is_3d_mode else DEFAULT_VOXEL_SIZE_2D
        return self.current_engine.weighted_voxel_fusion(tagged_stations, voxel_size=voxel_sz)

    def _export_cad(self):
        pts = self._get_merged_points()
        if len(pts) == 0:
            QMessageBox.warning(self, "Avviso", "Nessun punto da esportare.")
            return

        ext = "*.ply" if self.is_3d_mode else "*.dxf"
        filter_str = "File PLY 3D (*.ply)" if self.is_3d_mode else "File DXF (*.dxf)"
        def_name = "modello_fuso_3d.ply" if self.is_3d_mode else "pianta_fusa.dxf"

        path, _ = QFileDialog.getSaveFileName(self, "Esporta CAD / Mesh 3D", def_name, filter_str)
        if path:
            if self.current_exporter.export_cad(path, pts):
                QMessageBox.information(self, "Completato", f"File salvato con successo ({len(pts)} punti fusi)!")

    def _export_table(self):
        pts = self._get_merged_points()
        if len(pts) == 0:
            QMessageBox.warning(self, "Avviso", "Nessun punto da esportare.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Esporta CSV", "punti_fusi.csv", "File CSV (*.csv)")
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

    def _on_point_size_changed(self, val: int):
        self.lbl_pt_size.setText(f"Dimensione Punti: {val} px")
        self.canvas_2d.set_point_size(val)
        self.canvas_3d.set_point_size(val)