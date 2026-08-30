import numpy as np
from scipy.spatial import cKDTree
from core.base.base_engine import BaseRegistrationEngine

class ICPEngine2D(BaseRegistrationEngine):
    def transform_points(self, points: np.ndarray, transform_params: dict) -> np.ndarray:
        if len(points) == 0:
            return points
        rad = np.deg2rad(transform_params.get('yaw', 0.0))
        c, s = np.cos(rad), np.sin(rad)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        tx = transform_params.get('tx', 0.0)
        ty = transform_params.get('ty', 0.0)
        return (points[:, :2] @ R.T) + np.array([tx, ty], dtype=np.float32)

    def align_icp(self, source_pts: np.ndarray, target_pts: np.ndarray,
                  max_iterations: int = 100, tolerance: float = 1e-6, max_distance_m: float = 0.50):
        if len(source_pts) == 0 or len(target_pts) == 0:
            return source_pts, {'tx': 0.0, 'ty': 0.0, 'yaw': 0.0}, 0.0

        current_src = np.copy(source_pts[:, :2])
        target_tree = cKDTree(target_pts[:, :2])

        accum_tx, accum_ty, accum_yaw = 0.0, 0.0, 0.0
        prev_error = float('inf')

        for it in range(max_iterations):
            cur_thresh = max(0.06, max_distance_m * (1.0 - (it / max_iterations) * 0.70))
            distances, indices = target_tree.query(current_src)
            valid_mask = distances < cur_thresh

            if np.count_nonzero(valid_mask) < 15:
                break

            matched_src = current_src[valid_mask]
            matched_tgt = target_pts[indices[valid_mask], :2]

            src_centroid = np.mean(matched_src, axis=0)
            tgt_centroid = np.mean(matched_tgt, axis=0)

            src_centered = matched_src - src_centroid
            tgt_centered = matched_tgt - tgt_centroid

            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R_step = Vt.T @ U.T

            if np.linalg.det(R_step) < 0:
                Vt[1, :] *= -1
                R_step = Vt.T @ U.T

            t_step = tgt_centroid - (R_step @ src_centroid)
            d_yaw_deg = float(np.rad2deg(np.arctan2(R_step[1, 0], R_step[0, 0])))

            current_src = (current_src @ R_step.T) + t_step
            accum_tx += float(t_step[0])
            accum_ty += float(t_step[1])
            accum_yaw += d_yaw_deg

            mean_error = np.mean(distances[valid_mask])
            if abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        delta_params = {'tx': accum_tx, 'ty': accum_ty, 'yaw': accum_yaw}
        return current_src, delta_params, prev_error

    def _smooth_slice_seams(self, points: np.ndarray, seam_x_coords: list = None, seam_radius: float = 0.08) -> np.ndarray:
        """Filtro di regolarizzazione strutturale su raggio di vicinato per eliminare rumore isolato."""
        if len(points) < 5:
            return points

        tree = cKDTree(points)
        counts = tree.query_ball_point(points, r=0.08, return_sorted=False)
        valid_mask = np.array([len(c) >= 2 for c in counts], dtype=bool)
        return points[valid_mask]

    def weighted_voxel_fusion(self, tagged_points: list, voxel_size: float = 0.02, 
                              angular_step_deg: float = 1.0) -> np.ndarray:
        """
        FUSIONE ADATTIVA A SPICCHI POLARI (POLAR SECTOR DENSITY AUTHORITY):
        - Suddivide l'orizzonte di ciascuna stazione in spicchi polari da angular_step_deg gradi (es. 360 spicchi).
        - In ogni spicchio polare, valuta la competizione locale tra stazioni: la stazione che ha
          colpito la parete con la risoluzione angolare/densità migliore (minore distanza) ottiene l'autorità.
        - Verso le direzioni libere (senza stazioni rivali), il raggio si espande senza limiti catturando tutto il muro.
        - Unisce i settori e compatta la nuvola finale tramite Voxel Grid uniforme.
        """
        if not tagged_points:
            return np.empty((0, 2), dtype=np.float32)

        valid_stations = [
            (pts[:, :2].astype(np.float32), np.array(center[:2], dtype=np.float32)) 
            for pts, center in tagged_points if len(pts) > 0
        ]
        num_stations = len(valid_stations)
        if num_stations == 0:
            return np.empty((0, 2), dtype=np.float32)

        if num_stations == 1:
            return self.voxel_grid_filter(valid_stations[0][0], voxel_size)

        num_sectors = int(360.0 / angular_step_deg)
        all_centers = np.array([c for _, c in valid_stations], dtype=np.float32)
        centers_tree = cKDTree(all_centers)

        selected_clouds = []

        # Analisi per ciascuna stazione
        for st_idx, (pts, center) in enumerate(valid_stations):
            # Calcolo coordinate polari locali (raggio d e angolo theta) rispetto al centro della stazione
            rel_vecs = pts - center
            dists = np.linalg.norm(rel_vecs, axis=1)
            angles_rad = np.arctan2(rel_vecs[:, 1], rel_vecs[:, 0])
            angles_deg = (np.rad2deg(angles_rad) + 360.0) % 360.0

            # Discretizzazione nei canali angolari (spicchi)
            sector_indices = np.floor(angles_deg / angular_step_deg).astype(np.int32) % num_sectors

            # Trova la distanza di ogni punto rispetto alla stazione più vicina in assoluto
            closest_dists, nearest_st_ids = centers_tree.query(pts)

            # Maschera di autorità polare adattiva per ciascun punto
            # Un punto viene trattenuto se:
            # 1. La stazione corrente è la più vicina o entro un margine di tolleranza di 5 cm (massima risoluzione)
            # 2. Nessun'altra stazione ha campionato lo stesso punto con distanza significativamente inferiore
            margin_m = 0.05
            is_dominant = (dists <= closest_dists + margin_m) | (nearest_st_ids == st_idx)

            pts_dominant = pts[is_dominant]
            sec_dominant = sector_indices[is_dominant]
            dists_dominant = dists[is_dominant]

            if len(pts_dominant) == 0:
                continue

            # Pulizia interna per spicchio: preserva i fronti d'onda della parete senza raddoppi interni
            sort_sec = np.argsort(sec_dominant)
            sorted_sec = sec_dominant[sort_sec]
            sorted_pts = pts_dominant[sort_sec]

            _, split_idx = np.unique(sorted_sec, return_index=True)
            sec_groups = np.split(sorted_pts, split_idx[1:])

            for group in sec_groups:
                if len(group) > 0:
                    selected_clouds.append(group)

        if not selected_clouds:
            return np.empty((0, 2), dtype=np.float32)

        stitched_perimeter = np.vstack(selected_clouds)

        # Compattazione finale con Voxel Grid Filter per uniformare il passo di campionamento
        fused = self.voxel_grid_filter(stitched_perimeter, voxel_size=voxel_size)

        # Regolarizzazione sui bordi di contatto
        return self._smooth_slice_seams(fused)

    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float = 0.02) -> np.ndarray:
        """Filtro Voxel Grid isotropo 2D basato su hash grid."""
        if len(points) == 0:
            return points
        grid_indices = np.floor(points[:, :2] / voxel_size).astype(np.int32)
        x_min, y_min = grid_indices.min(axis=0)
        x_span = grid_indices[:, 0] - x_min
        y_span = grid_indices[:, 1] - y_min
        max_y = int(y_span.max()) + 1
        flat_keys = x_span.astype(np.int64) * max_y + y_span.astype(np.int64)

        sort_order = np.argsort(flat_keys)
        sorted_keys = flat_keys[sort_order]
        sorted_pts = points[sort_order, :2]

        _, split_indices = np.unique(sorted_keys, return_index=True)
        pt_groups = np.split(sorted_pts, split_indices[1:])
        return np.array([np.mean(g, axis=0) for g in pt_groups if len(g) > 0], dtype=np.float32)