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

    def _smooth_slice_seams(self, points: np.ndarray, seam_x_coords: list, seam_radius: float = 0.08) -> np.ndarray:
        """
        Saldatura delle giunzioni: identifica i punti in prossimità dei confini di taglio
        e ne raccorda le coordinate per eliminare gradini residui e micro-gap.
        """
        if len(points) == 0 or not seam_x_coords:
            return points

        tree = cKDTree(points)
        smoothed = np.copy(points)

        for sx in seam_x_coords:
            near_seam_idx = np.where(np.abs(points[:, 0] - sx) < seam_radius)[0]
            for idx in near_seam_idx:
                pt = points[idx]
                neighbors = tree.query_ball_point(pt, r=0.12)
                if len(neighbors) >= 3:
                    smoothed[idx] = np.mean(points[neighbors], axis=0)

        return smoothed

    def weighted_voxel_fusion(self, tagged_points: list, voxel_size: float = 0.02) -> np.ndarray:
        """
        RITAGLIO ADATTIVO CON CONFINI A PUNTO MEDIO (MIDPOINT / VORONOI 1D):
        - Calcola i confini di taglio esattamente a metà strada tra stazioni adiacenti.
        - Assegna ciascuna fascia spaziale alla stazione fisicamente più vicina.
        - Funziona con posizionamenti regolari, asimmetrici o arbitrari.
        """
        if not tagged_points:
            return np.empty((0, 2), dtype=np.float32)

        valid_stations = [(pts[:, :2], np.array(center[:2], dtype=np.float32)) for pts, center in tagged_points if len(pts) > 0]
        num_stations = len(valid_stations)
        if num_stations == 0:
            return np.empty((0, 2), dtype=np.float32)

        if num_stations == 1:
            return valid_stations[0][0]

        # 1. Bounding Box globale lungo X
        all_pts_flat = np.vstack([pts for pts, _ in valid_stations])
        x_min_glob = float(np.min(all_pts_flat[:, 0]))
        x_max_glob = float(np.max(all_pts_flat[:, 0]))

        # 2. Ordinamento stazioni da sinistra a destra lungo l'asse X
        station_centers_x = [center[0] for _, center in valid_stations]
        sorted_indices = np.argsort(station_centers_x)
        sorted_centers_x = [station_centers_x[i] for i in sorted_indices]

        # 3. Calcolo dinamico dei punti medi (confini di separazione)
        midpoints = []
        for i in range(num_stations - 1):
            mid = (sorted_centers_x[i] + sorted_centers_x[i + 1]) / 2.0
            midpoints.append(mid)

        sliced_clouds = []
        seam_coords = list(midpoints)

        # 4. Ritaglio per fasce di competenza dinamiche
        for sector_idx, st_idx in enumerate(sorted_indices):
            pts, _ = valid_stations[st_idx]

            if sector_idx == 0:
                x_start = x_min_glob - 0.5
                x_end = midpoints[0]
            elif sector_idx == num_stations - 1:
                x_start = midpoints[-1]
                x_end = x_max_glob + 0.5
            else:
                x_start = midpoints[sector_idx - 1]
                x_end = midpoints[sector_idx]

            # Margine di overlap di 2 cm sul confine per prevenire discontinuità
            mask = (pts[:, 0] >= x_start - 0.02) & (pts[:, 0] <= x_end + 0.02)

            isolated_slice = pts[mask]
            if len(isolated_slice) > 0:
                sliced_clouds.append(isolated_slice)

        if not sliced_clouds:
            return np.empty((0, 2), dtype=np.float32)

        stitched_perimeter = np.vstack(sliced_clouds)

        # 5. Micro-Snap sui confini a punto medio
        welded_perimeter = self._smooth_slice_seams(stitched_perimeter, seam_coords, seam_radius=0.08)

        # 6. Voxel Grid Filter uniforme
        grid_indices = np.floor(welded_perimeter / voxel_size).astype(np.int32)
        x_min, y_min = grid_indices.min(axis=0)
        x_span = grid_indices[:, 0] - x_min
        y_span = grid_indices[:, 1] - y_min
        max_y = int(y_span.max()) + 1
        flat_keys = x_span.astype(np.int64) * max_y + y_span.astype(np.int64)

        sort_order = np.argsort(flat_keys)
        sorted_keys = flat_keys[sort_order]
        sorted_pts = welded_perimeter[sort_order]

        _, split_indices = np.unique(sorted_keys, return_index=True)
        pt_groups = np.split(sorted_pts, split_indices[1:])

        fused = np.array([np.mean(g, axis=0) for g in pt_groups if len(g) > 0], dtype=np.float32)
        return fused

    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float = 0.02) -> np.ndarray:
        return points