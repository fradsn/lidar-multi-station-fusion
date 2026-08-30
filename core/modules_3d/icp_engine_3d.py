import numpy as np
from scipy.spatial import cKDTree
from core.base.base_engine import BaseRegistrationEngine

class ICPEngine3D(BaseRegistrationEngine):
    def _euler_to_rotation_matrix(self, yaw: float, pitch: float, roll: float) -> np.ndarray:
        """Crea la matrice R 3x3 a partire dagli angoli di Eulero in gradi (Yaw-Z, Pitch-Y, Roll-X)."""
        rz = np.deg2rad(yaw)
        ry = np.deg2rad(pitch)
        rx = np.deg2rad(roll)

        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0.0],
            [np.sin(rz),  np.cos(rz), 0.0],
            [0.0,         0.0,        1.0]
        ], dtype=np.float32)

        Ry = np.array([
            [np.cos(ry),  0.0, np.sin(ry)],
            [0.0,         1.0, 0.0],
            [-np.sin(ry), 0.0, np.cos(ry)]
        ], dtype=np.float32)

        Rx = np.array([
            [1.0, 0.0,         0.0],
            [0.0, np.cos(rx), -np.sin(rx)],
            [0.0, np.sin(rx),  np.cos(rx)]
        ], dtype=np.float32)

        return Rz @ Ry @ Rx

    def transform_points(self, points: np.ndarray, transform_params: dict) -> np.ndarray:
        """Applica la rototraslazione 6-DoF completa a una nuvola di punti 3D."""
        if len(points) == 0:
            return points

        yaw = transform_params.get('yaw', 0.0)
        pitch = transform_params.get('pitch', 0.0)
        roll = transform_params.get('roll', 0.0)
        R = self._euler_to_rotation_matrix(yaw, pitch, roll)

        t = np.array([
            transform_params.get('tx', 0.0),
            transform_params.get('ty', 0.0),
            transform_params.get('tz', 0.0)
        ], dtype=np.float32)

        pts_3d = points[:, :3]
        return (pts_3d @ R.T) + t

    def align_icp(self, source_pts: np.ndarray, target_pts: np.ndarray,
                  max_iterations: int = 60, tolerance: float = 1e-6, max_distance_m: float = 0.65):
        """Allineamento 6-DoF ICP con SVD su nuvole di punti tridimensionali."""
        if len(source_pts) == 0 or len(target_pts) == 0:
            return source_pts, {'tx': 0.0, 'ty': 0.0, 'tz': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 0.0

        current_src = np.copy(source_pts[:, :3])
        target_tree = cKDTree(target_pts[:, :3])

        accum_tx, accum_ty, accum_tz = 0.0, 0.0, 0.0
        accum_yaw, accum_pitch, accum_roll = 0.0, 0.0, 0.0
        prev_error = float('inf')

        for it in range(max_iterations):
            cur_thresh = max(0.06, max_distance_m * (1.0 - (it / max_iterations) * 0.70))
            distances, indices = target_tree.query(current_src)
            valid_mask = distances < cur_thresh

            if np.count_nonzero(valid_mask) < 20:
                break

            matched_src = current_src[valid_mask]
            matched_tgt = target_pts[indices[valid_mask], :3]

            src_centroid = np.mean(matched_src, axis=0)
            tgt_centroid = np.mean(matched_tgt, axis=0)

            src_centered = matched_src - src_centroid
            tgt_centered = matched_tgt - tgt_centroid

            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R_step = Vt.T @ U.T

            if np.linalg.det(R_step) < 0:
                Vt[2, :] *= -1
                R_step = Vt.T @ U.T

            t_step = tgt_centroid - (R_step @ src_centroid)

            d_yaw = float(np.rad2deg(np.arctan2(R_step[1, 0], R_step[0, 0])))
            d_pitch = float(np.rad2deg(np.arctan2(-R_step[2, 0], np.hypot(R_step[2, 1], R_step[2, 2]))))
            d_roll = float(np.rad2deg(np.arctan2(R_step[2, 1], R_step[2, 2])))

            current_src = (current_src @ R_step.T) + t_step
            accum_tx += float(t_step[0])
            accum_ty += float(t_step[1])
            accum_tz += float(t_step[2])
            accum_yaw += d_yaw
            accum_pitch += d_pitch
            accum_roll += d_roll

            mean_error = np.mean(distances[valid_mask])
            if abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        delta_params = {
            'tx': accum_tx,
            'ty': accum_ty,
            'tz': accum_tz,
            'yaw': accum_yaw,
            'pitch': accum_pitch,
            'roll': accum_roll
        }
        return current_src, delta_params, prev_error

    def _smooth_slice_seams_3d(self, points: np.ndarray, seam_x_coords: list, seam_radius: float = 0.08) -> np.ndarray:
        """Saldatura sferica k-d Tree 3D in prossimità dei piani di taglio verticali."""
        if len(points) == 0 or not seam_x_coords:
            return points

        tree = cKDTree(points[:, :3])
        smoothed = np.copy(points)

        for sx in seam_x_coords:
            near_seam_idx = np.where(np.abs(points[:, 0] - sx) < seam_radius)[0]
            for idx in near_seam_idx:
                pt = points[idx, :3]
                neighbors = tree.query_ball_point(pt, r=0.12)
                if len(neighbors) >= 3:
                    smoothed[idx, :3] = np.mean(points[neighbors, :3], axis=0)

        return smoothed

    def weighted_voxel_fusion(self, tagged_points: list, voxel_size: float = 0.03) -> np.ndarray:
        """
        FUSIONE A ZONE PLANARI + 3 FASCE VERTICALI:
        - Fascia Centrale (2/3 dell'altezza): ESCLUSIVA per la stazione di riferimento (niente doppioni sui muri).
        - Fasce Superiore e Inferiore (1/6 ciascuna): COOPERATIVE (preservano tetto e pavimento).
        """
        if not tagged_points:
            return np.empty((0, 3), dtype=np.float32)

        valid_stations = [(pts[:, :3], np.array(center, dtype=np.float32)) for pts, center in tagged_points if len(pts) > 0]
        num_stations = len(valid_stations)
        if num_stations == 0:
            return np.empty((0, 3), dtype=np.float32)

        if num_stations == 1:
            return valid_stations[0][0]

        # 1. Bounding box globale lungo X e Z
        all_pts_flat = np.vstack([pts for pts, _ in valid_stations])
        x_min_glob = float(np.min(all_pts_flat[:, 0]))
        x_max_glob = float(np.max(all_pts_flat[:, 0]))
        z_min_glob = float(np.min(all_pts_flat[:, 2]))
        z_max_glob = float(np.max(all_pts_flat[:, 2]))

        # Calcolo quote delle 3 fasce verticali (1/6 basso, 4/6 centro = 2/3, 1/6 alto)
        h_tot = max(0.1, z_max_glob - z_min_glob)
        z_low_thresh = z_min_glob + (1.0 / 6.0) * h_tot
        z_high_thresh = z_max_glob - (1.0 / 6.0) * h_tot

        # 2. Ordinamento stazioni lungo l'asse principale X
        station_centers_x = [center[0] for _, center in valid_stations]
        sorted_indices = np.argsort(station_centers_x)
        sorted_centers_x = [station_centers_x[i] for i in sorted_indices]

        # 3. Punti medi per la divisione in zone planari (confini X)
        midpoints = []
        for i in range(num_stations - 1):
            mid = (sorted_centers_x[i] + sorted_centers_x[i + 1]) / 2.0
            midpoints.append(mid)

        kept_points = []
        seam_coords = list(midpoints)

        # 4. Assegnazione a zone planari con esclusività verticale a 3 fasce
        for sector_idx, st_idx in enumerate(sorted_indices):
            ref_pts, _ = valid_stations[st_idx]

            if sector_idx == 0:
                x_start = x_min_glob - 1.0
                x_end = midpoints[0]
            elif sector_idx == num_stations - 1:
                x_start = midpoints[-1]
                x_end = x_max_glob + 1.0
            else:
                x_start = midpoints[sector_idx - 1]
                x_end = midpoints[sector_idx]

            # A) Tutti i punti della stazione di riferimento nella sua zona X vengono mantenuti
            mask_zone_ref = (ref_pts[:, 0] >= x_start - 0.02) & (ref_pts[:, 0] <= x_end + 0.02)
            pts_ref_zone = ref_pts[mask_zone_ref]
            if len(pts_ref_zone) > 0:
                kept_points.append(pts_ref_zone)

            # B) Punti delle ALTRE stazioni concorrenti in questo settore X:
            # - NELLA FASCIA CENTRALE (2/3): VENGONO SCARTATI RIGIDAMENTE (anti-ghosting sui muri).
            # - NELLE FASCE ESTERNE (tetto e pavimento): VENGONO ACCETTATI per colmare i vuoti.
            for other_idx, (other_pts, _) in enumerate(valid_stations):
                if other_idx == st_idx:
                    continue

                mask_zone_other = (other_pts[:, 0] >= x_start - 0.02) & (other_pts[:, 0] <= x_end + 0.02)
                pts_other_zone = other_pts[mask_zone_other]
                if len(pts_other_zone) == 0:
                    continue

                # Esclude la fascia centrale [z_low_thresh, z_high_thresh]
                mask_ext = (pts_other_zone[:, 2] < z_low_thresh) | (pts_other_zone[:, 2] > z_high_thresh)
                pts_ext = pts_other_zone[mask_ext]

                if len(pts_ext) > 0:
                    kept_points.append(pts_ext)

        if not kept_points:
            return np.empty((0, 3), dtype=np.float32)

        stitched_volume = np.vstack(kept_points)

        # 5. Saldatura Seam volumetrica sui piani di taglio
        welded_volume = self._smooth_slice_seams_3d(stitched_volume, seam_coords, seam_radius=0.10)

        # 6. Voxel Grid Filter finale per omogeneizzare la densità
        grid_indices = np.floor(welded_volume / voxel_size).astype(np.int32)
        min_idx = grid_indices.min(axis=0)
        shifted = grid_indices - min_idx
        max_dims = shifted.max(axis=0) + 1

        flat_keys = (
            shifted[:, 0].astype(np.int64) * (max_dims[1] * max_dims[2]) +
            shifted[:, 1].astype(np.int64) * max_dims[2] +
            shifted[:, 2].astype(np.int64)
        )

        sort_order = np.argsort(flat_keys)
        sorted_keys = flat_keys[sort_order]
        sorted_pts = welded_volume[sort_order]

        _, split_indices = np.unique(sorted_keys, return_index=True)
        pt_groups = np.split(sorted_pts, split_indices[1:])

        fused_3d = np.array([np.mean(g, axis=0) for g in pt_groups if len(g) > 0], dtype=np.float32)
        return fused_3d

    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float = 0.03) -> np.ndarray:
        return points