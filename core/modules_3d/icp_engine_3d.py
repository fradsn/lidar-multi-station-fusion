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

    def _smooth_slice_seams_3d(self, points: np.ndarray, radius: float = 0.08) -> np.ndarray:
        """Filtro di pulizia e regolarizzazione 3D su raggio di vicinato per eliminare residui spuri."""
        if len(points) < 5:
            return points

        tree = cKDTree(points[:, :3])
        counts = tree.query_ball_point(points[:, :3], r=radius, return_sorted=False)
        valid_mask = np.array([len(c) >= 3 for c in counts], dtype=bool)
        return points[valid_mask]

    def weighted_voxel_fusion(self, tagged_points: list, voxel_size: float = 0.03) -> np.ndarray:
        """
        FUSIONE 3D CALIBRATA SUL CONO OTTICO REALE (70° - 165°, Orizzonte 135°):
        - Angoli fisici: Max Up = +65° (Zenith cone), Max Down = -30° (Nadir cone).
        - Zona di visibilità standard (-30° <= elevazione <= +65°): Autorità esclusiva alla stazione
          più vicina. Elimina muri doppi e ghosting.
        - Coni d'ombra zenit/nadir: Se un punto cade nel cono cieco della stazione geometricamente più
          vicina, viene accettato il dato rilevato da stazioni più distanti, sigillando pavimento e soffitto.
        - Voxel Grid Filter finale: Compattazione isotropa a voxel_size metri.
        """
        if not tagged_points:
            return np.empty((0, 3), dtype=np.float32)

        valid_stations = [
            (pts[:, :3].astype(np.float32), np.array(center[:3], dtype=np.float32)) 
            for pts, center in tagged_points if len(pts) > 0
        ]
        num_stations = len(valid_stations)
        if num_stations == 0:
            return np.empty((0, 3), dtype=np.float32)

        if num_stations == 1:
            return self.voxel_grid_filter(valid_stations[0][0], voxel_size)

        all_centers = np.array([c for _, c in valid_stations], dtype=np.float32)
        centers_tree = cKDTree(all_centers)

        # Limiti fisici di elevazione del sensore (in radianti)
        max_elev_up_rad = np.deg2rad(65.0)     # 135° - 70°  = +65° (Soffitto/Zenith)
        max_elev_down_rad = np.deg2rad(-30.0)  # 135° - 165° = -30° (Pavimento/Nadir)

        selected_clouds = []
        margin_m = 0.06

        for st_idx, (pts, center) in enumerate(valid_stations):
            rel_vecs = pts - center
            dists_3d = np.linalg.norm(rel_vecs, axis=1)

            # 1. Trova la stazione più vicina in assoluto per ciascun punto 3D
            closest_dists, nearest_st_ids = centers_tree.query(pts)
            is_dominant = (dists_3d <= closest_dists + margin_m) | (nearest_st_ids == st_idx)

            # 2. Calcola l'angolo di elevazione rispetto alla stazione più vicina
            nearest_centers = all_centers[nearest_st_ids]
            vec_to_nearest = pts - nearest_centers
            dz_nearest = vec_to_nearest[:, 2]
            dxy_nearest = np.hypot(vec_to_nearest[:, 0], vec_to_nearest[:, 1])

            # Elevazione angolare (in radianti) rispetto al piano orizzontale del sensore più vicino
            elev_angle_nearest = np.arctan2(dz_nearest, np.maximum(0.01, dxy_nearest))

            # Verifica se il punto ricade nel cono d'ombra ottico della stazione più vicina
            in_nearest_blind_cone = (elev_angle_nearest > max_elev_up_rad) | (elev_angle_nearest < max_elev_down_rad)

            # 3. Maschera combinata: mantieni se dominante oppure se serve a colmare un cono cieco altrui
            keep_mask = is_dominant | in_nearest_blind_cone
            pts_kept = pts[keep_mask]

            if len(pts_kept) > 0:
                selected_clouds.append(pts_kept)

        if not selected_clouds:
            return np.empty((0, 3), dtype=np.float32)

        stitched_volume = np.vstack(selected_clouds)

        # 4. Compattazione volumetrica Voxel Grid 3D
        fused = self.voxel_grid_filter(stitched_volume, voxel_size=voxel_size)

        # 5. Pulizia dei residui isolati
        return self._smooth_slice_seams_3d(fused, radius=0.08)

    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float = 0.03) -> np.ndarray:
        """Filtro Voxel Grid 3D isotropo basato su hashing vettorizzato."""
        if len(points) == 0:
            return points

        grid_indices = np.floor(points[:, :3] / voxel_size).astype(np.int32)
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
        sorted_pts = points[sort_order, :3]

        _, split_indices = np.unique(sorted_keys, return_index=True)
        pt_groups = np.split(sorted_pts, split_indices[1:])
        return np.array([np.mean(g, axis=0) for g in pt_groups if len(g) > 0], dtype=np.float32)