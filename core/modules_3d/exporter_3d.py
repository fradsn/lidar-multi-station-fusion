import numpy as np

class Exporter3D:
    @staticmethod
    def export_cad(file_path: str, points: np.ndarray) -> bool:
        """Esporta la nuvola 3D fusa in formato standard PLY (punto di riferimento per mesh/point cloud)."""
        if len(points) == 0:
            return False
        try:
            pts = points[:, :3]
            header = (
                "ply\n"
                "format ascii 1.0\n"
                f"element vertex {len(pts)}\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "end_header\n"
            )
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(header)
                np.savetxt(f, pts, fmt="%.4f %.4f %.4f")
            return True
        except Exception as e:
            print(f"[Exporter3D] Errore salvataggio PLY: {e}")
            return False

    @staticmethod
    def export_table(file_path: str, points: np.ndarray) -> bool:
        """Esporta la tabella di coordinate 3D fuse in CSV (X, Y, Z)."""
        if len(points) == 0:
            return False
        try:
            pts = points[:, :3]
            np.savetxt(file_path, pts, delimiter=",", header="X,Y,Z", comments="", fmt="%.4f")
            return True
        except Exception as e:
            print(f"[Exporter3D] Errore salvataggio CSV: {e}")
            return False