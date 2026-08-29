import numpy as np
from core.base.base_exporter import BaseExporter

class Exporter3D(BaseExporter):
    """
    Predisposto per export 3D (PLY / PCD / DXF 3D).
    """
    def export_cad(self, filepath: str, points: np.ndarray) -> bool:
        # TODO: Export entità 3DFACE o 3D POINT su DXF
        raise NotImplementedError("Export CAD 3D in fase di sviluppo.")

    def export_table(self, filepath: str, points: np.ndarray) -> bool:
        # TODO: Export PLY (Stanford Mesh Format) o CSV (X, Y, Z)
        raise NotImplementedError("Export Nuvola 3D in fase di sviluppo.")