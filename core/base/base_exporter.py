from abc import ABC, abstractmethod
import numpy as np

class BaseExporter(ABC):
    @abstractmethod
    def export_cad(self, filepath: str, points: np.ndarray) -> bool:
        """Esporta nel formato CAD di riferimento (DXF 2D o DXF 3D)."""
        pass

    @abstractmethod
    def export_table(self, filepath: str, points: np.ndarray) -> bool:
        """Esporta in formato tabellare / nuvola (CSV, PLY, PCD)."""
        pass