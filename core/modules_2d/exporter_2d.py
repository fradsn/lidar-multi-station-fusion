import pandas as pd
import numpy as np
from core.base.base_exporter import BaseExporter

class Exporter2D(BaseExporter):
    def export_cad(self, filepath: str, points: np.ndarray) -> bool:
        """Esporta nuvola 2D in DXF R12."""
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write("0\nSECTION\n2\nHEADER\n0\nENDSEC\n")
                f.write("0\nSECTION\n2\nTABLES\n0\nENDSEC\n")
                f.write("0\nSECTION\n2\nBLOCKS\n0\nENDSEC\n")
                f.write("0\nSECTION\n2\nENTITIES\n")
                for pt in points:
                    f.write("0\nPOINT\n8\nMERGED_2D\n")
                    f.write(f"10\n{pt[0]:.4f}\n")
                    f.write(f"20\n{pt[1]:.4f}\n")
                    f.write("30\n0.0\n")
                f.write("0\nENDSEC\n0\nEOF\n")
            return True
        except Exception as e:
            print(f"Errore export DXF 2D: {e}")
            return False

    def export_table(self, filepath: str, points: np.ndarray) -> bool:
        """Esporta nuvola 2D in CSV unificato."""
        try:
            df = pd.DataFrame(points[:, :2], columns=['X', 'Y'])
            df.to_csv(filepath, index=False)
            return True
        except Exception as e:
            print(f"Errore export CSV 2D: {e}")
            return False