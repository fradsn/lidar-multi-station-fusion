import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QIcon, QFont
from PyQt6.QtCore import Qt
from ui.main_window import MainWindow

def main():
    # Abilita rendering ad alto DPI se supportato
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    
    # Stile e Font moderno
    app.setStyle('Fusion')
    default_font = QFont("Segoe UI", 9)
    app.setFont(default_font)

    # Palette Dark Mode nativa per coerenza visiva
    window = MainWindow()
    window.show()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()