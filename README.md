# 🗺️ LiDAR Multi-Station Fusion (`lidar-multi-station-fusion`)

A high-performance desktop application built with **Python & PyQt6** for multi-station LiDAR scan registration, automatic alignment, and high-fidelity geometric fusion (indoor SLAM / 2D architectural perimeter extraction).

Designed to produce clean, single-layer floorplans by eliminating double-wall tracks, phantom reflections, and overlapping artifacts between adjacent scanning stations.

---

## 🌟 Key Features

- **Coarse-to-Fine Point-to-Line ICP Alignment:**
  - Robust iterative alignment leveraging local normal estimation.
  - Mitigates wall-sliding drift and handles large initial misalignments down to millimeter precision.
- **Hard Bounding-Box Spatial Slicing:**
  - Dynamically partitions the global room footprint into exclusive sectors based on real sensor coordinates.
  - Grants absolute authority to the closest station for each sector, removing ghost lines and parallel wall artifacts.
- **Micro-Snap Seam Smoothing & Welding:**
  - Eliminates alignment height steps and bridges micro-gaps at partition cutlines.
  - Preserves structural corners while ensuring seamless perimeter continuity.
- **Uniform Voxel Grid Downsampling:**
  - Regularizes point distribution and density across the entire fused contour.
- **Interactive GUI (PyQt6 + PyQtGraph):**
  - Real-time station layer inspection and manual 3-DoF control (Translation $X, Y$ and Rotation $\theta$).
- **Vector & CAD Export:**
  - One-click export to AutoCAD-compliant **DXF** format and georeferenced **CSV**.

---

## 📁 Repository Structure

```text
lidar-multi-station-fusion/
├── core/
│   ├── base/
│   │   └── base_engine.py          # Abstract registration engine interface
│   └── modules_2d/
│       └── icp_engine_2d.py        # Point-to-Line ICP, Slicing, and Seam Welder
├── ui/                             # PyQt6 / PyQtGraph interface components
├── main.py                         # Application entry point
├── requirements.txt                # Python dependencies
├── .gitignore
└── README.md
