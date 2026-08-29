# 🗺️ LiDAR Multi-Station Fusion (`lidar-multi-station-fusion`)

A high-performance desktop application built with Python & PyQt6 for multi-station LiDAR scan registration, automatic alignment, and high-fidelity geometric fusion (indoor SLAM / 2D architectural perimeter extraction).

Designed to produce clean, single-layer floorplans by eliminating double-wall tracks, phantom reflections, and overlapping artifacts between adjacent scanning stations.

---

## 📸 Interface & Multi-Station Workflow

### 1. Multi-Station Scan Loading
Inspect individual station point clouds with dedicated layer controls and real-time 3-DoF manipulation.

| Station 1 (Base Layer) | Station 2 (Offset Alignment) | Station 3 (Full Span) |
| :---: | :---: | :---: |
| <img src="docs/images/screen1.png" width="100%"/> | <img src="docs/images/screen2.png" width="100%"/> | <img src="docs/images/screen3.png" width="100%"/> |

---

### 2. Multi-Station Fusion Pipeline (Before vs After)

Comparison showing the removal of parallel ghost walls through Hard Bounding-Box Spatial Slicing and Micro-Snap Boundary Welding:

| ❌ Raw Overlap (Pre-Fusion) | ✅ Clean Perimeter (Post-Fusion) |
| :---: | :---: |
| <img src="docs/images/preFusion.png" width="100%"/> | <img src="docs/images/postFusion.png" width="100%"/> |
| *Multi-station overlap with double-wall artifacts and angular drift* | *Unified single-layer perimeter with seamless boundary welding* |

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
  - Real-time station layer inspection and manual 3-DoF control (Translation X, Y and Rotation θ).
- **Vector & CAD Export:**
  - One-click export to AutoCAD-compliant DXF format and georeferenced CSV.


---

## 🚀 Usage Workflow

1. **Launch the Application:**
   python main.py

2. **Import Scans:**
   Click "Load CSV Scans" to import 2 or more station files from different scanning positions.

3. **Align Point Clouds:**
   Click "1-CLICK AUTO-ALIGN" to run the multi-station cascade ICP solver, or adjust station positions manually using the X, Y, and Yaw controls.

4. **Fuse & Extract Perimeter:**
   Click "GENERATE FUSED MAP" to apply spatial slicing, seam welding, and voxel downsampling.

5. **Export CAD Plan:**
   Click "Export DXF" to save the vector drawing for AutoCAD, Revit, or Rhino.

---

## 🔬 Algorithm Overview

### 1. Metric Point-to-Line Registration
The alignment phase minimizes the orthogonal point-to-plane distance against target surface normals, preventing longitudinal wall drift.

### 2. Spatial Authority Partitioning
The bounding box width is partitioned into N distinct sectors ordered by sensor positions. Each wall segment is retained solely from the station with optimal line-of-sight.

### 3. Boundary Micro-Snap Blending
Across each slice seam boundary, points within the neighborhood radius are smoothed using local k-d Tree adjacency to ensure continuous outlines without artificial steps.

---

