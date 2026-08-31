# 🗺️ LiDAR Universal Stitcher (`lidar-multi-station-fusion`)

A high-performance desktop application built with Python, PyQt6, and PyQtGraph for multi-station LiDAR scan registration, automatic 2D/3D alignment, and high-fidelity geometric fusion (indoor 2D architectural perimeter extraction & 3D volumetric cloud reconstruction).

Designed to produce clean, single-layer floorplans and watertight 3D models by eliminating double-wall tracks, phantom reflections, and overlapping artifacts across arbitrary multi-station layouts (supporting 2, 4, 10, or more stations regardless of room orientation).

---

## 📸 Interface & Multi-Station Workflow

### 1. Multi-Station Scan Loading (2D Planar Mode)
Inspect individual station point clouds with dedicated layer controls and real-time 3-DoF manipulation (including coordinate realignment for the Base Station).

| Station 1 (Base Layer) | Station 2 (Offset Alignment) | Station 3 (Full Span) |
| :---: | :---: | :---: |
| <img src="docs/images/screen1.png" width="100%"/> | <img src="docs/images/screen2.png" width="100%"/> | <img src="docs/images/screen3.png" width="100%"/> |

---

### 2. Multi-Station Fusion Pipeline (2D Before vs After)

Comparison showing the removal of parallel ghost walls through **Polar Sector Density Authority** and **Isotropic Voxel Centroiding**:

| ❌ Raw Overlap (Pre-Fusion) | ✅ Clean Perimeter (Post-Fusion) |
| :---: | :---: |
| <img src="docs/images/preFusion.png" width="100%"/> | <img src="docs/images/postFusion.png" width="100%"/> |
| *Multi-station overlap with double-wall artifacts and angular drift* | *Unified single-layer perimeter with seamless boundary welding* |

---

### 3. 3D Volumetric Multi-Station Alignment & Fusion

Full 3D volumetric workflow with 6-DoF cascade registration, **Hardware-Calibrated Optical Cone Geometry**, zenith/nadir blind cone cooperation, and OpenGL interactive rendering.

https://github.com/user-attachments/assets/a9956a8a-90a6-4aec-bc18-8eb90867c8ef

---

## 🌟 Key Features

### 🗺️ 2D Planimetric Suite
- **Coarse-to-Fine SVD Point-to-Point ICP Alignment:**
  - Robust iterative solver computing $(T_x, T_y, \text{Yaw } \theta)$.
  - Dynamic matching distance threshold decay (coarse $0.50\text{ m} \to$ fine $0.06\text{ m}$) ensuring rapid convergence and millimeter precision.
- **Polar Sector Density Authority (Isotropic 360° Partitioning):**
  - Evaluates local spatial authority using a $k\text{-d Tree}$ of sensor coordinates, giving precedence to the station with the closest Euclidean proximity (highest sampling density and SNR).
  - Groups validated points into 360 angular bins ($1^\circ$ resolution) along $\theta = \text{atan2}(\Delta y, \Delta x)$ to preserve wavefront continuity.
  - Fully rotation-invariant and isotropic: eliminates wall-clipping and ghosting on oblique walls, L-shaped corridors, and arbitrary layouts.
- **Micro-Snap Neighbor Pruning & 2D Voxel Grid Filter:**
  - Regularizes boundary transitions with a $2\text{ cm}$ isotropic hash grid ($0.02\text{ m}$).
  - Performs local $k\text{-d Tree}$ neighbor pruning ($r = 8\text{ cm}$) to discard airborne noise while ensuring continuous perimeter paths.

### 🧊 3D Volumetric Suite
- **Full 6-DoF SVD Point-to-Point ICP:**
  - Closed-form Kabsch/SVD rototranslation solver across all degrees of freedom ($\text{Yaw } \theta_z, \text{Pitch } \theta_y, \text{Roll } \theta_x + T_x, T_y, T_z$).
  - **1-Click Global Auto-Align:** Cascaded registration of all imported 3D station scans against the accumulated spatial reference.
- **Hardware-Calibrated Optical Cone Authority & Blind Cone Healing:**
  - **Exact Optical Field of View:** Calibrated to sensor limits ($70^\circ\text{--}165^\circ$ elevation range with horizon at $135^\circ$), defining an active vertical FoV of $[-30^\circ, +65^\circ]$.
  - **Exclusive Wall Authority ($-30^\circ \le \phi \le +65^\circ$):** Strict authority granted to the nearest sensor to eliminate ghost walls and artificial thickness.
  - **Zenith & Nadir Cooperative Healing ($\phi > +65^\circ$ or $\phi < -30^\circ$):** Automatically detects if a point falls within the blind cone of the closest station, seamlessly pulling complementary ceiling/floor data from farther stations to deliver watertight enclosures.
- **Fast 3D Hash Grid & Spatial Outlier Pruning:**
  - High-performance 64-bit vector hashing for $3\text{ cm}$ cubic voxel centroid downsampling.
  - Spherical $k\text{-d Tree}$ neighbor validation ($r = 8\text{ cm}$) to prune isolated reflections and sensor artifacts.

### 🖥️ Universal Core & GUI
- **Base Station Alignment Control:**
  - Real-time manual rototranslation controls enabled for all layers (including Station 1/Base) to align floor plans with CAD coordinate axes prior to registration.
- **Universal Smart Parser:**
  - Automatically detects delimiters (spaces, commas, tabs, semicolons) and headers (`X_m`, `Angle_deg`, `Distance_cm`, raw `X Y Z`).
  - Automatic unit normalization (centimeters to meters).
- **Interactive Dual Viewport (PyQt6 + PyQtGraph + OpenGL):**
  - Instant switching between 2D Planar and 3D Volumetric canvases.
  - Dynamic point size adjustment slider (`1` to `10 px`).
- **Multi-Format Export:**
  - **2D:** AutoCAD-compliant DXF vector drawings & clean CSV tables.
  - **3D:** Standard ASCII PLY point clouds (compatible with CloudCompare, MeshLab, Blender, Revit) & $(X, Y, Z)$ CSV tables.

---
