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

Full 3D volumetric workflow with 6-DoF cascade registration, **Spherical Sector Authority**, horizon-aware 3-band vertical partitioning, and OpenGL interactive rendering.

https://github.com/user-attachments/assets/a9956a8a-90a6-4aec-bc18-8eb90867c8ef

---

## 🌟 Key Features

### 🗺️ 2D Planimetric Suite
- **Coarse-to-Fine SVD Point-to-Point ICP Alignment:**
  - Robust iterative solver computing $(T_x, T_y, \text{Yaw } \theta)$.
  - Dynamic matching distance threshold decay (coarse $0.50\text{ m} \to$ fine $0.06\text{ m}$) ensuring rapid convergence and millimeter precision.
- **Polar Sector Density Authority (Isotropic 360° Partitioning):**
  - Discretizes each station's field of view into adaptive angular bins (polar sectors).
  - Assigns geometric authority along each directional vector to the station with the highest sampling density and signal-to-noise ratio (closest proximity).
  - Completely eliminates axis-orientation dependency: works seamlessly on oblique rooms, L-shaped corridors, and arbitrary sensor placements without wall clipping.
- **Micro-Snap Neighbor Welding & 2D Voxel Grid Filter:**
  - Regularizes boundary transitions with a $2\text{ cm}$ isotropic grid filter ($0.02\text{ m}$).
  - Performs local $k\text{-d Tree}$ neighbor pruning to discard spurious airborne noise while enforcing continuous structural perimeters.

### 🧊 3D Volumetric Suite
- **Full 6-DoF SVD Point-to-Point ICP:**
  - Closed-form Kabsch/SVD rototranslation solver across all degrees of freedom ($\text{Yaw } \theta_z, \text{Pitch } \theta_y, \text{Roll } \theta_x + T_x, T_y, T_z$).
  - **1-Click Global Auto-Align:** Cascaded registration of all imported 3D station scans against the accumulated spatial reference.
- **Spherical Sector Authority & Dual-Ratio 3-Band Vertical Partitioning:**
  - **Spherical Directional Binning:** Evaluates local authority along both horizontal azimuth ($\theta$) and vertical elevation ($\phi$) cones.
  - **Central Band (2/3 of Total Height — Eye-Level/Horizon Walls):** *Strict exclusivity* for the dominant local station, eliminating double-wall ghosting and wall thickness inflation.
  - **Upper & Lower Bands (1/6 each — Ceiling & Floor):** *Cooperative multi-station blending* to close zenith/nadir blind cones without clipping floor or ceiling surfaces.
- **Fast 3D Hash Grid & Spatial Outlier Pruning:**
  - High-performance vector hashing for $3\text{ cm}$ cubic voxel centroid downsampling.
  - Spherical $k\text{-d Tree}$ density validation to bridge boundary seams and eliminate isolated floating points.

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
