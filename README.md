# Static Solar Concentrator Code

This repository contains the implementation associated with the paper:

**"Parametric Surface Design for Static Solar Concentration: Enhancing Plastic Waste Pyrolysis in Sustainable Housing"**
https://doi.org/10.1016/j.matcom.2026.04.024

https://ars.els-cdn.com/content/image/1-s2.0-S0378475426001667-mmc1.pdf

The code combines a mathematical model for solar geometry and parametric mirror design with an embedded implementation for physical construction and validation.

---

## 📁 Repository Contents

### 🔹 Mathematical Model (Wolfram Mathematica)

The file `Notebook.nb` (also provided as `Notebook.pdf`) contains the full mathematical framework used in the paper.

Main components:

- Solar geometry modeling:
  - Day angle and solar declination (Spencer model)
  - Hour angle and true solar time
  - Solar elevation and azimuth

- Parametric mirror construction:
  - Local parabolic profiles driven by solar elevation
  - Time-dependent focal point evolution
  - Horizontal solar sweep and trajectory mapping

- 3D surface generation:
  - Assembly of the mirror as a family of rotated and translated parabolas
  - Construction of the full parametric surface

- Visualization tools:
  - Daily solar trajectory
  - Evolution of parabolic profiles
  - 3D representation of the mirror and focal line

This notebook provides a complete reproducible pipeline from solar geometry to mirror surface generation.

---

### 🔹 Arduino Implementation (Physical Construction)

The repository includes Arduino code for implementing the system in a physical setup:

- `Codigo_ESPESO.ino`
- `Codigo_Spline.ino`
- `libraries/ParabolaInterp.*`

This code is designed to run on an **Arduino Uno** and is used for:

- Guiding the construction of the mirror
- Verifying point placement in real space
- Assisting in the fabrication of the parametric surface

---

### 🔹 LIDAR-Based Positioning System

The Arduino system is coupled with a **LIDAR sensor**, which allows:

- Measurement of distances in real space
- Reconstruction of point positions in 3D
- Verification of the geometric accuracy of the constructed mirror

This setup enables a **point-by-point reconstruction of the mirror surface**, ensuring that the physical implementation matches the theoretical design.

---

### 🔹 Additional Materials

- `Notebook.pdf`: Exported version of the Mathematica notebook
- `.gif` and `.mp4` files: Visual demonstrations of the system behavior
- `flowchart.pdf`: System workflow and implementation logic

---

## ⚙️ Reproducibility

This repository contains all the necessary elements to reproduce:

1. The mathematical model of the mirror surface
2. The parametric generation of focal trajectories
3. The interpolation framework used in the paper
4. The physical construction methodology using embedded systems

---

## 🔗 Repository and Citation

This repository is publicly available and has been archived to ensure long-term reproducibility.

(👉 Aquí añadirás el DOI de Zenodo cuando lo tengas)

---

## 📌 Notes

- The Mathematica notebook is the reference implementation of the model.
- The Arduino code is intended for experimental validation and construction support.
- The system is designed to bridge theoretical modeling and real-world fabrication.

---

## 👨‍🔬 Authors

E. Moreno, P. González-Rodelas, V. Pascual

---

## 📄 License

This repository is released under the MIT License, allowing free use, modification, and distribution of the code.

The associated scientific article is published in a subscription-based journal and may be subject to publisher copyright restrictions. This repository provides the implementation and reproducibility resources independently of the publication access conditions.

