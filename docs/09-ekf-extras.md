---
marp: true
theme: default
math: katex
class:
  - lead
  <!-- - invert -->
---

# SCITOS: team 2

Tavo Annus
Timo Loomets
Mattias Kitsing

---

# Tasks

- Tweak EKF
- Add RRT based algorithm to path planning

---

# EKF
- Fixed bugs that Christian found in our Jacobians
- Added rotation-transition-rotation odometry model
- Experimented with some stuff to reduce the effect of incorrect mappings

---

# EKF TODO
- See if Lowe's ratio test can help
- Wrap up

---

# Path planning
- Added RRT* algorithm, have to analyse implementation complexity
- A* vs D* vs RRT* vs 'algorithm for feature map'
- TODO: Wrap up the overview and choose the algorithm

---

