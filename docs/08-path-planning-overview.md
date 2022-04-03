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

- Finish EKF
- Finish interpolation / extrapolation
- Path planning & Obstacle avoidance overview

---

# EKF
- Everything works _(unless unlucky feature mappings)_
- Kept motion model we had, likely switch `/cmd_vel` -> `/odom` in the future
- Not robust at all due to bad feature mappings

---

# Interpolation / Extrapolation

- Finally works and is quite stable
- ~98% is extrapolated

---

# Path planning & obstance avoidance

---

# A*
- Use it as a mark to beat
- Easy, but not the best

---

# D*
- Improvement on A*
- Plan from goal to start
- Re-evaluate only when needed

---

# Global path planning for feature maps
- Really good results on paper
- Exponential complexity for worst case _(n = amount of features)_

---

# Our choice
- D*
- Take inspiration from paper of planning with feature maps

---

