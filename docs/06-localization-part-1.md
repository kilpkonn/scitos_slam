---
marp: true
theme: default
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

- Finish localization overview
- EKF prediction step

---

# EKF - our choice
- No previous experience
- Seems logical as we have features extracted

---

# Refacto node structure
- Move localization and mapping to new node

---
# Message queue'ing
- Added closest timestamp based localisation msg getting
- Ordered queue with max size
---
# Problems
- `PoseWithCovariance` not visualized correctly
- Some other small `RViz` related problems

---

# TODO
- Fix `RViz` problems
- Clean up / refacto robot vs map frame
- Fix some mapping reated bugs
- Interpolation for localisation
- Extrapolation for localisation

---

