---
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

- Create grid to world transformation functions
- Read into mapping

---

# Transformations
- The task was rather easy
- We decided to slightly modify function signatures to use `Vec2`
- There was some ambiguity in what was expected

---

# Mapping models

- 'Raw' data points
- Occupancy grid
- Geometrical lines / objects

---

# Raw points

- Easy to implement
- Not effective
- Needs extra proccessing for SLAM

---

# Occupancy grid

- Discretizes space
- Can be used for SLAM

---

# Geometrical features
- Harder to implement
- Potentially the smallest memory usage
- Can be useful for SLAM
- Seems useful for indoor areas

---

# SLAM algorithms
- Graph SLAM
- EKF SLAM

---

# Graph SLAM
- Easiest to implement
- Can become computationally heavy
- Quite robust for incorrect associations

---

# EKF SLAM
- Not so easy to implement (?)
- Scales well in states domain
- Does not handle incorrect associations well

---
