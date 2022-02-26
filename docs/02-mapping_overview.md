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
- EAFC SLAM
- Particle filter SLAM

---

# Graph SLAM
- Easy to implement
- Can become computationally heavy
- Quite robust for incorrect associations

---

# EKF SLAM
- Not so easy to implement (?)
- Scales well in states domain
- Does not handle incorrect associations well

---

# Enhanced adaptive fuzzy clustering with noise clustering SLAM
- Hardest to implement
- Has quite many hyperparameters
- Potentially quite useful for indorrs use case

---

# Particle filter SLAM
- Easy to implement
- CPU heavy

---

# Ready made solutions
- GMapping
- Cartographer

---

# EKF SLAM vs ICP + DBSCAN + K-means

![img](https://journals.sagepub.com/na101/home/literatum/publisher/sage/journals/content/arxa/2015/arxa_12_3/59992/20161205/images/medium/10.5772_59992-fig24.gif)
[https://journals.sagepub.com/doi/full/10.5772/59992](https://journals.sagepub.com/doi/full/10.5772/59992)

---

# Compression by clustering

![img](https://journals.sagepub.com/na101/home/literatum/publisher/sage/journals/content/arxa/2015/arxa_12_3/59992/20161205/images/medium/10.5772_59992-fig12.gif)
[https://journals.sagepub.com/doi/full/10.5772/59992](https://journals.sagepub.com/doi/full/10.5772/59992)

--- 

# Iterative end point fitting
![img](https://journals.sagepub.com/na101/home/literatum/publisher/sage/journals/content/arxa/2015/arxa_12_3/59992/20161205/images/medium/10.5772_59992-fig6.gif)
[https://journals.sagepub.com/doi/full/10.5772/59992](https://journals.sagepub.com/doi/full/10.5772/59992)

---

