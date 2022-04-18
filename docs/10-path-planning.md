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

- Make final decision for path planning
- Implement it

---

# Our choice: RRT
- Easy to implement
- Comptutationally faster than RRT*

---

# Implementation
- Basics done
- Wall checks broken
- Pieces are not put together

---
# RRT

![bg right 70%](./fig/rrt_base.png)

---

# Path estimation

- 5 seconds of future
- tracks waypoints
- uses PID

![bg right](./fig/path_simulation.png)


---

# Problems
- Gazebo went crazy on our PID output
- Something wrong with wall checks

---

