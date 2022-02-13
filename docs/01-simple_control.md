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

- Create PID
- Create Utility items (`Vec2`, `Polar2`, etc)
- Set up project structure
- Make robot drive

---

# PID tuning
- Started with simple P
- TODO...

---

# What went well?
Utility items worked out very nicely
```cpp
Vec2<T>  <->  Polar2<T>  // Coverts very smoothly
```
PID is generic enough to work on vectors
```cpp
Vec2<float> error;
Polar2<float> command = pid.accumulate(error, dt);
// work with command.r and command.theta
```

--- 
# What went well?
- Everybody contributed
- RViz is nice :D
- Found out `catkin_tools` is more convenient than `catkin_make`

---
# Problems
- ROS internally differs quite a lot from ROS2
- ROS does not compile with `c++20`

---
