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

# PID
## PID tuning
- Started with simple P
- Added D parameter to "jump" start the movement between points
- to eliminate to offset I parameter was added
- Tuned with trial-error


## PID values
- PID values from parameter server
  - `Kp = 1.13`
  - `Ki = 1.2`
  - `Kd = 4`

```cpp
  float kp = nh_.param("/mission/pid_values/kp", 1.0f);
  float ki = nh_.param("/mission/pid_values/ki", 0.0f);
  float kd = nh_.param("/mission/pid_values/kd", 0.0f);
```

```cpp
  trajectoryPid_ = PID<Polar2<float>>(kp, ki, kd, maxErr, diffErrAlpha);
```

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
