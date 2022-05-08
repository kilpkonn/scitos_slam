---
marp: true
theme: default
math: katex
paginate: true
class: lead
html: true
style: |
  .columns {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 1rem;
  }
---

# SCITOS: team 2

Tavo Annus
Timo Loomets
Mattias Kitsing

---

# Tasks

For indoor differential drive robot:
- Choose and implement mapping algorithm
- Choose and implement localization algorithm
- Choose and implement path planning & obstacle avoidance algorithm

---

# Mapping overview

- Feature map (lines as features)
- Line is defined by 2 points
- Roughly inspired by **"Algorithms and a Framework for Indoor Robot Mapping in a Noisy Environment Using Clustering in Spatial and Hough Domains"** _([https://journals.sagepub.com/doi/full/10.5772/59992](https://journals.sagepub.com/doi/full/10.5772/59992))_

---

<div class="columns">
<div>

# Mapping flow 1: Extracting lines
1. Read the laser points
2. Perform noise filtering by eroding and dilating
3. Cluster into objects with DBSCAN
4. Fit lines with IEPF

</div>
<div>

<!-- ![bg right 80%](./fig/iepf.png) -->
<figure>
  <img src="./fig/iepf.png" alt="Map" style="width:100%">
  <figcaption>Fig.1 - Extracted lines.</figcaption>
</figure>
</div>
</div>

---

<div class="columns">
<div>

# Mapping flow 2: Accumulating map

1. Find the best match for every line
    - Needs to "overlap"
    - Uses Mahalanobis distance
2. Merge lines that are closer than threshold
3. Add lines that do not have good match
4. Remove lines that have low confidence

</div>
<div>

<!-- ![bg right 60%](./fig/mapv2.png) -->
<figure>
  <img src="./fig/mapv2.png" alt="Map" style="width:100%">
  <figcaption>Fig.2 - Feature map.</figcaption>
</figure>
</div>
</div>

---

# Localization overview

- EKF algorithm
  - Maximal likelyhood
  - Some sanity checks
  - No update if sanity checks fails
- We used the features from mapping
- Interpolation / Extrapolation

</div>
<div>

<!-- ![bg right 80%](./fig/ekf_good.png) -->
<figure>
  <img src="./fig/ekf_good.png" alt="Map" style="width:100%">
  <figcaption>Fig.3 - EKF at work.</figcaption>
</figure>
</div>
</div>

---

# Localization - EKF results 1

 - Much better resuls than odometry
 - Could recover from uncertanty and errors
 - 

<!-- ![bg right 80%](./fig/ekf_recovery.png) -->
<figure>
  <img src="./fig/ekf_recovery.png" alt="Map" style="width:100%">
  <figcaption>Fig.4 - EKF recovering.</figcaption>
</figure>
</div>
</div>

---

# Localization - EKF results 2

Had problems with 
- incorrectly mapped features
- long corridors 

<!-- ![bg right 80%](./fig/ekf_ugly.png) -->
<figure>
  <img src="./fig/ekf_ugly.png" alt="Map" style="width:100%">
  <figcaption>Fig.5 - EKF drift.</figcaption>
</figure>
</div>
</div>

---

# Global path planning (RRT)
<figure>
  <img src="./fig/final_rrt.png" alt="final_rrt" style="width:100%">
  <figcaption>Fig.6 - RRT graph on map.</figcaption>
</figure>
</div>
</div>
<figure>
  <img src="./fig/wall_padding.png" alt="wall_padding" style="width:100%">
  <figcaption>Fig.7 - Padding around walls.</figcaption>
</figure>
</div>
</div>

<!--![bg right 80%](./fig/final_rrt.png)
*RRT graph on map*
![bg right 80%](./fig/wall_padding.png)
*Padding around walls*-->

---

<div class="columns">
<div>

# Local planning (PID)
*PID errors while driving through waypoints*

</div>
<div>

<!-- ![bg right 70%](./fig/point_drive_PID.png) -->

<figure>
  <img src="./fig/point_drive_PID.png" alt="Map" style="width:100%">
  <figcaption>Fig.7 - PID errors.</figcaption>
</figure>
</div>
</div>

---

<div class="columns">
<div>

# Safety (Path simulation)

- Default clamp
- Stricter conditional clamp
- Simulate up to 5s of future
- Stop before potential collision
*Path prediction near walls*

</div>
<div>

<!-- ![bg right 70%](./fig/path_prediction_corridor.png) -->

<figure>
  <img src="./fig/path_prediction_corridor.png" alt="Map" style="width:100%">
  <figcaption>Fig.8 - Path prediction.</figcaption>
</figure>
</div>
</div>

---

# Project conclusion
**The good**
- Good teamwork and impressive result
- Learned interesting new technologies
- Decent project structure

**The bad**
- Solution lacks robustness
- Only unit tests

**The Ugly**
- Technological dept

---
