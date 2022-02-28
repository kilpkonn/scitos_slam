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

- Decide on how to store map
- Start implementing algorithms

---

# Map choice - EAFC (or inspired by it)
- Did not want grid
- Wanted something that could potentially store topology
- Non-standard, but not too hard to understand

---

# Erosion

- TODO

---

# DBSCAN

- Own imlementation
- Used to distinguish lines from one another
- Can be used for filtering out some noise

---

# DBSCAN
![bg right](./fig/dbscan.png)

---

# KMeans

- Used for reducing data size
- Maybe drop it and use lines instead of centroids

---

# KMeans
![bg right 95%](./fig/kmeans.png)

---

# Iterative end point fitting
- Douglas Peucker algorithm

--- 

# Iterative end point fitting
![img](https://journals.sagepub.com/na101/home/literatum/publisher/sage/journals/content/arxa/2015/arxa_12_3/59992/20161205/images/medium/10.5772_59992-fig6.gif)
[https://journals.sagepub.com/doi/full/10.5772/59992](https://journals.sagepub.com/doi/full/10.5772/59992)

---

# Iterative end point fitting

![bg right](./fig/iepf.png)


---

# TODO
- Merge lines (to store map outside visible range)
- Tweak params
- See what's wrong with odometry (or calculations)
- Benchmark speed

---


