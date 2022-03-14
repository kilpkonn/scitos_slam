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

- Finish mapping
- Localization overview

---

# Mapping
- Fix "raycast"
- Added saving to `yaml`
- Better lines merge (inspired by paper we found for EKF)
- Connect corners
- Attempt to make lines more parallel

---

# Connecting corners
- **Assumption: No gap less than 10 cm**
- DBScan on line end points (unknown nr of corners)
- Sanity checks for
  - Short lines
  - Messy corners 

---

# Parallel lines
- **Assumption: Similar walls indoors are alligned**
- K-Means clustering on line headings (2-4 main headings)
- Global lines
- Complex space for angle averaging

---

![bg 100%](./fig/mapv2.png)

---

# Localization

- Focus on algorithms suitable for feature maps
- Monte Carlo Localization
- Extended Kalman Filter localization

---

# Monte Carlo Localization

- Uses particle filter
- Works easily with both local and global localization
- Adaptive Monte Carlo Localization (AMCL) for improvement

---

# Grid based Monte Carlo Localization

- Uses histogram
- better at solving kidnapped robot
- sensitive to grid resolution

---

# Extended Kalman Filter

- Faster for maps with few features
- Needs wrapper for global localization (kidnapping problem etc.)

---

# Final choice

- EKF seems to naturally fit for our feature based map
_(Have not yet decided)_

---

