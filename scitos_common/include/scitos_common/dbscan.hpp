#pragma once

#include <cstdint>
#include <functional>
#include <set>
#include <vector>


/* 
 * DBSCAN algorithm
 *
 * @param: data - vector with datapoints
 * @param: s - distance function
 * @param n - number of core points
 * @param r - radius to merge to core
 *
 * @return vector with labels. 0 = unlabeled
 */
template <typename T>
std::vector<int> dbscan(const std::vector<T> &data,
                        std::function<float(T, T)> s, uint32_t n, float r) {
  int lbl = 1;
  std::vector<int> lables(0, data.size());
  for (int i = 0; i < data.size(); i++) {
    if (lables[i] != 0)
      continue; // Already classified

    // Create core
    std::set<int> cluster;

    for (int j = 0; j < data.size(); j++) {
      if (i == j)
        continue;
      auto a = data[i];
      auto b = data[j];
      if (s(a, b) < r)
        cluster.insert(j);
    }

    // Check if cluster can be made
    bool merged = cluster.size() > n;
    while (merged) {
      for (int j = 0; j < data.size(); j++) {
        if (lables[j] != 0 || cluster.find(j) == cluster.end()) {
          continue;
        }

        // Check if point can be merged
        auto p = data[j];
        for (int cidx : cluster) {
          auto c = data[cidx];
          if (s(p, c) < r) {
            cluster.insert(j);
            merged = true;
            break;
          }
        }
      }
    }

    if (cluster.size() > n) {
      for (int idx : cluster) {
        lables[idx] = lbl;
      }
      ++lbl;
    }
  }
  return lables;
}
