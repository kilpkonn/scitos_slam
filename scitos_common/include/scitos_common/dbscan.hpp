#pragma once

#include <cstdint>
#include <functional>
#include <set>
#include <stdexcept>
#include <vector>

namespace scitos_common {
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
  if (data.size() <= 0)
    return std::vector<int>();

  int lbl = 1;
  std::vector<int> lables(data.size(), 0);
  std::set<size_t> cluster;
  for (size_t i = 0; i < data.size(); i++) {
    if (lables.at(i) != 0)
      continue; // Already classified

    // Create core
    cluster.clear();

    for (size_t j = 0; j < data.size(); j++) {
      if (i == j)
        continue;
      auto a = data.at(i);
      auto b = data.at(j);
      if (s(a, b) < r)
        cluster.insert(j);
    }

    // Check if cluster can be made
    bool merged = cluster.size() > n;
    while (merged) {
      merged = false;
      for (size_t j = 0; j < data.size(); j++) {
        if (lables.at(j) != 0 || cluster.find(j) != cluster.end()) {
          continue;
        }

        // Check if point can be merged
        auto p = data.at(j);
        for (size_t cidx : cluster) {
          auto c = data.at(cidx);
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
        lables.at(idx) = lbl;
      }
      ++lbl;
    }
  }
  return lables;
}

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
std::vector<std::vector<T>> dbscan2(const std::vector<T> &data,
                                    std::function<float(T, T)> s, uint32_t n,
                                    float r) {
  if (data.size() <= 0)
    return {};

  int lbl = 1;
  std::vector<int> lables(data.size(), 0);
  std::vector<std::vector<T>> out;
  std::set<size_t> cluster;
  for (size_t i = 0; i < data.size(); i++) {
    if (lables.at(i) != 0)
      continue; // Already classified

    // Create core
    cluster.clear();

    for (size_t j = 0; j < data.size(); j++) {
      if (i == j)
        continue;
      auto a = data.at(i);
      auto b = data.at(j);
      if (s(a, b) < r)
        cluster.insert(j);
    }

    // Check if cluster can be made
    bool merged = cluster.size() > n;
    while (merged) {
      merged = false;
      for (size_t j = 0; j < data.size(); j++) {
        if (lables.at(j) != 0 || cluster.find(j) != cluster.end()) {
          continue;
        }

        // Check if point can be merged
        auto p = data.at(j);
        for (size_t cidx : cluster) {
          auto c = data.at(cidx);
          if (s(p, c) < r) {
            cluster.insert(j);
            merged = true;
            break;
          }
        }
      }
    }

    if (cluster.size() > n) {
      std::vector<T> tmp;
      tmp.reserve(cluster.size());
      for (int idx : cluster) {
        lables.at(idx) = lbl;
        tmp.push_back(data.at(idx));
      }
      out.push_back(tmp);
      ++lbl;
    }
  }
  return out;
}
} // namespace scitos_common
