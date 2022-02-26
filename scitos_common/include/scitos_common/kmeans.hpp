#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

/**
 * KMeans algorithm
 *
 * @param data - vector with datapoints
 * @param k - amount of clusters
 * @param s - distance function
 * @param n - amount of iterations to run (stopping criteria)
 *
 * @return centroids
 */
template <typename T>
std::vector<T> kmeans(const std::vector<T> &data, uint32_t k,
                      std::function<float(T, T)> &&s, uint32_t n) {
  const int nrow = data.size();

  std::vector<T> centroids;
  centroids.reserve(k);
  std::vector<std::vector<float>> distances(nrow);

  for (uint32_t i = 0; i < k; i++) {
    centroids.push_back(data[nrow / k * i]);
  }
  for (uint32_t i = 0; i < nrow; i++) {
    distances[i] = std::vector<float>(k);
  }

  for (uint32_t i = 0; i < n; i++) {
    for (int j = 0; j < nrow; j++) {
      for (uint32_t w = 0; w < k; w++) {
        auto centroid = centroids[w];
        auto p = data[j];
        distances[j][w] = s(p, centroid);
      }
    }

    std::vector<int> cluster_idxes;
    for (uint32_t j = 0; j < nrow; j++) {
      cluster_idxes[j] =
          std::min_element(distances[j].begin(), distances[j].end()) -
          distances[j].begin();
    }

    for (int i = 0; i < centroids.size(); i++)
      centroids[i] = T();

    std::vector<int> centroid_counts(k);
    for (int i = 0; i < nrow; i++) {
      centroids[cluster_idxes[i]] += data[i];
      centroid_counts[cluster_idxes[i]] += 1;
    }

    for (uint32_t w = 0; w < k; w++) {
      centroids[w] = centroids[w] / centroid_counts[w];
    }
  }

  return centroids;
}
