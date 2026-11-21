#pragma once

// Helpers shared by experiment runners: sample statistics over seeds and a
// uniform JSON row format consumed by scripts/benchmark/.

#include "multirobotcore/io/json.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace mrc_apps {

struct Sample {
  std::vector<double> values;

  void add(double v) { values.push_back(v); }
  [[nodiscard]] double mean() const {
    return values.empty() ? 0.0
                          : std::accumulate(values.begin(), values.end(), 0.0) /
                                static_cast<double>(values.size());
  }
  [[nodiscard]] double stddev() const {
    if (values.size() < 2) {
      return 0.0;
    }
    const double m = mean();
    double acc = 0.0;
    for (const double v : values) {
      acc += (v - m) * (v - m);
    }
    return std::sqrt(acc / static_cast<double>(values.size() - 1));
  }
  [[nodiscard]] double max() const {
    return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
  }
  [[nodiscard]] double min() const {
    return values.empty() ? 0.0 : *std::min_element(values.begin(), values.end());
  }
  [[nodiscard]] multirobotcore::io::Json json(int decimals = 4) const {
    return multirobotcore::io::Json{{"mean", multirobotcore::io::round_to(mean(), decimals)},
                                    {"std", multirobotcore::io::round_to(stddev(), decimals)},
                                    {"min", multirobotcore::io::round_to(min(), decimals)},
                                    {"max", multirobotcore::io::round_to(max(), decimals)},
                                    {"n", values.size()}};
  }
};

}  // namespace mrc_apps
