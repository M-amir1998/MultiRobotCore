// Regression tests: fixed inputs with recorded outputs, and a seeded crowd
// scenario with recorded bounds. A failure means the behaviour changed;
// update the recorded values only after reviewing why.

#include "multirobotcore/avoidance/avoidance.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace av = multirobotcore::avoidance;
using multirobotcore::Vec2;

namespace {

struct Snapshot {
  av::AvoidanceMethod method;
  std::vector<Vec2> expected;
};

}  // namespace

TEST(AvoidanceRegression, DecisionSnapshot) {
  multirobotcore::math::ObstacleSet obstacles;
  obstacles.circles.push_back({{1.5, 1.4}, 0.4});
  obstacles.boxes.push_back({{-2.0, -2.2}, {2.0, -1.9}});
  const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.8, 0.1}, 0.3},
                                           {{2.2, 0.3}, {-0.7, 0.0}, 0.3},
                                           {{1.0, -1.2}, {0.1, 0.9}, 0.3},
                                           {{-1.5, 0.9}, {0.5, -0.4}, 0.3}};
  const std::vector<Vec2> pref{{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.8, -0.6}};
  const std::vector<double> speeds{1.0, 1.0, 1.0, 1.0};
  const std::vector<Snapshot> snapshots{
      {av::AvoidanceMethod::kOrca,
       {{0.998292472608956, 0.0584135184036797},
        {-0.966660098204703, 0.210796522662137},
        {0.0747602782401526, 0.683075880543254},
        {0.8, -0.6}}},
      {av::AvoidanceMethod::kReciprocalVelocityObstacle,
       {{0.626461747190606, -0.228013428883779},
        {-0.939692620785908, 0.342020143325669},
        {0.115765451777954, 0.656538502008139},
        {0.8, -0.6}}},
      {av::AvoidanceMethod::kVelocityObstacle,
       {{0.383022221559489, -0.32139380484327},
        {-0.25, -0.433012701892219},
        {0.0, 0.333333333333333},
        {0.8, -0.6}}},
  };
  for (const auto& snapshot : snapshots) {
    av::AvoidanceConfig config;  // library defaults are part of the snapshot
    config.method = snapshot.method;
    const av::CollisionAvoider avoider(config, obstacles);
    const auto v = avoider.compute(agents, pref, speeds);
    ASSERT_EQ(v.size(), snapshot.expected.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
      EXPECT_NEAR(v[i].x(), snapshot.expected[i].x(), 1e-9)
          << av::to_string(snapshot.method) << " " << i;
      EXPECT_NEAR(v[i].y(), snapshot.expected[i].y(), 1e-9)
          << av::to_string(snapshot.method) << " " << i;
    }
  }
}

TEST(AvoidanceRegression, SeededOrcaCrowdSwap) {
  // 12 robots, ring radius 5 m, seeded placement noise and preference noise.
  av::AvoidanceConfig config;
  config.method = av::AvoidanceMethod::kOrca;
  config.time_horizon = 4.0;
  config.neighbor_distance = 8.0;
  config.max_neighbors = 16;
  config.safety_margin = 0.0;
  const av::CollisionAvoider avoider(config);
  std::mt19937_64 rng(2024);
  std::uniform_real_distribution<double> jitter(-0.05, 0.05);
  std::uniform_real_distribution<double> angle(0.0, multirobotcore::math::kTwoPi);
  std::vector<av::AgentState> agents;
  std::vector<Vec2> goals;
  for (int i = 0; i < 12; ++i) {
    const double a = multirobotcore::math::kTwoPi * i / 12.0;
    const Vec2 p = 5.0 * Vec2(std::cos(a), std::sin(a));
    goals.emplace_back(-p);
    agents.push_back({p + Vec2(jitter(rng), jitter(rng)), Vec2::Zero(), 0.3});
  }
  const std::vector<double> speeds(12, 1.0);
  const double dt = config.time_step;
  double min_clearance = std::numeric_limits<double>::infinity();
  double finish = -1.0;
  for (int k = 0; k < 2400 && finish < 0.0; ++k) {
    std::vector<Vec2> pref;
    for (std::size_t i = 0; i < agents.size(); ++i) {
      const double a = angle(rng);
      pref.push_back(av::preferred_velocity(agents[i].position, goals[i], 1.0, 0.5) +
                     0.05 * Vec2(std::cos(a), std::sin(a)));
    }
    const auto v = avoider.compute(agents, pref, speeds);
    bool done = true;
    for (std::size_t i = 0; i < agents.size(); ++i) {
      agents[i].velocity = v[i];
      agents[i].position += dt * v[i];
      done = done && (agents[i].position - goals[i]).norm() < 0.1;
    }
    for (std::size_t i = 0; i < agents.size(); ++i) {
      for (std::size_t j = i + 1; j < agents.size(); ++j) {
        min_clearance =
            std::min(min_clearance, (agents[i].position - agents[j].position).norm() - 0.6);
      }
    }
    if (done) {
      finish = (k + 1) * dt;
    }
  }
  EXPECT_GT(finish, 10.0);  // cannot be faster than the 10 m straight-line distance
  EXPECT_LT(finish, 40.0);
  EXPECT_GE(min_clearance, -1e-3);
}
