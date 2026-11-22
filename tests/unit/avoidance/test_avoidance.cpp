#include "multirobotcore/avoidance/avoidance.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace av = multirobotcore::avoidance;
namespace math = multirobotcore::math;
using multirobotcore::Vec2;

namespace {

struct Outcome {
  double min_clearance{std::numeric_limits<double>::infinity()};
  double min_obstacle_clearance{std::numeric_limits<double>::infinity()};
  bool all_reached{false};
  double time{0.0};
};

/// Ideal holonomic agents (exact single-integrator integration), which is the
/// model under which ORCA's guarantees hold. `perturbation` adds a tiny seeded
/// random vector to every preferred velocity to break exact symmetries.
Outcome run(const av::CollisionAvoider& avoider, std::vector<av::AgentState> agents,
            const std::vector<Vec2>& goals, double max_speed, double duration,
            const std::vector<bool>& active = {}, double perturbation = 1e-3) {
  const double dt = avoider.config().time_step;
  const std::vector<double> speeds(agents.size(), max_speed);
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * math::kPi);
  Outcome out;
  for (double t = 0.0; t < duration; t += dt) {
    std::vector<Vec2> pref;
    for (std::size_t i = 0; i < agents.size(); ++i) {
      const double a = angle(rng);
      pref.push_back(av::preferred_velocity(agents[i].position, goals[i], max_speed, 0.5) +
                     perturbation * Vec2(std::cos(a), std::sin(a)));
    }
    const auto v = avoider.compute(agents, pref, speeds, active);
    for (std::size_t i = 0; i < agents.size(); ++i) {
      agents[i].velocity = v[i];
      agents[i].position += dt * v[i];
    }
    for (std::size_t i = 0; i < agents.size(); ++i) {
      for (std::size_t j = i + 1; j < agents.size(); ++j) {
        out.min_clearance =
            std::min(out.min_clearance, (agents[i].position - agents[j].position).norm() -
                                            agents[i].radius - agents[j].radius);
      }
      for (const auto& d : avoider.discs()) {
        out.min_obstacle_clearance =
            std::min(out.min_obstacle_clearance,
                     (agents[i].position - d.center).norm() - d.radius - agents[i].radius);
      }
    }
    out.time = t + dt;
    bool reached = true;
    for (std::size_t i = 0; i < agents.size(); ++i) {
      if (!active.empty() && !active[i]) {
        continue;
      }
      reached = reached && (agents[i].position - goals[i]).norm() < 0.05;
    }
    if (reached) {
      out.all_reached = true;
      break;
    }
  }
  return out;
}

av::AvoidanceConfig config(av::AvoidanceMethod method) {
  av::AvoidanceConfig c;
  c.method = method;
  c.time_step = 0.05;
  c.time_horizon = 2.0;
  c.neighbor_distance = 6.0;
  c.max_neighbors = 10;
  c.safety_margin = 0.0;
  return c;
}

}  // namespace

TEST(Avoidance, MethodNamesRoundTrip) {
  for (const auto m :
       {av::AvoidanceMethod::kNone, av::AvoidanceMethod::kVelocityObstacle,
        av::AvoidanceMethod::kReciprocalVelocityObstacle, av::AvoidanceMethod::kOrca}) {
    EXPECT_EQ(av::parse_method(av::to_string(m)), m);
  }
  EXPECT_THROW((void)av::parse_method("orca2"), std::invalid_argument);
}

TEST(Avoidance, PreferredVelocitySlowsDownNearGoal) {
  EXPECT_TRUE(av::preferred_velocity({0.0, 0.0}, {10.0, 0.0}, 2.0, 0.5).isApprox(Vec2(2.0, 0.0)));
  EXPECT_TRUE(av::preferred_velocity({0.0, 0.0}, {0.25, 0.0}, 2.0, 0.5).isApprox(Vec2(1.0, 0.0)));
  EXPECT_EQ(av::preferred_velocity({1.0, 1.0}, {1.0, 1.0}, 2.0), Vec2::Zero());
}

TEST(Avoidance, BoxDiscsCoverTheBoxConservatively) {
  math::ObstacleSet obstacles;
  obstacles.boxes.push_back({{0.0, 0.0}, {2.0, 0.3}});
  obstacles.boxes.push_back({{-3.0, -3.0}, {-1.0, -1.0}});
  const auto discs = av::obstacle_discs(obstacles, 0.25);
  for (const auto& box : obstacles.boxes) {
    // Every boundary point of the box is covered by some disc.
    for (double s = 0.0; s <= 1.0; s += 0.01) {
      for (const Vec2& p : {Vec2(box.min.x() + s * box.width(), box.min.y()),
                            Vec2(box.min.x() + s * box.width(), box.max.y()),
                            Vec2(box.min.x(), box.min.y() + s * box.height()),
                            Vec2(box.max.x(), box.min.y() + s * box.height())}) {
        const bool covered = std::any_of(discs.begin(), discs.end(), [&](const math::Circle& c) {
          return (c.center - p).norm() <= c.radius + 1e-12;
        });
        EXPECT_TRUE(covered);
      }
    }
  }
  for (const auto& c : discs) {
    EXPECT_LE(c.radius, 0.5 * std::hypot(0.25, 0.25) + 1e-12);
  }
}

TEST(Avoidance, ExactlySymmetricHeadOnSwapIsAnOrcaEquilibrium) {
  // Known limitation: on an exactly collinear approach the ORCA constraint is
  // parallel to the line of centres, so no lateral motion ever arises and the
  // agents stop in front of each other (safely). Perturbations resolve it.
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kOrca));
  const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.0, 0.0}, 0.3},
                                           {{6.0, 0.0}, {0.0, 0.0}, 0.3}};
  const auto out = run(avoider, agents, {{6.0, 0.0}, {0.0, 0.0}}, 1.0, 20.0, {}, 0.0);
  EXPECT_FALSE(out.all_reached);
  EXPECT_GE(out.min_clearance, -1e-9);
}

TEST(Avoidance, HeadOnSwapIsCollisionFreeWithOrca) {
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kOrca));
  const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.0, 0.0}, 0.3},
                                           {{6.0, 0.0}, {0.0, 0.0}, 0.3}};
  const auto out = run(avoider, agents, {{6.0, 0.0}, {0.0, 0.0}}, 1.0, 30.0);
  EXPECT_TRUE(out.all_reached);
  EXPECT_GE(out.min_clearance, -1e-9);
}

TEST(Avoidance, ReciprocalMethodsResolveHeadOnSwap) {
  for (const auto m :
       {av::AvoidanceMethod::kReciprocalVelocityObstacle, av::AvoidanceMethod::kOrca}) {
    const av::CollisionAvoider avoider(config(m));
    // Slightly asymmetric start breaks the exact head-on symmetry.
    const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.0, 0.0}, 0.3},
                                             {{6.0, 0.05}, {0.0, 0.0}, 0.3}};
    const auto out = run(avoider, agents, {{6.0, 0.0}, {0.0, 0.0}}, 1.0, 30.0);
    EXPECT_TRUE(out.all_reached) << av::to_string(m);
    EXPECT_GE(out.min_clearance, -0.05) << av::to_string(m);
  }
}

TEST(Avoidance, NoAvoidanceCollides) {
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kNone));
  const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.0, 0.0}, 0.3},
                                           {{6.0, 0.0}, {0.0, 0.0}, 0.3}};
  const auto out = run(avoider, agents, {{6.0, 0.0}, {0.0, 0.0}}, 1.0, 30.0);
  EXPECT_LT(out.min_clearance, -0.5);
}

TEST(Avoidance, InactiveRobotIsAvoidedAndHeld) {
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kOrca));
  const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.0, 0.0}, 0.3},
                                           {{3.0, 0.02}, {0.0, 0.0}, 0.3}};
  const std::vector<bool> active{true, false};
  const auto out = run(avoider, agents, {{6.0, 0.0}, {3.0, 0.02}}, 1.0, 30.0, active);
  EXPECT_TRUE(out.all_reached);
  EXPECT_GE(out.min_clearance, -1e-6);
  const std::vector<Vec2> pref{{1.0, 0.0}, {1.0, 0.0}};
  const std::vector<double> speeds{1.0, 1.0};
  EXPECT_EQ(avoider.compute(agents, pref, speeds, active)[1], Vec2::Zero());
}

TEST(Avoidance, StaticObstacleIsAvoided) {
  math::ObstacleSet obstacles;
  obstacles.circles.push_back({{3.0, 0.05}, 0.6});
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kOrca), obstacles);
  const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.0, 0.0}, 0.3}};
  const auto out = run(avoider, agents, {{6.0, 0.0}}, 1.0, 30.0);
  EXPECT_TRUE(out.all_reached);
  EXPECT_GE(out.min_obstacle_clearance, -1e-6);
}

TEST(Avoidance, CircleSwapOfEightRobotsWithOrca) {
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kOrca));
  std::vector<av::AgentState> agents;
  std::vector<Vec2> goals;
  // Realistic placement noise; the exactly symmetric configuration resolves
  // only through the tiny preference perturbation and takes much longer.
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> jitter(-0.05, 0.05);
  for (int i = 0; i < 8; ++i) {
    const double a = 2.0 * math::kPi * i / 8.0;
    const Vec2 p = 4.0 * Vec2(std::cos(a), std::sin(a));
    goals.emplace_back(-p);
    agents.push_back({p + math::sample_vec2(rng, jitter), Vec2::Zero(), 0.3});
  }
  const auto out = run(avoider, agents, goals, 1.0, 90.0);
  EXPECT_TRUE(out.all_reached);
  EXPECT_GE(out.min_clearance, -1e-6);
}

TEST(Avoidance, ComputeIsDeterministic) {
  for (const auto m :
       {av::AvoidanceMethod::kVelocityObstacle, av::AvoidanceMethod::kReciprocalVelocityObstacle,
        av::AvoidanceMethod::kOrca}) {
    const av::CollisionAvoider avoider(config(m));
    const std::vector<av::AgentState> agents{{{0.0, 0.0}, {0.5, 0.0}, 0.3},
                                             {{2.0, 0.3}, {-0.5, 0.0}, 0.3},
                                             {{1.0, -1.5}, {0.0, 0.6}, 0.3}};
    const std::vector<Vec2> pref{{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}};
    const std::vector<double> speeds{1.0, 1.0, 1.0};
    EXPECT_EQ(avoider.compute(agents, pref, speeds), avoider.compute(agents, pref, speeds));
  }
}

TEST(Avoidance, DiagnosticsDescribeEveryNeighbour) {
  math::ObstacleSet obstacles;
  obstacles.circles.push_back({{0.0, 2.0}, 0.5});
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kOrca), obstacles);
  const std::vector<av::AgentState> agents{{{0.0, 0.0}, {1.0, 0.0}, 0.3},
                                           {{2.0, 0.0}, {-1.0, 0.0}, 0.3}};
  const std::vector<Vec2> pref{{1.0, 0.0}, {-1.0, 0.0}};
  const std::vector<double> speeds{1.0, 1.0};
  const auto d = avoider.diagnose(0, agents, pref, speeds);
  ASSERT_EQ(d.neighbors.size(), 2U);
  EXPECT_FALSE(d.neighbors[0].is_obstacle);
  EXPECT_TRUE(d.neighbors[0].is_robot);
  EXPECT_TRUE(d.neighbors[1].is_obstacle);
  EXPECT_FALSE(d.neighbors[1].is_robot);
  EXPECT_EQ(d.selected, avoider.compute(agents, pref, speeds)[0]);
  EXPECT_THROW((void)avoider.diagnose(5, agents, pref, speeds), std::invalid_argument);
}

TEST(Avoidance, SizeMismatchThrows) {
  const av::CollisionAvoider avoider(config(av::AvoidanceMethod::kOrca));
  const std::vector<av::AgentState> agents(2);
  const std::vector<Vec2> pref(1);
  const std::vector<double> speeds(2, 1.0);
  EXPECT_THROW((void)avoider.compute(agents, pref, speeds), std::invalid_argument);
  av::AvoidanceConfig bad;
  bad.time_horizon = 0.0;
  EXPECT_THROW(av::CollisionAvoider{bad}, std::invalid_argument);
}
