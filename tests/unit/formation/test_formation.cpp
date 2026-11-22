#include "multirobotcore/formation/controllers.hpp"
#include "multirobotcore/formation/metrics.hpp"
#include "multirobotcore/formation/shapes.hpp"
#include "multirobotcore/formation/transition.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/topologies.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

namespace f = multirobotcore::formation;
namespace g = multirobotcore::graph;
using multirobotcore::Pose2;
using multirobotcore::Vec2;

namespace {

double nearest_neighbour_distance(const std::vector<Vec2>& d, std::size_t i) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < d.size(); ++j) {
    if (j != i) {
      best = std::min(best, (d[i] - d[j]).norm());
    }
  }
  return best;
}

/// Rank of the rigidity matrix of the framework (graph, positions).
Eigen::Index rigidity_rank(const g::Graph& graph, const std::vector<Vec2>& p) {
  const auto edges = graph.edges();
  Eigen::MatrixXd r = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(edges.size()),
                                            static_cast<Eigen::Index>(2 * p.size()));
  for (std::size_t e = 0; e < edges.size(); ++e) {
    const auto i = edges[e].source;
    const auto j = edges[e].target;
    const Vec2 d = p[i] - p[j];
    const auto row = static_cast<Eigen::Index>(e);
    r.block<1, 2>(row, static_cast<Eigen::Index>(2 * i)) = d.transpose();
    r.block<1, 2>(row, static_cast<Eigen::Index>(2 * j)) = -d.transpose();
  }
  Eigen::FullPivLU<Eigen::MatrixXd> lu(r);
  lu.setThreshold(1e-9);
  return lu.rank();
}

std::vector<Vec2> perturbed(const std::vector<Vec2>& p, double amount, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(-amount, amount);
  std::vector<Vec2> out = p;
  for (Vec2& x : out) {
    x += multirobotcore::math::sample_vec2(rng, u);
  }
  return out;
}

void integrate(std::vector<Vec2>& p, const std::vector<Vec2>& u, double dt) {
  for (std::size_t i = 0; i < p.size(); ++i) {
    p[i] += dt * u[i];
  }
}

}  // namespace

TEST(Shapes, CentredWithRequestedSpacing) {
  for (const f::Shape s : {f::Shape::kLine, f::Shape::kColumn, f::Shape::kTriangle,
                           f::Shape::kSquare, f::Shape::kCircle, f::Shape::kV}) {
    for (std::size_t n : {1U, 2U, 5U, 9U}) {
      const auto d = f::make_shape(s, n, 1.5);
      ASSERT_EQ(d.size(), n);
      Vec2 sum = Vec2::Zero();
      for (const Vec2& x : d) {
        sum += x;
      }
      EXPECT_LT(sum.norm(), 1e-12) << f::to_string(s);
      if (n > 1) {
        for (std::size_t i = 0; i < n; ++i) {
          EXPECT_GE(nearest_neighbour_distance(d, i), 1.5 - 1e-9) << f::to_string(s);
        }
      }
    }
    EXPECT_EQ(f::shape_from_string(f::to_string(s)), s);
  }
  EXPECT_FALSE(f::shape_from_string("hexagon").has_value());
}

TEST(Shapes, CircleAndLineGeometry) {
  const auto circle = f::make_shape(f::Shape::kCircle, 6, 2.0);
  for (const Vec2& d : circle) {
    EXPECT_NEAR(d.norm(), 2.0, 1e-12);  // hexagon: radius equals side length
  }
  const auto line = f::make_shape(f::Shape::kLine, 4, 1.0);
  for (const Vec2& d : line) {
    EXPECT_NEAR(d.x(), 0.0, 1e-12);  // abreast: perpendicular to travel (+x)
  }
  const auto column = f::make_shape(f::Shape::kColumn, 4, 1.0);
  for (const Vec2& d : column) {
    EXPECT_NEAR(d.y(), 0.0, 1e-12);
  }
}

TEST(Shapes, WorldMappingAndLeaderFrame) {
  const Pose2 ref{{1.0, 2.0}, multirobotcore::math::kPi / 2.0};
  EXPECT_TRUE(f::to_world(ref, Vec2(1.0, 0.0)).isApprox(Vec2(1.0, 3.0)));
  const auto d = f::make_shape(f::Shape::kV, 5, 1.0);
  const auto rel = f::relative_to_slot(d, 0);
  EXPECT_TRUE(rel[0].isZero());
  EXPECT_TRUE((rel[3] - rel[1]).isApprox(d[3] - d[1]));
}

TEST(Shapes, HennebergGraphIsMinimallyRigidForGenericShapes) {
  for (const f::Shape s :
       {f::Shape::kTriangle, f::Shape::kV, f::Shape::kCircle, f::Shape::kSquare}) {
    const auto d = f::make_shape(s, 7, 1.0);
    const g::Graph graph = f::henneberg_graph(d);
    EXPECT_EQ(graph.num_edges(), 2U * 7U - 3U);
    EXPECT_TRUE(g::is_connected(graph));
    EXPECT_EQ(rigidity_rank(graph, d), 2 * 7 - 3) << f::to_string(s);
  }
  // Collinear frameworks are not infinitesimally rigid.
  const auto line = f::make_shape(f::Shape::kLine, 5, 1.0);
  EXPECT_LT(rigidity_rank(f::henneberg_graph(line), line), 2 * 5 - 3);
}

TEST(Metrics, AlignmentModes) {
  const auto d = f::make_shape(f::Shape::kTriangle, 6, 1.0);
  const Pose2 ref{{3.0, -1.0}, 0.7};
  const auto p = f::to_world(ref, d);
  const auto rigid = f::formation_error(p, d, f::Alignment::kRigid);
  EXPECT_LT(rigid.rmse, 1e-12);
  EXPECT_NEAR(rigid.rotation, 0.7, 1e-12);
  EXPECT_TRUE(rigid.translation.isApprox(Vec2(3.0, -1.0)));
  EXPECT_LT(f::formation_error(p, d, f::Alignment::kReference, ref).rmse, 1e-12);
  EXPECT_GT(f::formation_error(p, d, f::Alignment::kTranslation).rmse, 0.1);  // rotated
  const g::Graph graph = f::henneberg_graph(d);
  EXPECT_LT(f::distance_error(graph, p, d).max, 1e-12);
  EXPECT_NEAR(f::min_separation(p), 1.0, 1e-12);
  EXPECT_TRUE(f::relative_position_error(p[0], p[1], d[0], d[1], 0.7).isZero(1e-12));
}

TEST(LeaderFollower, StationaryLeaderFollowersReachSlots) {
  const auto d = f::relative_to_slot(f::make_shape(f::Shape::kV, 5, 1.0), 0);
  const Pose2 leader{{2.0, 2.0}, 0.3};
  std::vector<Vec2> p{leader.position, {-3.0, 0.0}, {0.0, 5.0}, {6.0, 1.0}, {4.0, -2.0}};
  const f::LeaderFollowerConfig cfg{1.0, 2.0, true};
  for (int k = 0; k < 600; ++k) {
    integrate(p, f::leader_follower_velocities(leader, Vec2::Zero(), 0, p, d, cfg), 0.02);
  }
  EXPECT_LT(f::formation_error(p, d, f::Alignment::kReference, leader).rmse, 1e-3);
}

TEST(LeaderFollower, MovingLeaderFeedforwardRemovesSteadyStateLag) {
  const auto d = f::relative_to_slot(f::make_shape(f::Shape::kColumn, 3, 1.0), 0);
  const Vec2 v(0.5, 0.0);
  auto run = [&](bool feedforward) {
    Pose2 leader{{0.0, 0.0}, 0.0};
    std::vector<Vec2> p{leader.position, {-1.0, 0.0}, {-2.0, 0.0}};
    const f::LeaderFollowerConfig cfg{1.0, 5.0, feedforward};
    const double dt = 0.01;
    for (int k = 0; k < 2000; ++k) {
      auto u = f::leader_follower_velocities(leader, v, 0, p, d, cfg);
      u[0] = v;
      integrate(p, u, dt);
      leader.position += dt * v;
    }
    return f::formation_error(p, d, f::Alignment::kReference, leader).max_error;
  };
  EXPECT_LT(run(true), 1e-6);
  EXPECT_NEAR(run(false), 0.5, 1e-3);  // lag = |v| / k
}

TEST(ConsensusFormation, ConvergesUpToTranslationAndPreservesCentroid) {
  const auto d = f::make_shape(f::Shape::kCircle, 6, 1.0);
  const g::Graph graph = g::ring_graph(6);
  std::vector<Vec2> p = perturbed(d, 2.0, 3);
  Vec2 centroid0 = Vec2::Zero();
  for (const Vec2& x : p) {
    centroid0 += x / 6.0;
  }
  for (int k = 0; k < 3000; ++k) {
    integrate(
        p,
        f::consensus_formation_velocities(graph, p, d, Pose2{}, Vec2::Zero(), {}, {1.0, 0.0, 10.0}),
        0.02);
  }
  const auto err = f::formation_error(p, d, f::Alignment::kTranslation);
  EXPECT_LT(err.rmse, 1e-6);
  EXPECT_TRUE(err.translation.isApprox(centroid0, 1e-9));
}

TEST(ConsensusFormation, PinningPlacesFormationExactly) {
  const auto d = f::make_shape(f::Shape::kTriangle, 6, 1.0);
  const g::Graph graph = g::path_graph(6);
  std::vector<double> pinning(6, 0.0);
  pinning[0] = 1.0;  // only robot 0 observes the reference
  const Pose2 ref{{5.0, 5.0}, -0.4};
  std::vector<Vec2> p = perturbed(f::to_world(Pose2{}, d), 3.0, 5);
  // Slowest mode: lambda_min(L + B) ~ 2 (1 - cos(pi / 13)) ~ 0.058 for a path
  // pinned at one end, so convergence is slow by design.
  for (int k = 0; k < 12000; ++k) {
    integrate(p,
              f::consensus_formation_velocities(graph, p, d, ref, Vec2::Zero(), pinning,
                                                {2.0, 2.0, 10.0}),
              0.02);
  }
  EXPECT_LT(f::formation_error(p, d, f::Alignment::kReference, ref).rmse, 1e-5);
}

TEST(DistanceFormation, ConvergesLocallyOnRigidGraph) {
  for (const f::Shape s : {f::Shape::kTriangle, f::Shape::kV}) {
    const auto d = f::make_shape(s, 6, 1.0);
    const g::Graph graph = f::henneberg_graph(d);
    std::vector<Vec2> p = perturbed(f::to_world(Pose2{{1.0, 1.0}, 0.2}, d), 0.2, 9);
    for (int k = 0; k < 6000; ++k) {
      integrate(p, f::distance_formation_velocities(graph, p, d, {0.5, 5.0}), 0.01);
    }
    EXPECT_LT(f::distance_error(graph, p, d).max, 1e-4) << f::to_string(s);
    EXPECT_LT(f::formation_error(p, d, f::Alignment::kRigid).rmse, 1e-3) << f::to_string(s);
  }
}

TEST(Transition, AssignmentMinimisesSquaredDistance) {
  std::mt19937_64 rng(4);
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  for (int trial = 0; trial < 50; ++trial) {
    std::vector<Vec2> from(5);
    std::vector<Vec2> to(5);
    for (std::size_t i = 0; i < 5; ++i) {
      from[i] = multirobotcore::math::sample_vec2(rng, u);
      to[i] = multirobotcore::math::sample_vec2(rng, u);
    }
    const auto sigma = f::assign_slots(from, to);
    double cost = 0.0;
    for (std::size_t i = 0; i < 5; ++i) {
      cost += (from[i] - to[sigma[i]]).squaredNorm();
    }
    std::vector<std::size_t> perm(5);
    std::iota(perm.begin(), perm.end(), std::size_t{0});
    double best = std::numeric_limits<double>::infinity();
    do {
      double c = 0.0;
      for (std::size_t i = 0; i < 5; ++i) {
        c += (from[i] - to[perm[i]]).squaredNorm();
      }
      best = std::min(best, c);
    } while (std::next_permutation(perm.begin(), perm.end()));
    EXPECT_NEAR(cost, best, 1e-9);
  }
}

TEST(Transition, SmoothBlendWithPinnedLeader) {
  const auto line = f::make_shape(f::Shape::kLine, 5, 1.0);
  const auto v = f::make_shape(f::Shape::kV, 5, 1.0);
  const f::FormationTransition tr(line, v, 2.0, 4.0, true,
                                  std::pair<std::size_t, std::size_t>{2, 0});
  EXPECT_EQ(tr.assignment()[2], 0U);
  const auto start = tr.offsets_at(0.0);
  const auto end = tr.offsets_at(10.0);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_TRUE(start[i].isApprox(line[i]));
    EXPECT_TRUE(end[i].isApprox(v[tr.assignment()[i]]));
  }
  EXPECT_FALSE(tr.finished(5.9));
  EXPECT_TRUE(tr.finished(6.0));
  // Offset rates match finite differences and vanish at both ends.
  const double t = 3.3;
  const double h = 1e-6;
  const auto a = tr.offsets_at(t - h);
  const auto b = tr.offsets_at(t + h);
  const auto rate = tr.offset_rates_at(t);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_TRUE(rate[i].isApprox((b[i] - a[i]) / (2.0 * h), 1e-5));
    EXPECT_TRUE(tr.offset_rates_at(2.0)[i].isZero());
  }
}
