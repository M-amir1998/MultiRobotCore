// Task-allocation demonstrations:
//   assignment  optimal (Hungarian) vs. greedy nearest-pair assignment, then
//               execution in the simulator
//   auction     traced synchronous (Jacobi) auction rounds: bids and prices
//   dynamic     tasks arrive and are cancelled during execution; the dynamic
//               allocator re-plans with commitment and a switching penalty
//
// Output: JSON consumed by scripts/render/render_allocation.py.
//
// mrc-modules: allocation simulation io

#include "demo_common.hpp"

#include "multirobotcore/allocation/auction.hpp"
#include "multirobotcore/allocation/dynamic.hpp"
#include "multirobotcore/allocation/hungarian.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"
#include "multirobotcore/simulation/events.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace mrc = multirobotcore;
namespace io = multirobotcore::io;
namespace al = multirobotcore::allocation;
using mrc_apps::Json;

namespace {

std::vector<mrc::Vec2> scatter(std::size_t n, const mrc::math::AABB& box, double min_spacing,
                               std::mt19937_64& rng) {
  std::uniform_real_distribution<double> ux(box.min.x() + 0.5, box.max.x() - 0.5);
  std::uniform_real_distribution<double> uy(box.min.y() + 0.5, box.max.y() - 0.5);
  std::vector<mrc::Vec2> out;
  while (out.size() < n) {
    const mrc::Vec2 p = mrc::math::sample_vec2(rng, ux, uy);
    if (std::all_of(out.begin(), out.end(),
                    [&](const mrc::Vec2& q) { return (q - p).norm() >= min_spacing; })) {
      out.push_back(p);
    }
  }
  return out;
}

/// Greedy baseline: repeatedly pick the globally cheapest remaining pair.
al::Assignment greedy_assignment(const al::CostMatrix& c) {
  const auto n = static_cast<std::size_t>(c.rows());
  const auto m = static_cast<std::size_t>(c.cols());
  std::vector<std::optional<std::size_t>> robot_to_task(n);
  std::vector<bool> robot_used(n, false);
  std::vector<bool> task_used(m, false);
  for (std::size_t k = 0; k < std::min(n, m); ++k) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t bi = 0;
    std::size_t bj = 0;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < m; ++j) {
        const double v = c(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
        if (!robot_used[i] && !task_used[j] && v < best) {
          best = v;
          bi = i;
          bj = j;
        }
      }
    }
    robot_used[bi] = task_used[bj] = true;
    robot_to_task[bi] = bj;
  }
  return al::make_assignment(c, robot_to_task);
}

Json assignment_json(const al::Assignment& a) {
  Json out = Json::array();
  for (const auto& t : a.robot_to_task) {
    out.push_back(t ? Json(*t) : Json(nullptr));
  }
  return out;
}

Json matrix_json(const al::CostMatrix& c) {
  Json out = Json::array();
  for (Eigen::Index i = 0; i < c.rows(); ++i) {
    Json row = Json::array();
    for (Eigen::Index j = 0; j < c.cols(); ++j) {
      row.push_back(io::round_to(c(i, j), 3));
    }
    out.push_back(std::move(row));
  }
  return out;
}

/// Drives every robot straight to its assigned task and records the run.
Json execute(const std::vector<mrc::Vec2>& robots, const std::vector<mrc::Vec2>& tasks,
             const al::Assignment& assignment, const Json& cfg) {
  mrc::simulation::WorldConfig world;
  world.dt = cfg.value("dt", 0.05);
  world.record_every = 2;
  mrc::simulation::RobotSpec spec;
  spec.radius = cfg.value("robot_radius", 0.22);
  spec.max_speed = cfg.value("max_speed", 1.0);
  std::vector<mrc::Pose2> poses;
  for (const auto& p : robots) {
    poses.push_back({p, 0.0});
  }
  mrc::simulation::Simulator sim(
      world, std::vector<mrc::simulation::RobotSpec>(robots.size(), spec), poses);
  const auto steps =
      static_cast<std::size_t>(std::llround(cfg.value("execution_time", 14.0) / world.dt));
  for (std::size_t k = 0; k < steps; ++k) {
    std::vector<mrc::Vec2> u(robots.size(), mrc::Vec2::Zero());
    for (std::size_t i = 0; i < robots.size(); ++i) {
      if (const auto t = assignment.robot_to_task[i]) {
        u[i] = mrc::math::clamp_norm(2.0 * (tasks[*t] - sim.robot(i).position), spec.max_speed);
      }
    }
    sim.step(u);
  }
  return Json{{"trajectory", io::frames_to_json(sim.frames())},
              {"metrics", io::to_json(sim.metrics())}};
}

Json dynamic_scenario(const Json& cfg, const Json& root, std::mt19937_64& rng) {
  const auto field = io::aabb_from_json(root.at("field"));
  const auto n = cfg.value("num_robots", std::size_t{6});
  const double service = cfg.value("service_time", 1.5);
  const double radius = root.value("robot_radius", 0.22);
  const double vmax = root.value("max_speed", 1.0);

  al::DynamicAllocatorConfig acfg;
  acfg.switching_penalty = cfg.value("switching_penalty", 1.0);
  acfg.commit_distance = cfg.value("commit_distance", 1.0);
  al::DynamicTaskAllocator allocator(acfg);

  const auto robot_pos = scatter(n, field, 1.0, rng);
  const auto initial = scatter(cfg.value("initial_tasks", std::size_t{6}), field, 1.0, rng);
  Json tasks_json = Json::array();
  auto add_task = [&](const mrc::Vec2& p, double t) {
    al::Task task;
    task.position = p;
    const std::size_t id = allocator.add_task(task);
    tasks_json.push_back(Json{{"id", id}, {"position", io::to_json(p)}, {"arrival", t}});
    return id;
  };
  for (const auto& p : initial) {
    add_task(p, 0.0);
  }
  mrc::simulation::EventSchedule schedule;
  for (const auto& a : cfg.at("arrivals")) {
    mrc::simulation::ScheduledEvent e;
    e.time = a.at("time").get<double>();
    e.type = mrc::simulation::EventType::kTaskArrival;
    e.position = io::vec2_from_json(a.at("position"));
    schedule.add(e);
  }
  for (const auto& c : cfg.at("cancellations")) {
    mrc::simulation::ScheduledEvent e;
    e.time = c.at("time").get<double>();
    e.type = mrc::simulation::EventType::kTaskCancelled;
    e.task = c.at("task").get<std::size_t>();
    schedule.add(e);
  }

  mrc::simulation::WorldConfig world;
  world.dt = root.value("dt", 0.05);
  world.record_every = 2;
  world.bounds = field;
  mrc::simulation::RobotSpec spec;
  spec.radius = radius;
  spec.max_speed = vmax;
  std::vector<mrc::Pose2> poses;
  for (const auto& p : robot_pos) {
    poses.push_back({p, 0.0});
  }
  mrc::simulation::Simulator sim(world, std::vector<mrc::simulation::RobotSpec>(n, spec), poses);

  std::vector<std::optional<std::size_t>> current(n);
  std::vector<double> service_left(n, 0.0);
  Json events = Json::array();
  Json robot_tasks = Json::array();  // per recorded frame: task id per robot
  Json open_tasks = Json::array();   // per recorded frame: open task ids
  std::size_t completed = 0;
  std::size_t reassignments = 0;
  double sum_completion = 0.0;
  bool dirty = true;
  auto robots_now = [&] {
    std::vector<al::Robot> rs(n);
    for (std::size_t i = 0; i < n; ++i) {
      rs[i].id = i;
      rs[i].position = sim.robot(i).position;
      rs[i].speed = vmax;
    }
    return rs;
  };
  auto record = [&] {
    Json rt = Json::array();
    for (const auto& c : current) {
      rt.push_back(c ? Json(*c) : Json(nullptr));
    }
    robot_tasks.push_back(std::move(rt));
    Json open = Json::array();
    for (const auto& t : allocator.open_tasks()) {
      open.push_back(t.id);
    }
    open_tasks.push_back(std::move(open));
  };
  record();
  const auto steps = static_cast<std::size_t>(std::llround(cfg.value("duration", 32.0) / world.dt));
  for (std::size_t k = 0; k < steps; ++k) {
    const double t = sim.time();
    for (const auto& e : schedule.pop_due(t)) {
      if (e.type == mrc::simulation::EventType::kTaskArrival) {
        const std::size_t id = add_task(e.position, e.time);
        events.push_back(Json{{"time", io::round_to(t, 2)}, {"type", "arrival"}, {"task", id}});
      } else if (allocator.cancel_task(e.task)) {
        events.push_back(
            Json{{"time", io::round_to(t, 2)}, {"type", "cancelled"}, {"task", e.task}});
      }
      dirty = true;
    }
    if (dirty) {
      const auto update = allocator.update(robots_now(), current);
      reassignments += update.reassigned;
      current = update.robot_task;
      events.push_back(Json{{"time", io::round_to(t, 2)},
                            {"type", "reallocation"},
                            {"reassigned", update.reassigned},
                            {"newly_assigned", update.newly_assigned},
                            {"released", update.released}});
      dirty = false;
    }
    std::vector<mrc::Vec2> u(n, mrc::Vec2::Zero());
    for (std::size_t i = 0; i < n; ++i) {
      if (!current[i]) {
        continue;
      }
      const auto task = allocator.find_task(*current[i]);
      if (!task) {
        current[i].reset();
        dirty = true;
        continue;
      }
      const mrc::Vec2 d = task->position - sim.robot(i).position;
      if (d.norm() < 0.1) {
        service_left[i] += world.dt;
        if (service_left[i] >= service) {
          allocator.complete_task(*current[i]);
          sum_completion += t;
          ++completed;
          events.push_back(Json{{"time", io::round_to(t, 2)},
                                {"type", "completed"},
                                {"task", *current[i]},
                                {"robot", i}});
          current[i].reset();
          service_left[i] = 0.0;
          dirty = true;
        }
      } else {
        u[i] = mrc::math::clamp_norm(2.0 * d, vmax);
      }
    }
    sim.step(u);
    if (sim.step_count() % world.record_every == 0) {
      record();
    }
  }
  return Json{
      {"robots", io::to_json(std::span<const mrc::Vec2>(robot_pos))},
      {"tasks", tasks_json},
      {"events", events},
      {"robot_tasks", robot_tasks},
      {"open_tasks", open_tasks},
      {"trajectory", io::frames_to_json(sim.frames())},
      {"summary",
       {{"completed", completed},
        {"reassignments", reassignments},
        {"mean_completion_time",
         io::round_to(completed > 0 ? sum_completion / static_cast<double>(completed) : 0.0, 3)},
        {"open_at_end", allocator.open_tasks().size()}}},
      {"metrics", io::to_json(sim.metrics())}};
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "allocation", "configs/allocation/allocation.json");
    const Json cfg = mrc_apps::load_config(args);
    std::mt19937_64 rng(cfg.value("seed", std::uint64_t{23}));
    const auto field = io::aabb_from_json(cfg.at("field"));
    const auto robots = scatter(cfg.value("num_robots", std::size_t{8}), field, 1.0, rng);
    const auto task_pos = scatter(cfg.value("num_tasks", std::size_t{8}), field, 1.0, rng);

    std::vector<al::Robot> rs;
    for (std::size_t i = 0; i < robots.size(); ++i) {
      rs.push_back(al::Robot{i, robots[i]});
    }
    std::vector<al::Task> ts;
    for (std::size_t j = 0; j < task_pos.size(); ++j) {
      al::Task t;
      t.id = j;
      t.position = task_pos[j];
      ts.push_back(t);
    }
    const al::CostMatrix costs = al::euclidean_costs(rs, ts);
    const al::Assignment optimal = al::solve_hungarian(costs);
    const al::Assignment greedy = greedy_assignment(costs);
    std::vector<al::AuctionRound> trace;
    const al::AuctionResult auction =
        al::solve_auction_traced(costs, cfg.value("auction_epsilon", 0.4), trace);
    std::cout << "hungarian cost " << optimal.total_cost << ", greedy " << greedy.total_cost
              << ", auction " << auction.assignment.total_cost << " in " << auction.rounds
              << " rounds\n";

    Json rounds = Json::array();
    for (const auto& r : trace) {
      Json bids = Json::array();
      for (const auto& b : r.bids) {
        if (b.task < task_pos.size()) {
          bids.push_back(Json::array({b.robot, b.task, io::round_to(b.price, 3)}));
        }
      }
      Json owner = Json::array();
      for (const auto& o : r.owner) {
        owner.push_back(o ? Json(*o) : Json(nullptr));
      }
      Json prices = Json::array();
      for (const double p : r.prices) {
        prices.push_back(io::round_to(p, 3));
      }
      rounds.push_back(Json{{"bids", bids}, {"owner", owner}, {"prices", prices}});
    }

    Json result = mrc_apps::metadata("mrc_demo_allocation", cfg);
    result["field"] = io::to_json(field);
    result["robots"] = io::to_json(std::span<const mrc::Vec2>(robots));
    result["tasks"] = io::to_json(std::span<const mrc::Vec2>(task_pos));
    result["costs"] = matrix_json(costs);
    result["hungarian"] = Json{{"assignment", assignment_json(optimal)},
                               {"cost", io::round_to(optimal.total_cost, 4)},
                               {"execution", execute(robots, task_pos, optimal, cfg)}};
    result["greedy"] =
        Json{{"assignment", assignment_json(greedy)}, {"cost", io::round_to(greedy.total_cost, 4)}};
    result["auction"] = Json{{"assignment", assignment_json(auction.assignment)},
                             {"cost", io::round_to(auction.assignment.total_cost, 4)},
                             {"epsilon", auction.epsilon},
                             {"rounds", auction.rounds},
                             {"bids", auction.bids},
                             {"messages", auction.messages},
                             {"trace", rounds},
                             {"execution", execute(robots, task_pos, auction.assignment, cfg)}};
    result["dynamic"] = dynamic_scenario(cfg.at("dynamic"), cfg, rng);
    mrc_apps::write_result(args, result);
  });
}
