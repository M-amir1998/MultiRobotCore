#include "multirobotcore/io/json.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

namespace multirobotcore::io {

double round_to(double value, int decimals) {
  if (!std::isfinite(value)) {
    return value;
  }
  const double scale = std::pow(10.0, decimals);
  const double rounded = std::round(value * scale) / scale;
  return rounded == 0.0 ? 0.0 : rounded;  // normalise -0.0
}

Json to_json(const Vec2& v, int decimals) {
  return Json::array({round_to(v.x(), decimals), round_to(v.y(), decimals)});
}

Json to_json(const Pose2& pose, int decimals) {
  return Json::array({round_to(pose.position.x(), decimals), round_to(pose.position.y(), decimals),
                      round_to(pose.heading, decimals)});
}

Json to_json(std::span<const Vec2> points, int decimals) {
  Json out = Json::array();
  for (const Vec2& p : points) {
    out.push_back(to_json(p, decimals));
  }
  return out;
}

Json to_json(const math::Circle& circle, int decimals) {
  return Json{{"center", to_json(circle.center, decimals)},
              {"radius", round_to(circle.radius, decimals)}};
}

Json to_json(const math::AABB& box, int decimals) {
  return Json{{"min", to_json(box.min, decimals)}, {"max", to_json(box.max, decimals)}};
}

Json to_json(const math::ObstacleSet& obstacles, int decimals) {
  Json circles = Json::array();
  for (const auto& c : obstacles.circles) {
    circles.push_back(to_json(c, decimals));
  }
  Json boxes = Json::array();
  for (const auto& b : obstacles.boxes) {
    boxes.push_back(to_json(b, decimals));
  }
  return Json{{"circles", circles}, {"boxes", boxes}};
}

Json to_json(const graph::Graph& graph) {
  Json edges = Json::array();
  for (const graph::Edge& e : graph.edges()) {
    edges.push_back(Json::array({e.source, e.target, round_to(e.weight, 6)}));
  }
  return Json{
      {"directed", graph.is_directed()}, {"num_nodes", graph.num_nodes()}, {"edges", edges}};
}

Json to_json(const communication::NetworkStats& stats) {
  return Json{{"messages_sent", stats.messages_sent},
              {"messages_delivered", stats.messages_delivered},
              {"dropped_loss", stats.dropped_loss},
              {"dropped_range", stats.dropped_range},
              {"dropped_outage", stats.dropped_outage},
              {"bytes_sent", stats.bytes_sent},
              {"bytes_delivered", stats.bytes_delivered},
              {"rounds", stats.rounds},
              {"delivery_ratio", round_to(stats.delivery_ratio(), 6)}};
}

namespace {

Json finite_or_null(double value, int decimals) {
  return std::isfinite(value) ? Json(round_to(value, decimals)) : Json(nullptr);
}

}  // namespace

Json to_json(const simulation::SimulationMetrics& metrics) {
  Json distance = Json::array();
  for (const double d : metrics.distance_travelled) {
    distance.push_back(round_to(d, 4));
  }
  Json effort = Json::array();
  for (const double e : metrics.control_effort) {
    effort.push_back(round_to(e, 4));
  }
  return Json{{"robot_collisions", metrics.robot_collisions},
              {"obstacle_collisions", metrics.obstacle_collisions},
              {"min_pair_clearance", finite_or_null(metrics.min_pair_clearance, 5)},
              {"min_obstacle_clearance", finite_or_null(metrics.min_obstacle_clearance, 5)},
              {"distance_travelled", distance},
              {"control_effort", effort}};
}

Json frames_to_json(const std::vector<simulation::Frame>& frames, int decimals) {
  Json t = Json::array();
  Json positions = Json::array();
  Json headings = Json::array();
  Json status = Json::array();
  for (const simulation::Frame& f : frames) {
    t.push_back(round_to(f.time, 4));
    Json p = Json::array();
    Json h = Json::array();
    Json s = Json::array();
    for (std::size_t i = 0; i < f.poses.size(); ++i) {
      p.push_back(to_json(f.poses[i].position, decimals));
      h.push_back(round_to(f.poses[i].heading, 3));
      s.push_back(std::string(models::to_string(f.status[i])));
    }
    positions.push_back(std::move(p));
    headings.push_back(std::move(h));
    status.push_back(std::move(s));
  }
  return Json{{"t", t}, {"positions", positions}, {"headings", headings}, {"status", status}};
}

Vec2 vec2_from_json(const Json& value) {
  if (!value.is_array() || value.size() != 2) {
    throw std::invalid_argument("vec2_from_json: expected [x, y]");
  }
  return Vec2(value[0].get<double>(), value[1].get<double>());
}

std::vector<Vec2> vec2_list_from_json(const Json& value) {
  std::vector<Vec2> out;
  for (const auto& v : value) {
    out.push_back(vec2_from_json(v));
  }
  return out;
}

Json read_json_file(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("read_json_file: cannot open " + path.string());
  }
  try {
    return Json::parse(in);
  } catch (const nlohmann::json::exception& e) {
    throw std::runtime_error("read_json_file: " + path.string() + ": " + e.what());
  }
}

void write_json_file(const std::filesystem::path& path, const Json& value, int indent) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("write_json_file: cannot open " + path.string());
  }
  out << value.dump(indent) << '\n';
  if (!out) {
    throw std::runtime_error("write_json_file: write failed for " + path.string());
  }
}

}  // namespace multirobotcore::io
