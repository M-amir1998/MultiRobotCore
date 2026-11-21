#include "multirobotcore/mapf/conflicts.hpp"

#include "conflict_scan.hpp"

namespace multirobotcore::mapf {

std::optional<Conflict> first_conflict(const std::vector<Path>& paths) {
  std::optional<Conflict> found;
  detail::scan_conflicts(
      paths.size(), [&](std::size_t i) -> const Path& { return paths[i]; },
      [&](const Conflict& c) {
        found = c;
        return false;
      });
  return found;
}

std::vector<Conflict> find_conflicts(const std::vector<Path>& paths) {
  std::vector<Conflict> all;
  detail::scan_conflicts(
      paths.size(), [&](std::size_t i) -> const Path& { return paths[i]; },
      [&](const Conflict& c) {
        all.push_back(c);
        return true;
      });
  return all;
}

std::size_t count_conflicts(const std::vector<Path>& paths) {
  std::size_t count = 0;
  detail::scan_conflicts(
      paths.size(), [&](std::size_t i) -> const Path& { return paths[i]; },
      [&](const Conflict& /*c*/) {
        ++count;
        return true;
      });
  return count;
}

bool is_collision_free(const std::vector<Path>& paths) {
  return !first_conflict(paths).has_value();
}

}  // namespace multirobotcore::mapf
