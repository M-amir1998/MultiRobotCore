#pragma once

// Shared command-line handling for demo and experiment executables.
//
//   <exe> [--config <file.json>] [--output <result.json>]
//
// Defaults: the repository configuration for the demo and
// results/<name>.json relative to the working directory.

#include "multirobotcore/io/json.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace mrc_apps {

namespace fs = std::filesystem;
using Json = multirobotcore::io::Json;

struct DemoArgs {
  fs::path config;
  fs::path output;
};

inline DemoArgs parse_args(int argc, char** argv, std::string_view name,
                           std::string_view default_config_relative) {
  DemoArgs args{fs::path(MRC_SOURCE_DIR) / fs::path(default_config_relative),
                fs::path("results") / (std::string(name) + ".json")};
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
      args.config = argv[++i];
    } else if ((arg == "--output" || arg == "-o") && i + 1 < argc) {
      args.output = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: " << argv[0] << " [--config file.json] [--output result.json]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + std::string(arg));
    }
  }
  return args;
}

/// Loads the configuration file, or an empty object when the default file is
/// absent (all parameters then take their documented defaults).
inline Json load_config(const DemoArgs& args) {
  if (!fs::exists(args.config)) {
    throw std::runtime_error("configuration not found: " + args.config.string());
  }
  return multirobotcore::io::read_json_file(args.config);
}

/// Standard metadata block written at the top of every result file.
inline Json metadata(std::string_view generator, const Json& config) {
  return Json{{"generator", std::string(generator)},
              {"library", "MultiRobotCore"},
              {"version", "0.1.0"},
              {"config", config}};
}

inline void write_result(const DemoArgs& args, const Json& result) {
  multirobotcore::io::write_json_file(args.output, result);
  std::cout << "wrote " << args.output.string() << '\n';
}

/// Runs main logic with uniform error reporting.
template <typename F>
int run_main(F&& body) {
  try {
    body();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}

/// Wall-clock stopwatch for runtime measurements in experiments.
class Stopwatch {
 public:
  Stopwatch() : start_(std::chrono::steady_clock::now()) {}
  [[nodiscard]] double elapsed_ms() const {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_)
        .count();
  }

 private:
  std::chrono::steady_clock::time_point start_;
};

}  // namespace mrc_apps
