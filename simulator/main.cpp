#include "controller_sim.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void usage(const char* program) {
  std::cerr << "Usage: " << program
            << " --scenario FILE [--duration MS] [--trace FILE] [--quiet]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario_path;
  std::string trace_path;
  uint64_t duration_ms = 10000;
  bool quiet = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scenario" && i + 1 < argc) scenario_path = argv[++i];
    else if (arg == "--duration" && i + 1 < argc) {
      const std::string text = argv[++i];
      try {
        std::size_t used = 0;
        if (text.empty() || text.front() == '-') {
          throw std::invalid_argument("invalid");
        }
        duration_ms = std::stoull(text, &used);
        if (used != text.size()) throw std::invalid_argument("trailing");
      } catch (const std::exception&) {
        std::cerr << "eppg-sim: invalid --duration '" << text << "'\n";
        return 2;
      }
    }
    else if (arg == "--trace" && i + 1 < argc) trace_path = argv[++i];
    else if (arg == "--quiet") quiet = true;
    else {
      usage(argv[0]);
      return 2;
    }
  }
  if (scenario_path.empty()) {
    usage(argv[0]);
    return 2;
  }

  try {
    eppg::sim::ControllerSim simulator;
    for (auto& event : eppg::sim::load_scenario(scenario_path)) {
      simulator.schedule(std::move(event));
    }
    std::ofstream trace;
    if (!trace_path.empty()) {
      trace.open(trace_path);
      if (!trace) throw std::runtime_error("cannot write trace: " + trace_path);
    }
    simulator.run(duration_ms, trace_path.empty() ? nullptr : &trace);
    if (!quiet) {
      for (const auto& event : simulator.event_log()) std::cerr << event << '\n';
    }
    std::cout << simulator.snapshot_json() << '\n';
  } catch (const std::exception& error) {
    std::cerr << "eppg-sim: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
