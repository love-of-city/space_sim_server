// Equivalence check between the delivered SARM thread SDF source and the
// platform build (native/sarm_thread_sdf.cpp). tools/build_sarm_plugin.py
// compiles this with DELIVERED_SOURCE and PLATFORM_SOURCE set to the two files
// and installs the plugin only if it exits with status 0.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#define MJ_STATIC
#include <mujoco/mujoco.h>

// Both sources register the plugin from a library constructor. The checker
// calls their distance/gradient functions directly and never registers.
#undef mjPLUGIN_LIB_INIT
#define mjPLUGIN_LIB_INIT(name) [[maybe_unused]] static void unused_##name()
extern "C" {
void mjp_defaultPlugin(mjpPlugin*) {}
int mjp_registerPlugin(const mjpPlugin*) { return 0; }
const char* mj_getPluginConfig(const mjModel*, int, const char*) { return nullptr; }
}

namespace delivered {
#include DELIVERED_SOURCE
double Distance(const mjtNum p[3], int mode) { return distance(p, mode); }
void Gradient(mjtNum g[3], const mjtNum p[3], int mode) { gradient(g, p, mode); }
}  // namespace delivered

namespace platform {
#include PLATFORM_SOURCE
double Distance(const mjtNum p[3], int mode) { return distance(p, mode); }
void Gradient(mjtNum g[3], const mjtNum p[3], int mode) { gradient(g, p, mode); }
}  // namespace platform

int main() {
  // Tolerances are far above the measured rounding differences (~1e-17 m and
  // ~1e-11) and far below anything a contact solver could notice.
  constexpr double kDistanceTolerance = 1e-12;  // metres
  constexpr double kGradientTolerance = 1e-6;   // unit-vector components
  constexpr int kPointsPerMode = 200000;
  std::mt19937_64 rng(20261013);
  std::uniform_real_distribution<double> radius(0.0, 0.03), angle(-3.14159265358979, 3.14159265358979),
      axial(-0.12, 0.01);
  double worst_distance = 0.0, worst_gradient = 0.0;
  bool ok = true;
  for (int mode = 0; mode < 5; ++mode) {
    for (int i = 0; i < kPointsPerMode; ++i) {
      // Every 1000th point lies on the thread axis, where atan2(0, 0) applies.
      const double r = i % 1000 == 0 ? 0.0 : radius(rng);
      const double t = angle(rng);
      const mjtNum p[3] = {r * std::cos(t), r * std::sin(t), axial(rng)};
      mjtNum a[3], b[3];
      delivered::Gradient(a, p, mode);
      platform::Gradient(b, p, mode);
      const double dd = std::fabs(delivered::Distance(p, mode) - platform::Distance(p, mode));
      const double dg = std::max({std::fabs(a[0] - b[0]), std::fabs(a[1] - b[1]), std::fabs(a[2] - b[2])});
      worst_distance = std::max(worst_distance, dd);
      worst_gradient = std::max(worst_gradient, dg);
      if (!(dd <= kDistanceTolerance) || !(dg <= kGradientTolerance)) ok = false;
    }
  }
  std::printf("{\"points\": %d, \"max_distance_difference_m\": %.3g, \"max_gradient_difference\": %.3g, \"pass\": %s}\n",
              5 * kPointsPerMode, worst_distance, worst_gradient, ok ? "true" : "false");
  return ok ? 0 : 1;
}
