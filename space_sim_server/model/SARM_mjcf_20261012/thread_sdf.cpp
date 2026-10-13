// Two-start SDF thread pair for the SARM bolt experiment.
// Dimensions are metres. The flank is smoothed intentionally; the specified
// 30-degree tooth angle is outside this experiment's scope.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <mujoco/mujoco.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLead = 0.016;        // one revolution advances 16 mm
constexpr double kPitch = 0.008;       // two starts, 8 mm apart axially
constexpr double kRootRadius = 0.0135; // measured from the source bolt STL
constexpr double kToothHeight = 0.0015;
constexpr double kClearance = 0.00015;
constexpr double kOuterRadius = 0.023;
constexpr double kMaleLength = 0.060;
constexpr double kFemaleLength = 0.020;
constexpr double kMalePhase = 0.004; // aligns the 60 mm initial body offset
// The source board1 STL has its bore crest at u=z-16*theta/(2*pi)=5 mm
// (board-local coordinates). The SDF geom origin is at z=5 mm, so a +4 mm
// phase puts the SDF crest at the same place. With a 16 mm lead this is a
// 90-degree angular correction relative to the previous zero phase.
constexpr double kFemalePhase = 0.004;
// The locking-rod/fastener pair is measured in its own mesh coordinates.
// rod local +Y is the thread axis, mapped to SDF +Z by the MJCF geom quat.
constexpr double kLockRootRadius = 0.0105;
constexpr double kLockToothHeight = 0.0015;  // major diameter 24 mm
constexpr double kLockFemaleRoot = 0.0106;
constexpr double kLockFemaleHeight = 0.0017; // source nut bore crest ~=12.3 mm
constexpr double kLockRodLength = 0.110;
constexpr double kLockNutLength = 0.065;
constexpr double kLockOuterRadius = 0.0130;
// A secondary clearance guide catches large radial excursions before the
// thin thread-only SDF shell loses contact during a manually driven swing.
// It is paired only with the locking rod, not with hook or other objects.
constexpr double kLockGuideBoreRadius = 0.01265;
constexpr double kLockGuideOuterRadius = 0.020;
constexpr double kLockRodPhase = 0.002035;
constexpr double kLockNutPhase = 0.001171;

struct RadialGeometry {
  std::uint64_t xy[2]{};
  double radius = 0;
  double theta = 0;
  bool valid = false;
  bool has_theta = false;
};

RadialGeometry& radial_geometry(const mjtNum p[3]) {
  // Exact XY reuse across the +/-Z finite differences and repeated SDF
  // queries. Immutable geometry parameters do not participate in this cache.
  // No coordinate quantization: signed zeros retain their atan2 semantics.
  thread_local RadialGeometry cache[32];
  static_assert(sizeof(mjtNum) == sizeof(std::uint64_t), "SDF cache requires double precision");
  std::uint64_t xy[2];
  std::memcpy(xy, p, sizeof(xy));
  const auto hash = xy[0] ^ (xy[1] + 0x9e3779b97f4a7c15ULL + (xy[0] << 6) + (xy[0] >> 2));
  auto& entry = cache[(hash ^ (hash >> 32)) & 31];
  if (!entry.valid || std::memcmp(entry.xy, xy, sizeof(xy)) != 0) {
    std::memcpy(entry.xy, xy, sizeof(xy));
    entry.radius = std::hypot(p[0], p[1]);
    entry.has_theta = false;
    entry.valid = true;
  }
  return entry;
}

double tooth_radius(const mjtNum p[3], RadialGeometry& geometry, double phase,
                    double root = kRootRadius, double height = kToothHeight) {
  if (!geometry.has_theta) {
    geometry.theta = std::atan2(p[1], p[0]);
    geometry.has_theta = true;
  }
  const double theta = geometry.theta;
  const double u = p[2] - kLead * theta / (2.0 * kPi) + phase;
  // cos(2*pi*u/pitch) has two starts per 16 mm lead.
  const double profile = 0.5 * (1.0 - std::cos(2.0 * kPi * u / kPitch));
  return root + height * profile;
}

double distance(const mjtNum p[3], int mode) {
  auto& geometry = radial_geometry(p);
  const double radius = geometry.radius;
  if (mode == 0) {  // threaded male cylinder, local z=-60..0 mm
    const double cap = std::max(-kMaleLength - p[2], p[2]);
    if (cap > radius - kRootRadius) return cap;
    const double radial = radius - tooth_radius(p, geometry, kMalePhase);
    return std::max({radial, -kMaleLength - p[2], p[2]});
  }
  if (mode == 1) {  // board1 threaded female bore
    const double outside = std::max({radius - kOuterRadius, -kFemaleLength - p[2], p[2]});
    if (outside > kRootRadius + kToothHeight + kClearance - radius) return outside;
    const double inside_hole = tooth_radius(p, geometry, kFemalePhase) + kClearance - radius;
    return std::max({inside_hole, radius - kOuterRadius,
                     -kFemaleLength - p[2], p[2]});
  }
  if (mode == 2) {  // locking rod, local z=-110..0 mm
    const double cap = std::max(-kLockRodLength - p[2], p[2]);
    if (cap > radius - kLockRootRadius) return cap;
    const double radial = radius - tooth_radius(
        p, geometry, kLockRodPhase, kLockRootRadius, kLockToothHeight);
    return std::max({radial, -kLockRodLength - p[2], p[2]});
  }
  if (mode == 3) {  // fastener's internal thread, local z=-65..0 mm
    const double outside = std::max({radius - kLockOuterRadius, -kLockNutLength - p[2], p[2]});
    if (outside > kLockFemaleRoot + kLockFemaleHeight - radius) return outside;
    const double inside_hole = tooth_radius(
        p, geometry, kLockNutPhase, kLockFemaleRoot, kLockFemaleHeight) - radius;
    return std::max({inside_hole, radius - kLockOuterRadius,
                     -kLockNutLength - p[2], p[2]});
  }
  // Smooth collision-only radial guide, with 0.65 mm clearance over the
  // 12-mm thread major radius.  Unlike a rigid constraint it can still be
  // unscrewed and does not prescribe any angular/axial relationship.
  return std::max({kLockGuideBoreRadius - radius,
                   radius - kLockGuideOuterRadius,
                   -kLockNutLength - p[2], p[2]});
}

void gradient(mjtNum grad[3], const mjtNum p[3], int mode) {
  // A line search may ask for the same gradient again after evaluating the
  // two surfaces. Retain only the last exact query per immutable SDF mode.
  struct Entry { mjtNum point[3]{}, gradient[3]{}; bool valid = false; };
  thread_local Entry cache[5];
  Entry* entry = mode >= 0 && mode < 5 ? &cache[mode] : nullptr;
  if (entry && entry->valid && std::memcmp(entry->point, p, sizeof(entry->point)) == 0) {
    std::memcpy(grad, entry->gradient, sizeof(entry->gradient));
    return;
  }
  constexpr double eps = 2e-6;
  for (int i = 0; i < 3; ++i) {
    mjtNum plus[3] = {p[0], p[1], p[2]};
    mjtNum minus[3] = {p[0], p[1], p[2]};
    plus[i] += eps;
    minus[i] -= eps;
    grad[i] = (distance(plus, mode) - distance(minus, mode)) / (2 * eps);
  }
  const double norm = std::sqrt(grad[0]*grad[0] + grad[1]*grad[1] + grad[2]*grad[2]);
  if (norm > 1e-12) {
    for (int i = 0; i < 3; ++i) grad[i] /= norm;
  } else {
    grad[0] = 1; grad[1] = 0; grad[2] = 0;
  }
  if (entry) {
    std::memcpy(entry->point, p, sizeof(entry->point));
    std::memcpy(entry->gradient, grad, sizeof(entry->gradient));
    entry->valid = true;
  }
}

const char* const kAttributes[] = {"mode"};

void register_plugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "sarm.sdf.two_start_thread";
  plugin.capabilityflags = mjPLUGIN_SDF;
  plugin.nattribute = 1;
  plugin.attributes = kAttributes;
  plugin.nstate = [](const mjModel*, int) { return 0; };
  plugin.init = [](const mjModel* m, mjData* d, int instance) {
    const char* config = mj_getPluginConfig(m, instance, "mode");
    const int mode = config ? std::atoi(config) : -1;
    if (mode < 0 || mode > 4) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new int(mode));
    return 0;
  };
  plugin.destroy = [](mjData* d, int instance) {
    delete reinterpret_cast<int*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.copy = [](mjData* dest, const mjModel*, const mjData* src, int instance) {
    dest->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new int(*reinterpret_cast<const int*>(src->plugin_data[instance])));
  };
  plugin.reset = [](const mjModel*, mjtNum*, void*, int) {};
  plugin.compute = [](const mjModel*, mjData*, int, int) {};
  plugin.sdf_distance = [](const mjtNum p[3], const mjData* d, int instance) {
    return distance(p, *reinterpret_cast<const int*>(d->plugin_data[instance]));
  };
  plugin.sdf_gradient = [](mjtNum grad[3], const mjtNum p[3],
                           const mjData* d, int instance) {
    gradient(grad, p, *reinterpret_cast<const int*>(d->plugin_data[instance]));
  };
  plugin.sdf_staticdistance = [](const mjtNum p[3], const mjtNum* attr) {
    return distance(p, static_cast<int>(attr[0]));
  };
  plugin.sdf_attribute = [](mjtNum* attr, const char** name,
                            const char** value) {
    attr[0] = -1;
    for (int i = 0; i < 1; ++i) {
      if (name[i] && value[i]) attr[0] = std::strtod(value[i], nullptr);
    }
  };
  plugin.sdf_aabb = [](mjtNum aabb[6], const mjtNum* attr) {
    const int mode = static_cast<int>(attr[0]);
    const double radius = mode == 0 ? kRootRadius + kToothHeight
                          : mode == 1 ? kOuterRadius
                          : mode == 2 ? kLockRootRadius + kLockToothHeight
                          : mode == 3 ? kLockOuterRadius
                                      : kLockGuideOuterRadius;
    const double length = mode == 0 ? kMaleLength
                          : mode == 1 ? kFemaleLength
                          : mode == 2 ? kLockRodLength : kLockNutLength;
    aabb[0] = 0; aabb[1] = 0; aabb[2] = -0.5 * length;
    aabb[3] = radius; aabb[4] = radius; aabb[5] = 0.5 * length;
  };
  mjp_registerPlugin(&plugin);
}

}  // namespace

mjPLUGIN_LIB_INIT(sarm_two_start_thread) { register_plugin(); }
