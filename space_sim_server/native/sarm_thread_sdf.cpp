// Platform build of the delivered SARM thread SDF plugin
// (model/SARM_mjcf_20261012/thread_sdf.cpp, kept byte-identical as delivered).
// Only tooth_radius() differs: the same function without atan2, which MuJoCo's
// SDF contact search calls tens of thousands of times per physics step.
// tools/build_sarm_plugin.py compiles both sources into one checker and refuses
// to install this plugin unless distances and gradients match the delivered code.
//
// Two-start SDF thread pair for the SARM bolt experiment.
// Dimensions are metres. The flank is smoothed intentionally; the specified
// 30-degree tooth angle is outside this experiment's scope.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
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

double tooth_radius(const mjtNum p[3], double phase,
                    double root = kRootRadius, double height = kToothHeight) {
  // Delivered form: theta = atan2(y, x), u = z - lead*theta/(2*pi) + phase,
  // profile = (1 - cos(2*pi*u/pitch)) / 2. With lead == 2*pitch the cosine
  // argument is w*(z + phase) - 2*theta, and cos/sin(2*theta) follow from x, y
  // directly, so atan2 and one cos are replaced by plain arithmetic.
  static_assert(kLead == 2.0 * kPitch, "trig-free form requires a two-start thread");
  constexpr double w = 2.0 * kPi / kPitch;
  const double r2 = p[0] * p[0] + p[1] * p[1];
  double cos2 = 1.0, sin2 = 0.0;  // atan2(0, 0) == 0 on the axis
  if (r2 > 0.0) {
    cos2 = (p[0] * p[0] - p[1] * p[1]) / r2;
    sin2 = 2.0 * p[0] * p[1] / r2;
  }
  const double a = w * (p[2] + phase);
  const double cosine = std::cos(a) * cos2 + std::sin(a) * sin2;
  return root + height * 0.5 * (1.0 - cosine);
}

double distance(const mjtNum p[3], int mode) {
  const double radius = std::hypot(p[0], p[1]);
  if (mode == 0) {  // threaded male cylinder, local z=-60..0 mm
    const double radial = radius - tooth_radius(p, kMalePhase);
    return std::max({radial, -kMaleLength - p[2], p[2]});
  }
  if (mode == 1) {  // board1 threaded female bore
    const double inside_hole = tooth_radius(p, kFemalePhase) + kClearance - radius;
    return std::max({inside_hole, radius - kOuterRadius,
                     -kFemaleLength - p[2], p[2]});
  }
  if (mode == 2) {  // locking rod, local z=-110..0 mm
    const double radial = radius - tooth_radius(
        p, kLockRodPhase, kLockRootRadius, kLockToothHeight);
    return std::max({radial, -kLockRodLength - p[2], p[2]});
  }
  if (mode == 3) {  // fastener's internal thread, local z=-65..0 mm
    const double inside_hole = tooth_radius(
        p, kLockNutPhase, kLockFemaleRoot, kLockFemaleHeight) - radius;
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
}

const char* const kAttributes[] = {"mode"};

// MuJoCo's public collision dispatch table also covers explicit <pair>s, which
// bypass mjcb_contactfilter. Preserve the stock detector for every uncertain
// configuration and for every other SDF plugin.
mjfCollision original_sdf_collision = nullptr;
int registered_slot = -1;

int model_mode(const mjModel* m, const mjData* d, int geom) {
  const int instance = m->geom_plugin[geom];
  if (instance < 0 || m->plugin[instance] != registered_slot || !d->plugin_data[instance]) return -1;
  return *reinterpret_cast<const int*>(d->plugin_data[instance]);
}

bool rod_inside_guide(const mjData* d, int rod, int guide, double margin) {
  const mjtNum* rp = d->geom_xpos + 3*rod;
  const mjtNum* gp = d->geom_xpos + 3*guide;
  const mjtNum* rm = d->geom_xmat + 9*rod;
  const mjtNum* gm = d->geom_xmat + 9*guide;
  double first[2]{}, last[2]{};
  double magnitude = 1.0;
  for (int j = 0; j < 3; ++j) magnitude = std::max({magnitude, std::abs(rp[j]), std::abs(gp[j])});
  for (int i = 0; i < 2; ++i) {
    double axis = 0.0;
    for (int j = 0; j < 3; ++j) {
      first[i] += gm[3*j+i] * (rp[j] - gp[j]);
      axis += gm[3*j+i] * rm[3*j+2];
    }
    last[i] = first[i] - kLockRodLength * axis;
    if (!std::isfinite(first[i]) || !std::isfinite(last[i])) return false;
  }
  // Every thread point lies in the radius-12 mm cylinder from z=-110..0 mm.
  // Projecting a disk cannot enlarge its radius; the norm of the axis segment
  // is bounded by its endpoints. If this entire cylinder is strictly inside
  // the 12.65 mm bore, the two signed-distance sublevel sets cannot intersect.
  // Inflate the bound for margins and world-coordinate subtraction roundoff.
  const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() * magnitude + 1e-9;
  const double bound = std::max(std::hypot(first[0], first[1]), std::hypot(last[0], last[1]))
                        + kLockRootRadius + kLockToothHeight + roundoff + std::max(0.0, margin);
  return std::isfinite(bound) && std::isfinite(margin) && bound < kLockGuideBoreRadius;
}

int collide_sdf(const mjModel* m, mjData* d, mjContact* contacts,
                int geom1, int geom2, mjtNum margin) {
  const int mode1 = model_mode(m, d, geom1);
  const int mode2 = model_mode(m, d, geom2);
  if ((mode1 == 2 && mode2 == 4 && rod_inside_guide(d, geom1, geom2, margin)) ||
      (mode1 == 4 && mode2 == 2 && rod_inside_guide(d, geom2, geom1, margin))) return 0;
  return original_sdf_collision(m, d, contacts, geom1, geom2, margin);
}

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
  registered_slot = mjp_registerPlugin(&plugin);
  original_sdf_collision = mjCOLLISIONFUNC[mjGEOM_SDF][mjGEOM_SDF];
  if (registered_slot >= 0 && original_sdf_collision)
    mjCOLLISIONFUNC[mjGEOM_SDF][mjGEOM_SDF] = collide_sdf;
}

}  // namespace

mjPLUGIN_LIB_INIT(sarm_two_start_thread) { register_plugin(); }
