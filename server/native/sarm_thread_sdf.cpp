// Platform build of the delivered SARM thread SDF plugin
// (model/SARM_mjcf_20261012/thread_sdf.cpp, kept byte-identical as delivered).
// tooth_radius() avoids atan2; gradient() shares phase arithmetic across the
// same six finite-difference samples used by the delivered contact search.
// tools/build_sarm_plugin.py compiles both sources into one checker and refuses
// to install this plugin unless distances and gradients match the delivered code.
//
// Two-start SDF thread pair for the SARM bolt experiment.
// Dimensions are metres. The flank is smoothed intentionally; the specified
// 30-degree tooth angle is outside this experiment's scope.
#include <algorithm>
#include <cmath>
#include <cstdlib>
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
  // Evaluate the same central differences while sharing the thread phase across
  // the four XY offsets. The Z offsets use trig addition; eps and normalization
  // remain identical to the delivered contact normal. No analytic-normal or
  // geometry approximation is introduced.
  constexpr double eps = 2e-6;
  constexpr double w = 2.0 * kPi / kPitch;
  const double phase = mode == 0 ? kMalePhase : mode == 1 ? kFemalePhase
                       : mode == 2 ? kLockRodPhase : kLockNutPhase;
  const double root = mode < 2 ? kRootRadius : mode == 2 ? kLockRootRadius : kLockFemaleRoot;
  const double height = mode < 2 ? kToothHeight : mode == 2 ? kLockToothHeight : kLockFemaleHeight;
  const double length = mode == 0 ? kMaleLength : mode == 1 ? kFemaleLength
                        : mode == 2 ? kLockRodLength : kLockNutLength;
  const double outer = mode == 1 ? kOuterRadius : mode == 3 ? kLockOuterRadius : kLockGuideOuterRadius;
  const double a = w * (p[2] + phase);
  const double ca = std::cos(a), sa = std::sin(a);
  const auto evaluate = [&](double x, double y, double z, double c, double s) {
    const double r2 = x*x + y*y;
    const double radius = std::hypot(x, y);
    double radial;
    if (mode == 4) {
      radial = std::max(kLockGuideBoreRadius - radius, radius - outer);
    } else {
      const double cos2 = r2 > 0 ? (x*x - y*y) / r2 : 1.;
      const double sin2 = r2 > 0 ? 2.*x*y / r2 : 0.;
      const double tooth = root + height * 0.5 * (1. - (c*cos2 + s*sin2));
      radial = mode == 0 || mode == 2 ? radius-tooth
               : std::max(tooth + (mode == 1 ? kClearance : 0.) - radius, radius-outer);
    }
    return std::max({radial, -length-z, z});
  };
  grad[0] = (evaluate(p[0]+eps,p[1],p[2],ca,sa) - evaluate(p[0]-eps,p[1],p[2],ca,sa)) / (2*eps);
  grad[1] = (evaluate(p[0],p[1]+eps,p[2],ca,sa) - evaluate(p[0],p[1]-eps,p[2],ca,sa)) / (2*eps);
  const double ce = std::cos(w*eps), se = std::sin(w*eps);
  grad[2] = (evaluate(p[0],p[1],p[2]+eps,ca*ce-sa*se,sa*ce+ca*se)
             - evaluate(p[0],p[1],p[2]-eps,ca*ce+sa*se,sa*ce-ca*se)) / (2*eps);
  const double norm = std::sqrt(grad[0]*grad[0] + grad[1]*grad[1] + grad[2]*grad[2]);
  if (norm > 1e-12) {
    for (int i = 0; i < 3; ++i) grad[i] /= norm;
  } else {
    grad[0] = 1; grad[1] = 0; grad[2] = 0;
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
