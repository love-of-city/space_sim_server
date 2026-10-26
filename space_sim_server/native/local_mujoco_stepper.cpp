// Local-frame fixed-step MuJoCo dynamics for the SARM "local" backend.
//
// Compile against EXACT MuJoCo 3.7.0 headers. No MuJoCo library is linked:
// every function is resolved from the mujoco.dll that Basilisk has ALREADY
// loaded into this process (looked up by its absolute path, never loaded
// here), so exactly one MuJoCo runtime exists in the process.
//
// The model is an independent mjModel compiled from the same MJCF as MJScene.
// Coordinates are the local translating frame L: origin at the orbital
// reference point O, axes parallel to the inertial frame N. Gravity is zero;
// the uniform part of the Earth/Sun field is carried by O (Basilisk), and the
// optional Earth tidal term is applied here as a body force.
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <new>
#include <vector>
#include <mujoco/mujoco.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#define API extern "C" __declspec(dllexport)
#else
#include <dlfcn.h>
#define API extern "C" __attribute__((visibility("default")))
#endif

namespace {

constexpr int ABI = 1;

enum Status {
    OK = 0,
    ERR_ARGUMENT = -1,
    ERR_RUNTIME = -2,
    ERR_VERSION = -3,
    ERR_LOAD = -4,
    ERR_DIVERGED = -5,
    ERR_FRICTION = -6,
};

struct Api {
    int (*version)();
    mjModel* (*loadXML)(const char*, const mjVFS*, char*, int);
    void (*deleteModel)(mjModel*);
    mjData* (*makeData)(const mjModel*);
    void (*deleteData)(mjData*);
    mjData* (*copyData)(mjData*, const mjModel*, const mjData*);
    void (*resetData)(const mjModel*, mjData*);
    void (*forward)(const mjModel*, mjData*);
    void (*step)(const mjModel*, mjData*);
    void (*step1)(const mjModel*, mjData*);
    void (*step2)(const mjModel*, mjData*);
    void (*kinematics)(const mjModel*, mjData*);
    void (*comPos)(const mjModel*, mjData*);
    void (*contactForce)(const mjModel*, const mjData*, int, mjtNum*);
    void (*objectVelocity)(const mjModel*, const mjData*, int, int, mjtNum*, int);

    void (*quat2Mat)(mjtNum*, const mjtNum*);
    int (*name2id)(const mjModel*, int, const char*);
    const char* (*id2name)(const mjModel*, int, int);
};

struct AxialFriction {
    int plug, base;
    bool enabled = false;
    double axis[3] = {0, 0, 1};  // outward, in the base frame
    double seat[3] = {0, 0, 0};  // seated plug origin in the base frame
    double travel, static_n, kinetic_n;
    double force[3] = {0, 0, 0};  // last applied inertial force on the plug
    double axial_force = 0, velocity = 0;
    double start_z = 0;
    int mode = 0;  // 0 free, 1 sticking/stopping, 2 sliding
};

struct Stepper {
    Api api{};
    mjModel* m = nullptr;
    mjData* d = nullptr;
    mjData* friction_trial = nullptr;  // candidate for the same physical step
    std::vector<AxialFriction> friction;
    std::vector<int> friction_ids;
    std::vector<double> friction_response;
    double friction_timestep = 0;
    std::vector<int> free_bodies;  // bodies whose root joint is free
    // Held external wrench per body: force in the inertial/local axes [N] at
    // the body COM, torque in the BODY frame [N*m] (Basilisk CmdForceInertial /
    // CmdTorqueBody conventions). Rotated with the body at every substep.
    std::vector<double> force_world;
    std::vector<double> torque_body;
    std::vector<double> orbital_force;  // BSK-computed differential force, inertial axes
    // Preserve the completed-step sample across rebase/set_state forward passes.
    std::vector<double> sensor_sample;
    bool orbital_forces_enabled = false;
};

void copy_error(char* out, int size, const char* text) {
    if (!out || size <= 0) return;
    std::strncpy(out, text, static_cast<std::size_t>(size) - 1);
    out[size - 1] = '\0';
}

#ifdef _WIN32
template <typename T>
bool resolve(HMODULE module, const char* name, T& target) {
    target = reinterpret_cast<T>(GetProcAddress(module, name));
    return target != nullptr;
}
#else
template <typename T>
bool resolve(void* module, const char* name, T& target) {
    target = reinterpret_cast<T>(dlsym(module, name));
    return target != nullptr;
}
#endif

bool finite(const mjtNum* values, int count) {
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) return false;
    }
    return true;
}

}  // namespace

API int lms_abi() { return ABI; }
API int lms_header_version() { return mjVERSION_HEADER; }

// runtime_path: absolute path of the mujoco.dll Basilisk already loaded.
#ifdef _WIN32
using RuntimePathChar = wchar_t;
#else
using RuntimePathChar = char;
#endif
API void* lms_create(const RuntimePathChar* runtime_path, const char* xml_path, char* error, int error_size) {
    if (!runtime_path || !xml_path) {
        copy_error(error, error_size, "runtime and model paths are required");
        return nullptr;
    }
    auto* s = new (std::nothrow) Stepper();
    if (!s) {
        copy_error(error, error_size, "out of memory");
        return nullptr;
    }
#ifdef _WIN32
    HMODULE module = GetModuleHandleW(runtime_path);
#else
    // Acquire only an existing mapping; Basilisk owns its lifetime.
    void* module = dlopen(runtime_path, RTLD_NOW | RTLD_NOLOAD);
#endif
    if (!module) {
        copy_error(error, error_size, "Basilisk MuJoCo runtime is not loaded in this process");
        delete s;
        return nullptr;
    }
    Api& a = s->api;
    bool ok = resolve(module, "mj_version", a.version) && resolve(module, "mj_loadXML", a.loadXML) &&
              resolve(module, "mj_deleteModel", a.deleteModel) && resolve(module, "mj_makeData", a.makeData) &&
              resolve(module, "mj_deleteData", a.deleteData) && resolve(module, "mj_resetData", a.resetData) &&
              resolve(module, "mj_copyData", a.copyData) && resolve(module, "mju_quat2Mat", a.quat2Mat) &&
              resolve(module, "mj_forward", a.forward) && resolve(module, "mj_step", a.step) &&
              resolve(module, "mj_step1", a.step1) && resolve(module, "mj_step2", a.step2) &&
              resolve(module, "mj_kinematics", a.kinematics) && resolve(module, "mj_comPos", a.comPos) &&
              resolve(module, "mj_name2id", a.name2id) &&
              resolve(module, "mj_id2name", a.id2name) &&
              resolve(module, "mj_contactForce", a.contactForce) &&
              resolve(module, "mj_objectVelocity", a.objectVelocity);
#ifndef _WIN32
    dlclose(module);  // Balance NOLOAD's reference; Basilisk retains its own.
#endif
    if (!ok) {
        copy_error(error, error_size, "required MuJoCo symbol is missing from the loaded runtime");
        delete s;
        return nullptr;
    }
    if (s->api.version() != mjVERSION_HEADER) {
        copy_error(error, error_size, "loaded MuJoCo runtime does not match the compiled 3.7.0 headers");
        delete s;
        return nullptr;
    }
    char load_error[1024] = {0};
    s->m = s->api.loadXML(xml_path, nullptr, load_error, sizeof(load_error));
    if (!s->m) {
        copy_error(error, error_size, load_error[0] ? load_error : "mj_loadXML failed");
        delete s;
        return nullptr;
    }
    s->d = s->api.makeData(s->m);
    if (!s->d) {
        copy_error(error, error_size, "mj_makeData failed");
        s->api.deleteModel(s->m);
        delete s;
        return nullptr;
    }
    // Numerical failure must surface to the caller, never be hidden by
    // MuJoCo's silent reset to qpos0.
    s->m->opt.disableflags |= mjDSBL_AUTORESET;
    s->m->opt.integrator = mjINT_IMPLICITFAST;
    s->m->opt.gravity[0] = s->m->opt.gravity[1] = s->m->opt.gravity[2] = 0;
    for (int b = 1; b < s->m->nbody; ++b) {
        int j = s->m->body_jntadr[b];
        if (s->m->body_jntnum[b] > 0 && s->m->jnt_type[j] == mjJNT_FREE) s->free_bodies.push_back(b);
    }
    s->force_world.assign(3 * s->m->nbody, 0.0);
    s->torque_body.assign(3 * s->m->nbody, 0.0);
    s->orbital_force.assign(3 * s->m->nbody, 0.0);
    s->sensor_sample.assign(s->m->nsensordata, 0.0);
    s->api.resetData(s->m, s->d);
    s->api.forward(s->m, s->d);
    return s;
}

API void lms_destroy(void* handle) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s) return;
    if (s->d) s->api.deleteData(s->d);
    if (s->friction_trial) s->api.deleteData(s->friction_trial);
    if (s->m) s->api.deleteModel(s->m);
    delete s;
}


// out: nq, nv, nu, nbody, njnt, nfree
API int lms_dims(void* handle, std::int64_t* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out) return ERR_ARGUMENT;
    out[0] = s->m->nq; out[1] = s->m->nv; out[2] = s->m->nu;
    out[3] = s->m->nbody; out[4] = s->m->njnt; out[5] = static_cast<std::int64_t>(s->free_bodies.size());
    return OK;
}

// type: mjtObj value (mjOBJ_JOINT=3, mjOBJ_ACTUATOR=19, mjOBJ_BODY=1).
API int lms_name2id(void* handle, int type, const char* name) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !name) return ERR_ARGUMENT;
    return s->api.name2id(s->m, type, name);
}

// Read an existing force/torque pair from the authoritative dynamics data.
// No forward pass: sensordata is the LAST substep's pre-integration sample.
// Return 1 when supported, 0 only when both sensors are absent, negative on
// invalid configuration. out = Fx,Fy,Fz,Tx,Ty,Tz in the common site frame.
API int lms_read_wrench(void* handle, const char* force_name, const char* torque_name, double* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !force_name || !torque_name || !out) return ERR_ARGUMENT;
    const mjModel* m = s->m;
    int f = s->api.name2id(m, mjOBJ_SENSOR, force_name);
    int t = s->api.name2id(m, mjOBJ_SENSOR, torque_name);
    if (f < 0 && t < 0) return 0;
    if (f < 0 || t < 0 || m->sensor_type[f] != mjSENS_FORCE || m->sensor_type[t] != mjSENS_TORQUE ||
        m->sensor_dim[f] != 3 || m->sensor_dim[t] != 3 ||
        m->sensor_objtype[f] != mjOBJ_SITE || m->sensor_objtype[t] != mjOBJ_SITE ||
        m->sensor_objid[f] != m->sensor_objid[t] || (m->opt.disableflags & mjDSBL_SENSOR)) return ERR_ARGUMENT;
    for (int i=0; i<3; ++i) {
        out[i] = s->sensor_sample[m->sensor_adr[f]+i];
        out[3+i] = s->sensor_sample[m->sensor_adr[t]+i];
    }
    return finite(out, 6) ? 1 : ERR_DIVERGED;
}

// Copies the object name (empty when unnamed); returns its length or an error.
API int lms_id2name(void* handle, int type, int id, char* out, int size) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out || size <= 0) return ERR_ARGUMENT;
    const char* name = s->api.id2name(s->m, type, id);
    copy_error(out, size, name ? name : "");
    return static_cast<int>(std::strlen(out));
}

// out: qposadr, dofadr, type
API int lms_joint(void* handle, int joint, std::int64_t* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out || joint < 0 || joint >= s->m->njnt) return ERR_ARGUMENT;
    out[0] = s->m->jnt_qposadr[joint]; out[1] = s->m->jnt_dofadr[joint]; out[2] = s->m->jnt_type[joint];
    return OK;
}

// out: mass [kg], ipos in the body frame [m]. Used to prove the publish-only
// MJScene copy and this independent model describe the same bodies.
API int lms_body_mass(void* handle, int body, double* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out || body < 0 || body >= s->m->nbody) return ERR_ARGUMENT;
    out[0] = s->m->body_mass[body];
    out[1] = s->m->body_ipos[3 * body];
    out[2] = s->m->body_ipos[3 * body + 1];
    out[3] = s->m->body_ipos[3 * body + 2];
    return OK;
}

// Compiled principal inertia and its orientation, including inferred geom inertia.
API int lms_body_inertial(void* handle, int body, double* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out || body < 0 || body >= s->m->nbody) return ERR_ARGUMENT;
    lms_body_mass(handle, body, out);
    for (int i=0; i<4; ++i) out[4+i] = s->m->body_iquat[4*body+i];
    for (int i=0; i<3; ++i) out[8+i] = s->m->body_inertia[3*body+i];
    return OK;
}

// Free-joint qpos/qvel addresses, in body order: [qposadr, dofadr] per body.
API int lms_free_joints(void* handle, std::int64_t* out, int capacity) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out || capacity < static_cast<int>(s->free_bodies.size())) return ERR_ARGUMENT;
    for (std::size_t i = 0; i < s->free_bodies.size(); ++i) {
        int j = s->m->body_jntadr[s->free_bodies[i]];
        out[2 * i] = s->m->jnt_qposadr[j];
        out[2 * i + 1] = s->m->jnt_dofadr[j];
    }
    return static_cast<int>(s->free_bodies.size());
}

// Express one actuator as a PD position servo evaluated INSIDE MuJoCo so the
// implicitfast integrator also differentiates the damping term:
//   force = clip(kp*ctrl - kp*q - kd*qd, -limit, limit)
// With ctrl = q_ref + kd/kp*qd_ref this equals the Basilisk PID -> limiter.
API int lms_set_servo(void* handle, int actuator, double kp, double kd, double limit) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || actuator < 0 || actuator >= s->m->nu) return ERR_ARGUMENT;
    if (!(kp > 0) || !(kd >= 0) || !(limit > 0) || !std::isfinite(kp + kd + limit)) return ERR_ARGUMENT;
    mjModel* m = s->m;
    m->actuator_ctrllimited[actuator] = 0;
    m->actuator_forcelimited[actuator] = 1;
    m->actuator_forcerange[2 * actuator] = -limit;
    m->actuator_forcerange[2 * actuator + 1] = limit;
    m->actuator_gaintype[actuator] = mjGAIN_FIXED;
    m->actuator_biastype[actuator] = mjBIAS_AFFINE;
    std::memset(m->actuator_gainprm + mjNGAIN * actuator, 0, sizeof(mjtNum) * mjNGAIN);
    std::memset(m->actuator_biasprm + mjNBIAS * actuator, 0, sizeof(mjtNum) * mjNBIAS);
    m->actuator_gainprm[mjNGAIN * actuator] = kp;
    m->actuator_biasprm[mjNBIAS * actuator + 1] = -kp;
    m->actuator_biasprm[mjNBIAS * actuator + 2] = -kd;
    return OK;
}

API int lms_set_state(void* handle, const double* qpos, const double* qvel) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !qpos || !qvel) return ERR_ARGUMENT;
    if (!finite(qpos, s->m->nq) || !finite(qvel, s->m->nv)) return ERR_ARGUMENT;
    std::memcpy(s->d->qpos, qpos, sizeof(mjtNum) * s->m->nq);
    std::memcpy(s->d->qvel, qvel, sizeof(mjtNum) * s->m->nv);
    std::memset(s->d->qacc_warmstart, 0, sizeof(mjtNum) * s->m->nv);
    s->api.forward(s->m, s->d);
    return OK;
}

API int lms_get_state(void* handle, double* qpos, double* qvel) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !qpos || !qvel) return ERR_ARGUMENT;
    std::memcpy(qpos, s->d->qpos, sizeof(mjtNum) * s->m->nq);
    std::memcpy(qvel, s->d->qvel, sizeof(mjtNum) * s->m->nv);
    return OK;
}

// Held external wrench on one body (see Stepper::force_world/torque_body).
API int lms_set_body_wrench(void* handle, int body, const double* force_world, const double* torque_body) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || body < 1 || body >= s->m->nbody || !force_world || !torque_body) return ERR_ARGUMENT;
    if (!finite(force_world, 3) || !finite(torque_body, 3)) return ERR_ARGUMENT;
    std::memcpy(&s->force_world[3 * body], force_world, sizeof(double) * 3);
    std::memcpy(&s->torque_body[3 * body], torque_body, sizeof(double) * 3);
    return OK;
}

// One relative dry-friction interface. It must connect two independent free
// bodies; it never adds a joint, changes geometry, or edits an integrated state.
API int lms_add_axial_friction(void* handle, int plug, int base, double static_n,
                              double kinetic_n, double travel) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || plug == base || plug < 1 || base < 1 || plug >= s->m->nbody || base >= s->m->nbody ||
        !(static_n >= kinetic_n) || !(kinetic_n > 0) || !(travel > 0) ||
        !std::isfinite(static_n + kinetic_n + travel)) return ERR_ARGUMENT;
    for (int b : {plug, base}) {
        if (s->m->body_jntnum[b] != 1 || s->m->jnt_type[s->m->body_jntadr[b]] != mjJNT_FREE)
            return ERR_ARGUMENT;
    }
    for (const auto& f : s->friction) if (f.plug == plug) return ERR_ARGUMENT;
    if (!s->friction_trial) s->friction_trial = s->api.makeData(s->m);
    if (!s->friction_trial) return ERR_RUNTIME;
    AxialFriction f{};
    f.plug = plug; f.base = base; f.static_n = static_n; f.kinetic_n = kinetic_n; f.travel = travel;
    s->friction.push_back(f);
    return static_cast<int>(s->friction.size()) - 1;
}

API int lms_set_axial_friction(void* handle, int id, int enabled, const double* axis, const double* seat) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || id < 0 || id >= static_cast<int>(s->friction.size()) || !axis || !seat ||
        !finite(axis, 3) || !finite(seat, 3)) return ERR_ARGUMENT;
    double length = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    if (std::abs(length - 1) > 1e-8) return ERR_ARGUMENT;
    auto& f = s->friction[id];
    f.enabled = enabled != 0;
    std::copy(axis, axis+3, f.axis);
    std::copy(seat, seat+3, f.seat);
    if (!f.enabled) {
        f.mode = 0; f.axial_force = f.velocity = 0;
        std::fill(f.force, f.force+3, 0);
    }
    return OK;
}

// Last completed substep, not a fresh force evaluation: Fx,Fy,Fz,axial F,v,mode.
API int lms_axial_friction_state(void* handle, int id, double* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || id < 0 || id >= static_cast<int>(s->friction.size()) || !out) return ERR_ARGUMENT;
    const auto& f = s->friction[id];
    std::copy(f.force, f.force+3, out);
    out[3] = f.axial_force; out[4] = f.velocity; out[5] = f.mode;
    return OK;
}

// Refresh only kinematics, never collision/constraint solving. Export all COMs
// from the authoritative local model, independent of MJScene publication stride.
API int lms_body_coms(void* handle, double* positions, double* masses) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !positions || !masses) return ERR_ARGUMENT;
    s->api.kinematics(s->m, s->d);
    s->api.comPos(s->m, s->d);
    std::memcpy(positions, s->d->xipos, 3 * s->m->nbody * sizeof(double));
    std::memcpy(masses, s->m->body_mass, s->m->nbody * sizeof(double));
    return OK;
}

// Separate from user wrenches: BSK gravity must not overwrite thrust/drag.
API int lms_set_orbital_forces(void* handle, const double* forces) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !forces || !finite(forces, 3 * s->m->nbody)) return ERR_ARGUMENT;
    std::memcpy(s->orbital_force.data(), forces, 3 * s->m->nbody * sizeof(double));
    s->orbital_forces_enabled = true;
    return OK;
}

// Sum user wrenches and provider gravity; mu>0 is ONLY the legacy regression
// path: F_i = m_i * mu/r^3 * (3 rhat rhat^T - I) * x_i.
static void apply_tidal(Stepper* s, const double* origin, double mu) {
    mjModel* m = s->m;
    mjData* d = s->d;
    for (int b = 0; b < m->nbody; ++b) {
        const mjtNum* R = d->xmat + 9 * b;  // body-to-world
        const double* f = &s->force_world[3 * b];
        const double* t = &s->torque_body[3 * b];
        mjtNum* out = d->xfrc_applied + 6 * b;
        for (int c = 0; c < 3; ++c) out[c] = f[c] + s->orbital_force[3 * b + c];
        out[3] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
        out[4] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
        out[5] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
    }
    if (!(mu > 0)) return;
    double r2 = origin[0] * origin[0] + origin[1] * origin[1] + origin[2] * origin[2];
    double r = std::sqrt(r2);
    double k = mu / (r2 * r);
    double u[3] = {origin[0] / r, origin[1] / r, origin[2] / r};
    for (int b = 1; b < m->nbody; ++b) {
        const mjtNum* x = d->xipos + 3 * b;
        double ux = u[0] * x[0] + u[1] * x[1] + u[2] * x[2];
        double mass = m->body_mass[b];
        for (int c = 0; c < 3; ++c) d->xfrc_applied[6 * b + c] += mass * k * (3.0 * u[c] * ux - x[c]);
    }
}

static void rotate(const double* R, const double* x, double* y) {
    for (int i=0; i<3; ++i) y[i] = R[3*i]*x[0] + R[3*i+1]*x[1] + R[3*i+2]*x[2];
}

static double dot3(const double* a, const double* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void cross3(const double* a, const double* b, double* out) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

// Relative axial velocity includes the moving/rotating base. Read integrated
// free-joint states directly: xmat/cvel still describe the start of a mj_step.
static double axial_velocity(Stepper* s, const mjData* d, const AxialFriction& f,
                             double* z = nullptr, double* axis_out = nullptr) {
    int pj = s->m->body_jntadr[f.plug], bj = s->m->body_jntadr[f.base];
    const double* pp = d->qpos + s->m->jnt_qposadr[pj];
    const double* bp = d->qpos + s->m->jnt_qposadr[bj];
    const double* pv = d->qvel + s->m->jnt_dofadr[pj];
    const double* bv = d->qvel + s->m->jnt_dofadr[bj];
    double R[9], axis[3], omega[3], offset[3], frame_speed[3], rel[3], seat[3];
    s->api.quat2Mat(R, bp+3);
    rotate(R, f.axis, axis); rotate(R, bv+3, omega); rotate(R, f.seat, seat);
    for (int k=0; k<3; ++k) offset[k] = pp[k] - bp[k];
    cross3(omega, offset, frame_speed);
    for (int k=0; k<3; ++k) rel[k] = pv[k] - bv[k] - frame_speed[k];
    if (z) *z = dot3(axis, offset) - dot3(axis, seat);
    if (axis_out) std::copy(axis, axis+3, axis_out);
    return dot3(axis, rel);
}

// Equal/opposite wrenches at the plug origin, shifted to BOTH centres of mass.
// The reaction includes base torque; never tie a floating socket to the world.
static void friction_wrench(Stepper* s, mjData* d, const AxialFriction& f,
                            const double* axis, double force) {
    const double* point = s->d->qpos + s->m->jnt_qposadr[s->m->body_jntadr[f.plug]];
    for (int b : {f.plug, f.base}) {
        const double* pose = s->d->qpos + s->m->jnt_qposadr[s->m->body_jntadr[b]];
        double R[9], com_offset[3], arm[3], applied[3], torque[3];
        s->api.quat2Mat(R, pose+3);
        rotate(R, s->m->body_ipos+3*b, com_offset);
        for (int k=0; k<3; ++k) {
            arm[k] = point[k] - pose[k] - com_offset[k];
            applied[k] = (b == f.plug ? force : -force) * axis[k];
        }
        cross3(arm, applied, torque);
        for (int k=0; k<3; ++k) {
            d->xfrc_applied[6*b+k] += applied[k];
            d->xfrc_applied[6*b+3+k] += torque[k];
        }
    }
}

// Bounded time-step friction solve. Predict the SAME implicitfast/contact step
// in scratch mjData and solve for the force that stops relative slip. Commit
// the converged candidate, including all integration/constraint state, rather
// than running the identical step2 again. Failed candidates never advance s->d.
// This avoids sign(v) chatter and viscous creep without an axial position spring.
// Sliding uses a fixed kinetic limit; the last stopping impulse may be smaller.
static int apply_axial_friction(Stepper* s, bool& integrated) {
    integrated = false;
    std::vector<int> ids;
    std::vector<double> axes, start_speed, previous_forces;
    for (int i=0; i<static_cast<int>(s->friction.size()); ++i) {
        auto& f = s->friction[i];
        double previous = f.axial_force;
        double previous_slip = f.velocity;
        int previous_mode = f.mode;
        f.axial_force = 0; f.mode = 0; std::fill(f.force, f.force+3, 0);
        double z, axis[3];
        f.velocity = axial_velocity(s, s->d, f, &z, axis);
        f.start_z = z;
        if (f.enabled && z < f.travel) {
            ids.push_back(i); axes.insert(axes.end(), axis, axis+3);
            start_speed.push_back(previous_mode == 1 ? previous_slip : f.velocity);
            previous_forces.push_back(previous);
        }
    }
    int n = static_cast<int>(ids.size());
    if (!n) return OK;
    auto predict = [&](const std::vector<double>& forces, std::vector<double>& speeds) {
        mjData* d = s->api.copyData(s->friction_trial, s->m, s->d);
        for (int j=0; j<n; ++j) friction_wrench(s, d, s->friction[ids[j]], axes.data()+3*j, forces[j]);
        // All candidates start at the same qpos/qvel/control. Reuse the real
        // step1's collision detection, mass factorization and velocity terms;
        // only applied forces, constraint solve and integration differ.
        s->api.step2(s->m, d);
        if (!finite(d->qpos, s->m->nq) || !finite(d->qvel, s->m->nv) ||
            d->warning[mjWARN_BADQACC].number != s->d->warning[mjWARN_BADQACC].number) return false;
        // MuJoCo 3.7 midpoint-integrates eligible free bodies but uses endpoint
        // velocities for articulated bases. Endpoint relative velocity == 0
        // therefore need not mean zero slip. Use the actual integrated relative
        // displacement / h; this is a per-step friction condition, not a spring
        // to a stored target pose or a correction of a past position error.
        for (int j=0; j<n; ++j) {
            double z;
            axial_velocity(s, d, s->friction[ids[j]], &z);
            speeds[j] = (z-s->friction[ids[j]].start_z) / s->m->opt.timestep;
        }
        return true;
    };
    constexpr double tolerance = 1e-6;  // m/s, contact-solver stiction tolerance
    std::vector<double> forces = previous_forces;
    std::vector<double> free_speed(n), measured(n), probe(n), response(n*n), limits(n);
    std::vector<bool> sliding(n, false);
    auto apply = [&]() {
        // Every successful exit immediately follows predict(forces, measured).
        // The trial therefore contains precisely the final applied forces and
        // a complete physical step, with the same initial solver warm start.
        // Both buffers are owned here; no mjData pointer escapes the ABI.
        // Swapping preserves contacts, effort, qacc, warnings and time without
        // copying the large model workspace a second time.
        std::swap(s->d, s->friction_trial);
        integrated = true;
        for (int i=0; i<n; ++i) {
            auto& f = s->friction[ids[i]];
            f.axial_force = forces[i]; f.mode = sliding[i] ? 2 : 1;
            for (int k=0; k<3; ++k) f.force[k] = forces[i] * axes[3*i+k];
        }
    };
    // Validate a previous solution against this substep's actual contact solve.
    // Usually loads change slowly: one candidate is sufficient.
    if (!predict(forces, measured)) return ERR_DIVERGED;
    bool warm_valid = true;
    for (int i=0; i<n; ++i) {
        const auto& f = s->friction[ids[i]];
        double limit = std::abs(start_speed[i]) <= tolerance ? f.static_n : f.kinetic_n;
        bool stopped = std::abs(measured[i]) <= tolerance && std::abs(forces[i]) <= limit;
        sliding[i] = std::abs(start_speed[i]) > tolerance && forces[i]*measured[i] < 0 &&
                     std::abs(std::abs(forces[i])-f.kinetic_n) < 1e-8;
        if (!stopped && !sliding[i]) warm_valid = false;
    }
    if (warm_valid) { apply(); return OK; }
    bool cached = s->friction_ids == ids && s->friction_response.size() == response.size() &&
                  s->friction_timestep == s->m->opt.timestep;
    for (int j=0; j<n; ++j) {
        const auto& f = s->friction[ids[j]];
        limits[j] = std::abs(start_speed[j]) <= tolerance ? f.static_n : f.kinetic_n;
    }
    if (cached) {
        response = s->friction_response;
        for (int i=0; i<n; ++i) {
            free_speed[i] = measured[i];
            for (int j=0; j<n; ++j) free_speed[i] -= response[i*n+j]*forces[j];
        }
    } else {
        std::fill(forces.begin(), forces.end(), 0);
        if (!predict(forces, free_speed)) return ERR_DIVERGED;
        // Measure the coupled force-to-slip response, including servo damping,
        // contact constraints, COM offsets, and the free base's reaction motion.
        for (int j=0; j<n; ++j) {
            double test = s->friction[ids[j]].kinetic_n;
            forces[j] = test;
            if (!predict(forces, probe)) return ERR_DIVERGED;
            for (int i=0; i<n; ++i) response[i*n+j] = (probe[i] - free_speed[i]) / test;
            forces[j] = 0;
        }
    }
    for (int outer=0; outer<12; ++outer) {
        for (int pass=0; pass<100; ++pass) {
            double change = 0;
            for (int i=0; i<n; ++i) {
                double residual = free_speed[i];
                for (int j=0; j<n; ++j) residual += response[i*n+j] * forces[j];
                double diagonal = response[i*n+i];
                double next = diagonal > 1e-12 ? forces[i] - residual/diagonal : 0;
                next = std::clamp(next, -limits[i], limits[i]);
                change = std::max(change, std::abs(next-forces[i]));
                forces[i] = next;
            }
            if (change < 1e-10) break;
        }
        if (!predict(forces, measured)) return ERR_DIVERGED;
        bool solved = true;
        for (int i=0; i<n; ++i) {
            const auto& f = s->friction[ids[i]];
            bool capped = std::abs(forces[i]) >= limits[i] - 1e-7;
            if (capped && std::abs(measured[i]) > tolerance && forces[i]*measured[i] < 0) {
                // Breakaway: kinetic friction replaces the larger static bound.
                if (limits[i] > f.kinetic_n) { limits[i] = f.kinetic_n; solved = false; }
                sliding[i] = true;
            } else if (std::abs(measured[i]) > tolerance) {
                solved = false;
                sliding[i] = false;
            } else sliding[i] = false;
            // A contact mode change makes the response piecewise affine.
            // Correct its intercept with the actual trial, then resolve.
            free_speed[i] = measured[i];
            for (int j=0; j<n; ++j) free_speed[i] -= response[i*n+j] * forces[j];
        }
        if (solved) break;
        // A large probe can cross a contact's breakaway point and overestimate
        // the local stopping response. Refresh the Jacobian near the candidate
        // instead of repeatedly applying that incorrect slope.
        if (outer >= 1 && outer < 11) {
            std::vector<double> minus(n), candidate = forces;
            for (int j=0; j<n; ++j) {
                double delta = .001 * std::max(1.0, s->friction[ids[j]].kinetic_n);
                candidate[j] = forces[j] + delta;
                if (!predict(candidate, probe)) return ERR_DIVERGED;
                candidate[j] = forces[j] - delta;
                if (!predict(candidate, minus)) return ERR_DIVERGED;
                candidate[j] = forces[j];
                for (int i=0; i<n; ++i) response[i*n+j] = (probe[i]-minus[i]) / (2*delta);
            }
            for (int i=0; i<n; ++i) {
                free_speed[i] = measured[i];
                for (int j=0; j<n; ++j) free_speed[i] -= response[i*n+j]*forces[j];
            }
        }
        // Safeguarded coordinate roots at contact mode switches. The contact
        // solver can have a tiny discontinuity where a local derivative is not
        // useful; a bracketed solve is slower but cannot take a Newton leap.
        if (outer == 11) {
            bool valid = false;
            for (int sweep=0; sweep<12; ++sweep) {
                for (int i=0; i<n; ++i) {
                    const auto& f = s->friction[ids[i]];
                    double low = -limits[i], high = limits[i];
                    forces[i] = low;
                    if (!predict(forces, probe)) return ERR_DIVERGED;
                    double vlo = probe[i];
                    forces[i] = high;
                    if (!predict(forces, probe)) return ERR_DIVERGED;
                    double vhi = probe[i];
                    sliding[i] = false;
                    if (vlo > tolerance || vhi < -tolerance) {
                        limits[i] = f.kinetic_n;
                        forces[i] = vlo > tolerance ? -f.kinetic_n : f.kinetic_n;
                        sliding[i] = true;
                    } else {
                        for (int bisect=0; bisect<40; ++bisect) {
                            forces[i] = (low+high)*.5;
                            if (!predict(forces, probe)) return ERR_DIVERGED;
                            if (std::abs(probe[i]) <= tolerance*.5) break;
                            if (probe[i] > 0) high = forces[i]; else low = forces[i];
                        }
                    }
                }
                if (!predict(forces, measured)) return ERR_DIVERGED;
                valid = true;
                for (int i=0; i<n; ++i) {
                    bool stopped = std::abs(measured[i]) <= tolerance;
                    bool slip = sliding[i] && forces[i]*measured[i] < 0;
                    if (!stopped && !slip) valid = false;
                    if (stopped) sliding[i] = false;
                }
                if (valid) break;
            }
            if (!valid) {
                for (int i=0; i<n; ++i) std::fprintf(stderr, "friction failure: t=%.6g id=%d v0=%.9g f=%.9g v=%.9g lim=%.9g\n",
                    s->d->time, ids[i], start_speed[i], forces[i], measured[i], limits[i]);
                return ERR_FRICTION;
            }
        }
    }
    s->friction_response = response;
    s->friction_ids = ids;
    s->friction_timestep = s->m->opt.timestep;
    apply();
    return OK;
}

// Advance by `substeps` fixed steps of dt/substeps with ctrl held.
// stats (optional, 6): max ncon, max nefc, max solver iterations, last ncon,
//                      bad-acceleration warnings, substeps executed.
API int lms_step(void* handle, const double* ctrl, double dt, int substeps, const double* origin, double mu,
                 double* qpos_out, double* qvel_out, double* qacc_out, double* actuator_force_out,
                 double* stats) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !ctrl || !origin || substeps < 1 || !(dt > 0) || !std::isfinite(dt)) return ERR_ARGUMENT;
    if (s->orbital_forces_enabled && mu != 0.0) return ERR_ARGUMENT;  // never double gravity
    mjModel* m = s->m;
    mjData* d = s->d;
    if (!finite(ctrl, m->nu)) return ERR_ARGUMENT;
    std::memcpy(d->ctrl, ctrl, sizeof(mjtNum) * m->nu);
    m->opt.timestep = dt / substeps;
    int max_ncon = 0, max_nefc = 0, max_iter = 0, executed = 0;
    int bad_before = d->warning[mjWARN_BADQACC].number;
    for (int k = 0; k < substeps; ++k) {
        apply_tidal(s, origin, mu);
        bool friction_enabled = std::any_of(s->friction.begin(), s->friction.end(),
                                             [](const AxialFriction& f) { return f.enabled; });
        if (friction_enabled) {
            // This split is the same implicitfast step used by mj_step. Its
            // position/velocity stages run ONCE on the authoritative data.
            s->api.step1(m, d);
            apply_tidal(s, origin, mu);  // fresh body rotations for held torques
            // The default contact objective tolerance can terminate before
            // micron/s stiction is resolved, making the force-to-slip map jump
            // during root finding. Solve trial AND authoritative constraints
            // to the same tighter accuracy; keep the friction tolerance and
            // authored physical parameters unchanged.
            const double contact_tolerance = m->opt.tolerance;
            m->opt.tolerance = std::min(contact_tolerance, 1e-12);
            bool integrated = false;
            int friction_status = apply_axial_friction(s, integrated);
            if (friction_status != OK) {
                m->opt.tolerance = contact_tolerance;
                return friction_status;
            }
            d = s->d;
            if (!integrated) s->api.step2(m, d);
            m->opt.tolerance = contact_tolerance;
        } else {
            bool integrated = false;
            apply_axial_friction(s, integrated);  // clear samples/start_z for free interfaces
            s->api.step(m, d);
        }
        for (auto& f : s->friction) {
            double z;
            axial_velocity(s, d, f, &z);
            f.velocity = (z-f.start_z) / m->opt.timestep;
        }
        ++executed;
        if (d->ncon > max_ncon) max_ncon = d->ncon;
        if (d->nefc > max_nefc) max_nefc = d->nefc;
        if (d->solver_niter[0] > max_iter) max_iter = d->solver_niter[0];
        if (d->warning[mjWARN_BADQACC].number != bad_before || !finite(d->qvel, m->nv) ||
            !finite(d->qpos, m->nq)) {
            if (stats) {
                stats[0] = max_ncon; stats[1] = max_nefc; stats[2] = max_iter; stats[3] = d->ncon;
                stats[4] = d->warning[mjWARN_BADQACC].number - bad_before; stats[5] = executed;
            }
            return ERR_DIVERGED;
        }
    }
    if (m->nsensordata > 0) {
        std::memcpy(s->sensor_sample.data(), d->sensordata, sizeof(mjtNum) * m->nsensordata);
    }
    // actuator_force/qacc are those of the LAST substep, i.e. the effort that
    // was actually applied over it. No extra forward pass: it would repeat
    // collision detection for publication only (it doubled the step cost).
    if (qpos_out) std::memcpy(qpos_out, d->qpos, sizeof(mjtNum) * m->nq);
    if (qvel_out) std::memcpy(qvel_out, d->qvel, sizeof(mjtNum) * m->nv);
    if (qacc_out) std::memcpy(qacc_out, d->qacc, sizeof(mjtNum) * m->nv);
    if (actuator_force_out) std::memcpy(actuator_force_out, d->actuator_force, sizeof(mjtNum) * m->nu);
    if (stats) {
        stats[0] = max_ncon; stats[1] = max_nefc; stats[2] = max_iter; stats[3] = d->ncon;
        stats[4] = 0; stats[5] = executed;
    }
    return OK;
}

// Linear momentum (3) and angular momentum about the system COM (3) in L,
// plus the system COM position (3) and velocity (3). For conservation checks.
API int lms_momentum(void* handle, double* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out) return ERR_ARGUMENT;
    mjModel* m = s->m;
    mjData* d = s->d;
    double mass = 0, com[3] = {0, 0, 0}, p[3] = {0, 0, 0};
    // Per-body angular velocity and COM linear velocity, from cvel, which is
    // [rotation; translation] expressed at the tree root's subtree COM.
    std::vector<double> v(6 * m->nbody);
    for (int b = 1; b < m->nbody; ++b) {
        double mb = m->body_mass[b];
        const mjtNum* x = d->xipos + 3 * b;
        const mjtNum* cv = d->cvel + 6 * b;          // about subtree_com[root], world axes
        const mjtNum* c0 = d->subtree_com + 3 * m->body_rootid[b];
        double r[3] = {x[0] - c0[0], x[1] - c0[1], x[2] - c0[2]};
        double lin[3] = {cv[3] + cv[1] * r[2] - cv[2] * r[1], cv[4] + cv[2] * r[0] - cv[0] * r[2],
                         cv[5] + cv[0] * r[1] - cv[1] * r[0]};
        for (int c = 0; c < 3; ++c) {
            v[6 * b + c] = cv[c];
            v[6 * b + 3 + c] = lin[c];
            com[c] += mb * x[c];
            p[c] += mb * lin[c];
        }
        mass += mb;
    }
    if (!(mass > 0)) return ERR_RUNTIME;
    for (int c = 0; c < 3; ++c) com[c] /= mass;
    double vcom[3] = {p[0] / mass, p[1] / mass, p[2] / mass};
    double h[3] = {0, 0, 0};
    for (int b = 1; b < m->nbody; ++b) {
        double mb = m->body_mass[b];
        const mjtNum* x = d->xipos + 3 * b;
        const mjtNum* R = d->ximat + 9 * b;
        const mjtNum* I = m->body_inertia + 3 * b;
        const double* w = &v[6 * b];
        const double* lin = &v[6 * b + 3];
        // I_world * w = R diag(I) R^T w
        double wb[3] = {R[0] * w[0] + R[3] * w[1] + R[6] * w[2], R[1] * w[0] + R[4] * w[1] + R[7] * w[2],
                        R[2] * w[0] + R[5] * w[1] + R[8] * w[2]};
        double Iw_b[3] = {I[0] * wb[0], I[1] * wb[1], I[2] * wb[2]};
        double Iw[3] = {R[0] * Iw_b[0] + R[1] * Iw_b[1] + R[2] * Iw_b[2],
                        R[3] * Iw_b[0] + R[4] * Iw_b[1] + R[5] * Iw_b[2],
                        R[6] * Iw_b[0] + R[7] * Iw_b[1] + R[8] * Iw_b[2]};
        double r[3] = {x[0] - com[0], x[1] - com[1], x[2] - com[2]};
        double u[3] = {lin[0] - vcom[0], lin[1] - vcom[1], lin[2] - vcom[2]};
        h[0] += Iw[0] + mb * (r[1] * u[2] - r[2] * u[1]);
        h[1] += Iw[1] + mb * (r[2] * u[0] - r[0] * u[2]);
        h[2] += Iw[2] + mb * (r[0] * u[1] - r[1] * u[0]);
    }
    for (int c = 0; c < 3; ++c) {
        out[c] = p[c];
        out[3 + c] = h[c];
        out[6 + c] = com[c];
        out[9 + c] = vcom[c];
    }
    return OK;
}

// Read-only contact snapshot from the authoritative native model. Returns ncon.
// Each row: geom1, geom2, body1, body2, signed separation [m]. No forward/step.
API int lms_contacts(void* handle, double* out, int capacity) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || capacity < 0 || (capacity > 0 && !out)) return ERR_ARGUMENT;
    for (int i = 0; i < s->d->ncon && i < capacity; ++i) {
        const auto& c = s->d->contact[i];
        out[5*i] = c.geom[0]; out[5*i+1] = c.geom[1];
        out[5*i+2] = c.geom[0] >= 0 ? s->m->geom_bodyid[c.geom[0]] : -1;
        out[5*i+3] = c.geom[1] >= 0 ? s->m->geom_bodyid[c.geom[1]] : -1;
        out[5*i+4] = c.dist;
    }
    return s->d->ncon;
}

// Read-only diagnostic samples in the local inertial axes. No forward/step calls.
API int lms_contact_detail(void* handle, double* out, int capacity) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || capacity < 0 || (capacity > 0 && !out)) return ERR_ARGUMENT;
    for (int i=0; i<s->d->ncon && i<capacity; ++i) {
        const auto& c=s->d->contact[i]; double* r=out+23*i;
        r[0]=c.geom[0]; r[1]=c.geom[1]; r[2]=c.dist;
        for(int k=0;k<3;++k) r[3+k]=c.pos[k];
        mjtNum force[6]={}; s->api.contactForce(s->m,s->d,i,force);
        for(int k=0;k<6;++k) r[6+k]=force[k];
        for(int k=0;k<9;++k) r[12+k]=c.frame[k];
        r[21]=c.efc_address; r[22]=s->d->time;
    }
    return s->d->ncon;
}
API int lms_body_detail(void* handle, int body, double* out) {
    auto* s=static_cast<Stepper*>(handle);
    if(!s || !out || body<0 || body>=s->m->nbody) return ERR_ARGUMENT;
    for(int k=0;k<3;++k) out[k]=s->d->xpos[3*body+k];
    for(int k=0;k<4;++k) out[3+k]=s->d->xquat[4*body+k];
    mjtNum vel[6]={}; s->api.objectVelocity(s->m,s->d,mjOBJ_BODY,body,vel,0);
    for(int k=0;k<6;++k) out[7+k]=vel[k];
    out[13]=s->d->time; return OK;
}
