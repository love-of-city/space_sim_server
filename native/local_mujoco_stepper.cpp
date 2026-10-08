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
#include <cstdint>
#include <cstring>
#include <new>
#include <vector>
#include <mujoco/mujoco.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define API extern "C" __declspec(dllexport)
#else
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
};

struct Api {
    int (*version)();
    mjModel* (*loadXML)(const char*, const mjVFS*, char*, int);
    void (*deleteModel)(mjModel*);
    mjData* (*makeData)(const mjModel*);
    void (*deleteData)(mjData*);
    void (*resetData)(const mjModel*, mjData*);
    void (*forward)(const mjModel*, mjData*);
    void (*step)(const mjModel*, mjData*);
    void (*kinematics)(const mjModel*, mjData*);
    void (*comPos)(const mjModel*, mjData*);
    int (*name2id)(const mjModel*, int, const char*);
    const char* (*id2name)(const mjModel*, int, int);
};

struct Stepper {
    Api api{};
    mjModel* m = nullptr;
    mjData* d = nullptr;
    std::vector<int> free_bodies;  // bodies whose root joint is free
    // Held external wrench per body: force in the inertial/local axes [N] at
    // the body COM, torque in the BODY frame [N*m] (Basilisk CmdForceInertial /
    // CmdTorqueBody conventions). Rotated with the body at every substep.
    std::vector<double> force_world;
    std::vector<double> torque_body;
    std::vector<double> orbital_force;  // BSK-computed differential force, inertial axes
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
API void* lms_create(const wchar_t* runtime_path, const char* xml_path, char* error, int error_size) {
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
    if (!module) {
        copy_error(error, error_size, "Basilisk mujoco.dll is not loaded in this process");
        delete s;
        return nullptr;
    }
    Api& a = s->api;
    bool ok = resolve(module, "mj_version", a.version) && resolve(module, "mj_loadXML", a.loadXML) &&
              resolve(module, "mj_deleteModel", a.deleteModel) && resolve(module, "mj_makeData", a.makeData) &&
              resolve(module, "mj_deleteData", a.deleteData) && resolve(module, "mj_resetData", a.resetData) &&
              resolve(module, "mj_forward", a.forward) && resolve(module, "mj_step", a.step) &&
              resolve(module, "mj_kinematics", a.kinematics) && resolve(module, "mj_comPos", a.comPos) &&
              resolve(module, "mj_name2id", a.name2id) &&
              resolve(module, "mj_id2name", a.id2name);
    if (!ok) {
        copy_error(error, error_size, "required MuJoCo symbol is missing from the loaded runtime");
        delete s;
        return nullptr;
    }
#else
    copy_error(error, error_size, "only the Windows runtime lookup is implemented");
    delete s;
    return nullptr;
#endif
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
    s->api.resetData(s->m, s->d);
    s->api.forward(s->m, s->d);
    return s;
}

API void lms_destroy(void* handle) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s) return;
    if (s->d) s->api.deleteData(s->d);
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
        s->api.step(m, d);
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
