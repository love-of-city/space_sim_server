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
#include <mutex>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
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
    mjModel* (*loadModel)(const char*, const mjVFS*);
    mjtSize (*sizeModel)(const mjModel*);
    void (*saveModel)(const mjModel*, const char*, void*, int);
    void (*defaultVFS)(mjVFS*);
    int (*addBufferVFS)(mjVFS*, const char*, const void*, int);
    void (*deleteVFS)(mjVFS*);
    void (*deleteModel)(mjModel*);
    mjData* (*makeData)(const mjModel*);
    void (*deleteData)(mjData*);
    mjData* (*copyData)(mjData*, const mjModel*, const mjData*);
    int (*stateSize)(const mjModel*, int);
    void (*getState)(const mjModel*, const mjData*, mjtNum*, int);
    void (*setState)(const mjModel*, mjData*, const mjtNum*, int);
    const mjpPlugin* (*pluginAtSlot)(int);
    mjfSensor* sensor_callback = nullptr;
    mjfAct* act_dyn_callback = nullptr;
    mjfAct* act_gain_callback = nullptr;
    mjfAct* act_bias_callback = nullptr;
    void (*resetData)(const mjModel*, mjData*);
    void (*forward)(const mjModel*, mjData*);
    void (*step)(const mjModel*, mjData*);
    void (*step1)(const mjModel*, mjData*);
    void (*step2)(const mjModel*, mjData*);
    void (*kinematics)(const mjModel*, mjData*);
    void (*comPos)(const mjModel*, mjData*);
    void (*quat2Mat)(mjtNum*, const mjtNum*);
    int (*name2id)(const mjModel*, int, const char*);
    const char* (*id2name)(const mjModel*, int, int);
    void (*contactForce)(const mjModel*, const mjData*, int, mjtNum*);
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

struct FrictionScratch {
    std::vector<int> ids;
    std::vector<double> axes,start_speed,previous_forces,forces,free_speed,measured,probe,response,limits,minus,candidate;
    std::vector<bool> sliding;
};
struct Stepper {

    Api api{};
    mjModel* m = nullptr;
    mjData* d = nullptr;
    mjData* friction_trial = nullptr;  // scratch data for the generic full-copy path
    std::vector<AxialFriction> friction;
    std::vector<int> friction_ids;
    std::vector<double> friction_response;
    double friction_timestep = 0;
    std::int64_t friction_predict_calls = 0;
    std::int64_t friction_reused_steps = 0;
    std::int64_t friction_unretained_steps = 0;
    std::int64_t friction_solver_retries = 0;
    std::int64_t friction_full_copies = 0;
    std::int64_t friction_state_restores = 0;
    std::vector<mjtNum> friction_start_state;
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
    FrictionScratch scratch;
};

// A single immutable compiled MJB, before servos/options are modified. Keeping
// bytes avoids mjModel/runtime lifetime coupling at library shutdown. New
// episodes deserialize into independent models and allocate fresh mjData.
std::mutex model_cache_mutex;
std::string model_cache_key;
std::vector<char> model_cache_bytes;

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

// Query-local scope for the private planning model only. It discards pair
// classes ignored by pose_collision. No model collision masks are mutated.
static thread_local const mjModel* planning_model=nullptr;
static thread_local std::vector<unsigned char> planning_relevant;
static thread_local unsigned long long planning_skipped=0;
static bool irrelevant_planning_pair(const mjModel* m,int a,int b) {
    if(m!=planning_model) return false;
    const int ba=m->geom_bodyid[a],bb=m->geom_bodyid[b];
    const bool skip=!planning_relevant[ba] && !planning_relevant[bb];
    planning_skipped+=skip;
    return skip;
}
API int lms_probe_relevance_begin(void* handle,const unsigned char* mask,int count) {
    auto* s=static_cast<Stepper*>(handle);
    if(!s || !mask || count!=s->m->nbody || planning_model) return ERR_ARGUMENT;
    planning_relevant.assign(mask,mask+count);
    planning_model=s->m;
    return OK;
}
API int lms_probe_relevance_end(void* handle) {
    auto* s=static_cast<Stepper*>(handle);
    if(!s || planning_model!=s->m) return ERR_ARGUMENT;
    planning_model=nullptr;
    return OK;
}
API unsigned long long lms_probe_relevance_skipped() {return planning_skipped;}

#ifdef _WIN32
using RuntimeModule = HMODULE;
static void* runtime_symbol(RuntimeModule module, const char* name) {
    return reinterpret_cast<void*>(GetProcAddress(module, name));
}
#else
using RuntimeModule = void*;
static void* runtime_symbol(RuntimeModule module, const char* name) { return dlsym(module, name); }
#endif
#include <unordered_set>
// Global MuJoCo callbacks also see Basilisk publication models. Only apply
// these filters to locally owned models whose lifetime we explicitly track.
static thread_local std::unordered_set<const mjModel*> filter_models;

// Experimental conservative separation test for cylinder/convex-mesh dispatch.
// A cylinder projection on each mesh-box axis is bounded analytically.
// The original callback handles every pair we cannot prove separated.
#include <cstdlib>
static mjfCollision original_cylinder_mesh = nullptr;
static int filter_mode = 0;  // 0 off, 1 shadow validation, 2 skip proven separated
static unsigned long long filter_calls=0, filter_separated=0, filter_missed=0;

static bool cylinder_mesh_separated(const mjModel* m, const mjData* d,
                                    int cylinder, int mesh, mjtNum margin) {
    const auto* c = d->geom_xpos + 3*cylinder;
    const auto* p = d->geom_xpos + 3*mesh;
    const auto* cr = d->geom_xmat + 9*cylinder;
    const auto* mr = d->geom_xmat + 9*mesh;
    const auto* box = m->geom_aabb + 6*mesh;
    const double radius=m->geom_size[3*cylinder], half=m->geom_size[3*cylinder+1];
    const double padding=std::max(0.,margin)+1e-9;
    for (int axis=0; axis<3; ++axis) {
        double center=0., dot=0.;
        for(int i=0;i<3;++i) {
            center += mr[3*i+axis]*(c[i]-p[i]);
            dot += mr[3*i+axis]*cr[3*i+2];
        }
        // Geom transforms are orthonormal. Roundoff is covered by padding.
        dot=std::clamp(dot,-1.,1.);
        const double extent=half*std::abs(dot)+radius*std::sqrt(std::max(0.,1.-dot*dot));
        if (std::abs(center-box[axis]) > box[axis+3]+extent+padding) return true;
    }
    return false;
}
static int filtered_cylinder_mesh(const mjModel* m, mjData* d, mjContact* contacts,
                                 int g1,int g2,mjtNum margin) {
    if(irrelevant_planning_pair(m,g1,g2)) return 0;
    if(!filter_models.count(m)) return original_cylinder_mesh(m,d,contacts,g1,g2,margin);
    ++filter_calls;
    const bool separated=cylinder_mesh_separated(m,d,g1,g2,margin);
    filter_separated += separated;
    if (separated && filter_mode==2) return 0;
    const int n=original_cylinder_mesh(m,d,contacts,g1,g2,margin);
    if (separated && n) ++filter_missed;
    return n;
}
static void install_cylinder_filter(RuntimeModule module) {
    if(original_cylinder_mesh) return;
    const char* value=std::getenv("CYLINDER_FILTER_MODE");
    filter_mode=value ? std::atoi(value) : 0;
    if (!filter_mode) return;
    auto* table=reinterpret_cast<mjfCollision*>(runtime_symbol(module,"mjCOLLISIONFUNC"));
    if(!table) return;
    auto& callback=table[mjGEOM_CYLINDER*mjNGEOMTYPES+mjGEOM_MESH];
    original_cylinder_mesh=callback;
    if(callback) callback=filtered_cylinder_mesh;
}
API void experiment_filter_reset() { filter_calls=filter_separated=filter_missed=0; }
API void experiment_filter_stats(unsigned long long* out) {
    out[0]=filter_calls;out[1]=filter_separated;out[2]=filter_missed;out[3]=filter_mode;
}

// Experimental proof of convex mesh separation. A cached direction is only a
// search hint: every invocation reprojects ALL current vertices. No prior
// contact result, force, distance, or pose is reused.
#include <unordered_map>
#include <array>
#include <limits>
struct MeshPairKey {
  const mjModel* model;
  int a,b;
  bool operator==(const MeshPairKey& x) const {return model==x.model && a==x.a && b==x.b;}
};
struct MeshPairHash {
  size_t operator()(const MeshPairKey& x) const {
    return (reinterpret_cast<uintptr_t>(x.model)>>4) ^
           (static_cast<size_t>(x.a)*2654435761u) ^ (static_cast<size_t>(x.b)*2246822519u);
  }
};
struct MeshHint {std::array<double,3> local_axis{}; int support[4]={}; int separation_side=0; bool valid=false; unsigned cooldown=0;};
static thread_local std::unordered_map<MeshPairKey,MeshHint,MeshPairHash> mesh_hints;
static mjfCollision original_mesh_mesh=nullptr;
static int mesh_mode=0;
static unsigned long long mesh_calls=0,mesh_separated=0,mesh_missed=0,mesh_searches=0,mesh_hits=0;

// Warm-start a linear support query on MuJoCo's compiled convex hull graph.
// A graph is only an acceleration structure for the same original vertices.
// Negative and positive directions are searched independently. Each update
// rechecks the current direction against adjacent hull vertices.
static double mesh_support(const mjModel* m,int mesh,const double direction[3],
                           int sign,int& cached_index) {
  const auto* vertices=m->mesh_vert+3*m->mesh_vertadr[mesh];
  const int adr=m->mesh_graphadr[mesh];
  if(adr<0) {
    double best=-INFINITY;
    for(int i=0;i<m->mesh_vertnum[mesh];++i) {
      double value=sign*(direction[0]*vertices[3*i]+direction[1]*vertices[3*i+1]+direction[2]*vertices[3*i+2]);
      best=std::max(best,value);
    }
    return best;
  }
  const auto* graph=m->mesh_graph+adr;
  const int n=graph[0];const auto* edgeadr=graph+2;const auto* ids=graph+2+n;
  const auto* neighbors=graph+2+2*n;
  int vertex=(cached_index>=0 && cached_index<n)?cached_index:0;
  const auto dot=[&](int k) {
    const auto* v=vertices+3*ids[k];
    return sign*(direction[0]*v[0]+direction[1]*v[1]+direction[2]*v[2]);
  };
  double best=dot(vertex);
  for(int steps=0;steps<n;++steps) {
    int next=vertex;
    for(int i=edgeadr[vertex];neighbors[i]>=0;++i) {
      int neighbor=neighbors[i];double value=dot(neighbor);
      if(value>best){best=value;next=neighbor;}
    }
    if(next==vertex)break;
    vertex=next;
  }
  cached_index=vertex;
  return best;
}

// Independent numerical audit: compare warm support to a full vertex scan.
API int experiment_support_audit(void* handle,unsigned long long* result) {
  auto* s=static_cast<Stepper*>(handle);if(!s)return -1;
  const auto* m=s->m;
  unsigned long long checked=0,failed=0;
  std::uint64_t seed=20261030;
  for(int mesh=0;mesh<m->nmesh;++mesh) {
    if(m->mesh_vertnum[mesh]>256)continue;
    int cached[2]={};
    for(int sample=0;sample<64;++sample) {
      double direction[3];
      for(auto& x:direction) {seed=seed*6364136223846793005ULL+1442695040888963407ULL;x=double(seed>>11)/9007199254740992.*2.-1.;}
      const auto* v=m->mesh_vert+3*m->mesh_vertadr[mesh];
      for(int k=0;k<2;++k) {
        int sign=k?1:-1;double exact=-INFINITY;
        for(int j=0;j<m->mesh_vertnum[mesh];++j)
          exact=std::max(exact,sign*(direction[0]*v[3*j]+direction[1]*v[3*j+1]+direction[2]*v[3*j+2]));
        const double fast=mesh_support(m,mesh,direction,sign,cached[k]);
        ++checked;failed+=std::abs(fast-exact)>1e-12;
      }
    }
  }
  result[0]=checked;result[1]=failed;return failed?1:0;
}

// Both sets are the exact float vertex arrays consumed by MuJoCo's convex
// support function. Separation of their projections proves separation of the
// entire convex hulls. Positive contact margin and numerical guard are added.
static bool mesh_axis_separates(const mjModel* m,const mjData* d,int a,int b,
                                const double axis[3],double margin,MeshHint& hint) {
  const double norm=std::sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
  if (!(norm>1e-12)) return false;
  const auto* ra=d->geom_xmat+9*a;const auto* rb=d->geom_xmat+9*b;
  const auto* pa=d->geom_xpos+3*a;const auto* pb=d->geom_xpos+3*b;
  double world[3]={},other[3]={},offset=0;
  for(int i=0;i<3;++i) {
    for(int j=0;j<3;++j)world[i]+=ra[3*i+j]*axis[j];
    offset+=world[i]*(pb[i]-pa[i]);
  }
  for(int j=0;j<3;++j)for(int i=0;i<3;++i)other[j]+=rb[3*i+j]*world[i];
  const int ma=m->geom_dataid[a],mb=m->geom_dataid[b];
  const double pad=(std::max(0.,margin)+1e-8)*norm;
  // A single strict interval inequality already proves separation. Recheck
  // its two current support values first; only compute the other two when
  // necessary. The remembered side is merely an evaluation-order hint.
  // The support arrays, arithmetic, margins and fallback remain unchanged.
  for(int attempt=0;attempt<2;++attempt) {
    const int side=attempt ? 1-hint.separation_side : hint.separation_side;
    bool separated;
    if(side==0) {
      const double hi_a=mesh_support(m,ma,axis,1,hint.support[1]);
      const double lo_b=-mesh_support(m,mb,other,-1,hint.support[2]);
      separated=hi_a+pad < lo_b+offset;
    } else {
      const double lo_a=-mesh_support(m,ma,axis,-1,hint.support[0]);
      const double hi_b=mesh_support(m,mb,other,1,hint.support[3]);
      separated=hi_b+offset+pad < lo_a;
    }
    if(separated) {hint.separation_side=side;return true;}
  }
  return false;
}

static bool prove_mesh_separation(const mjModel* m,const mjData* d,int a,int b,double margin) {
  const int ma=m->geom_dataid[a],mb=m->geom_dataid[b];
  // Bound probe overhead on dense meshes; original solver always handles them.
  if(ma<0 || mb<0 || m->mesh_vertnum[ma]>256 || m->mesh_vertnum[mb]>256) return false;
  if(mesh_hints.size()>200000)mesh_hints.clear();
  auto& hint=mesh_hints[{m,a,b}];
  if(hint.valid && mesh_axis_separates(m,d,a,b,hint.local_axis.data(),margin,hint)) {
    ++mesh_hits;return true;
  }
  if(hint.cooldown) {--hint.cooldown;return false;}
  ++mesh_searches;
  const auto* ra=d->geom_xmat+9*a;const auto* rb=d->geom_xmat+9*b;
  const auto* pa=d->geom_xpos+3*a;const auto* pb=d->geom_xpos+3*b;
  // Seven cheap directions: geom-A axes, geom-B axes, center displacement.
  for(int k=0;k<7;++k) {
    double axis[3]={};
    if(k<3)axis[k]=1;
    else for(int j=0;j<3;++j)for(int i=0;i<3;++i)
      axis[j]+=ra[3*i+j]*(k<6?rb[3*i+k-3]:pb[i]-pa[i]);
    if(mesh_axis_separates(m,d,a,b,axis,margin,hint)) {
      std::copy(axis,axis+3,hint.local_axis.begin());hint.valid=true;hint.cooldown=0;return true;
    }
  }
  // Only delay a future optimization attempt, never delay the original query.
  hint.valid=false;hint.cooldown=31;return false;
}
static int filtered_mesh_mesh(const mjModel* m,mjData* d,mjContact* con,int a,int b,mjtNum margin) {
  if(irrelevant_planning_pair(m,a,b)) return 0;
  if(!filter_models.count(m)) return original_mesh_mesh(m,d,con,a,b,margin);
  ++mesh_calls;
  bool separated=prove_mesh_separation(m,d,a,b,margin);mesh_separated+=separated;
  if(separated && mesh_mode==2)return 0;
  int n=original_mesh_mesh(m,d,con,a,b,margin);
  if(separated && n)++mesh_missed;
  return n;
}
static void install_mesh_filter(RuntimeModule module) {
  if(original_mesh_mesh)return;
  const char* mode=std::getenv("MESH_FILTER_MODE");mesh_mode=mode?std::atoi(mode):0;
  if(!mesh_mode)return;
  auto* table=reinterpret_cast<mjfCollision*>(runtime_symbol(module,"mjCOLLISIONFUNC"));
  if(!table)return;
  auto& callback=table[mjGEOM_MESH*mjNGEOMTYPES+mjGEOM_MESH];
  original_mesh_mesh=callback;if(callback)callback=filtered_mesh_mesh;
}
API void experiment_mesh_reset() {mesh_calls=mesh_separated=mesh_missed=mesh_searches=mesh_hits=0;}
API void experiment_mesh_stats(unsigned long long* out) {
  out[0]=mesh_calls;out[1]=mesh_separated;out[2]=mesh_missed;out[3]=mesh_searches;out[4]=mesh_hits;out[5]=mesh_mode;
}

// Geometric containment certificate for the CURRENT SARM two-start male (mode
// 0) / female (mode 1) plugin. R,t map male plugin coordinates to female ones.
// This only proves nonintersection; it never computes approximate contacts.
#include <cmath>
#include <algorithm>
static double certified_thread_gap(const double R[9], const double t[3]) {
  constexpr double root=.0135,height=.0015,clearance=.00015,length=.060;
  constexpr double radius=root+height,w=2*3.14159265358979323846/.008;
  const double yaw=std::atan2(R[3],R[0]),c=std::cos(yaw),s=std::sin(yaw);
  const double Q[9]={c,-s,0,s,c,0,0,0,1};
  if(!(R[8]>.99))return -1.;
  // Only male points within the female's axial solid [-.020,0] can collide.
  // Enlarge that necessary interval to cover the maximum XY-induced z shift.
  const double zerror=radius*std::hypot(R[6],R[7])+1e-9;
  const double zmin=std::max(-length,(-.020-t[2]-zerror)/R[8]);
  const double zmax=std::min(0.,(-t[2]+zerror)/R[8]);
  if(zmin>zmax+1e-7)return 1.;
  if(zmin>zmax)return -1.;
  const double zcenter=.5*(zmin+zmax),zhalf=.5*(zmax-zmin);
  double exy[2];
  for(int i=0;i<2;++i) {
    const double a=R[3*i]-Q[3*i],b=R[3*i+1]-Q[3*i+1],z=R[3*i+2];
    // All points in the male solid are inside this finite cylinder.
    exy[i]=std::abs(t[i]+zcenter*z)+radius*std::hypot(a,b)+zhalf*std::abs(z);
  }
  const double xy=std::hypot(exy[0],exy[1]);
  if (!(xy<clearance))return -1.;
  const double dz=radius*std::hypot(R[6],R[7])+std::max(std::abs(zmin),std::abs(zmax))*std::abs(R[8]-1.);
  // For r>=root, angle error <=asin(xy/root). For r<root, xy<clearance
  // already guarantees containment within the female's minimum bore radius.
  const double phase=w*t[2]-2*yaw;  // male/female phases both exactly .004 m
  const double nominal=height*std::abs(std::sin(.5*phase));
  const double perturbation=.5*height*(w*dz+2*std::asin(xy/root));
  return clearance-nominal-perturbation-xy;
}

static mjfCollision original_thread_sdf=nullptr;
static const mjpPlugin* (*thread_plugin_at_slot)(int)=nullptr;
static const char* (*thread_plugin_config)(const mjModel*,int,const char*)=nullptr;
static int thread_mode=0;
static unsigned long long thread_calls=0,thread_eligible=0,thread_separated=0,thread_missed=0;

static int uncached_sarm_mode(const mjModel* m,int g) {
  int instance=m->geom_plugin[g];
  if(instance<0 || !thread_plugin_at_slot || !thread_plugin_config)return -1;
  const auto* p=thread_plugin_at_slot(m->plugin[instance]);
  if(!p || std::strcmp(p->name,"sarm.sdf.two_start_thread"))return -1;
  const char* value=thread_plugin_config(m,instance,"mode");
  if(!value)return -1;
  return !std::strcmp(value,"0")?0:!std::strcmp(value,"1")?1:-1;
}
static thread_local std::unordered_map<const mjModel*,std::vector<int>> thread_modes;
static int current_sarm_mode(const mjModel* m,int g) {
  auto& modes=thread_modes[m];
  if(modes.empty()) modes.assign(m->ngeom,-2);
  if(modes[g]==-2) modes[g]=uncached_sarm_mode(m,g);
  return modes[g];
}
static bool thread_pair_separated(const mjModel* m,const mjData* d,int a,int b,double margin) {
  // Positive collision margins require a Euclidean distance bound, not merely
  // a proof of containment. They always use the original callback.
  if(margin!=0.)return false;
  int ma=current_sarm_mode(m,a),mb=current_sarm_mode(m,b);
  if(!((ma==0 && mb==1)||(ma==1 && mb==0)))return false;
  ++thread_eligible;
  const int male=ma==0?a:b,female=ma==1?a:b;
  const auto* ra=d->geom_xmat+9*male;const auto* rb=d->geom_xmat+9*female;
  const auto* pa=d->geom_xpos+3*male;const auto* pb=d->geom_xpos+3*female;
  double R[9]={},t[3]={};
  for(int i=0;i<3;++i)for(int k=0;k<3;++k) {
    t[i]+=rb[3*k+i]*(pa[k]-pb[k]);
    for(int j=0;j<3;++j)R[3*i+j]+=rb[3*k+i]*ra[3*k+j];
  }
  return certified_thread_gap(R,t)>1e-7;  // 0.1 micrometre numerical guard
}
static int filtered_thread_sdf(const mjModel* m,mjData* d,mjContact* con,int a,int b,mjtNum margin) {
  if(irrelevant_planning_pair(m,a,b)) return 0;
  if(!filter_models.count(m)) return original_thread_sdf(m,d,con,a,b,margin);
  ++thread_calls;
  const bool separated=thread_pair_separated(m,d,a,b,margin);thread_separated+=separated;
  if(separated && thread_mode==2)return 0;
  const int n=original_thread_sdf(m,d,con,a,b,margin);
  if(separated && n)++thread_missed;
  return n;
}
static void install_thread_filter(RuntimeModule module) {
  if(original_thread_sdf)return;
  const char* value=std::getenv("THREAD_FILTER_MODE");thread_mode=value?std::atoi(value):0;
  if(!thread_mode)return;
  thread_plugin_at_slot=reinterpret_cast<decltype(thread_plugin_at_slot)>(runtime_symbol(module,"mjp_getPluginAtSlot"));
  thread_plugin_config=reinterpret_cast<decltype(thread_plugin_config)>(runtime_symbol(module,"mj_getPluginConfig"));
  auto* table=reinterpret_cast<mjfCollision*>(runtime_symbol(module,"mjCOLLISIONFUNC"));
  if(!table || !thread_plugin_at_slot || !thread_plugin_config)return;
  auto& callback=table[mjGEOM_SDF*mjNGEOMTYPES+mjGEOM_SDF];
  original_thread_sdf=callback;if(callback)callback=filtered_thread_sdf;
}
API void experiment_thread_reset() {thread_calls=thread_eligible=thread_separated=thread_missed=0;}
API void experiment_thread_stats(unsigned long long* out) {
  out[0]=thread_calls;out[1]=thread_eligible;out[2]=thread_separated;out[3]=thread_missed;out[4]=thread_mode;
}

static void (*probe_fwd_position)(const mjModel*, mjData*) = nullptr;
API int lms_abi() { return ABI; }
API int lms_header_version() { return mjVERSION_HEADER; }

// runtime_path: absolute path of the mujoco.dll Basilisk already loaded.
#ifdef _WIN32
using RuntimePathChar = wchar_t;
#else
using RuntimePathChar = char;
#endif
API void* lms_create_cached(const RuntimePathChar* runtime_path, const char* xml_path,
                           const char* cache_key, std::int64_t* cache_info, char* error, int error_size) {
    if (cache_info) cache_info[0] = cache_info[1] = cache_info[2] = 0;
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
    install_cylinder_filter(module);
    install_mesh_filter(module);
    install_thread_filter(module);
    probe_fwd_position = reinterpret_cast<decltype(probe_fwd_position)>(runtime_symbol(module, "mj_fwdPosition"));
    Api& a = s->api;
    bool ok = resolve(module, "mj_version", a.version) && resolve(module, "mj_loadXML", a.loadXML) &&
              resolve(module, "mj_loadModel", a.loadModel) && resolve(module, "mj_sizeModel", a.sizeModel) &&
              resolve(module, "mj_saveModel", a.saveModel) && resolve(module, "mj_defaultVFS", a.defaultVFS) &&
              resolve(module, "mj_addBufferVFS", a.addBufferVFS) && resolve(module, "mj_deleteVFS", a.deleteVFS) &&
              resolve(module, "mj_deleteModel", a.deleteModel) && resolve(module, "mj_makeData", a.makeData) &&
              resolve(module, "mj_deleteData", a.deleteData) && resolve(module, "mj_resetData", a.resetData) &&
              resolve(module, "mj_copyData", a.copyData) && resolve(module, "mju_quat2Mat", a.quat2Mat) &&
              resolve(module, "mj_stateSize", a.stateSize) && resolve(module, "mj_getState", a.getState) &&
              resolve(module, "mj_setState", a.setState) && resolve(module, "mjp_getPluginAtSlot", a.pluginAtSlot) &&
              resolve(module, "mjcb_sensor", a.sensor_callback) && resolve(module, "mjcb_act_dyn", a.act_dyn_callback) &&
              resolve(module, "mjcb_act_gain", a.act_gain_callback) && resolve(module, "mjcb_act_bias", a.act_bias_callback) &&
              resolve(module, "mj_forward", a.forward) && resolve(module, "mj_step", a.step) &&
              resolve(module, "mj_step1", a.step1) && resolve(module, "mj_step2", a.step2) &&
              resolve(module, "mj_kinematics", a.kinematics) && resolve(module, "mj_comPos", a.comPos) &&
              resolve(module, "mj_name2id", a.name2id) &&
              resolve(module, "mj_id2name", a.id2name) &&
              resolve(module, "mj_contactForce", a.contactForce);
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
    {
        std::lock_guard<std::mutex> lock(model_cache_mutex);
        const bool cacheable = cache_key && cache_key[0];
        if (cacheable && model_cache_key == cache_key && !model_cache_bytes.empty()) {
            mjVFS vfs;
            a.defaultVFS(&vfs);
            if (a.addBufferVFS(&vfs, "physics.mjb", model_cache_bytes.data(),
                               static_cast<int>(model_cache_bytes.size())) == 0)
                s->m = a.loadModel("physics.mjb", &vfs);
            a.deleteVFS(&vfs);
            if (cache_info && s->m) cache_info[0] = 1;
        }
        if (!s->m) {
            s->m = a.loadXML(xml_path, nullptr, load_error, sizeof(load_error));
            if (s->m && cacheable) {
                const mjtSize size = a.sizeModel(s->m);
                // One entry, bounded independently of the worker RSS limit.
                if (size > 0 && size <= 1024 * 1024 * 1024) {
                    model_cache_bytes.resize(size);
                    a.saveModel(s->m, nullptr, model_cache_bytes.data(), static_cast<int>(size));
                    model_cache_key = cache_key;
                } else {
                    std::vector<char>().swap(model_cache_bytes);
                    model_cache_key.clear();
                }
            }
        }
        if (cache_info) cache_info[1] = model_cache_bytes.size();
        if (cache_info && s->m) cache_info[2] = a.sizeModel(s->m);
    }
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
    filter_models.insert(s->m);
    s->api.forward(s->m, s->d);
    return s;
}

API void* lms_create(const RuntimePathChar* runtime_path, const char* xml_path, char* error, int error_size) {
    return lms_create_cached(runtime_path, xml_path, nullptr, nullptr, error, error_size);
}

API void lms_destroy(void* handle) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s) return;
    if (s->d) s->api.deleteData(s->d);
    if (s->friction_trial) s->api.deleteData(s->friction_trial);
    if (s->m) {
        if(planning_model==s->m) planning_model=nullptr;
        filter_models.erase(s->m);
        thread_modes.erase(s->m);
        for(auto it=mesh_hints.begin();it!=mesh_hints.end();) {
            if(it->first.model==s->m) it=mesh_hints.erase(it); else ++it;
        }
        s->api.deleteModel(s->m);
    }
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

// Body origin and body-to-local rotation at the same kinematics phase as
// d->contact (the last substep start). Do not call forward on the live data.
API int lms_body_geometry_pose(void* handle, int body, double* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out || body < 0 || body >= s->m->nbody) return ERR_ARGUMENT;
    for (int i=0; i<3; ++i) out[i] = s->d->xpos[3*body+i];
    for (int i=0; i<9; ++i) out[3+i] = s->d->xmat[9*body+i];
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

// Private planning model only: geometry queries never consume force/acceleration.
API int lms_probe_set_state(void* handle, const double* qpos, const double* qvel) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !qpos || !qvel) return ERR_ARGUMENT;
    if (!finite(qpos, s->m->nq) || !finite(qvel, s->m->nv)) return ERR_ARGUMENT;
    std::memcpy(s->d->qpos, qpos, sizeof(mjtNum) * s->m->nq);
    std::memcpy(s->d->qvel, qvel, sizeof(mjtNum) * s->m->nv);
    std::memset(s->d->qacc_warmstart, 0, sizeof(mjtNum) * s->m->nv);
    if (!probe_fwd_position) return ERR_RUNTIME;
    probe_fwd_position(s->m, s->d);
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
// and solve for the force that stops relative slip. Eligible models can restore
// a complete integration checkpoint between trials inside this synchronous call
// and retain the accepted step2 result. Other models use the original scratch
// copy/predict/apply path, including its callback and plugin behavior.
// This avoids sign(v) chatter and viscous creep without an axial position spring.
// Sliding uses a fixed kinetic limit; the last stopping impulse may be smaller.
static bool can_restore_friction_trial(const Stepper* s) {
    const mjModel* m = s->m;
    // This optimization is specific to the pinned MuJoCo 3.7 implicitfast /
    // Newton path: step2 recomputes acceleration/constraints but leaves step1's
    // geometry, velocity terms and mass factorization intact. Sleeping, history,
    // stateful actuators/plugins and arbitrary callbacks require a full copy.
    if (m->opt.integrator != mjINT_IMPLICITFAST || m->opt.solver != mjSOL_NEWTON ||
        (m->opt.enableflags & (mjENBL_SLEEP | mjENBL_FWDINV)) || m->nhistory || m->na ||
        m->npluginstate || *s->api.sensor_callback || *s->api.act_dyn_callback ||
        *s->api.act_gain_callback || *s->api.act_bias_callback) return false;
    for (int i=0; i<m->nplugin; ++i) {
        const mjpPlugin* plugin = s->api.pluginAtSlot(m->plugin[i]);
        if (!plugin || plugin->capabilityflags != mjPLUGIN_SDF || plugin->advance) return false;
    }
    return true;
}

static int apply_axial_friction(Stepper* s, mjData*& accepted_trial) {
    accepted_trial = nullptr;
    auto& w=s->scratch;
    auto& ids=w.ids; auto& axes=w.axes; auto& start_speed=w.start_speed; auto& previous_forces=w.previous_forces;
    ids.clear(); axes.clear(); start_speed.clear(); previous_forces.clear();
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
    const bool restore_trial = can_restore_friction_trial(s);
    mjWarningStat initial_warnings[mjNWARNING];
    std::memcpy(initial_warnings, s->d->warning, sizeof(initial_warnings));
    if (restore_trial) {
        s->friction_start_state.resize(s->api.stateSize(s->m, mjSTATE_INTEGRATION));
        s->api.getState(s->m, s->d, s->friction_start_state.data(), mjSTATE_INTEGRATION);
    }
    // The core is synchronous and the eligible model has no step2 callbacks.
    // Intermediate candidates are private to this call. Roll back integration
    // state on every failure before retrying at stricter precision or returning.
    struct RestoreFailedTrial {
        Stepper* s;
        bool enabled;
        mjData*& accepted;
        const mjWarningStat* warnings;
        ~RestoreFailedTrial() {
            if (enabled && !accepted) {
                s->api.setState(s->m, s->d, s->friction_start_state.data(), mjSTATE_INTEGRATION);
                std::memcpy(s->d->warning, warnings, sizeof(s->d->warning));
            }
        }
    } rollback{s, restore_trial, accepted_trial, initial_warnings};
    auto predict = [&](const std::vector<double>& forces, std::vector<double>& speeds) {
        ++s->friction_predict_calls;
        mjData* d = restore_trial ? s->d : s->friction_trial;
        if (restore_trial) {
            // Same physical step, same immutable step1 inputs. Restore time,
            // qpos/qvel, warmstart, controls and ALL external forces; never use
            // the previous candidate's integrated state or friction wrench.
            s->api.setState(s->m, d, s->friction_start_state.data(), mjSTATE_INTEGRATION);
            std::memcpy(d->warning, initial_warnings, sizeof(d->warning));
            ++s->friction_state_restores;
        } else {
            s->api.copyData(d, s->m, s->d);
            ++s->friction_full_copies;
        }
        for (int j=0; j<n; ++j) friction_wrench(s, d, s->friction[ids[j]], axes.data()+3*j, forces[j]);
        // All candidates start at the same qpos/qvel/control. Reuse the real
        // step1's collision detection, mass factorization and velocity terms;
        // only applied forces, constraint solve and integration differ.
        s->api.step2(s->m, d);
        if (!finite(d->qpos, s->m->nq) || !finite(d->qvel, s->m->nv) ||
            d->warning[mjWARN_BADQACC].number != initial_warnings[mjWARN_BADQACC].number) return false;
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
    auto& forces=w.forces; forces=previous_forces;
    auto& free_speed=w.free_speed; free_speed.assign(n,0.);
    auto& measured=w.measured; measured.assign(n,0.);
    auto& probe=w.probe; probe.assign(n,0.);
    auto& response=w.response; response.assign(n*n,0.);
    auto& limits=w.limits; limits.assign(n,0.);
    auto& sliding=w.sliding; sliding.assign(n,false);
    auto apply = [&]() {
        // Every successful branch reaches this only after predict evaluated
        // these exact forces and validated ALL interfaces at the same precision.
        accepted_trial = restore_trial ? s->d : s->friction_trial;
        for (int i=0; i<n; ++i) {
            auto& f = s->friction[ids[i]];
            f.axial_force = forces[i]; f.mode = sliding[i] ? 2 : 1;
            for (int k=0; k<3; ++k) f.force[k] = forces[i] * axes[3*i+k];
            if (!restore_trial) friction_wrench(s, s->d, f, axes.data()+3*i, forces[i]);
        }
    };
    // Validate a previous solution against this substep's actual contact solve.
    // Usually loads change slowly: one discarded prediction is sufficient.
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
            auto& minus=w.minus; minus.assign(n,0.);
            auto& candidate=w.candidate; candidate=forces;
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
            // A coordinate candidate also changes the common base/contact
            // solve. Accept a jointly valid evaluated point immediately: later
            // coordinate updates can otherwise cross its numerical branch and
            // discard a solution that already met every interface's tolerance.
            auto stopped_candidate = [&]() {
                for (int j=0; j<n; ++j) {
                    if (std::abs(probe[j]) > tolerance || std::abs(forces[j]) > limits[j]) return false;
                }
                measured = probe;
                std::fill(sliding.begin(), sliding.end(), false);
                return true;
            };
            for (int sweep=0; sweep<12; ++sweep) {
                for (int i=0; i<n; ++i) {
                    const auto& f = s->friction[ids[i]];
                    double low = -limits[i], high = limits[i];
                    forces[i] = low;
                    if (!predict(forces, probe)) return ERR_DIVERGED;
                    if (stopped_candidate()) { valid = true; break; }
                    double vlo = probe[i];
                    forces[i] = high;
                    if (!predict(forces, probe)) return ERR_DIVERGED;
                    if (stopped_candidate()) { valid = true; break; }
                    double vhi = probe[i];
                    sliding[i] = false;
                    if (vlo > tolerance || vhi < -tolerance) {
                        limits[i] = f.kinetic_n;
                        forces[i] = vlo > tolerance ? -f.kinetic_n : f.kinetic_n;
                        sliding[i] = true;
                    } else {
                        // The inner contact solver has a finite stopping
                        // accuracy: slip can jump across zero before reaching
                        // the half-tolerance target. Keep the best *evaluated*
                        // bracket point, rather than using an arbitrary last
                        // midpoint which can lie on the invalid side of that
                        // jump. The coupled validation below still requires
                        // the original full tolerance for every interface.
                        double best_force = std::abs(vlo) < std::abs(vhi) ? low : high;
                        double best_speed = std::min(std::abs(vlo), std::abs(vhi));
                        for (int bisect=0; bisect<40; ++bisect) {
                            forces[i] = (low+high)*.5;
                            if (!predict(forces, probe)) return ERR_DIVERGED;
                            if (stopped_candidate()) { valid = true; break; }
                            if (std::abs(probe[i]) < best_speed) {
                                best_force = forces[i];
                                best_speed = std::abs(probe[i]);
                            }
                            if (best_speed <= tolerance*.5) break;
                            if (probe[i] > 0) high = forces[i]; else low = forces[i];
                        }
                        if (valid) break;
                        forces[i] = best_force;
                    }
                }
                if (valid) break;
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
                // The caller can retry the same step with a more accurate
                // contact solve. Only diagnose a final, strict-precision failure.
                if (s->m->opt.tolerance <= 1e-14) {
                    for (int i=0; i<n; ++i) std::fprintf(stderr, "friction failure: t=%.6g id=%d v0=%.9g f=%.9g v=%.9g lim=%.9g\n",
                        s->d->time, ids[i], start_speed[i], forces[i], measured[i], limits[i]);
                }
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
    if(s && planning_model==s->m) return ERR_ARGUMENT;
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
            const double contact_tolerance = m->opt.tolerance;
            struct RestoreContactTolerance {
                mjtNum& value;
                double original;
                ~RestoreContactTolerance() { value = original; }
            } restore_contact_tolerance{m->opt.tolerance, contact_tolerance};
            const auto previous_friction = s->friction;
            mjData* accepted_trial = nullptr;
            int friction_status = apply_axial_friction(s, accepted_trial);
            if (friction_status == ERR_FRICTION && contact_tolerance > 1e-14) {
                // Newton's global cost stopping criterion can leave a tiny
                // force-to-slip discontinuity in a light connector coupled to
                // a massive articulated base. A bracket may then contain no
                // jointly acceptable point at that contact accuracy. Re-solve
                // the SAME step1 state with a tighter inner contact solve;
                // retain all original friction bounds and slip tolerances.
                // Failed trials applied no wrench to d and advanced no state.
                s->friction = previous_friction;
                m->opt.tolerance = 1e-14;
                ++s->friction_solver_retries;
                friction_status = apply_axial_friction(s, accepted_trial);
            }
            if (friction_status != OK) {
                s->friction = previous_friction;
                return friction_status;
            }
            if (accepted_trial == d) {
                // Only the accepted result escapes this synchronous call. Its
                // sensors, contacts, solver state and warmstart are all intact.
                ++s->friction_reused_steps;
            } else {
                // Generic models keep the original apply/integrate path. This
                // also covers enabled channels already outside their travel,
                // where no prediction was produced at all.
                s->api.step2(m, d);
                if (!accepted_trial) ++s->friction_unretained_steps;
            }
        } else {
            mjData* accepted_trial = nullptr;
            apply_axial_friction(s, accepted_trial);  // clear samples/start_z for free interfaces
            s->api.step(m, d);
            ++s->friction_unretained_steps;
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

// Per-handle lifetime counts; no global counters or cross-episode state.
API int lms_friction_diagnostics(void* handle, std::int64_t* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out) return ERR_ARGUMENT;
    out[0] = s->friction_predict_calls;
    out[1] = s->friction_reused_steps;
    out[2] = s->friction_unretained_steps;
    out[3] = s->friction_solver_retries;
    return OK;
}

API int lms_friction_copy_diagnostics(void* handle, std::int64_t* out) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !out) return ERR_ARGUMENT;
    out[0] = s->friction_full_copies;
    out[1] = s->friction_state_restores;
    return OK;
}

// Pure counterpart of PlugFriction.wrench. This does not step, constrain or
// rewrite MuJoCo state: the existing message-connected force port still owns
// application of these forces, including the equal/opposite COM wrench pair.
// Input: relative position(3), rotation(9), velocity(3), angular velocity(3).
// Socket row: position(3), socket rotation(9), plug rotation(9), initial, keyed.
// State: active socket (-1 = free), first sample, status, axial displacement.
namespace retention_guidance {
double norm(const double* v, int n=3) {
    double squared = 0;
    for (int i=0; i<n; ++i) squared += v[i]*v[i];
    return std::sqrt(squared);
}
void transpose_rotate(const double* r, const double* v, double* out) {
    for (int i=0; i<3; ++i) out[i] = r[i]*v[0]+r[3+i]*v[1]+r[6+i]*v[2];
}
void limit(double* v, int n, double maximum) {
    double length = norm(v, n);
    double scale = length ? std::min(1.0, maximum/length) : 1.0;
    for (int i=0; i<n; ++i) v[i] *= scale;
}
void rotation_error(const double* r, double* out) {
    double q[4], trace=r[0]+r[4]+r[8];
    if (trace > 0) {
        double s=std::sqrt(trace+1)*2;
        q[0]=.25*s; q[1]=(r[7]-r[5])/s; q[2]=(r[2]-r[6])/s; q[3]=(r[3]-r[1])/s;
    } else {
        int i=0;
        if (r[4]>r[0]) i=1;
        if (r[8]>r[4*i]) i=2;
        if (i==0) {
            double s=std::sqrt(1+r[0]-r[4]-r[8])*2;
            q[0]=(r[7]-r[5])/s; q[1]=.25*s; q[2]=(r[1]+r[3])/s; q[3]=(r[2]+r[6])/s;
        } else if (i==1) {
            double s=std::sqrt(1+r[4]-r[0]-r[8])*2;
            q[0]=(r[2]-r[6])/s; q[1]=(r[1]+r[3])/s; q[2]=.25*s; q[3]=(r[5]+r[7])/s;
        } else {
            double s=std::sqrt(1+r[8]-r[0]-r[4])*2;
            q[0]=(r[3]-r[1])/s; q[1]=(r[2]+r[6])/s; q[2]=(r[5]+r[7])/s; q[3]=.25*s;
        }
    }
    double length=norm(q,4);
    for (double& value:q) value/=length;
    if (q[0]<0) for (double& value:q) value=-value;
    double sine=norm(q+1), scale=sine<1e-10 ? 2 : 2*std::atan2(sine,q[0])/sine;
    for (int i=0;i<3;++i) out[i]=q[i+1]*scale;
}
void coordinates(const double* input, const double* socket, double* delta, double* speed, double* error) {
    double offset[3];
    for (int i=0;i<3;++i) offset[i]=input[i]-socket[i];
    transpose_rotate(socket+3,offset,delta);
    transpose_rotate(socket+3,input+12,speed);
    if (socket[22]) {
        double difference[9];
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
            difference[3*i+j]=0;
            for (int k=0;k<3;++k) difference[3*i+j]+=input[3+3*i+k]*socket[12+3*j+k];
        }
        rotation_error(difference,error);
    } else {
        double desired[3]={socket[14],socket[17],socket[20]};
        double actual[3]={input[5],input[8],input[11]};
        cross3(desired,actual,error);
        double sine=norm(error), cosine=std::clamp(dot3(desired,actual),-1.0,1.0);
        if (sine<1e-10) {
            for (int i=0;i<3;++i) error[i]=cosine>=0 ? 0 : std::acos(-1.0)*socket[12+3*i];
        } else {
            double scale=std::atan2(sine,cosine)/sine;
            for (int i=0;i<3;++i) error[i]*=scale;
        }
    }
}
}  // namespace retention_guidance

API int lms_plug_guidance(const double* input, const double* sockets, int count,
                          const double* p, double* state, double* out) {
    using namespace retention_guidance;
    if (!input || !sockets || !p || !state || !out || count<1 || !finite(input,18)) return ERR_ARGUMENT;
    int active=static_cast<int>(state[0]);
    if (active < -1 || active >= count) return ERR_ARGUMENT;
    double delta[3],speed[3],error[3];
    if (active>=0) {
        coordinates(input,sockets+23*active,delta,speed,error);
        if (delta[2]>=p[0] || norm(delta,2)>p[13] || norm(error)>p[14]) active=-1;
    }
    if (active<0) for (int i=0;i<count;++i) {
        const double* socket=sockets+23*i;
        coordinates(input,socket,delta,speed,error);
        bool initial=state[1] && (socket[21] || std::abs(speed[2])<=1e-6);
        if (norm(delta,2)<=p[10] && norm(error)<=p[12] && delta[2]>=-.001 && delta[2]<p[0]
                && (speed[2]<-1e-6 || initial)) { active=i; break; }
    }
    state[0]=active; state[1]=0;
    std::fill(out,out+6,0.0);
    if (active<0) { state[2]=0; state[3]=0; return OK; }
    const double* socket=sockets+23*active;
    double force[3]={-p[1]*delta[0]-p[2]*speed[0],-p[1]*delta[1]-p[2]*speed[1],0};
    limit(force,2,p[3]);
    force[2]=delta[2]<0 ? std::min(-p[8]*delta[2],p[9]) : 0;
    rotate(socket+3,force,out);
    double omega[3]={input[15],input[16],input[17]},torsion[3]={0,0,0};
    if (!socket[22]) {
        double axis[3]={input[5],input[8],input[11]}, roll=dot3(omega,axis);
        for (int i=0;i<3;++i) { omega[i]-=roll*axis[i]; torsion[i]=-p[6]*roll*axis[i]; }
    }
    for (int i=0;i<3;++i) out[3+i]=-p[4]*error[i]-p[5]*omega[i]+torsion[i];
    limit(out+3,3,p[7]);
    state[3]=delta[2];
    bool seated=std::abs(delta[2])<p[11] && norm(delta,2)<=p[10] && norm(error)<=p[12];
    state[2]=seated ? 1 : std::abs(speed[2])<=1e-6 ? 2 : speed[2]>0 ? 3 : 4;
    return finite(out,6) ? OK : ERR_ARGUMENT;
}

// Same scene-message kinematics and COM shifts as PlugRetentionModel. Each
// body's sample is qpos(7), qvel(6); inputs remain the published inertial state.
// This deliberately does not read live mjData (which has a different frame).
API int lms_plug_guidance_wrenches(const double* sample, const double* coms,
                                   const double* sockets, int count, const double* parameters,
                                   double* state, double* out) {
    using namespace retention_guidance;
    if (!sample || !coms || !out || !finite(sample,26)) return ERR_ARGUMENT;
    double r[2][9],omega[2][3],com[2][3];
    for (int body=0;body<2;++body) {
        const double* s=sample+13*body;
        double length=norm(s+3,4);
        if (length<=1e-12) return ERR_ARGUMENT;
        double w=s[3]/length,x=s[4]/length,y=s[5]/length,z=s[6]/length;
        double* m=r[body];
        m[0]=1-2*(y*y+z*z); m[1]=2*(x*y-z*w); m[2]=2*(x*z+y*w);
        m[3]=2*(x*y+z*w); m[4]=1-2*(x*x+z*z); m[5]=2*(y*z-x*w);
        m[6]=2*(x*z-y*w); m[7]=2*(y*z+x*w); m[8]=1-2*(x*x+y*y);
        rotate(m,s+10,omega[body]);
        rotate(m,coms+3*body,com[body]);
        for (int i=0;i<3;++i) com[body][i]+=s[i];
    }
    double offset[3],cross[3],velocity[3],angular_velocity[3],input[18];
    for (int i=0;i<3;++i) offset[i]=sample[13+i]-sample[i];
    cross3(omega[0],offset,cross);
    for (int i=0;i<3;++i) {
        velocity[i]=sample[20+i]-sample[7+i]-cross[i];
        angular_velocity[i]=omega[1][i]-omega[0][i];
    }
    transpose_rotate(r[0],offset,input);
    transpose_rotate(r[0],velocity,input+12);
    transpose_rotate(r[0],angular_velocity,input+15);
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
        input[3+3*i+j]=0;
        for (int k=0;k<3;++k) input[3+3*i+j]+=r[0][3*k+i]*r[1][3*k+j];
    }
    double guidance[6];
    int status=lms_plug_guidance(input,sockets,count,parameters,state,guidance);
    if (status!=OK) return status;
    double force[3],torque[3],plug_lever[3],base_lever[3],plug_moment[3],base_moment[3];
    rotate(r[0],guidance,force);
    rotate(r[0],guidance+3,torque);
    for (int i=0;i<3;++i) {
        out[i]=force[i]; out[6+i]=-force[i];
        plug_lever[i]=sample[13+i]-com[1][i];
        base_lever[i]=sample[13+i]-com[0][i];
    }
    cross3(plug_lever,force,plug_moment);
    cross3(base_lever,out+6,base_moment);
    for (int i=0;i<3;++i) { plug_moment[i]+=torque[i]; base_moment[i]=-torque[i]+base_moment[i]; }
    transpose_rotate(r[1],plug_moment,out+3);
    transpose_rotate(r[0],base_moment,out+9);
    return finite(out,12) ? OK : ERR_ARGUMENT;
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

// Only for an independently loaded planning model. Include nearby separated
// surfaces in collision queries; never call this on a production stepper.
// Gap keeps these additional positive-distance contacts inactive in forward.
API int lms_probe_clearance(void* handle, double clearance) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || !std::isfinite(clearance) || clearance < 0 || clearance > .02) return ERR_ARGUMENT;
    for (int g = 0; g < s->m->ngeom; ++g) {
        s->m->geom_margin[g] = std::max(s->m->geom_margin[g], clearance);
        s->m->geom_gap[g] = std::max(s->m->geom_gap[g], clearance);
    }
    for (int p = 0; p < s->m->npair; ++p) {
        s->m->pair_margin[p] = std::max(s->m->pair_margin[p], clearance);
        s->m->pair_gap[p] = std::max(s->m->pair_gap[p], clearance);
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

// Read the completed authoritative solver result without advancing physics.
// Rows: geom1, geom2, body1, body2, separation [m], contact-frame wrench [N,N*m].
API int lms_contact_materials(void* handle, double* out, int capacity) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || capacity < 0 || (capacity > 0 && !out)) return ERR_ARGUMENT;
    const int count = std::min(capacity, s->d->ncon);
    for (int i = 0; i < count; ++i) {
        const mjContact& c = s->d->contact[i];
        double* row = out + 8*i;
        row[0] = c.geom[0]; row[1] = c.geom[1]; row[2] = c.dim;
        for (int j = 0; j < 5; ++j) row[3+j] = c.friction[j];
    }
    return s->d->ncon;
}

API int lms_contact_forces(void* handle, double* out, int capacity) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || capacity < 0 || (capacity > 0 && !out)) return ERR_ARGUMENT;
    for (int i = 0; i < s->d->ncon && i < capacity; ++i) {
        const auto& c = s->d->contact[i];
        out[11*i] = c.geom[0]; out[11*i+1] = c.geom[1];
        out[11*i+2] = c.geom[0] >= 0 ? s->m->geom_bodyid[c.geom[0]] : -1;
        out[11*i+3] = c.geom[1] >= 0 ? s->m->geom_bodyid[c.geom[1]] : -1;
        out[11*i+4] = c.dist;
        mjtNum wrench[6] = {0,0,0,0,0,0};
        s->api.contactForce(s->m, s->d, i, wrench);
        for (int j = 0; j < 6; ++j) out[11*i+5+j] = wrench[j];
    }
    return s->d->ncon;
}

// Geometric engagement evidence: ids, separation, world position and normal.
// This reads existing contacts only; it never runs forward or changes state.
API int lms_contact_geometry(void* handle, double* out, int capacity) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s || capacity < 0 || (capacity > 0 && !out)) return ERR_ARGUMENT;
    for (int i = 0; i < s->d->ncon && i < capacity; ++i) {
        const auto& c = s->d->contact[i];
        out[11*i] = c.geom[0]; out[11*i+1] = c.geom[1];
        out[11*i+2] = c.geom[0] >= 0 ? s->m->geom_bodyid[c.geom[0]] : -1;
        out[11*i+3] = c.geom[1] >= 0 ? s->m->geom_bodyid[c.geom[1]] : -1;
        out[11*i+4] = c.dist;
        for (int j = 0; j < 3; ++j) {
            out[11*i+5+j] = c.pos[j];
            out[11*i+8+j] = c.frame[j];
        }
    }
    return s->d->ncon;
}

// Read-only task counts. Match existing <= threshold and pair membership exactly.
API int lms_finger_contact_counts(void* handle, int target1, int target2,
                                 int finger1, int finger2, double threshold, int* out) {
    auto* s=static_cast<Stepper*>(handle);
    if(!s || !out || !std::isfinite(threshold)) return ERR_ARGUMENT;
    out[0]=out[1]=0;
    for(int i=0;i<s->d->ncon;++i) {
        const auto& c=s->d->contact[i];
        if(!(c.dist<=threshold) || c.geom[0]<0 || c.geom[1]<0) continue;
        const int a=s->m->geom_bodyid[c.geom[0]], b=s->m->geom_bodyid[c.geom[1]];
        if(a==target1 || b==target1 || (target2>=0 && (a==target2 || b==target2))) {
            out[0]+=(a==finger1 || b==finger1);
            out[1]+=(a==finger2 || b==finger2);
        }
    }
    return 0;
}

// Batch the existing per-body sums in their original source order. Caller owns
// stable channel and output arrays. No physics cadence or force law changes.
API int lms_sum_body_wrenches(void* handle, int groups, const int* bodies,
                            const int* offsets, const double* const* forces,
                            const double* const* torques, double* const* sum_force,
                            double* const* sum_torque) {
    auto* s=static_cast<Stepper*>(handle);
    if(!s || groups<0 || !bodies || !offsets || !forces || !torques || !sum_force || !sum_torque)
        return ERR_ARGUMENT;
    if(offsets[0]!=0) return ERR_ARGUMENT;
    for(int g=0;g<groups;++g) {
        if(offsets[g+1]<offsets[g] || !sum_force[g] || !sum_torque[g]) return ERR_ARGUMENT;
        for(int k=0;k<3;++k) {sum_force[g][k]=0.;sum_torque[g][k]=0.;}
        for(int c=offsets[g];c<offsets[g+1];++c) {
            if(!forces[c] || !torques[c]) return ERR_ARGUMENT;
            for(int k=0;k<3;++k) {
                sum_force[g][k]+=forces[c][k];
                sum_torque[g][k]+=torques[c][k];
            }
        }
        const int status=lms_set_body_wrench(handle,bodies[g],sum_force[g],sum_torque[g]);
        if(status<0) return status;
    }
    return OK;
}

// Reset episode state in the same translation unit as Stepper: no external ABI casts.
API int lms_reset_episode(void* handle) {
    auto* s = static_cast<Stepper*>(handle);
    if (!s) return ERR_ARGUMENT;
    s->api.resetData(s->m, s->d);
    if (s->friction_trial) s->api.resetData(s->m, s->friction_trial);
    for (auto& f : s->friction) {
        f.enabled = false;
        std::fill(f.force, f.force + 3, 0.);
        f.axial_force = f.velocity = f.start_z = 0.;
        f.mode = 0;
    }
    s->friction_ids.clear();
    s->friction_response.clear();
    s->friction_start_state.clear();
    s->friction_timestep = 0.;
    s->friction_predict_calls = s->friction_reused_steps = 0;
    s->friction_unretained_steps = s->friction_solver_retries = 0;
    s->friction_full_copies = s->friction_state_restores = 0;
    for (auto* values : {&s->force_world, &s->torque_body, &s->orbital_force, &s->sensor_sample})
        std::fill(values->begin(), values->end(), 0.);
    return OK;
}
