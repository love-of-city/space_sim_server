// Experimental instrumentation only. Compile against EXACT MuJoCo 3.7.0 headers.
// No MuJoCo library is linked/loaded here; calls use Basilisk's already loaded DLL.
#include <cstdint>
#include <mujoco/mujoco.h>

#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C" __attribute__((visibility("default")))
#endif

API int probe_abi() { return 1; }
API int probe_header_version() { return mjVERSION_HEADER; }
API int probe_stats(const mjData* d, const mjModel* m, std::int64_t* out) {
    if(!d || !m || !out)return -1;
    out[0]=d->ncon;out[1]=d->nefc;out[2]=d->nisland;
    out[3]=m->nq;out[4]=m->nv;out[5]=m->ngeom;
    out[6]=static_cast<std::int64_t>(d->narena);
    out[7]=static_cast<std::int64_t>(d->pstack);
    out[8]=static_cast<std::int64_t>(d->pbase);
    out[9]=static_cast<std::int64_t>(d->threadpool);
    return 0;
}
API int probe_detach_finished(mjData* d, std::uintptr_t expected) {
    // Called only AFTER synchronous ExecuteSimulation has returned and the
    // benchmark has finished. Never change a binding while workers are active.
    // There is no public unbind in MuJoCo 3.7.0; reject a nonempty main stack.
    if(!d || !expected || d->threadpool!=expected)return -1;
    if(d->pstack || d->pbase)return -2;
    d->threadpool=0;
    return 0;
}
