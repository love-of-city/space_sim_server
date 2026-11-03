// Compare the installed SARM dispatch with MuJoCo's original detector in the
// same process/runtime. This tests full contact output, not only SDF samples.
#include <mujoco/mujoco.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  HMODULE runtime = LoadLibraryExA(argv[1], nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
  if (!runtime) return 3;
#define LOAD(name) auto name = reinterpret_cast<decltype(&::name)>(GetProcAddress(runtime, #name)); if (!name) return 4
  LOAD(mj_version); LOAD(mj_loadPluginLibrary); LOAD(mj_loadXML);
  LOAD(mj_makeData); LOAD(mj_deleteData); LOAD(mj_deleteModel); LOAD(mj_forward); LOAD(mj_name2id);
#undef LOAD
  if (mj_version() != 3007000) return 5;
  auto table = reinterpret_cast<mjfCollision (*)[mjNGEOMTYPES]>(GetProcAddress(runtime, "mjCOLLISIONFUNC"));
  if (!table) return 6;
  const auto reference = table[mjGEOM_SDF][mjGEOM_SDF];
  mj_loadPluginLibrary(argv[2]);
  const auto candidate = table[mjGEOM_SDF][mjGEOM_SDF];
  if (!reference || reference == candidate) return 7;
  char error[1024]{};
  mjModel* model = mj_loadXML(argv[3], nullptr, error, sizeof(error));
  if (!model) { std::cerr << error << std::endl; return 8; }
  mjData* data = mj_makeData(model);
  mj_forward(model, data);
  int geom[5];
  for (int i = 0; i < 5; ++i) {
    char name[] = "sdf0";
    name[3] += i;
    geom[i] = mj_name2id(model, mjOBJ_GEOM, name);
    if (geom[i] < 0) return 9;
  }
  int checks = 0, mismatches = 0, contacts = 0, empty = 0;
  auto compare = [&](int first, int second, double margin) {
    mjContact a[mjMAXCONPAIR]{}, b[mjMAXCONPAIR]{};
    const int na = reference(model, data, a, geom[first], geom[second], margin);
    const int nb = candidate(model, data, b, geom[first], geom[second], margin);
    ++checks;
    contacts += na;
    empty += na == 0;
    if (na != nb || std::memcmp(a, b, sizeof(mjContact) * na)) ++mismatches;
  };
  auto pose = [&](double radial, double axial, double tilt, double translation) {
    const double c = std::cos(tilt), s = std::sin(tilt);
    for (int i = 0; i < 5; ++i) {
      auto* p = data->geom_xpos + 3*geom[i];
      p[0] = translation; p[1] = -translation; p[2] = translation;
      auto* r = data->geom_xmat + 9*geom[i];
      std::fill(r, r+9, 0.0);
      r[0] = r[4] = r[8] = 1.0;
    }
    auto* p = data->geom_xpos + 3*geom[2];
    p[0] += radial; p[2] += axial;
    auto* r = data->geom_xmat + 9*geom[2];
    r[0] = c; r[2] = s; r[6] = -s; r[8] = c;
  };
  for (double shift : {0.0, 6.9e6})
    for (double radius : {0.0, .00049, .00050, .000649, .000650, .000651, .001, .003, .02})
      for (double angle : {0.0, .001, .01, .03})
        for (double z : {-.20, -.06, 0.0, .02, .12}) {
          pose(radius, z, angle, shift);
          for (double margin : {0.0, .00015, .0007}) {
            compare(2, 4, margin); compare(4, 2, margin);
          }
          // Other modes must still use the original detector unchanged.
          compare(0, 1, 0.0); compare(2, 3, 0.0);
        }
  mj_deleteData(data); mj_deleteModel(model);
  std::cout << "{\"checks\":" << checks << ",\"mismatches\":" << mismatches
            << ",\"reference_contacts\":" << contacts << ",\"empty_pairs\":" << empty << "}" << std::endl;
  return mismatches || !contacts || !empty ? 1 : 0;
}
