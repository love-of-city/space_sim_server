"""AM0 solar colour and illuminance; offline calculation only."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

root = Path(sys.argv[0]).resolve().parent / "data" / "solar"
cie_path = root / "CIE_xyz_1931_2deg.csv"
am0_path = root / "e490_00a.dat"
cie = np.loadtxt(cie_path, delimiter=",")
am0 = np.loadtxt(am0_path)

assert cie.ndim == 2 and cie.shape[1] == 4
assert am0.ndim == 2 and am0.shape[1] == 2
assert np.isfinite(cie).all() and np.isfinite(am0).all()
assert (np.diff(cie[:, 0]) > 0).all()
assert (np.diff(am0[:, 0]) > 0).all()
assert (am0[:, 1] >= 0).all()

# Convert BOTH wavelength and spectral density.
# micrometres -> nm; W/(m2 micrometre) -> W/(m2 nm).
am0_nm = am0[:, 0] * 1000.0
am0_per_nm = am0[:, 1] / 1000.0
wavelength = cie[:, 0]
assert am0_nm[0] <= wavelength[0] <= wavelength[-1] <= am0_nm[-1]

raw_irradiance = float(np.trapezoid(am0_per_nm, am0_nm))
assert raw_irradiance > 0.0
spectral_power = np.interp(wavelength, am0_nm, am0_per_nm)
xyz = np.trapezoid(
    spectral_power[:, None] * cie[:, 1:4],
    wavelength,
    axis=0,
)

# CIE photopic illuminance, normal incidence, no atmosphere.
raw_lux = 683.0 * float(xyz[1])
target_irradiance = 1361.0
scale = target_irradiance / raw_irradiance
reference_lux = raw_lux * scale

# Linear sRGB, without gamma encoding or chromatic adaptation.
matrix = np.array([
    [3.2406, -1.5372, -0.4986],
    [-0.9689, 1.8758, 0.0415],
    [0.0557, -0.2040, 1.0570],
])
rgb = matrix @ xyz
assert (rgb >= 0).all() and rgb.max() > 0, "RGB needs gamut review"
rgb = rgb / rgb.max()

result = {
    "source": "PySpectral E490-00a AM0 + CIE 1931 2 degree",
    "sources": {
        "AM0": "https://github.com/pytroll/pyspectral/blob/4bf06825780d5814bf3615637a0b6818756ddc88/pyspectral/data/e490_00a.dat",
        "CIE": "https://doi.org/10.25039/CIE.DS.xvudnb9b",
        "nominal_TSI": "https://arxiv.org/abs/1510.07674",
    },
    "input_sha256": {
        "CIE_xyz_1931_2deg.csv": hashlib.sha256(cie_path.read_bytes()).hexdigest(),
        "e490_00a.dat": hashlib.sha256(am0_path.read_bytes()).hexdigest(),
    },
    "method": "linear interpolation to CIE grid, trapezoidal integration",
    "cie_range_nm": [float(wavelength[0]), float(wavelength[-1])],
    "raw_irradiance_W_m2": raw_irradiance,
    "raw_XYZ_weighted_integrals_W_m2": xyz.tolist(),
    "raw_illuminance_lux_at_1_AU": raw_lux,
    "target_irradiance_W_m2": target_irradiance,
    "spectrum_scale": scale,
    "reference_illuminance_lux_at_1_AU": reference_lux,
    "linear_sRGB_normalized": rgb.tolist(),
    "chromaticity_xy": (xyz[:2] / xyz.sum()).tolist(),
    "assumptions": "1 AU; normal incidence; no atmosphere or occultation",
    "note": "E490 spectral shape rescaled to 1361 W/m2; not unmodified ASTM data. RGB is peak-normalized. UE photometric output requires separate validation.",
}

out_path = root / "solar_rgb_result.json"
out_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, indent=2))
