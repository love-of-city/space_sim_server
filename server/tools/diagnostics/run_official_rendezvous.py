"""Run the pinned, unmodified AVS Lab example offline and retain its plots/data."""

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np

SOURCE = Path(__file__).parent / "vendor/scenarioRendezVous.py"
SHA256 = "1fec9a5659f038118a0542a4a384a99f20dbf139d3b2b23b13665557b38f4a3f"
URL = "https://raw.githubusercontent.com/AVSLab/basilisk/43701e745cea88e49615b93ec68558715b2538aa/examples/scenarioRendezVous.py"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest() != SHA256:
        raise ValueError("Pinned official example hash mismatch")
    spec = importlib.util.spec_from_file_location("official_rendezvous", SOURCE)
    example = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(example)
    # Keep any example-generated output in the chosen evidence directory.
    os.chdir(output)
    figures = example.run(False)
    arrays, descriptions = {}, []
    for name, figure in figures.items():
        figure.savefig(output / f"{name}.png", dpi=140)
        for axis_index, axis in enumerate(figure.axes):
            for line_index, line in enumerate(axis.lines):
                key = f"{name}_{axis_index}_{line_index}"
                arrays[key] = np.column_stack((line.get_xdata(), line.get_ydata()))
                descriptions.append({"key": key, "xlabel": axis.get_xlabel(),
                                     "ylabel": axis.get_ylabel(), "label": line.get_label()})
    passed = len(figures) == 3 and bool(arrays) and all(
        len(values) > 10 and np.isfinite(values).all() for values in arrays.values()
    )
    np.savez_compressed(output / "official_plot_data.npz", **arrays)
    report = {"passed": bool(passed), "source_url": URL, "source_sha256": SHA256,
              "basilisk_version": importlib.metadata.version("bsk"),
              "figures": list(figures), "series": descriptions,
              "scope": "Unmodified official impulsive rendezvous / attitude demo; not the team finite-force flight."}
    (output / "official_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"passed": bool(passed), "figures": len(figures), "output": str(output)}))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
