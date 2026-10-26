#!/usr/bin/env bash
set -euo pipefail
platform_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
config_path="$platform_root/deploy/linux.local.json"
arguments=("$@")
for ((index=0; index<${#arguments[@]}; index++)); do
    if [[ "${arguments[index]}" == --config ]]; then
        config_path="${arguments[index+1]:?--config requires a path}"
    fi
done
python_exe="${SPACE_SIM_PYTHON:-}"
if [[ -z "$python_exe" && -f "$config_path" ]]; then
    python_exe="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("python", ""))' "$config_path")"
fi
python_exe="${python_exe:-python3}"
export PYTHONPATH="$platform_root/backend:$platform_root${PYTHONPATH:+:$PYTHONPATH}"
exec "$python_exe" -m space_arm_platform.linux_deployment "${arguments[@]}"
