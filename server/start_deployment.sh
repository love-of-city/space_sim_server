#!/usr/bin/env bash
set -euo pipefail
exec "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/scripts/linux_platform.sh" start "$@"
