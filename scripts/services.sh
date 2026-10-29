#!/usr/bin/env bash
set -eo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
setup="${EMBODIED_NEXUS_SETUP:-$root/install/setup.bash}"
args=("$@")
for ((i=0; i<${#args[@]}; i++)); do
  if [[ "${args[i]}" == --setup && $((i+1)) -lt ${#args[@]} ]]; then setup="${args[i+1]}"; fi
done
if [[ ! -f "$setup" ]]; then
  echo "Workspace setup not found: $setup. Set EMBODIED_NEXUS_SETUP." >&2
  exit 2
fi
source "$setup"
exec python3 "$root/scripts/services.py" "$@"
