#!/usr/bin/env bash
set -eo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -n "${EMBODIED_NEXUS_SETUP:-}" ]]; then
  setup="$EMBODIED_NEXUS_SETUP"
elif [[ "$root" == /mnt/* && -f "${XDG_CACHE_HOME:-$HOME/.cache}/embodied-nexus/install/setup.bash" ]]; then
  setup="${XDG_CACHE_HOME:-$HOME/.cache}/embodied-nexus/install/setup.bash"
else
  setup="$root/install/setup.bash"
fi
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
