#!/usr/bin/env bash
set -eo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="all"
target_set=false
serial=false
build_base=""
install_base=""
log_base=""

usage() {
  cat <<'USAGE'
Usage:
  bash scripts/build.sh [all|core|sim|<ros-package>] [options]

Targets:
  all                 Build every ROS package under src (default)
  core                Build embodied_nexus_core and its dependencies
  sim                 Build embodied_nexus_sim_gazebo and its dependencies
  <ros-package>       Build the named package and its dependencies

Options:
  --serial            Build packages sequentially and limit Make to one job
  --build-base PATH   Set the colcon build directory
  --install-base PATH Set the colcon install directory
  --log-base PATH     Set the colcon log directory
  -h, --help          Show this help

The script expects ROS 2 Humble at /opt/ros/humble and runs from the repo root.
For a repository under /mnt/<drive>, default output directories use Linux-side cache storage.
USAGE
}

while (($#)); do
  case "$1" in
    all|core|sim)
      if [[ "$target_set" == true ]]; then
        echo "Select only one build target." >&2
        usage >&2
        exit 2
      fi
      target="$1"
      target_set=true
      shift
      ;;
    --serial)
      serial=true
      shift
      ;;
    --build-base|--install-base|--log-base)
      option="$1"
      if (($# < 2)) || [[ -z "$2" || "$2" == -* ]]; then
        echo "$option requires a path." >&2
        exit 2
      fi
      case "$option" in
        --build-base) build_base="$2" ;;
        --install-base) install_base="$2" ;;
        --log-base) log_base="$2" ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -* )
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ "$target_set" == true ]]; then
        echo "Unexpected argument: $1" >&2
        usage >&2
        exit 2
      fi
      target="$1"
      target_set=true
      shift
      ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash." >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
cd "$root"

if [[ "$root" == /mnt/* ]]; then
  native_output="${XDG_CACHE_HOME:-$HOME/.cache}/embodied-nexus"
  [[ -n "$build_base" ]] || build_base="$native_output/build"
  [[ -n "$install_base" ]] || install_base="$native_output/install"
  [[ -n "$log_base" ]] || log_base="$native_output/log"
fi

package_targets=()
package_target=""
case "$target" in
  all) package_target="" ;;
  core) package_target="embodied_nexus_core" ;;
  sim) package_targets=(embodied_nexus_sim_gazebo embodied_nexus_sim_mujoco) ;;
  *) package_targets=("$target") ;;
esac

if [[ -n "${package_target:-}" ]]; then
  package_targets=("$package_target")
fi

if ((${#package_targets[@]})); then
  packages="$(colcon list --base-paths src | awk '{print $1}')"
  for package_target in "${package_targets[@]}"; do
    if ! grep -Fxq -- "$package_target" <<<"$packages"; then
      echo "Unknown ROS package target: $package_target" >&2
      echo "Discover packages with: colcon list --base-paths src" >&2
      exit 2
    fi
  done
fi

colcon_args=(--base-paths src --symlink-install)
((${#package_targets[@]})) && colcon_args+=(--packages-up-to "${package_targets[@]}")
[[ -n "$build_base" ]] && colcon_args+=(--build-base "$build_base")
[[ -n "$install_base" ]] && colcon_args+=(--install-base "$install_base")
[[ "$serial" == true ]] && { colcon_args+=(--executor sequential); export MAKEFLAGS=-j1; }

printf 'Build output directories: build=%s install=%s log=%s\n' \
  "${build_base:-$root/build}" "${install_base:-$root/install}" "${log_base:-$root/log}"
if [[ -n "$log_base" ]]; then
  colcon --log-base "$log_base" build "${colcon_args[@]}"
else
  colcon build "${colcon_args[@]}"
fi
printf '\nWorkspace setup: source "%s/setup.bash"\n' "${install_base:-install}"
