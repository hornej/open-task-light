#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: tools/use-local-profile.sh <prod|debug>

Rebuilds the local sdkconfig from:
  1. sdkconfig.defaults
  2. sdkconfig.local.shared
  3. sdkconfig.local.<profile>

This overwrites the current local sdkconfig after first creating a timestamped
backup if one already exists.
EOF
}

if [[ $# -ne 1 ]]; then
  usage
  exit 1
fi

profile="$1"
case "$profile" in
  prod|debug)
    ;;
  *)
    usage
    exit 1
    ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"

shared_fragment="${project_dir}/sdkconfig.local.shared"
profile_fragment="${project_dir}/sdkconfig.local.${profile}"

for required_file in \
  "${project_dir}/sdkconfig.defaults" \
  "${shared_fragment}" \
  "${profile_fragment}"
do
  if [[ ! -f "${required_file}" ]]; then
    echo "Missing required config fragment: ${required_file}" >&2
    exit 1
  fi
done

if ! command -v idf.py >/dev/null 2>&1; then
  if [[ -n "${IDF_PATH:-}" && -f "${IDF_PATH}/export.sh" ]]; then
    # shellcheck disable=SC1090
    . "${IDF_PATH}/export.sh" >/dev/null 2>&1
  elif [[ -f "${HOME}/esp/esp-idf/export.sh" ]]; then
    # shellcheck disable=SC1091
    . "${HOME}/esp/esp-idf/export.sh" >/dev/null 2>&1
  fi
fi

if ! command -v idf.py >/dev/null 2>&1; then
  echo "idf.py not found. Source ESP-IDF first or install it under ~/esp/esp-idf." >&2
  exit 1
fi

cd "${project_dir}"

if [[ -f sdkconfig ]]; then
  backup_file="sdkconfig.backup.$(date +%Y%m%d-%H%M%S)"
  cp sdkconfig "${backup_file}"
  echo "Backed up existing sdkconfig to ${backup_file}"
fi

rm -f sdkconfig sdkconfig.old

defaults_value="sdkconfig.defaults;sdkconfig.local.shared;sdkconfig.local.${profile}"

idf.py -DIDF_TARGET=esp32s3 -DSDKCONFIG_DEFAULTS="${defaults_value}" reconfigure

echo
echo "Applied local profile: ${profile}"
echo "Fragments:"
echo "  sdkconfig.defaults"
echo "  sdkconfig.local.shared"
echo "  sdkconfig.local.${profile}"
