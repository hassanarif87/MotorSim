#!/usr/bin/env bash
set -euo pipefail

python_executable=$("${PYTHON:-python3}" -c 'import sys; print(sys.executable)')
extension_suffix=$("$python_executable" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
build_links=$(mktemp -d)
temporary_extension=""
trap 'rm -rf -- "$build_links"; rm -f -- "$temporary_extension"' EXIT

bazel "$@" build --symlink_prefix="$build_links/bazel-" \
    "--repo_env=MOTORSIM_PYTHON=$python_executable" //:_pmsmcpp.so
temporary_extension=$(mktemp ../motorsim/.pmsmcpp.XXXXXXXX)
install -m 755 -- "$build_links/bazel-bin/_pmsmcpp.so" "$temporary_extension"
mv -- "$temporary_extension" "../motorsim/_pmsmcpp${extension_suffix}"
