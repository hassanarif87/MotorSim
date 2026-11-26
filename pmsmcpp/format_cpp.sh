#!/usr/bin/env bash
set -euo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
format_args=(--style=file -i)

case ${1:-} in
    --check)
        format_args=(--style=file --dry-run --Werror)
        shift
        ;;
    -h|--help)
        printf 'Usage: %s [--check] [file ...]\n' "$0"
        exit 0
        ;;
esac

formatter=${CLANG_FORMAT:-clang-format}
if [[ -z ${CLANG_FORMAT:-} ]] && ! command -v "$formatter" >/dev/null; then
    for bundled in "$HOME"/.vscode{,-server}/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-format; do
        if [[ -x $bundled ]]; then
            formatter=$bundled
            break
        fi
    done
fi

if ! command -v "$formatter" >/dev/null; then
    printf 'Formatter %s not found. Install clang-format or set CLANG_FORMAT.\n' "$formatter" >&2
    exit 127
fi

if (( $# > 0 )); then
    exec "$formatter" "${format_args[@]}" -- "$@"
fi

find "$project_dir" -maxdepth 1 -type f \( -name '*.cpp' -o -name '*.h' \) \
    -exec "$formatter" "${format_args[@]}" -- {} +
find "$project_dir/src" "$project_dir/test" -type f \( -name '*.cpp' -o -name '*.h' \) \
    -exec "$formatter" "${format_args[@]}" -- {} +
