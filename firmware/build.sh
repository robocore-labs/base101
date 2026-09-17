#!/usr/bin/env bash
#
# build.sh — pull the latest link101 libraries, then build.
#
# The libraries in lib/ are git submodules: the firmware repo pins each one
# to a specific commit, and a plain `cmake --build` just compiles whatever
# is checked out at that commit -- it never talks to git. So a fix pushed
# to hardware_link101 an hour ago does not show up here on its own. This is
# the one command that does both steps: move every submodule to its
# remote's current tip, then build.
#
#   ./build.sh              pull the libraries, then build
#   ./build.sh --offline    skip the pull -- build whatever's on disk now
#   ./build.sh --clean      wipe build/ first (e.g. after changing PICO_BOARD)
#
# Set PICO_SDK_PATH before running if it isn't already in your environment;
# this falls back to ~/pico/pico-sdk if that exists and PICO_SDK_PATH doesn't.
#
# Pulling only moves the submodules' working trees -- it never stages or
# commits the new pins. That stays a deliberate step of its own
# (git add lib && git commit), so a library update never lands in this
# repo's history just because someone ran a build.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

OFFLINE=0
CLEAN=0
for arg in "$@"; do
    case "$arg" in
        --offline) OFFLINE=1 ;;
        --clean)   CLEAN=1 ;;
        -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown option: $arg (try --help)" >&2; exit 2 ;;
    esac
done

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }

# --- Libraries --------------------------------------------------------

if [[ $OFFLINE -eq 0 ]]; then
    bold "Pulling libraries"

    # Covers a fresh clone (lib/* would otherwise be empty) as well as one
    # that's already checked out -- cheap and safe either way.
    git submodule update --init --quiet

    # Move each submodule to its remote's current tip. A submodule with no
    # branch set in .gitmodules (none of ours do) tracks its remote's
    # default branch, which is main for all of these.
    submodule_err="$(mktemp)"
    if git submodule update --remote --quiet 2>"$submodule_err"; then
        git submodule foreach -q 'echo "  $name  $(git log --oneline -1)"'
    else
        warn "couldn't reach one or more library remotes -- building with what's already on disk"
        sed 's/^/  /' "$submodule_err"
        git submodule foreach -q 'echo "  $name  $(git log --oneline -1)"'
    fi
    rm -f "$submodule_err"

    if ! git diff --quiet -- lib; then
        echo
        warn "libraries moved. Once you're happy with the build, record the new pins:"
        echo "      git add lib && git commit -m 'Bump libraries'"
    fi
    echo
fi

# --- Build --------------------------------------------------------------

: "${PICO_SDK_PATH:=$HOME/pico/pico-sdk}"
export PICO_SDK_PATH
if [[ ! -d "$PICO_SDK_PATH" ]]; then
    echo "PICO_SDK_PATH ('$PICO_SDK_PATH') doesn't exist -- set it and try again." >&2
    exit 1
fi

if [[ $CLEAN -eq 1 ]]; then
    rm -rf build
fi

bold "Configuring"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

bold "Building"
cmake --build build -j"$(nproc)"

echo
bold "Done: build/base101_firmware.uf2"
