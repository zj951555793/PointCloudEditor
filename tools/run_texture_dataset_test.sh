#!/usr/bin/env sh
set -eu
BUILD="${1:-build}"
ctest --test-dir "$BUILD" -R JMEngine_texture_bop --output-on-failure
