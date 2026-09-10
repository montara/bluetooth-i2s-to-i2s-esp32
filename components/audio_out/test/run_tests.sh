#!/usr/bin/env bash
# Build and run the frame_ring tests on the host. No ESP-IDF needed.
set -euo pipefail

cd "$(dirname "$0")"
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

cc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined \
   -DFRAME_RING_HOST_TEST \
   -I../include \
   test_frame_ring.c ../frame_ring.c \
   -o "$out/test_frame_ring"

"$out/test_frame_ring"
