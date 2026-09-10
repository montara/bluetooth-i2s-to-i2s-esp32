#!/usr/bin/env bash
# Build and run the rate_table tests on the host. No ESP-IDF needed.
set -euo pipefail

cd "$(dirname "$0")"
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

cc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined \
   -I../include \
   test_rate_table.c ../rate_table.c \
   -o "$out/test_rate_table"

"$out/test_rate_table"
