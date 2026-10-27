#!/usr/bin/env bash
# Compile & run the PIDController host test. Uses real cJSON + JsonWrapper from
# managed_components (present after one `idf.py build`), a stub NVS and stub
# esp_log. Run from this directory.
set -e
ROOT=../../../..                 # stillerate2 project root
MC=$ROOT/managed_components
CJSON=$MC/espressif__cjson/cJSON
JSON=$MC/jsonwrapper/include

if [ ! -d "$CJSON" ]; then
  echo "managed_components not found — run 'idf.py build' once first." >&2
  exit 1
fi

c++ -std=c++17 -Wall -Wno-deprecated-declarations \
    -Istubs -I../../include -I"$JSON" -I"$CJSON" \
    test_pidcontroller.cpp "$CJSON/cJSON.c" \
    -o test_pidcontroller
./test_pidcontroller
