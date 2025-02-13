#!/usr/bin/env bash
cmake -B build_linx -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build_linx