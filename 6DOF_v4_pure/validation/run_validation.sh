#!/bin/bash
# Numerical validation: Python test vectors vs C++ implementation
set -e
cd "$(dirname "$0")"

echo "=== Step 1: Generate Python test vectors ==="
python3 generate_test_vectors.py

echo ""
echo "=== Step 2: Build C++ validator ==="
g++ -std=c++17 -O2 -Wall -o validate validate_cpp.cpp -lm

echo ""
echo "=== Step 3: Run C++ validation ==="
./validate test_vectors.json

echo ""
echo "=== Done ==="
