#!/bin/bash
# ======================================================================
#  Cross-compile acados core libraries for arm64-v8a (Android)
#
#  Builds libacados.a, libblasfeo.a, libhpipm.a from acados-main/
#  source using Android NDK cmake toolchain.
#
#  Prerequisites:
#    - Android NDK 27 installed
#    - acados-main/ with source code
#
#  Output:
#    AndroidApp/app/src/main/cpp/acados_arm64/lib/libacados.a
#    AndroidApp/app/src/main/cpp/acados_arm64/lib/libblasfeo.a
#    AndroidApp/app/src/main/cpp/acados_arm64/lib/libhpipm.a
# ======================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---- Find NDK ----
if [ -n "${ANDROID_NDK_HOME:-}" ]; then
    NDK="$ANDROID_NDK_HOME"
elif [ -d "$HOME/Android/Sdk/ndk/27.2.12479018" ]; then
    NDK="$HOME/Android/Sdk/ndk/27.2.12479018"
else
    echo "ERROR: Cannot find Android NDK 27. Set ANDROID_NDK_HOME." >&2
    exit 1
fi

TOOLCHAIN="$NDK/build/cmake/android.toolchain.cmake"
if [ ! -f "$TOOLCHAIN" ]; then
    echo "ERROR: CMake toolchain not found: $TOOLCHAIN" >&2
    exit 1
fi

ACADOS_SRC="$REPO_ROOT/acados-main"
OUT_DIR="$REPO_ROOT/AndroidApp/app/src/main/cpp/acados_arm64/lib"
BUILD_DIR="/tmp/acados_arm64_build_$$"

mkdir -p "$BUILD_DIR" "$OUT_DIR"

echo "Building acados ARM64 libraries..."
echo "  NDK:    $NDK"
echo "  Source: $ACADOS_SRC"
echo "  Output: $OUT_DIR"

cd "$BUILD_DIR"

cmake "$ACADOS_SRC" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DANDROID_ABI=arm64-v8a \
    -DANDROID_PLATFORM=android-26 \
    -DCMAKE_BUILD_TYPE=Release \
    -DACADOS_WITH_QPOASES=OFF \
    -DACADOS_WITH_DAQP=OFF \
    -DACADOS_WITH_OSQP=OFF \
    -DACADOS_WITH_QPDUNES=OFF \
    -DACADOS_WITH_HPMPC=OFF \
    -DACADOS_WITH_OOQP=OFF \
    -DTARGET=ARMV8A_ARM_CORTEX_A57 \
    -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 \
    -DHPIPM_TARGET=GENERIC \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_C_FLAGS="-DOS_LINUX" \
    -DCMAKE_ASM_FLAGS="-DOS_LINUX" \
    2>&1 | tee /tmp/acados_arm64_cmake.log | tail -40

echo ""
echo "Compiling (this may take a few minutes)..."
make -j$(nproc) acados blasfeo hpipm 2>&1 | tee /tmp/acados_arm64_make.log | tail -40

# Fail fast if make actually failed (tee swallows exit code)
if ! grep -qE "^(Built target|\[100%\])" /tmp/acados_arm64_make.log; then
    if grep -qiE "error:|Error [0-9]+|undefined reference" /tmp/acados_arm64_make.log; then
        echo "ERROR: build failed. Full log: /tmp/acados_arm64_make.log" >&2
        exit 1
    fi
fi

# Copy libraries
for lib in libacados.a libblasfeo.a libhpipm.a; do
    found=$(find "$BUILD_DIR" -name "$lib" -type f 2>/dev/null | head -1)
    if [ -n "$found" ]; then
        cp "$found" "$OUT_DIR/$lib"
        echo "  Copied: $lib ($(stat -c%s "$OUT_DIR/$lib") bytes)"
    else
        echo "  WARNING: $lib not found in build directory"
    fi
done

# Copy blasfeo_target.h (MUST match the library's TARGET — GENERIC vs ARMV8A
# have different struct padding, mixing them causes memory corruption).
# BLASFEO's configure_file() writes it into the SOURCE tree, not build dir.
INCLUDE_DIR="$REPO_ROOT/AndroidApp/app/src/main/cpp/acados_arm64/include/blasfeo/include"
mkdir -p "$INCLUDE_DIR"

TARGET_H=""
if [ -f "$ACADOS_SRC/external/blasfeo/include/blasfeo_target.h" ]; then
    TARGET_H="$ACADOS_SRC/external/blasfeo/include/blasfeo_target.h"
else
    TARGET_H=$(find "$BUILD_DIR" -name "blasfeo_target.h" -type f 2>/dev/null | head -1)
fi

if [ -n "$TARGET_H" ]; then
    cp "$TARGET_H" "$INCLUDE_DIR/blasfeo_target.h"
    # Sanity check: must define the expected TARGET macro (else struct layout mismatch)
    if grep -q "TARGET_ARMV8A_ARM_CORTEX_A57" "$INCLUDE_DIR/blasfeo_target.h"; then
        echo "  Copied: blasfeo_target.h (TARGET_ARMV8A_ARM_CORTEX_A57 confirmed)"
    else
        echo "  ERROR: blasfeo_target.h does not define TARGET_ARMV8A_ARM_CORTEX_A57!" >&2
        echo "         Source: $TARGET_H" >&2
        echo "         This means libraries and headers disagree on struct layout." >&2
        exit 1
    fi
else
    echo "  ERROR: blasfeo_target.h not found — cannot guarantee header/library match!" >&2
    exit 1
fi

rm -rf "$BUILD_DIR"

echo ""
echo "Done! ARM64 acados libraries updated in $OUT_DIR"
echo "Now run: ./scripts/build_m130_solvers_arm64.sh && cd AndroidApp && ./gradlew assembleDebug"
