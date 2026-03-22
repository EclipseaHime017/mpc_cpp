#!/usr/bin/env bash
# =============================================================================
# install_deps.sh — Install all dependencies for mpc_cpp
# =============================================================================
# Tested on: Ubuntu 22.04 (x86_64 and ARM64)
# Run as:    bash scripts/install_deps.sh
#
# Dependencies installed:
#   System: Eigen3, yaml-cpp, libusb, cmake, build-essential
#   Source: OSQP 0.6.3, osqp-cpp, CasADi 3.6 (with IPOPT)
# =============================================================================

set -euo pipefail

INSTALL_PREFIX="${INSTALL_PREFIX:-/usr/local}"
BUILD_DIR="${BUILD_DIR:-/tmp/mpc_deps_build}"
JOBS=$(nproc)

echo "=== Installing system packages ==="
sudo apt-get update -qq
sudo apt-get install -y \
    build-essential cmake git pkg-config \
    libeigen3-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    liblapack-dev libopenblas-dev \
    gfortran \
    wget unzip

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# ─── OSQP 0.6.3 ──────────────────────────────────────────────────────────────
echo ""
echo "=== Building OSQP 0.6.3 ==="
if ! pkg-config --exists osqp 2>/dev/null; then
    git clone --depth 1 --branch v0.6.3 https://github.com/osqp/osqp.git osqp
    cmake -B osqp/build -S osqp \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DOSQP_BUILD_UNITTESTS=OFF
    cmake --build osqp/build -j"$JOBS"
    sudo cmake --install osqp/build
    echo "OSQP installed."
else
    echo "OSQP already installed, skipping."
fi

# ─── osqp-cpp (Google wrapper) ────────────────────────────────────────────────
echo ""
echo "=== Building osqp-cpp ==="
if [ ! -f "$INSTALL_PREFIX/include/osqp++.h" ]; then
    git clone --depth 1 https://github.com/google/osqp-cpp.git osqp-cpp
    cmake -B osqp-cpp/build -S osqp-cpp \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DOSQP_CPP_BUILD_TESTS=OFF
    cmake --build osqp-cpp/build -j"$JOBS"
    sudo cmake --install osqp-cpp/build
    echo "osqp-cpp installed."
else
    # Fallback: try OsqpEigen (Robotics Toolbox wrapper, simpler API)
    echo "osqp-cpp already present."
fi

# ─── OsqpEigen (alternative lighter wrapper) ─────────────────────────────────
# Enable if osqp-cpp build fails on your platform
if false; then
    echo ""
    echo "=== Building OsqpEigen ==="
    git clone --depth 1 https://github.com/robotology/osqp-eigen.git osqp-eigen
    cmake -B osqp-eigen/build -S osqp-eigen \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"
    cmake --build osqp-eigen/build -j"$JOBS"
    sudo cmake --install osqp-eigen/build
fi

# ─── IPOPT (prerequisite for CasADi) ─────────────────────────────────────────
echo ""
echo "=== Installing IPOPT via apt ==="
# Ubuntu 22.04 ships coinor-libipopt-dev 3.14.x — sufficient for CasADi
sudo apt-get install -y coinor-libipopt-dev || {
    echo "apt IPOPT not available, building from source..."
    # Source build omitted for brevity; use the script from:
    # https://github.com/coin-or/Ipopt/releases
    echo "Please install IPOPT manually from https://github.com/coin-or/Ipopt"
    exit 1
}

# ─── CasADi 3.6.x ────────────────────────────────────────────────────────────
echo ""
echo "=== Building CasADi 3.6 ==="
if ! find "$INSTALL_PREFIX/lib" -name "libcasadi*" 2>/dev/null | grep -q casadi; then
    CASADI_VER="3.6.5"
    wget -q "https://github.com/casadi/casadi/releases/download/${CASADI_VER}/casadi-${CASADI_VER}.tar.gz"
    tar xf "casadi-${CASADI_VER}.tar.gz"
    cmake -B "casadi-${CASADI_VER}/build" -S "casadi-${CASADI_VER}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DWITH_IPOPT=ON \
        -DWITH_EXAMPLES=OFF \
        -DWITH_PYTHON=OFF \
        -DWITH_MATLAB=OFF
    cmake --build "casadi-${CASADI_VER}/build" -j"$JOBS"
    sudo cmake --install "casadi-${CASADI_VER}/build"
    echo "CasADi installed."
else
    echo "CasADi already installed, skipping."
fi

# ─── ldconfig ────────────────────────────────────────────────────────────────
echo ""
echo "=== Updating linker cache ==="
sudo ldconfig

echo ""
echo "=== All dependencies installed successfully ==="
echo ""
echo "To build mpc_cpp:"
echo "  cd \$(git rev-parse --show-toplevel)/BFYP/mpc_cpp"
echo "  cmake -B build -DCMAKE_BUILD_TYPE=Release"
echo "  cmake --build build -j\$(nproc)"
echo "  ./build/test_mpc_solver config/robot_params.yaml"
