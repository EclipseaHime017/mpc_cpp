cmake_minimum_required(VERSION 3.16)
project(mpc_cpp VERSION 1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
# Release: -O3 + native SIMD (NEON on Jetson Orin, AVX on x86)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

# ============================================================
# Dependencies
# ============================================================

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# --- Eigen3 ---
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")

# --- yaml-cpp ---
find_package(yaml-cpp REQUIRED)
# Fallback via pkg-config
if(NOT yaml-cpp_FOUND)
    pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
endif()

# --- OSQP ---
# Install: git clone https://github.com/osqp/osqp --recursive
#          cmake -S osqp -B osqp/build -DCMAKE_BUILD_TYPE=Release
#          cmake --build osqp/build -j4 && cmake --install osqp/build
find_package(osqp REQUIRED
    HINTS /usr/local/lib/cmake/osqp
          /usr/local/share/osqp/cmake
          $ENV{CONDA_PREFIX}/lib/cmake/osqp
          ${CMAKE_PREFIX_PATH})
message(STATUS "OSQP found")

# --- osqp-cpp (Google C++ wrapper) ---
# Install: git clone https://github.com/google/osqp-cpp
#          cmake -S osqp-cpp -B osqp-cpp/build && cmake --build ... --install
find_package(osqp-cpp QUIET
    HINTS /usr/local/lib/cmake/osqp-cpp
          $ENV{CONDA_PREFIX}/lib/cmake/osqp-cpp)
if(osqp-cpp_FOUND)
    message(STATUS "osqp-cpp found (Google wrapper)")
    set(OSQP_CPP_LIB osqp-cpp::osqp-cpp)
else()
    message(WARNING "osqp-cpp not found; using OsqpEigen instead")
    # Fallback: OsqpEigen (https://github.com/robotology/osqp-eigen)
    find_package(OsqpEigen REQUIRED
        HINTS /usr/local/lib/cmake/OsqpEigen
              $ENV{CONDA_PREFIX}/lib/cmake/OsqpEigen)
    set(OSQP_CPP_LIB OsqpEigen::OsqpEigen)
    add_compile_definitions(USE_OSQP_EIGEN)
endif()

# --- CasADi (with IPOPT backend) ---
# Install: git clone https://github.com/casadi/casadi
#          cmake -S casadi -B casadi/build -DWITH_IPOPT=ON
#          cmake --build casadi/build -j4 && cmake --install casadi/build
# Requires: sudo apt install coinor-libipopt-dev
find_package(casadi REQUIRED
    HINTS /usr/local/lib/cmake/casadi
          /usr/local/share/casadi
          $ENV{CONDA_PREFIX}/lib/cmake/casadi
          ${CMAKE_PREFIX_PATH})
message(STATUS "CasADi found")

# --- libusb (for CAN interfaces, inherited from pure_cpp) ---
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ============================================================
# Source files from pure_cpp (hardware communication layer)
# ============================================================

set(PURE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../pure_cpp/Main)

set(HW_SOURCES
    ${PURE_CPP_DIR}/src/robstride.cpp
    ${PURE_CPP_DIR}/src/can/can_interface.cpp
    ${PURE_CPP_DIR}/src/imu/serial.c
    ${PURE_CPP_DIR}/src/imu/wit_c_sdk.c
    ${PURE_CPP_DIR}/src/observations.cpp       # For IMUComponent, Gamepad
    ${PURE_CPP_DIR}/src/keyboard_input.cpp
)

# ============================================================
# MPC controller sources
# ============================================================

set(MPC_SOURCES
    src/common/robot_config.cpp
    src/estimator/state_estimator.cpp
    src/gait/gait_generator.cpp
    src/kinematics/quadruped_kin.cpp
    src/mpc/mpc_controller.cpp
    src/wbc/wbc_controller.cpp
    src/main.cpp
)

# ============================================================
# Main executable
# ============================================================

add_executable(mpc_robot_control
    ${MPC_SOURCES}
    ${HW_SOURCES}
)

target_include_directories(mpc_robot_control PUBLIC
    include
    ${PURE_CPP_DIR}/include
    ${LIBUSB_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(mpc_robot_control
    Threads::Threads
    Eigen3::Eigen
    yaml-cpp
    osqp::osqp
    ${OSQP_CPP_LIB}
    casadi::casadi
    ${LIBUSB_LIBRARIES}
)

# ============================================================
# Test executables (no hardware required)
# ============================================================

# Offline unit test for MPC solver (feed fixed inputs, compare to Python)
add_executable(test_mpc_solver
    src/common/robot_config.cpp
    src/kinematics/quadruped_kin.cpp
    src/gait/gait_generator.cpp
    src/mpc/mpc_controller.cpp
    src/mpc/nmpc_footstep.cpp
    tests/test_mpc_solver.cpp
)
target_include_directories(test_mpc_solver PUBLIC
    include
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_mpc_solver
    Eigen3::Eigen
    yaml-cpp
    osqp::osqp         # mpc_controller.cpp uses OSQP C API directly
    casadi::casadi
)

# ============================================================
# Build summary
# ============================================================
message(STATUS "")
message(STATUS "=== mpc_cpp Build Summary ===")
message(STATUS "Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "CXX flags:     ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "Eigen3:        ${EIGEN3_VERSION}")
message(STATUS "OSQP wrapper:  ${OSQP_CPP_LIB}")
message(STATUS "pure_cpp dir:  ${PURE_CPP_DIR}")
message(STATUS "=============================")
message(STATUS "")
