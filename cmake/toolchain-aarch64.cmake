# Cross-compile the dashboard for the Radxa CM3 (RK3566, aarch64 Linux).
#
# Requires on the dev box:
#   sudo apt install g++-aarch64-linux-gnu
#   sudo dpkg --add-architecture arm64 && sudo apt install libdrm-dev:arm64 libgpiod-dev:arm64
#   (arm64 packages come from ports.ubuntu.com; see /etc/apt/sources.list.d/arm64-ports.list)
#
# Usage:
#   cmake -S . -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake
#   cmake --build build-arm64 -j

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

# Multiarch: :arm64 dev packages install under the host root's aarch64 triplet dirs.
set(CMAKE_FIND_ROOT_PATH /usr/lib/aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# Point pkg-config at the arm64 .pc files (libdrm) rather than the host amd64 ones.
set(ENV{PKG_CONFIG_LIBDIR}      "/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "/")
