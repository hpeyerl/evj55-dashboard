cmake_minimum_required(VERSION 3.16)
project(ev_dashboard_pi5 C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- LVGL configuration: point LVGL at our Linux lv_conf.h ---
set(LV_BUILD_CONF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h" CACHE FILEPATH "" FORCE)
# The dashboard uses no vector graphics; skip the internal ThorVG build.
set(CONFIG_LV_USE_THORVG_INTERNAL OFF CACHE BOOL "" FORCE)

# --- libdrm: needed to compile LVGL's DRM driver and to link ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBDRM REQUIRED libdrm)
include_directories(${LIBDRM_INCLUDE_DIRS})   # so lv_linux_drm.c finds <xf86drm.h> etc.

# --- LVGL: use the vendored 9.5.0 copy if present, else fetch it ---
set(LVGL_LOCAL "${CMAKE_CURRENT_SOURCE_DIR}/managed_components/lvgl__lvgl")
if(EXISTS "${LVGL_LOCAL}/lvgl.h")
  message(STATUS "Using vendored LVGL: ${LVGL_LOCAL}")
  add_subdirectory(${LVGL_LOCAL} ${CMAKE_BINARY_DIR}/lvgl)
else()
  include(FetchContent)
  FetchContent_Declare(lvgl
    GIT_REPOSITORY https://github.com/lvgl/lvgl.git
    GIT_TAG        v9.5.0)
  FetchContent_MakeAvailable(lvgl)
endif()

set(REPO ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(dashboard
  src/main_linux.cpp
  src/stubs.cpp
  src/vss_reader.cpp
  src/bms_http.c
  src/http_api.c
  ${REPO}/src/dashboard_ui.cpp
  ${REPO}/src/can_parser.cpp
  ${REPO}/src/bms_data.c
  ${REPO}/include/cJSON.c
  ${REPO}/src/lv_font_montserrat_72.c
  ${REPO}/src/lv_font_montserrat_110.c
)

# DISPLAY_TARGET=2 -> Waveshare 1920x720 layout; SIM_DATA drives the demo.
# LCD_*_RES are the landscape UI dimensions (ESP sets these via platformio flags).
# UNITS_METRIC: km/h, km, km/kWh (default); -DUNITS_METRIC=OFF for mph/mi.
option(UNITS_METRIC "Display metric units" ON)
if(UNITS_METRIC)
  set(UNITS_METRIC_VAL 1)
else()
  set(UNITS_METRIC_VAL 0)
endif()
target_compile_definitions(dashboard PRIVATE
    DISPLAY_TARGET=2 SIM_DATA=1 LCD_H_RES=1920 LCD_V_RES=720 BMS_HTTP=1
    UNITS_METRIC=${UNITS_METRIC_VAL})

# Include order matters: compat shims first (shadow esp_timer.h / wifi_manager.h /
# display_driver.h), then our linux dir (lv_conf.h) before the ESP src/ copy.
target_include_directories(dashboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/compat
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${REPO}/include
  ${REPO}/src
)

find_package(Threads REQUIRED)
target_link_libraries(dashboard PRIVATE lvgl m ${LIBDRM_LIBRARIES} Threads::Threads gpiod)

# Strip symbols in Release (the ~142 KB symbol table; the fonts/rodata stay).
# Cosmetic — the app loads+inits in ~0.1s — but keeps the deployed binary lean.
target_link_options(dashboard PRIVATE $<$<CONFIG:Release>:-s>)
