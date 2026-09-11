# VSS reader status

- 2026-09-10: draft written (vss_reader.h/.cpp + vss_test.cpp). Interrupt-driven
  libgpiod v1 port of esp32p4/src/vss_sensor.c. Not wired into the dashboard.
- 2026-09-11: **RUNTIME VALIDATED on the box.** Built with libgpiod-dev, ran
  vss_test against the bench generator: 43 Hz -> steady 8.0 mph (= 43*3600/19312,
  exact), EMA smooth, odometer accumulating. Pulse->speed path proven correct.
  VSS = gpiochip3 line 27 (pin18/GPIO3_D3). libgpiod-dev now installed on eMMC.

## Remaining
1. Calibrate `pulses_per_mile` in-vehicle at a known speed (default 19312 = the
   ~12 pulses/m estimate; likely wrong until measured on the Toyota HF2A + tyre).
2. Integrate: add vss_reader.cpp to the dashboard CMake, `vss_start("gpiochip3",27,..)`
   in main_linux.cpp, feed `vss_speed_mph()` into the speed path (currently CAN-only,
   0x257/RPM). Eventually fuse VSS + motor-RPM + GPS (see HAT_CONTEXT speed fusion).
3. Odometer persistence (a file; the ESP32 used NVS).
