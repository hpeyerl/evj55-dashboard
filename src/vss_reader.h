// =============================================================
//  vss_reader.h - Linux/CM3 Vehicle Speed Sensor reader (DRAFT)
//
//  Interrupt-driven (libgpiod v1 line events) port of the ESP32
//  esp32p4/src/vss_sensor.c pulse->speed algorithm.
//
//  STATUS 2026-09-10: DRAFT, compile-checked only. NOT wired into the
//  dashboard build/deploy yet. Runtime-validate with the bench function
//  generator (drive VSS_SIG, watch vss_test print speed) before integrating.
//  Hardware path is proven (see evj55_contexts/HAT_BRINGUP.md 2026-09-10):
//  VSS = gpiochip3 line 27 (header pin 18 / GPIO3_D3).
//
//  Speed is MPH internally (matches the app's CAN speed convention).
// =============================================================
#ifndef VSS_READER_H
#define VSS_READER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Calibration. Default derived from ~12 pulses/metre (HAT_CONTEXT):
//   12 pulses/m * 1609.344 m/mile ~= 19312 pulses/mile.
// Override once calibrated at a known speed in the vehicle.
typedef struct {
    double pulses_per_mile;      // the one number that maps pulses -> distance
    double max_plausible_mph;    // discard faster than this (glitch gate)
    int    avg_depth;            // period-averaging window (<= VSS_MAX_AVG_DEPTH)
    double ema_alpha_slow;       // EMA when change is small
    double ema_alpha_fast;       // EMA when change exceeds fast_threshold
    double ema_fast_threshold_mph;
    double stale_timeout_ms;     // no pulse for this long -> speed decays to 0
} vss_cal_t;

// Sensible defaults (fills *c).
void vss_default_cal(vss_cal_t *c);

// Start the reader on the given gpiochip + line (VSS = "gpiochip3", 27).
// Spawns an event thread. Returns 0 on success, <0 on error.
int  vss_start(const char *chip, unsigned int line, const vss_cal_t *cal);

// Current smoothed speed in MPH (0.0 when stopped/stale). Thread-safe.
double vss_speed_mph(void);

// Total accumulated distance since start, in miles (odometer; no persistence yet).
double vss_miles(void);

// Stop the event thread and release the line.
void vss_stop(void);

#ifdef __cplusplus
}
#endif
#endif // VSS_READER_H
