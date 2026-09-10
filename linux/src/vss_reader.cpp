// =============================================================
//  vss_reader.cpp - Linux/CM3 VSS reader (DRAFT, compile-checked only)
//  See vss_reader.h. Port of esp32p4/src/vss_sensor.c to libgpiod v1.
// =============================================================
#include "vss_reader.h"

#include <gpiod.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <atomic>

#define VSS_MAX_AVG_DEPTH 16

static struct gpiod_chip *s_chip = nullptr;
static struct gpiod_line *s_line = nullptr;
static pthread_t          s_thread;
static std::atomic<bool>  s_run{false};

static vss_cal_t s_cal;
static double    s_usec_per_pulse_at_1mph = 0.0;

// Shared outputs (written by event thread, read by UI thread).
static std::atomic<double>   s_speed_mph{0.0};
static std::atomic<uint64_t> s_total_pulses{0};

// Event-thread-local period state.
static uint64_t s_periods[VSS_MAX_AVG_DEPTH];
static int      s_period_idx   = 0;
static int      s_period_count = 0;
static uint64_t s_last_ts_us   = 0;
static double   s_ema          = 0.0;
static bool     s_ema_init     = false;

static uint64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static void derive_cal(void)
{
    // usec per pulse when travelling 1 mph:
    //   3.6e9 us/hr / pulses_per_mile
    s_usec_per_pulse_at_1mph = 3600000000.0 / s_cal.pulses_per_mile;
}

// Faithful port of prv_process_ts: returns mph, or -1 to discard.
static double process_ts(uint64_t ts_us)
{
    if (s_last_ts_us == 0) { s_last_ts_us = ts_us; return -1.0; }
    uint64_t period = ts_us - s_last_ts_us;
    s_last_ts_us = ts_us;
    if (period == 0) return -1.0;

    double instant = s_usec_per_pulse_at_1mph / (double)period;
    if (instant > s_cal.max_plausible_mph || instant < 0.1) return -1.0;

    s_periods[s_period_idx] = period;
    s_period_idx = (s_period_idx + 1) % s_cal.avg_depth;
    if (s_period_count < s_cal.avg_depth) s_period_count++;

    uint64_t sum = 0;
    for (int i = 0; i < s_period_count; i++) sum += s_periods[i];
    double avg_period = (double)(sum / (uint64_t)s_period_count);
    return s_usec_per_pulse_at_1mph / avg_period;
}

static double ema_update(double sample)
{
    if (!s_ema_init) { s_ema = sample; s_ema_init = true; return s_ema; }
    double err   = sample - s_ema;
    double alpha = (fabs(err) > s_cal.ema_fast_threshold_mph)
                   ? s_cal.ema_alpha_fast : s_cal.ema_alpha_slow;
    s_ema += alpha * err;
    return s_ema;
}

static void reset_state(void)
{
    s_period_idx = s_period_count = 0;
    s_last_ts_us = 0;
    s_ema = 0.0; s_ema_init = false;
    s_speed_mph.store(0.0);
}

static void *event_loop(void *)
{
    while (s_run.load()) {
        // Wake either on an edge or on a timeout (so a stopped vehicle decays to 0).
        struct timespec to;
        to.tv_sec  = (time_t)(s_cal.stale_timeout_ms / 1000.0);
        to.tv_nsec = (long)((s_cal.stale_timeout_ms - to.tv_sec * 1000.0) * 1e6);

        int r = gpiod_line_event_wait(s_line, &to);
        if (r < 0) { continue; }          // error - retry
        if (r == 0) {                      // timeout: no pulses -> stopped
            reset_state();
            continue;
        }
        struct gpiod_line_event ev;
        if (gpiod_line_event_read(s_line, &ev) < 0) continue;
        if (ev.event_type != GPIOD_LINE_EVENT_RISING_EDGE) continue;  // count 1 edge/pulse

        s_total_pulses.fetch_add(1);
        double spd = process_ts(now_us());
        if (spd >= 0.0)
            s_speed_mph.store(ema_update(spd));
    }
    return nullptr;
}

extern "C" void vss_default_cal(vss_cal_t *c)
{
    c->pulses_per_mile         = 19312.0;   // ~12 pulses/m; CALIBRATE in vehicle
    c->max_plausible_mph       = 160.0;
    c->avg_depth               = 8;
    c->ema_alpha_slow          = 0.15;
    c->ema_alpha_fast          = 0.55;
    c->ema_fast_threshold_mph  = 5.0;
    c->stale_timeout_ms        = 750.0;     // ~ below this pulse rate = "stopped"
}

extern "C" int vss_start(const char *chip, unsigned int line, const vss_cal_t *cal)
{
    if (s_run.load()) return -1;
    s_cal = *cal;
    if (s_cal.avg_depth < 1) s_cal.avg_depth = 1;
    if (s_cal.avg_depth > VSS_MAX_AVG_DEPTH) s_cal.avg_depth = VSS_MAX_AVG_DEPTH;
    derive_cal();
    reset_state();

    s_chip = gpiod_chip_open_by_name(chip);
    if (!s_chip) { fprintf(stderr, "vss: open chip %s failed\n", chip); return -2; }
    s_line = gpiod_chip_get_line(s_chip, line);
    if (!s_line) { fprintf(stderr, "vss: get line %u failed\n", line); gpiod_chip_close(s_chip); s_chip=nullptr; return -3; }
    if (gpiod_line_request_both_edges_events(s_line, "vss") < 0) {
        fprintf(stderr, "vss: request edges failed (line busy?)\n");
        gpiod_chip_close(s_chip); s_chip=nullptr; return -4;
    }
    s_run.store(true);
    if (pthread_create(&s_thread, nullptr, event_loop, nullptr) != 0) {
        s_run.store(false);
        gpiod_line_release(s_line);
        gpiod_chip_close(s_chip); s_chip=nullptr;
        return -5;
    }
    return 0;
}

extern "C" double vss_speed_mph(void) { return s_speed_mph.load(); }

extern "C" double vss_miles(void)
{
    return (double)s_total_pulses.load() / s_cal.pulses_per_mile;
}

extern "C" void vss_stop(void)
{
    if (!s_run.load()) return;
    s_run.store(false);
    pthread_join(s_thread, nullptr);
    if (s_line) gpiod_line_release(s_line);
    if (s_chip) gpiod_chip_close(s_chip);
    s_line = nullptr; s_chip = nullptr;
}
