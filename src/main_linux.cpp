// SPDX-FileCopyrightText: 2026 Herb Peyerl
// SPDX-License-Identifier: BSD-3-Clause

// =============================================================
//  main_linux.c — EV Dashboard, Linux/Pi5 prototype entry point
//
//  Reuses the portable LVGL UI (../src/dashboard_ui.cpp) unchanged.
//  Replaces the ESP-IDF FreeRTOS/TWAI main with:
//    - DRM/KMS display (the DSI panel is already a DRM device)
//    - evdev touch (the Goodix GT911 shows up as a Linux input dev)
//    - the same SIM_DATA sinf() generator the ESP build uses
//
//  Usage:
//    ./dashboard [drm_device] [touch_event_device]
//    e.g. ./dashboard /dev/dri/card0 /dev/input/event4
//  Both args optional; drm defaults to /dev/dri/card0, touch is
//  skipped if not given.
// =============================================================
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "lvgl.h"
#include "dashboard_ui.h"
#include "can_parser.h"      // DashData
#include "vss_reader.h"    // VSS speed reader (GPIO3_D3 pulse -> mph)
#include "gear_shifter.h"    // gear_shifter_current()
#include "bms_http.h"        // bms_http_poll()
#include "http_api.h"        // http_api_start()

// Auto-detect the touchscreen's evdev node by scanning input device names.
// Returns a static "/dev/input/eventN" path, or NULL if none found.
static const char *find_touch_device(void)
{
    static char path[64];
    DIR *d = opendir("/sys/class/input");
    if (!d) return NULL;
    struct dirent *e;
    const char *found = NULL;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, "event", 5) != 0) continue;
        char namep[160], name[128] = {0};
        snprintf(namep, sizeof namep, "/sys/class/input/%s/device/name", e->d_name);
        FILE *f = fopen(namep, "r");
        if (!f) continue;
        if (fgets(name, sizeof name, f) &&
            (strstr(name, "Goodix") || strstr(name, "goodix") ||   /* Rockchip BSP names it "goodix-ts" */
             strstr(name, "Touch")  || strstr(name, "touch"))) {
            snprintf(path, sizeof path, "/dev/input/%s", e->d_name);
            found = path;
        }
        fclose(f);
        if (found) break;
    }
    closedir(d);
    return found;
}

// Open a non-blocking SocketCAN RAW socket bound to `iface`; -1 if unavailable.
static int open_can_socket(const char *iface)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { close(s); return -1; }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0) { close(s); return -1; }
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
    return s;
}

// CAN TX socket, set once the RX socket is bound (SocketCAN raw is bidirectional).
// -1 until then / in SIM mode, in which case a param write is a no-op + log.
static int s_can_tx_fd = -1;
// Set by a regenmax write; the main loop re-reads it so the UI shows the VCU's value.
static bool s_regen_resync = false;
#define REGEN_PARAM 61   // ZombieVerter regenmax (see dashboard_ui.cpp)
#define REGEN_RESYNC_MS 5000

// Platform hook the shared UI calls on regen-slider release. Sends an expedited
// SDO download (mirrors src/sdo_manager.c prv_send_frame): CAN 0x603,
// cmd 0x23, index 0x2100|(id>>8), subindex id&0xFF, value x32 little-endian.
extern "C" bool dashboard_vcu_set_param(uint16_t param_id, float value)
{
    int32_t raw = (int32_t)(value * 32.0f);
    if (s_can_tx_fd < 0) {
        printf("VCU set param %u = %.1f (raw %d): no CAN — not sent\n",
               param_id, value, raw);
        return false;
    }
    struct can_frame fr;
    memset(&fr, 0, sizeof fr);
    fr.can_id  = 0x603;                       // SDO_TX_ID (standard 11-bit)
    fr.can_dlc = 8;
    uint16_t index = 0x2100 | (param_id >> 8);
    fr.data[0] = 0x23;                        // SDO_CMD_WRITE (expedited, 4 bytes)
    fr.data[1] = (uint8_t)(index & 0xFF);
    fr.data[2] = (uint8_t)(index >> 8);
    fr.data[3] = (uint8_t)(param_id & 0xFF);
    fr.data[4] = (uint8_t)( raw        & 0xFF);
    fr.data[5] = (uint8_t)((raw >>  8) & 0xFF);
    fr.data[6] = (uint8_t)((raw >> 16) & 0xFF);
    fr.data[7] = (uint8_t)((raw >> 24) & 0xFF);
    ssize_t n = write(s_can_tx_fd, &fr, sizeof fr);
    bool ok = (n == (ssize_t)sizeof fr);
    printf("VCU set param %u = %.1f (raw %d): %s\n",
           param_id, value, raw, ok ? "sent 0x603" : "TX FAIL");
    if (ok && param_id == REGEN_PARAM) s_regen_resync = true;
    return ok;
}

// Send an SDO expedited-upload (read) request for `param_id` (TX 0x603,
// cmd 0x40). The VCU replies on 0x583, decoded in the RX drain below.
static bool vcu_send_sdo_read(uint16_t param_id)
{
    if (s_can_tx_fd < 0) return false;
    struct can_frame fr;
    memset(&fr, 0, sizeof fr);
    fr.can_id  = 0x603;
    fr.can_dlc = 8;
    uint16_t index = 0x2100 | (param_id >> 8);
    fr.data[0] = 0x40;                        // SDO_CMD_READ (initiate upload)
    fr.data[1] = (uint8_t)(index & 0xFF);
    fr.data[2] = (uint8_t)(index >> 8);
    fr.data[3] = (uint8_t)(param_id & 0xFF);
    return write(s_can_tx_fd, &fr, sizeof fr) == (ssize_t)sizeof fr;
}

// LVGL tick source: milliseconds from a monotonic clock.
static uint32_t millis_cb(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL);
}

int main(int argc, char **argv)
{
    const char *fb_dev    = (argc > 1) ? argv[1] : "/dev/fb0";
    const char *touch_dev = (argc > 2) ? argv[2] : NULL;
    if (!touch_dev) touch_dev = find_touch_device();   // auto-detect GT911

    setvbuf(stdout, NULL, _IONBF, 0);   // unbuffered so logs show over ssh
    lv_init();
    lv_tick_set_cb(millis_cb);

    // --- display: fbdev ---
    // We use fbdev (not DRM) because LVGL's DRM driver renders DIRECT into the
    // framebuffer and cannot do software rotation, whereas the fbdev driver
    // rotates in software in its flush. The panel is 720x1920 native; we rotate
    // to 1920x720 landscape below.
    lv_display_t *disp = lv_linux_fbdev_create();
    if (!disp) { fprintf(stderr, "lv_linux_fbdev_create failed\n"); return 1; }
    lv_linux_fbdev_set_file(disp, fb_dev);

    // Panel is 720x1920 portrait; the dashboard UI is 1920x720 landscape.
    // 270 is the correct upright orientation for this panel (90 is upside down);
    // override with LV_ROTATE=0|90|180|270.
    int rot = 270;
    const char *renv = getenv("LV_ROTATE");
    if (renv) rot = atoi(renv);
    lv_display_rotation_t r = LV_DISPLAY_ROTATION_0;
    if      (rot == 90)  r = LV_DISPLAY_ROTATION_90;
    else if (rot == 180) r = LV_DISPLAY_ROTATION_180;
    else if (rot == 270) r = LV_DISPLAY_ROTATION_270;
    lv_display_set_rotation(disp, r);
    printf("rotation: %d\n", rot);

    printf("fbdev display %s  logical %dx%d\n", fb_dev,
           (int)lv_display_get_horizontal_resolution(disp),
           (int)lv_display_get_vertical_resolution(disp));

    // --- touch: evdev (Goodix GT911) ---
    if (touch_dev) {
        lv_indev_t *touch = lv_evdev_create(LV_INDEV_TYPE_POINTER, touch_dev);
        if (touch) { lv_indev_set_display(touch, disp); lv_evdev_set_swap_axes(touch, true); lv_evdev_set_calibration(touch, 0, 0, 720, 1920); printf("touch: %s\n", touch_dev); }
        else fprintf(stderr, "warning: lv_evdev_create(%s) failed — running without touch\n", touch_dev);
    } else {
        printf("touch: (none — pass event device as arg 2 to enable)\n");
    }

    // --- build the dashboard UI ---
    dashboard_ui_create(disp);

    // --- HTTP control/status API (remote nav + live state) ---
    http_api_start(8080);

    // --- data source: real vehicle CAN (can0) if available, else simulated ---
    const char *can_iface = getenv("CAN_IFACE");
    if (!can_iface) can_iface = "can0";
    int can_fd = open_can_socket(can_iface);
    bool use_can = (can_fd >= 0);
    s_can_tx_fd = can_fd;   // enable SDO param writes (regen slider) over the same socket
    // Second bus (can1): RX-only / passive per the HAT CAN map (vehicle + charger).
    // -1 until it exists (no HAT yet); retried below like can0 for the boot race.
    int can1_fd = open_can_socket("can1");
    // SIM sine-wave data is for bench/animation testing ONLY — off by default so a real
    // vehicle boot never shows fake numbers. Enable with DASH_SIM=1 for demos.
    const bool sim_enabled = (getenv("DASH_SIM") != NULL);
    printf("data source: %s\n", use_can ? can_iface
           : (sim_enabled ? "SIM (sine-wave demo)" : "waiting for CAN (blank until can0 is up)"));
    // If CAN isn't up yet (the interface is often brought up by systemd AFTER this
    // process starts — a boot race), keep retrying ~1 Hz and switch over the instant
    // it appears, instead of being stuck for the whole session.
    uint32_t can_retry_t0 = millis_cb();

    // g_dash is defined + initialized in can_parser.cpp (dir_confirmed=INT8_MIN);
    // do NOT memset it here. In CAN mode parse_can_frame() fills it from frames.
    float t = 0.0f;
    uint64_t canload_bits = 0;               // CAN bus-load accumulator
    uint32_t canload_t0   = millis_cb();
    bool     regen_readback_done = false;    // set once the VCU answers our read
    uint32_t regen_req_t0 = 0;               // last read-request time (0 = never)
    uint32_t regen_ok_t0  = 0;               // last time the VCU answered
    dash_screen_t prev_screen = DASH_SCREEN_HOME;   // for STATUS-entry edge detect
    // Process one frame from either bus: shared parser + SDO readback + bus-load.
    auto process_frame = [&](const struct can_frame &fr) {
        uint32_t id = fr.can_id & CAN_EFF_MASK;
        parse_can_frame(id, fr.data, millis_cb());
        if (id == 0x583) {   // SDO reply: pick off a regenmax read for the slider
            uint8_t  cmd   = fr.data[0];
            uint16_t index = (uint16_t)(fr.data[1] | (fr.data[2] << 8));
            uint16_t pid   = (uint16_t)(((index & 0xFF) << 8) | fr.data[3]);
            if ((cmd == 0x43 || cmd == 0x4B) && pid == REGEN_PARAM) {
                int32_t raw = (int32_t)(fr.data[4] | (fr.data[5] << 8) |
                                        (fr.data[6] << 16) | (fr.data[7] << 24));
                dashboard_ui_set_regen_current(raw / 32.0f);
                regen_readback_done = true;
                regen_ok_t0 = millis_cb();
            }
        }
        int ov = (fr.can_id & CAN_EFF_FLAG) ? 67 : 47;
        canload_bits += (uint64_t)((ov + 8 * fr.can_dlc) * 1.15f);
    };
    // VSS speed sensor: interrupt-driven pulse counter on gpiochip3 line 27
    // (pin18/GPIO3_D3). Overrides CAN speed when active. Calibrate pulses_per_mile
    // in-vehicle later; default is the ~12 pulses/m estimate.
    { vss_cal_t vcal; vss_default_cal(&vcal);
      if (vss_start("gpiochip3", 27, &vcal) == 0) printf("vss: started (gpiochip3 line 27)\n");
      else fprintf(stderr, "vss: start failed - speed will use CAN only\n"); }

    for (;;) {
        // Not on CAN yet (or can1 still down)? retry ~1 Hz; switch over as ifaces appear.
        if (!use_can || can1_fd < 0) {
            uint32_t now = millis_cb();
            if (now - can_retry_t0 >= 1000) {
                can_retry_t0 = now;
                if (!use_can) {
                    can_fd = open_can_socket(can_iface);
                    if (can_fd >= 0) {
                        use_can = true;
                        s_can_tx_fd = can_fd;
                        printf("data source: switched to %s\n", can_iface);
                    }
                }
                if (can1_fd < 0) can1_fd = open_can_socket("can1");
            }
        }
        if (use_can) {
            // drain both buses -> shared parser; stamp per-bus liveness by interface
            struct can_frame fr;
            while (read(can_fd, &fr, sizeof fr) == (ssize_t)sizeof fr) {
                g_dash.last_ms_can0 = millis_cb();
                process_frame(fr);
            }
            if (can1_fd >= 0)
                while (read(can1_fd, &fr, sizeof fr) == (ssize_t)sizeof fr) {
                    g_dash.last_ms_can1 = millis_cb();
                    process_frame(fr);
                }
        } else if (sim_enabled) {
            t += 0.05f;
            g_dash.soc_pct         = 50.0f + 45.0f * sinf(t * 0.3f);
            g_dash.speed           = 60.0f + 50.0f * sinf(t * 0.7f);
            g_dash.power_kw        = 150.0f * sinf(t * 1.1f);
            g_dash.pack_volts      = 390.0f + 10.0f * sinf(t * 0.2f);
            g_dash.pack_amps       = g_dash.power_kw * 1000.0f / g_dash.pack_volts;
            g_dash.inverter_temp_c = 40.0f + 20.0f * sinf(t * 0.15f);
            g_dash.motor_temp_c    = 60.0f + 30.0f * sinf(t * 0.2f);
            g_dash.batt_temp_c     = 25.0f + 10.0f * sinf(t * 0.1f);
            g_dash.aux_volts       = 13.5f + 0.5f * sinf(t * 0.4f);
            g_dash.range_dist      = g_dash.soc_pct * 2.5f;
            g_dash.gear            = 3;
            g_dash.odo_total_miles = 12345.0f;
            g_dash.trip_miles      = 42.0f;
        }

        // VSS overrides CAN speed when the sensor is active (pulses present)
        if (!sim_enabled) {
            double vss = vss_speed_mph();
            if (vss > 0.0) g_dash.speed = (float)vss;
        }

        // CAN bus load %, refreshed ~1 Hz (500 kbps bus)
        uint32_t cl_now = millis_cb();
        if (cl_now - canload_t0 >= 1000) {
            float secs = (cl_now - canload_t0) / 1000.0f;
            float load = (float)canload_bits / (500000.0f * secs) * 100.0f;
            g_dash.can_load_pct = load > 100.0f ? 100.0f : load;
            canload_bits = 0;
            canload_t0   = cl_now;
        }

        // Re-sync regenmax each time the STATUS screen is opened, so a value
        // changed out-of-band (e.g. via the Zombie's HTTP port) shows up.
        dash_screen_t scr = dashboard_ui_get_screen();
        if (scr == DASH_SCREEN_VCU && prev_screen != DASH_SCREEN_VCU) {
            regen_readback_done = false;
            regen_req_t0 = 0;
        }
        prev_screen = scr;

        // Regenmax readback: poll ~1 Hz until the VCU answers, so the slider and
        // regen button show the real value. Re-armed after our own write (read
        // ~200 ms later) and every REGEN_RESYNC_MS, so a Zombie reboot back to
        // its saved value or an M5Dial write shows up.
        if (s_regen_resync) {
            s_regen_resync = false;
            regen_readback_done = false;
            regen_req_t0 = millis_cb() - 800;
        }
        if (regen_readback_done && millis_cb() - regen_ok_t0 >= REGEN_RESYNC_MS)
            regen_readback_done = false;
        if (use_can && !regen_readback_done) {
            uint32_t now = millis_cb();
            if (regen_req_t0 == 0 || now - regen_req_t0 >= 1000) {
                vcu_send_sdo_read(REGEN_PARAM);
                regen_req_t0 = now;
            }
        }

        // let a touched gear button override immediately (UI feedback)
        int8_t req = gear_shifter_current();
        if (req >= 0) g_dash.gear = (uint8_t)req;

        // poll the BMW i3 BMS web API (only while the BMS screen is showing)
        bms_http_poll(millis_cb(), scr == DASH_SCREEN_BMS);

        dashboard_ui_update(&g_dash);
        lv_timer_handler();
        usleep(30 * 1000);   // ~33 fps
    }
    return 0;
}
