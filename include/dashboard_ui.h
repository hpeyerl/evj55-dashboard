// =============================================================
//  dashboard_ui.h — EV Dashboard LVGL UI public API
// =============================================================
#pragma once
#include "lvgl.h"
#include "can_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DASH_SCREEN_HOME     = 0,
    DASH_SCREEN_SETTINGS = 1,
    DASH_SCREEN_VCU      = 2,   // live VCU values grid (formerly "STATUS")
    DASH_SCREEN_BMS      = 3,
} dash_screen_t;

// Create all LVGL widgets. Call once after lv_init() and display setup.
void dashboard_ui_create(lv_display_t *disp);

// Update all widgets from a snapshot of DashData. Call each frame.
void dashboard_ui_update(const DashData *d);

// Switch active screen. Safe to call from any task (queues the switch).
void dashboard_ui_set_screen(dash_screen_t screen);

// Get current screen
dash_screen_t dashboard_ui_get_screen(void);

// Platform-provided: write a ZombieVerter parameter over CAN via an expedited
// SDO download (TX 0x603). `value` is in human units (e.g. regenmax % = -35..0);
// the transport scales x32 internally. Returns true if the frame was
// queued/sent. Implemented by main.cpp (ESP → sdo_write) and main_linux.cpp
// (Linux → SocketCAN). The UI calls this on regen-slider release.
bool dashboard_vcu_set_param(uint16_t param_id, float value);

// Platform → UI: report the VCU's current regenmax so the slider boots at the
// real value instead of a default. `pct` is the signed value read back (e.g.
// -22.0). Clamped to the guarded range; ignored while the user is dragging.
// Safe to call before the status screen exists (applied when it's built).
void dashboard_ui_set_regen_current(float pct);

// Platform -> UI: the VCU's current value of a RANGE/MOTOR param (27 Gear,
// 129 MotActive), read back over SDO. Clears a pending badge write.
void dashboard_ui_set_mode_current(uint16_t param_id, int value);

#ifdef __cplusplus
}
#endif
