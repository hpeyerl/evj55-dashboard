// SPDX-FileCopyrightText: 2026 Herb Peyerl
// SPDX-License-Identifier: BSD-3-Clause

// =============================================================
//  dashboard_ui.cpp — EV Dashboard LVGL UI  v3
//
//  Layout (landscape 1280×720):
//
//  ┌──────────────┬──┬──────────────────────┬──┬────────────┐
//  │ 5× half-arc  │S │                      │P │ Efficiency │
//  │ meter gauges │O │    157  km/h          │W │ Trip kWh  │
//  │ inv/mot/bat  │C │    D  PRND            │R │ Range     │
//  │ pack V/A     │  │                      │  │ 12V aux   │
//  ├──────────────┴──┴──────────────────────┴──┴────────────┤
//  │  CAN 500kbps            ESP32-P4 | M5Stack Tab5    ●   │
//  └──────────────────────────────────────────────────────────┘
//
//  Left/right panels have large inner border-radius for bracket effect.
//  Meter gauges: half-circle arc + colored zone arcs + needle line.
//  SOC arc:   "(" shape left of speed. fills CW. cyan≥50% amber 21-49% red≤20%.
//  Power arc: ")" shape right of speed. zero=center, orange=drive, green=regen.
// =============================================================

#include "dashboard_ui.h"
#include "dashboard_layout.h"
#include "can_signals.h"
#include "units.h"
#include "wifi_manager.h"
#include "gear_shifter.h"
#include "lvgl.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>
#include <cstring>
#ifdef __linux__
#include <sys/statvfs.h>   // read-only-root health check (Pi/CM3 build)
#endif
#ifdef BMS_HTTP
#include "bms_data.h"
#endif

LV_FONT_DECLARE(lv_font_montserrat_72)
LV_FONT_DECLARE(lv_font_montserrat_110)

// ── Colour palette ────────────────────────────────────────────────────────
#define CLR_BG          lv_color_hex(0x07090E)
#define CLR_PANEL       lv_color_hex(0x0B0F18)
#define CLR_BORDER      lv_color_hex(0x161C26)
#define CLR_CYAN        lv_color_hex(0x00E5FF)
#define CLR_GREEN       lv_color_hex(0x10B981)
#define CLR_AMBER       lv_color_hex(0xF59E0B)
#define CLR_RED         lv_color_hex(0xEF4444)
#define CLR_TEXT_DIM    lv_color_hex(0x2D3A50)
#define CLR_TEXT_MID    lv_color_hex(0x4A5A70)
#define CLR_TEXT_BRIGHT lv_color_hex(0xC8D6E5)
#define CLR_WHITE       lv_color_hex(0xFFFFFF)
#define CLR_ORANGE      lv_color_hex(0xF59E0B)

// Sub-screen "safe area": keep content clear of the top-left HOME button and
// the far-right ◀▶ arrow strip so the nav controls never occlude content.
#define NAV_SAFE_TOP    16
#define NAV_SAFE_RIGHT  240   // reserves the 🏠 ◀ ▶ cluster (3 x 64, 16px gaps)

// ── Layout — see dashboard_layout.h (selected by DISPLAY_TARGET) ─────────
// Layout constants (LEFT_W, RIGHT_W, BOT_H, ARC_R, METER_R etc.)
// are defined in layout_waveshare.h / layout_tab5.h / layout_stub.h

// Meter spacing — not in layout header since it's derived from screen height
#define METER_GAP  90    // vertical spacing between meter centers

// ── Widget handles ────────────────────────────────────────────────────────
// Meter: arc indicator + needle line + value label
typedef struct {
    lv_obj_t *arc_green;
    lv_obj_t *arc_yellow;
    lv_obj_t *arc_red;
    lv_obj_t *needle;
    lv_obj_t *lbl_val;
} Meter;

static Meter s_inv_meter;
static Meter s_mot_meter;
static Meter s_bat_meter;
static Meter s_pv_meter;
static Meter s_pa_meter;

#define SOC_TICKS 100
#define PWR_TICKS 100
#define MARK_OVER 12             // white marker overhang past each end of the comb band
#define MARK_W    6              // white marker width (a touch wider than a tick)
static lv_obj_t *s_arc_combs   = NULL;   // container for both comb arcs (single fade handle)
static lv_obj_t *s_soc_tick[SOC_TICKS];  // SOC comb: fills bottom->top
static lv_obj_t *s_pwr_tick[PWR_TICKS];  // power comb: center-zero, drive up / regen down
static lv_obj_t *s_soc_mark    = NULL;   // longer white current-level marker (SOC)
static lv_obj_t *s_pwr_mark    = NULL;   // longer white current-level marker (power)
static lv_coord_t s_soc_cx, s_soc_cy, s_pwr_cx, s_pwr_cy;  // arc centers (for markers)
static lv_obj_t *s_lbl_soc_pct = NULL;
static lv_obj_t *s_lbl_range   = NULL;
static lv_obj_t *s_lbl_pwr_val = NULL;
static lv_obj_t *s_lbl_speed   = NULL;
static lv_obj_t *s_lbl_prnd[4];
static lv_obj_t *s_lbl_range_badge = NULL;   // M5Dial hi/lo range (display only)
static lv_obj_t *s_lbl_motor_badge = NULL;   // M5Dial motor config (display only)
static void prv_add_arrow_nav(lv_obj_t *screen);   // fwd decl (used in home builder)
static void prv_add_regen_button(lv_obj_t *screen); // fwd decl (used in home builder)
static lv_obj_t *s_lbl_cruise    = NULL;  // cruise speed target
static lv_obj_t *s_lbl_cruise_st = NULL;  // CC / SET / RES indicator
static lv_obj_t *s_lbl_eff     = NULL;
static lv_obj_t *s_lbl_trip    = NULL;
static lv_obj_t *s_lbl_aux_v   = NULL;
static lv_obj_t *s_dot_can     = NULL;   // CAN0 liveness
static lv_obj_t *s_dot_can1    = NULL;   // CAN1 liveness
static lv_obj_t *s_dot_wifi    = NULL;
static lv_obj_t *s_dot_rw      = NULL;   // read-write-root warning (should be ro)
static lv_obj_t *s_lbl_odo_val      = NULL;
static lv_obj_t *s_lbl_trip_odo_val = NULL;


// ── Helpers ───────────────────────────────────────────────────────────────
static lv_obj_t *make_label(lv_obj_t *parent, const char *txt,
                              lv_color_t col, const lv_font_t *font)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_set_style_text_font(l, font, 0);
    return l;
}

static lv_obj_t *make_rect(lv_obj_t *parent,
                             lv_coord_t x, lv_coord_t y,
                             lv_coord_t w, lv_coord_t h,
                             lv_color_t color, lv_coord_t radius)
{
    lv_obj_t *r = lv_obj_create(parent);
    lv_obj_set_pos(r, x, y);
    lv_obj_set_size(r, w, h);
    lv_obj_set_style_bg_color(r, color, 0);
    lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(r, 0, 0);
    lv_obj_set_style_radius(r, radius, 0);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

// Create one meter gauge.
// parent: container object
// cx, cy: center of the half-circle (flat side at cy, arc opens upward)
// green_end_deg, yellow_end_deg: arc zone boundaries (0-180°, left=0 right=180)
// label: short tag string
static void make_meter(Meter *m, lv_obj_t *parent,
                        lv_coord_t cx, lv_coord_t cy,
                        int16_t green_end, int16_t yellow_end,
                        const char *tag)
{
    lv_coord_t sz = METER_R * 2;
    lv_coord_t ax = cx - METER_R;
    lv_coord_t ay = cy - METER_R;

    // Background arc (full 180° sweep, flat bottom)
    // In LVGL arc: 0=right, 90=bottom, 180=left, 270=top
    // Half circle flat-bottom = bg from 180 to 360(=0) going CW
    // But we want arc that goes from LEFT to RIGHT across the top.
    // bg_start=180 (left), bg_end=360(=0) (right) → 180° CW sweep
    lv_obj_t *bg = lv_arc_create(parent);
    lv_obj_set_pos(bg, ax, ay);
    lv_obj_set_size(bg, sz, sz);
    lv_arc_set_bg_angles(bg, 180, 360);
    lv_arc_set_angles(bg, 180, 180);
    lv_obj_set_style_arc_color(bg, CLR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_arc_color(bg, CLR_BORDER, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(bg, METER_W, LV_PART_MAIN);
    lv_obj_set_style_arc_width(bg, METER_W, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bg, LV_OPA_TRANSP, 0);
    lv_obj_remove_style(bg, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_CLICKABLE);

    // Green zone arc (from 180° toward right by green_end degrees)
    int16_t g_end = 180 + green_end;
    lv_obj_t *ag = lv_arc_create(parent);
    lv_obj_set_pos(ag, ax, ay);
    lv_obj_set_size(ag, sz, sz);
    lv_arc_set_bg_angles(ag, 180, g_end);
    lv_arc_set_angles(ag, 180, g_end);
    lv_obj_set_style_arc_color(ag, CLR_GREEN, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ag, CLR_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(ag, METER_W, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ag, METER_W, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ag, LV_OPA_TRANSP, 0);
    lv_obj_remove_style(ag, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ag, LV_OBJ_FLAG_CLICKABLE);
    m->arc_green = ag;

    // Yellow zone
    int16_t y_end = 180 + yellow_end;
    lv_obj_t *ay2 = lv_arc_create(parent);
    lv_obj_set_pos(ay2, ax, ay);
    lv_obj_set_size(ay2, sz, sz);
    lv_arc_set_bg_angles(ay2, g_end, y_end);
    lv_arc_set_angles(ay2, g_end, y_end);
    lv_obj_set_style_arc_color(ay2, CLR_AMBER, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ay2, CLR_AMBER, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(ay2, METER_W, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ay2, METER_W, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ay2, LV_OPA_TRANSP, 0);
    lv_obj_remove_style(ay2, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ay2, LV_OBJ_FLAG_CLICKABLE);
    m->arc_yellow = ay2;

    // Red zone (from yellow_end to 360)
    lv_obj_t *ar = lv_arc_create(parent);
    lv_obj_set_pos(ar, ax, ay);
    lv_obj_set_size(ar, sz, sz);
    lv_arc_set_bg_angles(ar, y_end, 360);
    lv_arc_set_angles(ar, y_end, 360);
    lv_obj_set_style_arc_color(ar, CLR_RED, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ar, CLR_RED, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(ar, METER_W, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ar, METER_W, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ar, LV_OPA_TRANSP, 0);
    lv_obj_remove_style(ar, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ar, LV_OBJ_FLAG_CLICKABLE);
    m->arc_red = ar;

    // Needle: thin line widget rotated by transform
    // We use a narrow rect rotated via style transform
    lv_obj_t *nd = lv_obj_create(parent);
    lv_obj_set_size(nd, 2, METER_R - 6);
    lv_obj_set_style_bg_color(nd, CLR_WHITE, 0);
    lv_obj_set_style_bg_opa(nd, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(nd, 0, 0);
    lv_obj_set_style_radius(nd, 1, 0);
    // Position at center; rotation via transform_angle
    lv_obj_set_pos(nd, cx - 1, cy - METER_R + 6);
    lv_obj_set_style_transform_pivot_x(nd, 1, 0);
    lv_obj_set_style_transform_pivot_y(nd, METER_R - 6, 0);
    lv_obj_set_style_transform_angle(nd, 0, 0);
    lv_obj_clear_flag(nd, LV_OBJ_FLAG_SCROLLABLE);
    m->needle = nd;

    // Center dot
    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_set_size(dot, 6, 6);
    lv_obj_set_pos(dot, cx - 3, cy - 3);
    lv_obj_set_style_bg_color(dot, CLR_WHITE, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);

    // Tag label
    lv_obj_t *tl = make_label(parent, tag, CLR_CYAN, &lv_font_montserrat_10);
    lv_obj_set_pos(tl, cx - 14, cy + 4);

    // Value label
    m->lbl_val = make_label(parent, "--", CLR_GREEN, &lv_font_montserrat_14);
    lv_obj_set_pos(m->lbl_val, cx - 20, cy + 16);
}

// Angle (deg, CW) of comb tick `i` of `n` along a0..a1 (a1 may exceed 360).
static float comb_angle(int i, int n, float a0, float a1)
{
    float t = (n > 1) ? (float)i / (float)(n - 1) : 0.0f;
    return a0 + t * (a1 - a0);
}

// Position a rect `o` (size tw×len) centered at radius r / angle `ang` from
// (cx,cy), rotated so its long axis is radial. Mirrors the meter-needle
// transform pattern; reused for both comb ticks and the longer marker.
static void place_radial(lv_obj_t *o, lv_coord_t cx, lv_coord_t cy, lv_coord_t r,
                         lv_coord_t len, lv_coord_t tw, float ang)
{
    float rad = ang * (float)M_PI / 180.0f;
    lv_coord_t x = cx + (lv_coord_t)(r * cosf(rad));
    lv_coord_t y = cy + (lv_coord_t)(r * sinf(rad));
    lv_obj_set_size(o, tw, len);
    lv_obj_set_pos(o, x - tw / 2, y - len / 2);
    lv_obj_set_style_transform_pivot_x(o, tw / 2, 0);
    lv_obj_set_style_transform_pivot_y(o, len / 2, 0);
    int a10 = (int)lroundf((ang - 90.0f) * 10.0f);   // 0.1° units, radial
    a10 %= 3600; if (a10 < 0) a10 += 3600;
    lv_obj_set_style_transform_angle(o, a10, 0);
}

// Build `n` radial tick marks along a CW arc a0..a1 (a1 may cross 0°) centered
// at (cx,cy) radius r. Unlit ticks (CLR_BORDER) form the track; the update loop
// recolors them. Stored in out[].
static void make_comb(lv_obj_t *parent, lv_obj_t **out, int n,
                      lv_coord_t cx, lv_coord_t cy, lv_coord_t r,
                      lv_coord_t len, lv_coord_t tw, float a0, float a1)
{
    for (int i = 0; i < n; i++) {
        lv_obj_t *tk = lv_obj_create(parent);
        lv_obj_set_style_border_width(tk, 0, 0);
        lv_obj_set_style_radius(tk, 1, 0);
        lv_obj_set_style_bg_opa(tk, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(tk, CLR_BORDER, 0);   // start unlit
        place_radial(tk, cx, cy, r, len, tw, comb_angle(i, n, a0, a1));
        lv_obj_clear_flag(tk, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(tk, LV_OBJ_FLAG_SCROLLABLE);
        out[i] = tk;
    }
}

// Create the longer white current-level marker (protrudes past both comb ends).
static lv_obj_t *make_marker(lv_obj_t *parent)
{
    lv_obj_t *m = lv_obj_create(parent);
    lv_obj_set_style_border_width(m, 0, 0);
    lv_obj_set_style_radius(m, 1, 0);
    lv_obj_set_style_bg_opa(m, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(m, CLR_WHITE, 0);
    lv_obj_add_flag(m, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(m, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(m, LV_OBJ_FLAG_SCROLLABLE);
    return m;
}

// Update needle angle for a meter.
// pct: 0.0 (left/min) to 1.0 (right/max)
// Needle at pct=0 points left (270° in LVGL = pointing up when flat-bottom arc)
// Actually: flat-bottom arc left edge = 180°, right edge = 360°
// pct=0 → angle 270° (pointing straight up-left)
// pct=0.5 → angle 270° (pointing straight up)  
// Wait — let's think in terms of the needle rotation:
// Arc goes from 180° (left) to 360°(right) = 180° sweep
// pct=0 → needle points at 180° = left side = angle from vertical = -90°
// pct=1 → needle points at 360° = right side = angle from vertical = +90°
// pct=0.5 → needle points at 270° = straight up = angle 0°
// transform_angle in LVGL is in tenths of degrees, 0=pointing up
// At pct: needle_angle = (pct - 0.5) * 180 degrees
static void update_needle(Meter *m, float pct)
{
    if (pct < 0) pct = 0;
    if (pct > 1) pct = 1;
    int16_t angle_deg = (int16_t)((pct - 0.5f) * 180.0f);
    lv_obj_set_style_transform_angle(m->needle, angle_deg * 10, 0);
    lv_obj_invalidate(m->needle);
}

// ── Screen management ────────────────────────────────────────
static lv_obj_t    *s_scr_home     = NULL;
static lv_obj_t    *s_scr_settings = NULL;
static lv_obj_t    *s_scr_vcu   = NULL;
static dash_screen_t s_cur_screen  = DASH_SCREEN_HOME;
static lv_display_t *s_disp        = NULL;
static volatile dash_screen_t s_pending_screen    = DASH_SCREEN_HOME;
static volatile bool          s_screen_change_req = false;

// ── PRND tap callback ─────────────────────────────────────────────────────
static void prv_gear_tap_cb(lv_event_t *e)
{
    int8_t gear = (int8_t)(intptr_t)lv_event_get_user_data(e);
    gear_shifter_request(gear);
}

// Nav-bar button tap → switch screen. user_data is the dash_screen_t index
// (HOME=0, SETTINGS=1, STATUS=2), matching nav_labels order.
static void prv_nav_tap_cb(lv_event_t *e)
{
    int scr = (int)(intptr_t)lv_event_get_user_data(e);
    dashboard_ui_set_screen((dash_screen_t)scr);
}

// ── dashboard_ui_create ───────────────────────────────────────────────────
void dashboard_ui_create(lv_display_t *disp)
{
    s_disp = disp;
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    s_scr_home = scr;
    lv_obj_set_style_bg_color(scr, CLR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    const lv_coord_t W      = LCD_H_RES;
    const lv_coord_t H      = LCD_V_RES;
    const lv_coord_t MAIN_H = H;          // full height — bottom tab bar removed


    // ── Left panel — bracket shape via radius on inner-top and inner-bottom corners
    lv_obj_t *left = lv_obj_create(scr);
    lv_obj_set_pos(left, 0, 0);
    lv_obj_set_size(left, LEFT_W, MAIN_H);
    lv_obj_set_style_bg_opa(left, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(left, 0, 0);
    lv_obj_set_style_radius(left, 0, 0);
    // Bracket curve: large radius on right side
    lv_obj_set_style_pad_all(left, 0, 0);
    lv_obj_clear_flag(left, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(left, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    // Meters are children of left panel — coords relative to left panel
    lv_coord_t meter_cx = LEFT_W / 2;
    lv_coord_t meter_spacing = (MAIN_H - 20) / 5;
    lv_coord_t cy0 = meter_spacing / 2;

    make_meter(&s_inv_meter, left, meter_cx, cy0 + 0*meter_spacing,
               126, 153, "Inverter");
    make_meter(&s_mot_meter, left, meter_cx, cy0 + 1*meter_spacing,
               126, 153, "Motor");
    make_meter(&s_bat_meter, left, meter_cx, cy0 + 2*meter_spacing,
               126, 153, "HV Battery");
    // Pack V: range 290-450V = 160V = 180°
    // Zones: 290-299 (10°)=red, 300-340 (45°)=orange, 341-430 (100°)=green, 430-450 (22°)=red
    // green_end=10°, yellow_end=55° passed to make_meter; arc_red resized + extra arc added
    make_meter(&s_pv_meter,  left, meter_cx, cy0 + 3*meter_spacing,
               10, 55, "Pack V");
    lv_obj_set_style_arc_color(s_pv_meter.arc_green,  CLR_RED,    LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_pv_meter.arc_green,  CLR_RED,    LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_pv_meter.arc_yellow, CLR_ORANGE, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_pv_meter.arc_yellow, CLR_ORANGE, LV_PART_INDICATOR);
    // arc_red was 235-360°; resize to 235-338° (green zone) and recolor
    lv_arc_set_bg_angles(s_pv_meter.arc_red, 235, 338);
    lv_arc_set_angles(s_pv_meter.arc_red,    235, 338);
    lv_obj_set_style_arc_color(s_pv_meter.arc_red, CLR_GREEN, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_pv_meter.arc_red, CLR_GREEN, LV_PART_INDICATOR);
    // Extra arc for 430+V (red) — 338-358°
    {
        lv_coord_t pv_cy = cy0 + 3 * meter_spacing;
        lv_coord_t sz    = METER_R * 2;
        lv_obj_t  *ar2   = lv_arc_create(left);
        lv_obj_set_pos(ar2, meter_cx - METER_R, pv_cy - METER_R);
        lv_obj_set_size(ar2, sz, sz);
        lv_arc_set_bg_angles(ar2, 338, 358);
        lv_arc_set_angles(ar2,    338, 358);
        lv_obj_set_style_arc_color(ar2, CLR_RED, LV_PART_MAIN);
        lv_obj_set_style_arc_color(ar2, CLR_RED, LV_PART_INDICATOR);
        lv_obj_set_style_arc_width(ar2, METER_W, LV_PART_MAIN);
        lv_obj_set_style_arc_width(ar2, METER_W, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(ar2, LV_OPA_TRANSP, 0);
        lv_obj_remove_style(ar2, NULL, LV_PART_KNOB);
        lv_obj_clear_flag(ar2, LV_OBJ_FLAG_CLICKABLE);
    }

    make_meter(&s_pa_meter,  left, meter_cx, cy0 + 4*meter_spacing,
               90, 90, "Pack A");

    // Pack A: green left half, orange right half (regen / drive)
    lv_obj_set_style_arc_color(s_pa_meter.arc_green, CLR_GREEN, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_pa_meter.arc_green, CLR_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_pa_meter.arc_yellow, CLR_ORANGE, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_pa_meter.arc_yellow, CLR_ORANGE, LV_PART_INDICATOR);

    // ── SOC arc — "(" shape ───────────────────────────────────────────────
    // Arc center is placed to the RIGHT of left panel edge so only the
    // left-curving inner portion is visible on screen.
    // LVGL arc: 0=right/3-o'clock, goes CW. 
    // For a "(" shape we need the arc to sweep from ~top-left to bottom-left.
    // Center at (LEFT_W + ARC_R, MAIN_H/2), radius ARC_R.
    // Sweep from 120° to 240° (CW) = left-facing 120° arc.
    // 50% thicker than SVG sketch: stroke width = 21
    // ARC_R, ARC_W, ARC_START, ARC_END from dashboard_layout.h

    lv_coord_t soc_cx = LEFT_W + ARC_INSET;    // center x — off right edge of left panel
    lv_coord_t soc_cy = MAIN_H / 2;         // center y — vertical midpoint

    // Comb container — holds both arcs' ticks; single handle for the stale fade.
    // Full main area, transparent, overflow visible (ticks sit at arc radius).
    s_arc_combs = lv_obj_create(scr);
    lv_obj_set_pos(s_arc_combs, 0, 0);
    lv_obj_set_size(s_arc_combs, W, MAIN_H);
    lv_obj_set_style_bg_opa(s_arc_combs, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_arc_combs, 0, 0);
    lv_obj_set_style_pad_all(s_arc_combs, 0, 0);
    lv_obj_add_flag(s_arc_combs, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_clear_flag(s_arc_combs, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_arc_combs, LV_OBJ_FLAG_SCROLLABLE);

    // SOC comb — radial ticks along the "(" arc. Unlit ticks form the track;
    // update lights ticks bottom->top; a longer white marker rides the level.
    s_soc_cx = soc_cx; s_soc_cy = soc_cy;
    make_comb(s_arc_combs, s_soc_tick, SOC_TICKS,
              soc_cx, soc_cy, ARC_R, ARC_W, 3, ARC_START, ARC_END);
    s_soc_mark = make_marker(s_arc_combs);

    // SOC % label — centered inside the bottom of the arc stroke
    // RANGE label — to the left of the arc at the same height
    {
        float a = ARC_START * (float)M_PI / 180.0f;
        lv_coord_t bx = soc_cx + (lv_coord_t)(ARC_R * cosf(a));
        lv_coord_t by = soc_cy + (lv_coord_t)(ARC_R * sinf(a));
        s_lbl_soc_pct = make_label(scr, "0%", CLR_CYAN, &lv_font_montserrat_24);
        lv_obj_set_pos(s_lbl_soc_pct, bx - 32, by - 116);
    }

    // ── Center ────────────────────────────────────────────────────────────
    const lv_coord_t CTR_X  = LEFT_W + 30;
    const lv_coord_t CTR_W  = W - RIGHT_W - 30 - CTR_X;
    lv_obj_t *ctr = lv_obj_create(scr);
    lv_obj_set_pos(ctr, CTR_X, 0);
    lv_obj_set_size(ctr, CTR_W, MAIN_H);
    lv_obj_set_style_bg_opa(ctr, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctr, 0, 0);
    lv_obj_set_style_radius(ctr, 0, 0);
    lv_obj_clear_flag(ctr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ctr, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    s_lbl_speed = make_label(ctr, "0", CLR_WHITE, &lv_font_montserrat_110);
    lv_obj_align(s_lbl_speed, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t *unit = make_label(ctr, UNITS_SPEED_LABEL, CLR_TEXT_MID,
                                 &lv_font_montserrat_18);
    lv_obj_align(unit, LV_ALIGN_CENTER, 0, 8);

    // PRND labels moved to right panel — see below

    // ── Odometer / Trip ───────────────────────────────────────────────────
    {
        lv_obj_t *h = make_label(ctr, "ODO", CLR_CYAN, &lv_font_montserrat_18);
        lv_obj_align(h, LV_ALIGN_CENTER, -60, 70);
    }
    s_lbl_odo_val = make_label(ctr, "0.0 " UNITS_DIST_LABEL, CLR_WHITE, &lv_font_montserrat_18);
    lv_obj_align(s_lbl_odo_val, LV_ALIGN_CENTER, 30, 70);

    {
        lv_obj_t *h = make_label(ctr, "TRIP", CLR_CYAN, &lv_font_montserrat_18);
        lv_obj_align(h, LV_ALIGN_CENTER, -60, 98);
    }
    s_lbl_trip_odo_val = make_label(ctr, "0.0 " UNITS_DIST_LABEL, CLR_WHITE, &lv_font_montserrat_18);
    lv_obj_align(s_lbl_trip_odo_val, LV_ALIGN_CENTER, 30, 98);

    {
        lv_obj_t *h = make_label(ctr, "RANGE", CLR_CYAN, &lv_font_montserrat_18);
        lv_obj_align(h, LV_ALIGN_CENTER, -60, 126);
    }
    s_lbl_range = make_label(ctr, "-- " UNITS_DIST_LABEL, CLR_WHITE, &lv_font_montserrat_18);
    lv_obj_align(s_lbl_range, LV_ALIGN_CENTER, 30, 126);

    // Cruise indicator — sits above PRND row, hidden until active
    s_lbl_cruise_st = make_label(ctr, "", CLR_TEXT_DIM,
                                  &lv_font_montserrat_14);
    lv_obj_align(s_lbl_cruise_st, LV_ALIGN_BOTTOM_MID, -40, -70);

    s_lbl_cruise = make_label(ctr, "", CLR_CYAN,
                               &lv_font_montserrat_24);
    lv_obj_align(s_lbl_cruise, LV_ALIGN_BOTTOM_MID, 20, -66);

    // ── Power arc — ")" shape ─────────────────────────────────────────────
    // Mirror of SOC arc. Center to the LEFT of right panel edge.
    // Sweep from 300° to 60° (CW) = right-facing 120° arc.
    // PWR_ARC_START, PWR_ARC_END from dashboard_layout.h

    lv_coord_t pwr_cx = W - RIGHT_W - ARC_INSET;
    lv_coord_t pwr_cy = MAIN_H / 2;

    // Power comb — radial ticks along the ")" arc. a0..a0+160 crosses 0° (=360°)
    // at the midpoint, so tick PWR_TICKS/2 is the center/zero: drive lights
    // upward (orange), regen downward (green); a longer white marker rides it.
    s_pwr_cx = pwr_cx; s_pwr_cy = pwr_cy;
    make_comb(s_arc_combs, s_pwr_tick, PWR_TICKS,
              pwr_cx, pwr_cy, ARC_R, ARC_W, 3,
              PWR_ARC_START, PWR_ARC_START + 160);
    s_pwr_mark = make_marker(s_arc_combs);

    // Power labels — at the bottom endpoint of the arc (PWR_ARC_END angle)
    {
        float a = PWR_ARC_END * (float)M_PI / 180.0f;
        lv_coord_t bx = pwr_cx + (lv_coord_t)(ARC_R * cosf(a));
        lv_coord_t by = pwr_cy + (lv_coord_t)(ARC_R * sinf(a));
        s_lbl_pwr_val = make_label(scr, "+0", CLR_CYAN, &lv_font_montserrat_24);
        lv_obj_set_pos(s_lbl_pwr_val, bx - 32, by - 116);
        lv_obj_t *kw_hdr = make_label(scr, "kW", CLR_TEXT_DIM,
                                       &lv_font_montserrat_10);
        lv_obj_set_pos(kw_hdr, bx - 5, by - 130);
    }

    // ── Right panel ───────────────────────────────────────────────────────
    lv_obj_t *right = lv_obj_create(scr);
    lv_obj_set_pos(right, W - RIGHT_W, 0);
    lv_obj_set_size(right, RIGHT_W, MAIN_H);
    lv_obj_set_style_bg_opa(right, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(right, 0, 0);
    lv_obj_set_style_radius(right, 0, 0);
    lv_obj_set_style_pad_left(right, 14, 0);
    lv_obj_set_style_pad_top(right, 14, 0);
    lv_obj_clear_flag(right, LV_OBJ_FLAG_SCROLLABLE);

    struct { lv_obj_t **lbl; const char *tag; const char *init; } rows[] = {
        {&s_lbl_eff,  "EFFICIENCY\n" UNITS_EFF_LABEL, "--"},
        {&s_lbl_trip, "TRIP kWh",   "--"},
        {NULL,        "EST RANGE",  "--"},
        {&s_lbl_aux_v,"12V AUX",    "0.0 V"},
    };
    // We reuse s_lbl_range for range — just add 12V aux here
    // Actually range is shown below SOC bar — add separate range label here too
    static lv_obj_t *s_lbl_range2 = NULL;  // second instance for right panel

    lv_obj_t *r_eff_tag = make_label(right, "EFFICIENCY", CLR_CYAN,
                                      &lv_font_montserrat_10);
    lv_obj_align(r_eff_tag, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_t *r_eff_unit = make_label(right, UNITS_EFF_LABEL, CLR_CYAN,
                                       &lv_font_montserrat_10);
    lv_obj_align(r_eff_unit, LV_ALIGN_TOP_LEFT, 0, 14);
    s_lbl_eff = make_label(right, "--", CLR_TEXT_BRIGHT,
                            &lv_font_montserrat_48);
    lv_obj_align(s_lbl_eff, LV_ALIGN_TOP_LEFT, 0, 30);

    lv_obj_t *d1 = lv_obj_create(right);
    lv_obj_set_size(d1, RIGHT_W - 14, 1);
    lv_obj_set_style_bg_color(d1, CLR_BORDER, 0);
    lv_obj_set_style_bg_opa(d1, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(d1, 0, 0);
    lv_obj_align(d1, LV_ALIGN_TOP_LEFT, 0, 100);

    make_label(right, "TRIP kWh", CLR_CYAN, &lv_font_montserrat_10);
    lv_obj_t *trip_tag = lv_obj_get_child(right, lv_obj_get_child_cnt(right)-1);
    lv_obj_align(trip_tag, LV_ALIGN_TOP_LEFT, 0, 112);
    s_lbl_trip = make_label(right, "--", CLR_TEXT_BRIGHT,
                             &lv_font_montserrat_48);
    lv_obj_align(s_lbl_trip, LV_ALIGN_TOP_LEFT, 0, 126);

    lv_obj_t *d2 = lv_obj_create(right);
    lv_obj_set_size(d2, RIGHT_W - 14, 1);
    lv_obj_set_style_bg_color(d2, CLR_BORDER, 0);
    lv_obj_set_style_bg_opa(d2, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(d2, 0, 0);
    lv_obj_align(d2, LV_ALIGN_TOP_LEFT, 0, 200);

    make_label(right, "12V AUX", CLR_CYAN, &lv_font_montserrat_10);
    lv_obj_t *aux_tag = lv_obj_get_child(right, lv_obj_get_child_cnt(right)-1);
    lv_obj_align(aux_tag, LV_ALIGN_TOP_LEFT, 0, 212);
    s_lbl_aux_v = make_label(right, "0.0 V", CLR_TEXT_BRIGHT,
                              &lv_font_montserrat_48);
    lv_obj_align(s_lbl_aux_v, LV_ALIGN_TOP_LEFT, 0, 226);

    // ── CAN0 + CAN1 + WiFi + rw-root status (bottom of right panel) ──────────
    {
        lv_coord_t panel_cx = W - RIGHT_W / 2;   // horizontal center of right panel
        lv_coord_t row_y    = MAIN_H - 42;       // near bottom of the panel
        s_dot_can = make_label(scr, LV_SYMBOL_LOOP, CLR_RED, &lv_font_montserrat_14);
        lv_obj_set_pos(s_dot_can, panel_cx - 66, row_y);
        s_dot_can1 = make_label(scr, LV_SYMBOL_LOOP, CLR_RED, &lv_font_montserrat_14);
        lv_obj_set_pos(s_dot_can1, panel_cx - 28, row_y);
        s_dot_wifi = make_label(scr, LV_SYMBOL_WIFI, CLR_RED, &lv_font_montserrat_14);
        lv_obj_set_pos(s_dot_wifi, panel_cx + 10, row_y);
        // rw-root warning (⚠) — hidden unless the eMMC (/mnt/ro) is mounted rw
        s_dot_rw = make_label(scr, LV_SYMBOL_WARNING, CLR_AMBER, &lv_font_montserrat_14);
        lv_obj_set_pos(s_dot_rw, panel_cx + 48, row_y);
        lv_obj_add_flag(s_dot_rw, LV_OBJ_FLAG_HIDDEN);
    }

    // ── PRND in right panel (above status indicators) ────────────────────
    {
        lv_coord_t panel_cx  = W - RIGHT_W / 2;
        lv_coord_t prnd_y    = MAIN_H - 110;   // above indicators at MAIN_H-42
        const char *gnames[] = {"P","R","N","D"};
        // Wider spacing (60px vs 40) + shift the whole cluster left so "D"
        // isn't jammed against the right edge and each letter is an easy,
        // non-overlapping tap target.
        int x_off[]                 = {-90, -30, 30, 90};
        const lv_coord_t prnd_shift = 40;
        for (int i = 0; i < 4; i++) {
            s_lbl_prnd[i] = make_label(scr, gnames[i], CLR_TEXT_DIM,
                                        &lv_font_montserrat_40);
            // Constant padding + (transparent) panel background: the "acked"
            // box is toggled on in the update by just flipping bg opacity, so
            // the glyph never shifts. Position compensates for the 8px pad.
            lv_obj_set_style_pad_all(s_lbl_prnd[i], 8, 0);
            lv_obj_set_style_radius(s_lbl_prnd[i], 8, 0);
            lv_obj_set_style_bg_color(s_lbl_prnd[i], CLR_PANEL, 0);
            lv_obj_set_style_bg_opa(s_lbl_prnd[i], LV_OPA_TRANSP, 0);
            lv_obj_set_pos(s_lbl_prnd[i],
                           panel_cx + x_off[i] - prnd_shift - 22, prnd_y - 8);
            // Read-only: the M5Dial is the shifter. PRND just reflects
            // selected gear (cyan, from 0x312) + confirmed dir (box, from VCU).
        }
        lv_obj_set_style_text_color(s_lbl_prnd[0], CLR_CYAN, 0);

        // Range + motor-config badges (display only) above the PRND row.
        lv_coord_t cluster_cx = panel_cx - prnd_shift;
        lv_coord_t badge_y    = prnd_y - 46;
        lv_obj_t  *rcap = make_label(scr, "RANGE", CLR_TEXT_MID, &lv_font_montserrat_10);
        lv_obj_set_pos(rcap, cluster_cx - 84, badge_y - 13);
        lv_obj_t  *mcap = make_label(scr, "MOTOR", CLR_TEXT_MID, &lv_font_montserrat_10);
        lv_obj_set_pos(mcap, cluster_cx + 14, badge_y - 13);

        s_lbl_range_badge = make_label(scr, "--", CLR_TEXT_BRIGHT, &lv_font_montserrat_18);
        lv_obj_set_style_bg_color(s_lbl_range_badge, CLR_PANEL, 0);
        lv_obj_set_style_bg_opa(s_lbl_range_badge, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(s_lbl_range_badge, 6, 0);
        lv_obj_set_style_pad_all(s_lbl_range_badge, 4, 0);
        lv_obj_set_pos(s_lbl_range_badge, cluster_cx - 84, badge_y);

        s_lbl_motor_badge = make_label(scr, "--", CLR_TEXT_BRIGHT, &lv_font_montserrat_18);
        lv_obj_set_style_bg_color(s_lbl_motor_badge, CLR_PANEL, 0);
        lv_obj_set_style_bg_opa(s_lbl_motor_badge, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(s_lbl_motor_badge, 6, 0);
        lv_obj_set_style_pad_all(s_lbl_motor_badge, 4, 0);
        lv_obj_set_pos(s_lbl_motor_badge, cluster_cx + 14, badge_y);
    }

    // (bottom tab bar removed — navigation is the 🏠 ◀ ▶ cluster added below;
    //  MAIN_H now spans full height so the combs aren't cut off.)
    prv_add_arrow_nav(scr);
    prv_add_regen_button(scr);
}
// Forward declarations for screen helpers
static void prv_ensure_settings_screen(void);
static void prv_ensure_vcu_screen(void);
static void prv_update_vcu(const DashData *d);
static void prv_add_arrow_nav(lv_obj_t *screen);
#ifdef BMS_HTTP
static lv_obj_t *s_scr_bms = NULL;
static void prv_ensure_bms_screen(void);
static void prv_update_bms(void);
#endif

// ── dashboard_ui_update ───────────────────────────────────────────────────
void dashboard_ui_update(const DashData *d)
{
    // ── Pending screen switch (set from httpd task via dashboard_ui_set_screen) ─
    if (s_screen_change_req) {
        s_screen_change_req = false;
        switch (s_pending_screen) {
            case DASH_SCREEN_HOME:
                if (s_scr_home) lv_screen_load(s_scr_home);
                break;
            case DASH_SCREEN_SETTINGS:
                prv_ensure_settings_screen();
                lv_screen_load(s_scr_settings);
                break;
            case DASH_SCREEN_VCU:
                prv_ensure_vcu_screen();
                lv_screen_load(s_scr_vcu);
                break;
#ifdef BMS_HTTP
            case DASH_SCREEN_BMS:
                prv_ensure_bms_screen();
                lv_screen_load(s_scr_bms);
                break;
#endif
            default: break;
        }
        s_cur_screen = s_pending_screen;
    }

    char buf[32];

    // ── CAN staleness dim ─────────────────────────────────────────────────
    // g_dash values FREEZE when frames stop (held, not zeroed — see the CAN read
    // loop in main_linux.cpp), so on a stale/dropped feed the numbers already hold;
    // we just dim the primary readouts by opacity to signal "held, not live". The
    // CAN dot (green<2s / amber<10s / red) conveys the severity; this is the cluster.
    {
        uint32_t sn_ms    = (uint32_t)(esp_timer_get_time() / 1000);
        bool     can_stale = (d->last_ms_any != 0) && ((sn_ms - d->last_ms_any) > 3000);
        static int stale_prev = -1;
        if ((int)can_stale != stale_prev) {
            lv_opa_t o = can_stale ? LV_OPA_40 : LV_OPA_COVER;
            if (s_lbl_speed)   lv_obj_set_style_text_opa(s_lbl_speed,   o, 0);
            if (s_lbl_soc_pct) lv_obj_set_style_text_opa(s_lbl_soc_pct, o, 0);
            if (s_lbl_pwr_val) lv_obj_set_style_text_opa(s_lbl_pwr_val, o, 0);
            if (s_arc_combs)   lv_obj_set_style_opa      (s_arc_combs, o, 0);
            stale_prev = (int)can_stale;
        }
    }

    // ── Meter gauge update helper ─────────────────────────────────────────
    // Updates needle position and value label; only calls set_style on color change
    // `stale` greys the readout: on GS450H the inverter-sourced motor/heatsink
    // temps are only meaningful while running, so dim them when opmode==Off.
    auto uptemp = [](Meter *m, float v, float vmin, float vmax,
                     float warn, float crit, lv_color_t *last_col, bool stale) {
        if (stale) {
            lv_color_t nc = CLR_TEXT_DIM;
            if (memcmp(&nc, last_col, sizeof nc) != 0) {
                update_needle(m, 0.0f);
                lv_label_set_text(m->lbl_val, "--");
                lv_obj_set_style_text_color(m->lbl_val, nc, 0);
                *last_col = nc;
            }
            return;
        }
        float pct = (v - vmin) / (vmax - vmin);
        update_needle(m, pct);
        char b[12];
        snprintf(b, sizeof(b), "%.0f C", v);
        lv_label_set_text(m->lbl_val, b);
        lv_color_t nc = v >= crit ? CLR_RED : v >= warn ? CLR_AMBER : CLR_GREEN;
        if (memcmp(&nc, last_col, sizeof(lv_color_t)) != 0) {
            lv_obj_set_style_text_color(m->lbl_val, nc, 0);
            *last_col = nc;
        }
    };

    static lv_color_t lc_inv = {0}, lc_mot = {0}, lc_bat = {0};
    static lv_color_t lc_pv  = {0}, lc_pa  = {0};

    // Temp meters: scale 0-150°C. Motor/inverter grey out when the VCU is Off.
    bool temp_stale = (d->vcu_opmode == 0);
    uptemp(&s_inv_meter, d->inverter_temp_c, 0, 150,
           WARN_INV_TEMP_C, CRIT_INV_TEMP_C, &lc_inv, temp_stale);
    uptemp(&s_mot_meter, d->motor_temp_c,    0, 150,
           WARN_MOTOR_TEMP_C, CRIT_MOTOR_TEMP_C, &lc_mot, temp_stale);
    uptemp(&s_bat_meter, d->batt_temp_c,     0,  60,
           WARN_BATT_TEMP_C, CRIT_BATT_TEMP_C, &lc_bat, false);

    // Pack voltage meter: scale 300-420V
    {
        float pct = (d->pack_volts - 290.0f) / 160.0f;
        update_needle(&s_pv_meter, pct);
        snprintf(buf, sizeof(buf), "%.0f V", d->pack_volts);
        lv_label_set_text(s_pv_meter.lbl_val, buf);
        lv_color_t nc = d->pack_volts < 300.0f ? CLR_RED   :
                        d->pack_volts < 341.0f ? CLR_ORANGE :
                        d->pack_volts <= 430.0f ? CLR_GREEN : CLR_RED;
        if (memcmp(&nc, &lc_pv, sizeof(lv_color_t)) != 0) {
            lv_obj_set_style_text_color(s_pv_meter.lbl_val, nc, 0);
            lc_pv = nc;
        }
    }

    // Pack amps meter: scale -500 to +500A, zero at center
    {
        float pct = (d->pack_amps + 500.0f) / 1000.0f;
        update_needle(&s_pa_meter, pct);
        snprintf(buf, sizeof(buf), "%.0f A", d->pack_amps);
        lv_label_set_text(s_pa_meter.lbl_val, buf);
        lv_color_t nc = d->pack_amps < 0 ? CLR_GREEN : CLR_ORANGE;
        if (memcmp(&nc, &lc_pa, sizeof(lv_color_t)) != 0) {
            lv_obj_set_style_text_color(s_pa_meter.lbl_val, nc, 0);
            lc_pa = nc;
        }
    }

    // ── SOC arc ───────────────────────────────────────────────────────────
    // Arc sweeps from ARC_START(120°) to ARC_END(240°) = 120° total.
    // Fill from start(120°) upward: 0%=empty, 100%=full 120° sweep.
    static lv_color_t last_soc_col = {0};
    static int        last_soc_lit = -1;
    float soc_f = d->soc_pct / 100.0f;
    if (soc_f < 0.0f) soc_f = 0.0f; if (soc_f > 1.0f) soc_f = 1.0f;
    lv_color_t soc_col = soc_f >= 0.50f ? CLR_GREEN :
                         soc_f >= 0.21f ? CLR_AMBER : CLR_RED;
    int soc_lit = (int)lroundf(soc_f * SOC_TICKS);
    if (soc_lit != last_soc_lit ||
        memcmp(&soc_col, &last_soc_col, sizeof(lv_color_t)) != 0) {
        for (int i = 0; i < SOC_TICKS; i++)
            lv_obj_set_style_bg_color(s_soc_tick[i],
                                      (i < soc_lit) ? soc_col : CLR_BORDER, 0);
        if (soc_lit > 0) {   // white marker rides the top of the fill
            place_radial(s_soc_mark, s_soc_cx, s_soc_cy, ARC_R,
                         ARC_W + 2 * MARK_OVER, MARK_W,
                         comb_angle(soc_lit - 1, SOC_TICKS, ARC_START, ARC_END));
            lv_obj_clear_flag(s_soc_mark, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_soc_mark, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_set_style_text_color(s_lbl_soc_pct, soc_col, 0);
        last_soc_lit = soc_lit;
        last_soc_col = soc_col;
    }
    snprintf(buf, sizeof(buf), "%d%%", (int)d->soc_pct);
    lv_label_set_text(s_lbl_soc_pct, buf);
    snprintf(buf, sizeof(buf), "%.0f %s",
             DIST_TO_DISPLAY(d->range_dist), UNITS_DIST_LABEL);
    lv_label_set_text(s_lbl_range, buf);

    // ── Speed ─────────────────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%d", (int)SPEED_TO_DISPLAY(d->speed));
    lv_label_set_text(s_lbl_speed, buf);

    // ── Gear ──────────────────────────────────────────────────────────────
    // Optimistic gear ack, matching the M5Dial JLR shifter
    // (m5dial-jlr1-shifter): the Zombie has no dedicated gear-ack broadcast,
    // so a selection is treated as acked GEAR_ACK_MS after it changes. Cyan =
    // selected (immediate); Home-button-style box = acked (after the delay).
    const uint32_t GEAR_ACK_MS = 200;
    uint32_t now_ms = lv_tick_get();
    static int      last_gear  = -1;
    static int      last_boxed = -2;
    static int      gear_prev  = -1;
    static uint32_t gear_ms    = 0;
    int g = (d->gear >= 0 && d->gear < 4) ? d->gear : 0;
    if (g != gear_prev) { gear_prev = g; gear_ms = now_ms; }
    // Prefer the Zombie's confirmed direction (dir, DIRS: -1=R 0=N 1=D 2=P) for
    // a REAL ack; fall back to the M5Dial-style optimistic timer if 0x510 isn't
    // being received.
    int conf;
    switch (d->vcu_dir) {
        case  2: conf = 0; break;   // Park    -> P
        case -1: conf = 1; break;   // Reverse -> R
        case  0: conf = 2; break;   // Neutral -> N
        case  1: conf = 3; break;   // Drive   -> D
        default: conf = -1; break;  // unknown
    }
    bool dir_known = (d->vcu_dir >= -1 && d->vcu_dir <= 2);
    int boxed = d->vcu_park ? 0                 // P: pawl engaged (lever contact)
              : dir_known   ? conf              // R/N/D from Zombie dir
              : (((uint32_t)(now_ms - gear_ms) >= GEAR_ACK_MS) ? g : -1);
    if (g != last_gear || boxed != last_boxed) {
        for (int i = 0; i < 4; i++) {
            lv_obj_set_style_text_color(s_lbl_prnd[i],
                i == g ? CLR_CYAN : CLR_TEXT_DIM, 0);
            lv_obj_set_style_bg_opa(s_lbl_prnd[i],
                i == boxed ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        }
        last_gear  = g;
        last_boxed = boxed;
    }

    // ── Range + motor-config badges (M5Dial aux pages, display only) ───────
    static const char *hl_names[4] = { "LOW", "HIGH", "AUTO", "HI-LO" };
    static const char *mg_names[4] = { "MG1+2", "MG1", "MG2", "BLEND" };
    static int last_hl = -2, last_mg = -2;
    if (s_lbl_range_badge && d->hl_mode != last_hl) {
        lv_label_set_text(s_lbl_range_badge,
            (d->hl_mode >= 0 && d->hl_mode < 4) ? hl_names[d->hl_mode] : "--");
        last_hl = d->hl_mode;
    }
    if (s_lbl_motor_badge && d->mg_mode != last_mg) {
        lv_label_set_text(s_lbl_motor_badge,
            (d->mg_mode >= 0 && d->mg_mode < 4) ? mg_names[d->mg_mode] : "--");
        last_mg = d->mg_mode;
    }

    // ── Power arc ─────────────────────────────────────────────────────────
    // Arc sweeps PWR_ARC_START(300°) to PWR_ARC_END(60°) = 120° total.
    // Zero at arc midpoint (300+60°/2 = 330°... wraps = center at 0°/360°).
    // Drive (+kW): fills upper half from center toward PWR_ARC_END(60°).
    // Regen (-kW): fills lower half from center toward PWR_ARC_START(300°).
    static lv_color_t last_pwr_col    = {0};
    static int        last_pwr_signed = -99999;
    float kw      = d->power_kw;
    float kw_frac = fabsf(kw) / PWR_FULL;
    if (kw_frac > 1.0f) kw_frac = 1.0f;
    lv_color_t pwr_col = kw >= 0.0f ? CLR_ORANGE : CLR_GREEN;

    // Comb center (index PWR_HALF) = zero. Drive lights upward (toward top),
    // regen downward (toward bottom); one white tick marks the current level.
    const int PWR_HALF = PWR_TICKS / 2;
    int pwr_k      = (int)lroundf(kw_frac * PWR_HALF);   // lit ticks this side
    int pwr_signed = (kw >= 0.0f) ? pwr_k : -pwr_k;      // change-detect key
    if (pwr_signed != last_pwr_signed ||
        memcmp(&pwr_col, &last_pwr_col, sizeof(lv_color_t)) != 0) {
        for (int i = 0; i < PWR_TICKS; i++)
            lv_obj_set_style_bg_color(s_pwr_tick[i], CLR_BORDER, 0);
        int mk;
        if (kw >= 0.0f) {                                // drive: up from center
            for (int i = 0; i < pwr_k; i++)
                lv_obj_set_style_bg_color(s_pwr_tick[PWR_HALF - 1 - i], pwr_col, 0);
            mk = pwr_k > 0 ? PWR_HALF - pwr_k : PWR_HALF - 1;
        } else {                                         // regen: down from center
            for (int i = 0; i < pwr_k; i++)
                lv_obj_set_style_bg_color(s_pwr_tick[PWR_HALF + i], pwr_col, 0);
            mk = pwr_k > 0 ? PWR_HALF + pwr_k - 1 : PWR_HALF;
        }
        // white marker rides the current level (sits at center when idle)
        place_radial(s_pwr_mark, s_pwr_cx, s_pwr_cy, ARC_R,
                     ARC_W + 2 * MARK_OVER, MARK_W,
                     comb_angle(mk, PWR_TICKS, PWR_ARC_START, PWR_ARC_START + 160));
        lv_obj_clear_flag(s_pwr_mark, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_text_color(s_lbl_pwr_val, pwr_col, 0);
        last_pwr_signed = pwr_signed;
        last_pwr_col    = pwr_col;
    }
    snprintf(buf, sizeof(buf), "%+d", (int)kw);
    lv_label_set_text(s_lbl_pwr_val, buf);

    // ── Right panel ───────────────────────────────────────────────────────
    if (d->speed > 2.0f && d->power_kw > 5.0f)
        snprintf(buf, sizeof(buf), "%.1f", EFF_TO_DISPLAY(d->speed / d->power_kw));
    else
        snprintf(buf, sizeof(buf), "--");
    lv_label_set_text(s_lbl_eff, buf);

    lv_label_set_text(s_lbl_trip, "--");  // TODO: trip kWh accumulator

    // ── Odometer ──────────────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%.1f %s",
             DIST_TO_DISPLAY(d->odo_total_miles), UNITS_DIST_LABEL);
    lv_label_set_text(s_lbl_odo_val, buf);
    snprintf(buf, sizeof(buf), "%.1f %s",
             DIST_TO_DISPLAY(d->trip_miles), UNITS_DIST_LABEL);
    lv_label_set_text(s_lbl_trip_odo_val, buf);

    snprintf(buf, sizeof(buf), "%.1f V", d->aux_volts);
    lv_label_set_text(s_lbl_aux_v, buf);

    // ── CAN + WiFi status indicators ──────────────────────────────────────
    {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        // Per-bus CAN liveness (each icon keyed on its own interface's last frame):
        //   <2s fresh (green), 2-10s stale/holding (amber), >10s or never dead (red)
        auto bus_state = [now_ms](uint32_t last) -> int {
            if (last == 0) return 2;
            uint32_t age = now_ms - last;
            return age > 10000 ? 2 : (age > 2000 ? 1 : 0);
        };
        auto st_color = [](int s) -> lv_color_t {
            return s == 0 ? CLR_GREEN : (s == 1 ? CLR_AMBER : CLR_RED);
        };
        int  can0_state = bus_state(d->last_ms_can0);
        int  can1_state = bus_state(d->last_ms_can1);
        bool wifi_ok = wifi_manager_is_connected();
        static int  last_can0    = -1;
        static int  last_can1    = -1;
        static bool last_wifi_ok = false;
        if (can0_state != last_can0) {
            lv_obj_set_style_text_color(s_dot_can,  st_color(can0_state), 0);
            last_can0 = can0_state;
        }
        if (can1_state != last_can1) {
            lv_obj_set_style_text_color(s_dot_can1, st_color(can1_state), 0);
            last_can1 = can1_state;
        }
        if (wifi_ok != last_wifi_ok) {
            lv_obj_set_style_text_color(s_dot_wifi, wifi_ok ? CLR_GREEN : CLR_RED, 0);
            last_wifi_ok = wifi_ok;
        }
#ifdef __linux__
        // read-only-root health: warn (⚠) if the eMMC is unlocked. Root "/" is a
        // tmpfs overlay (always rw); the real disk is the ro lower at /mnt/ro, so
        // check THAT — rw there = someone left it unlocked after an upload.
        {
            static int32_t  last_rw    = -1;
            static uint32_t rw_next_ms = 0;
            uint32_t t = lv_tick_get();
            if (s_dot_rw && (int32_t)(t - rw_next_ms) >= 0) {
                rw_next_ms = t + 2000;   // poll every 2s
                struct statvfs vfs;
                int rw = (statvfs("/mnt/ro", &vfs) == 0) ? !(vfs.f_flag & ST_RDONLY) : 0;
                if (rw != last_rw) {
                    if (rw) lv_obj_clear_flag(s_dot_rw, LV_OBJ_FLAG_HIDDEN);
                    else    lv_obj_add_flag  (s_dot_rw, LV_OBJ_FLAG_HIDDEN);
                    last_rw = rw;
                }
            }
        }
#endif
    }

    // ── Cruise control ────────────────────────────────────────────────────
    {
        uint8_t cs = d->cruise_state;
        if (cs == CRUISE_CC_NONE || cs == CRUISE_CC_CANCEL) {
            lv_label_set_text(s_lbl_cruise_st, "");
            lv_label_set_text(s_lbl_cruise, "");
        } else if (cs & CRUISE_CC_SET) {
            lv_label_set_text(s_lbl_cruise_st, "SET");
            lv_obj_set_style_text_color(s_lbl_cruise_st, CLR_CYAN, 0);
            snprintf(buf, sizeof(buf), "%.0f", SPEED_TO_DISPLAY(d->cruise_kph));
            lv_label_set_text(s_lbl_cruise, buf);
        } else if (cs & CRUISE_CC_RESUME) {
            lv_label_set_text(s_lbl_cruise_st, "RES");
            lv_obj_set_style_text_color(s_lbl_cruise_st, CLR_AMBER, 0);
            snprintf(buf, sizeof(buf), "%.0f", SPEED_TO_DISPLAY(d->cruise_kph));
            lv_label_set_text(s_lbl_cruise, buf);
        } else if (cs & CRUISE_CC_ON) {
            lv_label_set_text(s_lbl_cruise_st, "CC");
            lv_obj_set_style_text_color(s_lbl_cruise_st, CLR_TEXT_MID, 0);
            lv_label_set_text(s_lbl_cruise, "");
        }
    }

    // ── Status screen live values (cheap; no-ops until that screen is built) ─
    prv_update_vcu(d);
#ifdef BMS_HTTP
    prv_update_bms();   // no-op until BMS screen built
#endif

    // ── Invalidate ────────────────────────────────────────────────────────
    lv_obj_invalidate(lv_screen_active());
}

// ── Screen switch implementation ──────────────────────────────
// Give a sub-screen a way back HOME: the whole screen is tap-to-return
// (robust against z-order/hit-test quirks), plus a visible ◀ HOME button.
// Prev/next screen cycling for the far-right arrow buttons.
static void prv_nav_arrow_cb(lv_event_t *e)
{
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    if (dir == 0) { dashboard_ui_set_screen(DASH_SCREEN_HOME); return; }  // 🏠
#ifdef BMS_HTTP
    static const dash_screen_t order[] = { DASH_SCREEN_HOME, DASH_SCREEN_VCU,
                                           DASH_SCREEN_BMS, DASH_SCREEN_SETTINGS };
#else
    static const dash_screen_t order[] = { DASH_SCREEN_HOME, DASH_SCREEN_VCU,
                                           DASH_SCREEN_SETTINGS };
#endif
    const int n = (int)(sizeof(order) / sizeof(order[0]));
    dash_screen_t cur = dashboard_ui_get_screen();
    int idx = 0;
    for (int i = 0; i < n; i++) if (order[i] == cur) { idx = i; break; }
    idx = (idx + dir + n) % n;
    dashboard_ui_set_screen(order[idx]);
}

// Two thumb-friendly ◀ ▶ buttons at far-right-middle to cycle screens.
static void prv_add_arrow_nav(lv_obj_t *screen)
{
    // Far-right-middle cluster, one reachable row: 🏠 ◀ ▶ (filled home at the
    // left, then the screen-cycle arrows) — no more reaching for the top-left.
    // 64px targets with 16px gaps so fat fingers don't fat-finger a neighbour.
    const struct { const char *sym; int dir; int xoff; int yoff; int home; } b[3] = {
        { LV_SYMBOL_HOME,   0, -168, 0, 1 },
        { LV_SYMBOL_LEFT,  -1,  -88, 0, 0 },
        { LV_SYMBOL_RIGHT, +1,   -8, 0, 0 },
    };
    for (int i = 0; i < 3; i++) {
        lv_obj_t *btn = lv_obj_create(screen);
        lv_obj_set_size(btn, 64, 64);
        lv_obj_align(btn, LV_ALIGN_RIGHT_MID, b[i].xoff, b[i].yoff);
        lv_obj_set_style_bg_color(btn, b[i].home ? CLR_CYAN : CLR_PANEL, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(btn, CLR_CYAN, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_radius(btn, 12, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_ext_click_area(btn, 8);   // 8+8 = 16 = the gap, no overlap
        lv_obj_add_event_cb(btn, prv_nav_arrow_cb, LV_EVENT_CLICKED,
                            (void *)(intptr_t)b[i].dir);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, b[i].sym);
        lv_obj_set_style_text_color(lbl, b[i].home ? CLR_BG : CLR_CYAN, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
        lv_obj_center(lbl);
    }
}

static void prv_add_home_button(lv_obj_t *screen)
{
    // No visible top-left button (unreachable when steering-wheel-mounted — the
    // 🏠 in the right-middle nav cluster replaces it). Keep tap-anywhere-home as
    // a bonus reachability fallback on the (non-interactive) sub-screens.
    lv_obj_add_flag(screen, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(screen, prv_nav_tap_cb, LV_EVENT_CLICKED, (void *)(intptr_t)DASH_SCREEN_HOME);
}

static void prv_ensure_settings_screen(void)
{
    if (s_scr_settings) return;
    s_scr_settings = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_settings, CLR_BG, 0);
    lv_obj_set_style_bg_opa(s_scr_settings, LV_OPA_COVER, 0);

    lv_obj_t *lbl = lv_label_create(s_scr_settings);
    lv_label_set_text(lbl, "Settings\nUse browser: http://ev-dashboard.local/settings");
    lv_obj_set_style_text_color(lbl, CLR_TEXT_MID, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    prv_add_home_button(s_scr_settings);
    prv_add_arrow_nav(s_scr_settings);
}

// ── Status screen: clean live-values grid, fed from g_dash (CAN) ─────────
enum { ST_SOC, ST_SPEED, ST_POWER, ST_PACKV, ST_PACKA, ST_RANGE,
       ST_MOTOR, ST_INV, ST_BATT, ST_SHUNT, ST_AUX, ST_GEAR, ST_ODO,
       ST_TRIP, ST_CANLOAD, ST_N };
static const char *st_name[ST_N] = {
    "SOC", "SPEED", "POWER", "PACK V", "PACK A", "RANGE",
    "MOTOR", "INVERTER", "BATTERY", "SHUNT", "AUX 12V", "GEAR", "ODO",
    "TRIP", "CAN LOAD" };
static lv_obj_t *s_st_val[ST_N];

// ── Regen limit slider (guarded live-tune of ZombieVerter regenmax #61) ──────
// Regen is the truck's PRIMARY decel right now (no service brakes yet), so this
// widget is deliberately guarded: a FLOOR so you can't weaken braking past
// -10%, and it applies ONLY on finger-release (not during drag). Park-test only.
#define REGEN_PARAM_ID   61        // ZombieVerter "regenmax"
#define REGEN_MAG_MIN    10        // % floor: always at least -10% regen
#define REGEN_MAG_MAX    35        // % ceiling of regen magnitude
#define REGEN_MAG_DEFAULT 20       // slider start position (not applied until release)
static lv_obj_t *s_regen_slider = NULL;
static lv_obj_t *s_regen_lbl    = NULL;
static int  s_regen_current_mag = REGEN_MAG_DEFAULT;  // magnitude to show; from VCU once read
static bool s_regen_have_current = false;             // true once the VCU has reported a value

// Regen ON/OFF button (home screen, snow/ice). OFF writes regenmax = 0; ON
// restores the last non-zero value the VCU reported. Long-press only, so a
// stray tap while driving can't drop regen. The button shows the VCU's
// read-back value, never the value we asked for. Never saved to flash: a
// Zombie power cycle returns to its saved regenmax.
static lv_obj_t *s_regen_btn     = NULL;
static lv_obj_t *s_regen_btn_lbl = NULL;
static int  s_regen_live_mag = -1;             // VCU regenmax magnitude, -1 = not read yet
static int  s_regen_on_mag   = REGEN_MAG_MIN;  // restore value for ON (last non-zero read)
static bool s_regen_pending  = false;          // write sent, waiting for read-back

static void prv_regen_btn_refresh(void)
{
    lv_color_t fg, bg;
    lv_opa_t   bg_opa;

    if (!s_regen_btn) return;
    if (s_regen_pending || s_regen_live_mag < 0) {
        lv_label_set_text(s_regen_btn_lbl, s_regen_pending ? "REGEN ..." : "REGEN ?");
        fg = CLR_TEXT_MID; bg = CLR_PANEL; bg_opa = LV_OPA_COVER;
    } else if (s_regen_live_mag == 0) {
        lv_label_set_text(s_regen_btn_lbl, "REGEN OFF");
        fg = CLR_BG; bg = CLR_AMBER; bg_opa = LV_OPA_COVER;
    } else {
        lv_label_set_text_fmt(s_regen_btn_lbl, "REGEN -%d%%", s_regen_live_mag);
        fg = CLR_GREEN; bg = CLR_PANEL; bg_opa = LV_OPA_COVER;
    }
    lv_obj_set_style_text_color(s_regen_btn_lbl, fg, 0);
    lv_obj_set_style_bg_color(s_regen_btn, bg, 0);
    lv_obj_set_style_bg_opa(s_regen_btn, bg_opa, 0);
    lv_obj_set_style_border_color(s_regen_btn,
                                  s_regen_live_mag == 0 ? CLR_AMBER : CLR_GREEN, 0);
}

// Long-press: toggle. Unknown state (no read-back yet) does nothing.
static void prv_regen_btn_cb(lv_event_t *e)
{
    int target;

    (void)e;
    if (s_regen_live_mag < 0 || s_regen_pending) return;
    target = (s_regen_live_mag == 0) ? s_regen_on_mag : 0;
    if (dashboard_vcu_set_param(REGEN_PARAM_ID, -(float)target)) {
        s_regen_pending = true;
    } else {
        lv_label_set_text(s_regen_btn_lbl, "NO VCU");
        lv_obj_set_style_text_color(s_regen_btn_lbl, CLR_AMBER, 0);
        return;
    }
    prv_regen_btn_refresh();
}

// Right panel, under the nav cluster; same 224px span as the three nav buttons.
static void prv_add_regen_button(lv_obj_t *screen)
{
    lv_obj_t *btn;

    btn = lv_obj_create(screen);
    lv_obj_set_size(btn, 224, 64);
    lv_obj_align(btn, LV_ALIGN_RIGHT_MID, -8, 88);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_border_width(btn, 3, LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn, 12, 0);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(btn, 8);
    lv_obj_add_event_cb(btn, prv_regen_btn_cb, LV_EVENT_LONG_PRESSED, NULL);
    s_regen_btn_lbl = lv_label_create(btn);
    lv_obj_set_style_text_font(s_regen_btn_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(s_regen_btn_lbl);
    s_regen_btn = btn;
    prv_regen_btn_refresh();
}

// While dragging: update the readout only — do NOT touch the VCU.
static void prv_regen_changed_cb(lv_event_t *e)
{
    (void)e;
    int mag = lv_slider_get_value(s_regen_slider);
    lv_label_set_text_fmt(s_regen_lbl, "REGEN LIMIT  -%d%%", mag);
    lv_obj_set_style_text_color(s_regen_lbl, CLR_TEXT_BRIGHT, 0);
}

// On release: commit the (negative) magnitude to regenmax and flag it applied.
static void prv_regen_released_cb(lv_event_t *e)
{
    (void)e;
    int mag = lv_slider_get_value(s_regen_slider);
    bool ok = dashboard_vcu_set_param(REGEN_PARAM_ID, -(float)mag);
    lv_label_set_text_fmt(s_regen_lbl, ok ? "REGEN  -%d%%  " LV_SYMBOL_OK
                                          : "REGEN  -%d%%  (no VCU)", mag);
    lv_obj_set_style_text_color(s_regen_lbl, ok ? CLR_GREEN : CLR_AMBER, 0);
}

// Platform → UI: the VCU's current regenmax. Boots the slider at the real value;
// won't yank the knob out from under a finger that's mid-drag.
extern "C" void dashboard_ui_set_regen_current(float pct)
{
    int mag = (int)lroundf(-pct);                 // regenmax is negative → magnitude
    if (mag < 0) mag = 0;
    s_regen_live_mag = mag;                       // regen button: raw, 0 = OFF
    if (mag > 0) s_regen_on_mag = mag;
    s_regen_pending = false;
    prv_regen_btn_refresh();
    if (mag < REGEN_MAG_MIN) mag = REGEN_MAG_MIN;
    if (mag > REGEN_MAG_MAX) mag = REGEN_MAG_MAX;
    s_regen_current_mag  = mag;
    s_regen_have_current = true;
    if (!s_regen_slider) return;                  // not built yet → applied on build
    if (lv_obj_has_state(s_regen_slider, LV_STATE_PRESSED)) return;  // don't fight a drag
    lv_slider_set_value(s_regen_slider, mag, LV_ANIM_OFF);
    if (s_regen_live_mag == 0) {
        lv_label_set_text(s_regen_lbl, "REGEN OFF  (release slider to set)");
        lv_obj_set_style_text_color(s_regen_lbl, CLR_AMBER, 0);
        return;
    }
    lv_label_set_text_fmt(s_regen_lbl, "REGEN LIMIT  -%d%%", mag);
    lv_obj_set_style_text_color(s_regen_lbl, CLR_TEXT_BRIGHT, 0);
}

static void prv_ensure_vcu_screen(void)
{
    if (s_scr_vcu) return;
    s_scr_vcu = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_vcu, CLR_BG, 0);
    lv_obj_set_style_bg_opa(s_scr_vcu, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_scr_vcu, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *grid = lv_obj_create(s_scr_vcu);
    lv_obj_set_size(grid, LCD_H_RES - 12 - NAV_SAFE_RIGHT, LCD_V_RES - NAV_SAFE_TOP - 12);
    lv_obj_align(grid, LV_ALIGN_TOP_LEFT, 12, NAV_SAFE_TOP);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_set_style_pad_row(grid, 12, 0);
    lv_obj_set_style_pad_column(grid, 12, 0);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);

    const lv_coord_t card_w = (LCD_H_RES - 12 - NAV_SAFE_RIGHT - 4 * 12) / 5;  // 5/row
    for (int i = 0; i < ST_N; i++) {
        lv_obj_t *card = lv_obj_create(grid);
        lv_obj_set_size(card, card_w, 118);
        lv_obj_set_style_bg_color(card, CLR_PANEL, 0);
        lv_obj_set_style_border_color(card, CLR_BORDER, 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_pad_all(card, 10, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *nm = lv_label_create(card);
        lv_label_set_text(nm, st_name[i]);
        lv_obj_set_style_text_color(nm, CLR_TEXT_MID, 0);
        lv_obj_set_style_text_font(nm, &lv_font_montserrat_18, 0);
        lv_obj_align(nm, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_t *vl = lv_label_create(card);
        lv_label_set_text(vl, "--");
        lv_obj_set_style_text_color(vl, CLR_TEXT_BRIGHT, 0);
        lv_obj_set_style_text_font(vl, &lv_font_montserrat_40, 0);
        lv_obj_align(vl, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        s_st_val[i] = vl;
    }

    // ── Regen limit strip: lower band, clear of the right-middle nav cluster ──
    lv_obj_t *rc = lv_obj_create(s_scr_vcu);
    lv_obj_set_size(rc, LCD_H_RES - NAV_SAFE_RIGHT - 24, 96);
    lv_obj_align(rc, LV_ALIGN_BOTTOM_LEFT, 12, -12);
    lv_obj_set_style_bg_color(rc, CLR_PANEL, 0);
    lv_obj_set_style_bg_opa(rc, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(rc, CLR_BORDER, 0);
    lv_obj_set_style_border_width(rc, 1, 0);
    lv_obj_set_style_radius(rc, 8, 0);
    lv_obj_set_style_pad_all(rc, 14, 0);
    lv_obj_clear_flag(rc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(rc, LV_OBJ_FLAG_CLICKABLE);   // absorb taps so a near-miss doesn't nav home

    // Start at the VCU's real regenmax if we've read it back; else a safe default
    // (shown greyed with a "?" so it's obvious it isn't the confirmed live value).
    int start_mag = s_regen_have_current ? s_regen_current_mag : REGEN_MAG_DEFAULT;

    s_regen_lbl = lv_label_create(rc);
    if (s_regen_live_mag == 0)
        lv_label_set_text(s_regen_lbl, "REGEN OFF  (release slider to set)");
    else if (s_regen_have_current)
        lv_label_set_text_fmt(s_regen_lbl, "REGEN LIMIT  -%d%%", start_mag);
    else
        lv_label_set_text_fmt(s_regen_lbl, "REGEN LIMIT  -%d%%  ?", start_mag);
    lv_obj_set_style_text_color(s_regen_lbl, s_regen_live_mag == 0 ? CLR_AMBER :
                                s_regen_have_current ? CLR_TEXT_BRIGHT : CLR_TEXT_MID, 0);
    lv_obj_set_style_text_font(s_regen_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(s_regen_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

    s_regen_slider = lv_slider_create(rc);
    // -68 (not -28): extra inset so the knob, which bulges 14px past the bar,
    // isn't clipped by rc's right edge at the full-right (max) position.
    lv_obj_set_width(s_regen_slider, LCD_H_RES - NAV_SAFE_RIGHT - 24 - 68);
    lv_obj_set_height(s_regen_slider, 28);                 // fat bar for a fingertip
    lv_obj_align(s_regen_slider, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_slider_set_range(s_regen_slider, REGEN_MAG_MIN, REGEN_MAG_MAX);  // magnitude; floor guards weak end
    lv_slider_set_value(s_regen_slider, start_mag, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_regen_slider, CLR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_regen_slider, CLR_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_regen_slider, CLR_GREEN, LV_PART_KNOB);
    lv_obj_set_style_pad_all(s_regen_slider, 14, LV_PART_KNOB);  // knob bulges past the bar
    lv_obj_set_ext_click_area(s_regen_slider, 22);        // vertical near-misses still grab it
    lv_obj_add_event_cb(s_regen_slider, prv_regen_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_regen_slider, prv_regen_released_cb, LV_EVENT_RELEASED, NULL);

    // NOTE: no prv_add_home_button() here — that makes the WHOLE screen a
    // tap-to-home target, which steals taps meant for the slider. This screen is
    // now interactive; home is still reachable via the 🏠 in the arrow cluster.
    prv_add_arrow_nav(s_scr_vcu);
}

static void prv_update_vcu(const DashData *d)
{
    if (!s_st_val[0]) return;   // status screen not built yet
    static const char *gnm[] = {"P", "R", "N", "D"};
    char b[24];
    snprintf(b, sizeof b, "%.0f %%", d->soc_pct);         lv_label_set_text(s_st_val[ST_SOC],   b);
    snprintf(b, sizeof b, "%.0f",    d->speed);           lv_label_set_text(s_st_val[ST_SPEED], b);
    snprintf(b, sizeof b, "%.0f kW", d->power_kw);        lv_label_set_text(s_st_val[ST_POWER], b);
    snprintf(b, sizeof b, "%.0f V",  d->pack_volts);      lv_label_set_text(s_st_val[ST_PACKV], b);
    snprintf(b, sizeof b, "%.0f A",  d->pack_amps);       lv_label_set_text(s_st_val[ST_PACKA], b);
    snprintf(b, sizeof b, "%.0f mi", d->range_dist);      lv_label_set_text(s_st_val[ST_RANGE], b);
    // Motor/inverter temps are inverter-sourced (GS450H) — grey + "--" when Off.
    int t_stale = (d->vcu_opmode == 0);
    static int last_t_stale = -1;
    if (t_stale != last_t_stale) {
        lv_color_t c = t_stale ? CLR_TEXT_DIM : CLR_TEXT_BRIGHT;
        lv_obj_set_style_text_color(s_st_val[ST_MOTOR], c, 0);
        lv_obj_set_style_text_color(s_st_val[ST_INV],   c, 0);
        last_t_stale = t_stale;
    }
    if (t_stale) {
        lv_label_set_text(s_st_val[ST_MOTOR], "--");
        lv_label_set_text(s_st_val[ST_INV],   "--");
    } else {
        snprintf(b, sizeof b, "%.0f C", d->motor_temp_c);    lv_label_set_text(s_st_val[ST_MOTOR], b);
        snprintf(b, sizeof b, "%.0f C", d->inverter_temp_c); lv_label_set_text(s_st_val[ST_INV],   b);
    }
    snprintf(b, sizeof b, "%.0f C",  d->batt_temp_c);     lv_label_set_text(s_st_val[ST_BATT],  b);
    snprintf(b, sizeof b, "%.0f C",  d->aux_temp_c);      lv_label_set_text(s_st_val[ST_SHUNT], b);
    snprintf(b, sizeof b, "%.1f V",  d->aux_volts);       lv_label_set_text(s_st_val[ST_AUX],   b);
    lv_label_set_text(s_st_val[ST_GEAR], d->gear < 4 ? gnm[d->gear] : "-");
    snprintf(b, sizeof b, "%.0f",    d->odo_total_miles); lv_label_set_text(s_st_val[ST_ODO],   b);
    snprintf(b, sizeof b, "%.1f",    d->trip_miles);      lv_label_set_text(s_st_val[ST_TRIP],  b);
    snprintf(b, sizeof b, "%.0f %%", d->can_load_pct);    lv_label_set_text(s_st_val[ST_CANLOAD], b);
}

#ifdef BMS_HTTP
// ── BMS screen: pack summary + per-module cell grid (from HTTP /api/data) ──
#define BMS_UI_MODS   8
#define BMS_UI_CELLS  12
enum { BS_PACK, BS_SOC, BS_LOW, BS_HIGH, BS_DELTA, BS_MODDLT, BS_TEMP, BS_CURR, BS_N };
static const char *bs_name[BS_N] =
    { "PACK", "SOC", "MIN CELL", "MAX CELL", "CELL DLT", "MOD DLT", "AVG TEMP", "CURRENT" };
static lv_obj_t *s_bms_sumval[BS_N];
static lv_obj_t *s_bms_modcard[BMS_UI_MODS];
static lv_obj_t *s_bms_modv[BMS_UI_MODS];
static lv_obj_t *s_bms_modt[BMS_UI_MODS];
static lv_obj_t *s_bms_cell[BMS_UI_MODS][BMS_UI_CELLS];
static lv_obj_t *s_bms_celllbl[BMS_UI_MODS][BMS_UI_CELLS];

// Physical mounting in the truck: which modules sit at the FRONT (rest = rear),
// so front/rear thermal groups are visually separable. 0-based (idx 0 = M1).
// Front = M1, M4, M5 (all confirmed 2026-08-26).
// Edit this array to re-map front vs rear.
static const bool bms_mod_front[BMS_UI_MODS] =
    { true, false, false, true, true, false, false, false };

// f: 0 (lowest cell) .. 1 (highest cell). blue -> green -> red, dimmed by scale.
static lv_color_t bms_heat(float f, float scale)
{
    if (f < 0) f = 0; if (f > 1) f = 1;
    float r, g, bl;
    if (f < 0.5f) { float u = f / 0.5f;
        r  = 0x22 + u * (0x10 - 0x22);
        g  = 0x66 + u * (0xB9 - 0x66);
        bl = 0xEF + u * (0x81 - 0xEF);
    } else { float u = (f - 0.5f) / 0.5f;
        r  = 0x10 + u * (0xEF - 0x10);
        g  = 0xB9 + u * (0x44 - 0xB9);
        bl = 0x81 + u * (0x44 - 0x81);
    }
    return lv_color_make((uint8_t)(r * scale), (uint8_t)(g * scale), (uint8_t)(bl * scale));
}

static void prv_ensure_bms_screen(void)
{
    if (s_scr_bms) return;
    const lv_coord_t W = LCD_H_RES, H = LCD_V_RES;
    s_scr_bms = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_bms, CLR_BG, 0);
    lv_obj_set_style_bg_opa(s_scr_bms, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_scr_bms, LV_OBJ_FLAG_SCROLLABLE);

    // ── summary strip ──
    lv_obj_t *sum = lv_obj_create(s_scr_bms);
    lv_obj_set_size(sum, W - 12 - NAV_SAFE_RIGHT, 92);   // match module-row span below
    lv_obj_align(sum, LV_ALIGN_TOP_LEFT, 12, 10);        // left-align with the module cards
    lv_obj_set_style_bg_opa(sum, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(sum, 0, 0);
    lv_obj_set_style_pad_all(sum, 0, 0);
    lv_obj_set_style_pad_column(sum, 8, 0);
    lv_obj_set_flex_flow(sum, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(sum, LV_OBJ_FLAG_SCROLLABLE);
    for (int i = 0; i < BS_N; i++) {
        lv_obj_t *tile = lv_obj_create(sum);
        lv_obj_set_flex_grow(tile, 1);
        lv_obj_set_height(tile, LV_PCT(100));
        lv_obj_set_style_bg_color(tile, CLR_PANEL, 0);
        lv_obj_set_style_border_color(tile, CLR_BORDER, 0);
        lv_obj_set_style_border_width(tile, 1, 0);
        lv_obj_set_style_radius(tile, 8, 0);
        lv_obj_set_style_pad_all(tile, 8, 0);
        lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *nm = lv_label_create(tile);
        lv_label_set_text(nm, bs_name[i]);
        lv_obj_set_style_text_color(nm, CLR_TEXT_MID, 0);
        lv_obj_set_style_text_font(nm, &lv_font_montserrat_14, 0);
        lv_obj_align(nm, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_t *vl = lv_label_create(tile);
        lv_label_set_text(vl, "--");
        lv_obj_set_style_text_color(vl, CLR_TEXT_BRIGHT, 0);
        lv_obj_set_style_text_font(vl, &lv_font_montserrat_24, 0);
        lv_obj_align(vl, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        s_bms_sumval[i] = vl;
    }

    // ── module cards ──
    lv_obj_t *mods = lv_obj_create(s_scr_bms);
    lv_obj_set_size(mods, W - 12 - NAV_SAFE_RIGHT, H - BOT_H - 92 - 34);  // clear arrows
    lv_obj_align(mods, LV_ALIGN_TOP_LEFT, 12, 110);
    lv_obj_set_style_bg_opa(mods, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mods, 0, 0);
    lv_obj_set_style_pad_all(mods, 0, 0);
    lv_obj_set_style_pad_column(mods, 8, 0);
    lv_obj_set_flex_flow(mods, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(mods, LV_OBJ_FLAG_SCROLLABLE);

    for (int m = 0; m < BMS_UI_MODS; m++) {
        lv_obj_t *card = lv_obj_create(mods);
        s_bms_modcard[m] = card;
        const bool     front = bms_mod_front[m];          // truck front vs rear
        const lv_color_t grp  = front ? CLR_CYAN : CLR_AMBER;
        lv_obj_set_flex_grow(card, 1);
        lv_obj_set_height(card, LV_PCT(100));
        lv_obj_set_style_bg_color(card, CLR_PANEL, 0);
        lv_obj_set_style_border_color(card, grp, 0);       // group-colored ring
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_pad_all(card, 6, 0);
        lv_obj_set_style_pad_row(card, 3, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        char hb[20]; snprintf(hb, sizeof hb, "M%d  %s", m + 1, front ? "FWD" : "REAR");
        lv_obj_t *hdr = lv_label_create(card);
        lv_label_set_text(hdr, hb);
        lv_obj_set_style_text_color(hdr, grp, 0);
        lv_obj_set_style_text_font(hdr, &lv_font_montserrat_18, 0);
        lv_obj_t *mv = lv_label_create(card);
        lv_label_set_text(mv, "--");
        lv_obj_set_style_text_color(mv, CLR_TEXT_BRIGHT, 0);
        lv_obj_set_style_text_font(mv, &lv_font_montserrat_14, 0);
        s_bms_modv[m] = mv;
        lv_obj_t *mt = lv_label_create(card);
        lv_label_set_text(mt, "--");
        lv_obj_set_style_text_color(mt, CLR_TEXT_MID, 0);
        lv_obj_set_style_text_font(mt, &lv_font_montserrat_14, 0);
        s_bms_modt[m] = mt;

        lv_obj_t *cg = lv_obj_create(card);
        lv_obj_set_width(cg, LV_PCT(100));
        lv_obj_set_flex_grow(cg, 1);
        lv_obj_set_style_bg_opa(cg, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(cg, 0, 0);
        lv_obj_set_style_pad_all(cg, 0, 0);
        lv_obj_set_style_pad_row(cg, 3, 0);
        lv_obj_set_style_pad_column(cg, 3, 0);
        lv_obj_set_flex_flow(cg, LV_FLEX_FLOW_ROW_WRAP);
        lv_obj_clear_flag(cg, LV_OBJ_FLAG_SCROLLABLE);
        for (int c = 0; c < BMS_UI_CELLS; c++) {
            lv_obj_t *cell = lv_obj_create(cg);
            lv_obj_set_size(cell, LV_PCT(48), LV_PCT(15));
            lv_obj_set_style_bg_color(cell, CLR_BG, 0);
            lv_obj_set_style_border_width(cell, 0, 0);
            lv_obj_set_style_radius(cell, 4, 0);
            lv_obj_set_style_pad_all(cell, 0, 0);
            lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
            s_bms_cell[m][c] = cell;
            lv_obj_t *cl = lv_label_create(cell);
            lv_label_set_text(cl, "--");
            lv_obj_set_style_text_color(cl, CLR_TEXT_BRIGHT, 0);
            lv_obj_set_style_text_font(cl, &lv_font_montserrat_14, 0);
            lv_obj_center(cl);
            s_bms_celllbl[m][c] = cl;
        }
    }
    prv_add_home_button(s_scr_bms);
    prv_add_arrow_nav(s_scr_bms);
}

static void prv_update_bms(void)
{
    if (!s_bms_sumval[BS_PACK]) return;   // screen not built yet
    char b[24];
    if (!g_bms.valid) {
        lv_label_set_text(s_bms_sumval[BS_PACK], "...");
        return;
    }
    float lo = g_bms.lowCell, hi = g_bms.highCell;
    float span = hi - lo; if (span < 0.001f) span = 0.001f;

    snprintf(b, sizeof b, "%.1f V", g_bms.packV);    lv_label_set_text(s_bms_sumval[BS_PACK], b);
    snprintf(b, sizeof b, "%.1f %%", g_bms.soc);     lv_label_set_text(s_bms_sumval[BS_SOC],  b);
    snprintf(b, sizeof b, "%.3f", lo);               lv_label_set_text(s_bms_sumval[BS_LOW],  b);
    snprintf(b, sizeof b, "%.3f", hi);               lv_label_set_text(s_bms_sumval[BS_HIGH], b);
    int dmv = (int)((hi - lo) * 1000.0f + 0.5f);
    snprintf(b, sizeof b, "%d mV", dmv);             lv_label_set_text(s_bms_sumval[BS_DELTA], b);
    lv_obj_set_style_text_color(s_bms_sumval[BS_DELTA],
        dmv < 30 ? CLR_GREEN : dmv < 100 ? CLR_ORANGE : CLR_RED, 0);
    snprintf(b, sizeof b, "%.1f C", g_bms.avgTemp);  lv_label_set_text(s_bms_sumval[BS_TEMP], b);
    snprintf(b, sizeof b, "%.1f A", g_bms.currentA); lv_label_set_text(s_bms_sumval[BS_CURR], b);

    int nm = g_bms.numModules; if (nm > BMS_UI_MODS) nm = BMS_UI_MODS;

    // Module-to-module spread — the number that matters for this pack. The CSCs
    // balance perfectly WITHIN a module but never across, so this is the real
    // imbalance (and CELL DLT above is ~1/12 of it). modLo/modHi also drive the
    // per-module voltage colour so the high/low outliers pop.
    float modLo = 1e9f, modHi = -1e9f;
    for (int m = 0; m < nm; m++) {
        float v = g_bms.modules[m].voltage;
        if (v < modLo) modLo = v;
        if (v > modHi) modHi = v;
    }
    float modSpan = (nm > 0) ? (modHi - modLo) : 0.0f;
    snprintf(b, sizeof b, "%.2f V", modSpan); lv_label_set_text(s_bms_sumval[BS_MODDLT], b);
    lv_obj_set_style_text_color(s_bms_sumval[BS_MODDLT],
        modSpan < 0.30f ? CLR_GREEN : modSpan < 1.0f ? CLR_ORANGE : CLR_RED, 0);

    for (int m = 0; m < BMS_UI_MODS; m++) {
        if (m >= nm) { lv_obj_add_flag(s_bms_modcard[m], LV_OBJ_FLAG_HIDDEN); continue; }
        lv_obj_clear_flag(s_bms_modcard[m], LV_OBJ_FLAG_HIDDEN);
        BmsModule *bm = &g_bms.modules[m];
        snprintf(b, sizeof b, "%.2f V", bm->voltage);     lv_label_set_text(s_bms_modv[m], b);
        // colour the module voltage by its offset: blue = low SoC, red = high
        float mf = (modSpan > 0.001f) ? (bm->voltage - modLo) / modSpan : 0.5f;
        lv_obj_set_style_text_color(s_bms_modv[m], bms_heat(mf, 1.0f), 0);
        snprintf(b, sizeof b, "%d/%d C", bm->t1, bm->t2); lv_label_set_text(s_bms_modt[m], b);
        lv_obj_set_style_border_color(s_bms_modcard[m], bm->faulted ? CLR_RED : CLR_BORDER, 0);
        int nc = bm->num_cells; if (nc > BMS_UI_CELLS) nc = BMS_UI_CELLS;
        for (int c = 0; c < BMS_UI_CELLS; c++) {
            if (c >= nc) { lv_obj_add_flag(s_bms_cell[m][c], LV_OBJ_FLAG_HIDDEN); continue; }
            lv_obj_clear_flag(s_bms_cell[m][c], LV_OBJ_FLAG_HIDDEN);
            float v = bm->cells[c];
            lv_obj_set_style_bg_color(s_bms_cell[m][c], bms_heat((v - lo) / span, 0.55f), 0);
            snprintf(b, sizeof b, "%.3f", v); lv_label_set_text(s_bms_celllbl[m][c], b);
        }
    }
}
#endif // BMS_HTTP

extern "C" void dashboard_ui_set_screen(dash_screen_t screen)
{
    // Queue the screen switch — actual LVGL call happens in dashboard_ui_update()
    // on the UI task, since LVGL is not thread-safe.
    s_pending_screen    = screen;
    s_screen_change_req = true;
}

extern "C" dash_screen_t dashboard_ui_get_screen(void)
{
    return s_cur_screen;
}

// ── C-compatible g_dash getters (used by status_page.c) ──────
extern "C" {
float dash_get_speed(void)        { return g_dash.speed; }
float dash_get_soc(void)          { return g_dash.soc_pct; }
float dash_get_power_kw(void)     { return g_dash.power_kw; }
float dash_get_pack_volts(void)   { return g_dash.pack_volts; }
float dash_get_pack_amps(void)    { return g_dash.pack_amps; }
float dash_get_inverter_temp(void){ return g_dash.inverter_temp_c; }
float dash_get_motor_temp(void)   { return g_dash.motor_temp_c; }
float dash_get_batt_temp(void)    { return g_dash.batt_temp_c; }
float dash_get_aux_volts(void)    { return g_dash.aux_volts; }
}
