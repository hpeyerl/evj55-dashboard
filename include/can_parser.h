// SPDX-FileCopyrightText: 2026 Herb Peyerl
// SPDX-License-Identifier: BSD-3-Clause

// =============================================================
//  can_parser.h — CAN Frame Parser Public API
//
//  Call parse_can_frame() from your TWAI receive task.
//  Read g_dash (protected by a mutex) from your UI task.
// =============================================================

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "can_signals.h"

// ── Live dashboard snapshot ───────────────────────────────────────────────
typedef struct {
    // Drivetrain
    float   speed;          // vehicle speed, native CAN units (MPH from BMS)
    float   power_kw;       // derived: pack_volts × pack_amps / 1000

    // Battery / BMS (0x355, 0x356)
    float   soc_pct;        // 0–100 %
    float   pack_volts;     // V
    float   pack_amps;      // A  (+ = discharge / motoring, – = regen)
    float   batt_temp_c;    // °C

    // Thermal (0x513: tmpm, tmphs, tmpaux). On GS450H tmpm/tmphs come from the
    // inverter and are only valid when running; aux_temp_c (tmpaux) is a real
    // Zombie analog input that reads ambient/coolant.
    float   motor_temp_c;
    float   inverter_temp_c;
    float   aux_temp_c;

    // 12V system (0x210)
    float   aux_volts;

    // Derived
    float   range_dist;     // estimated range, native miles (convert at display)
    float   odo_total_miles;
    float   trip_miles;

    // Gear:  0=P  1=R  2=N  3=D
    uint8_t gear;          // requested gear from shifter (0x312)

    // VCU range / motor config, from SDO read-back. -1 = not yet read.
    int8_t  hl_mode;       // Gear (27):      0=L 1=H 2=A 3=HL
    int8_t  mg_mode;       // MotActive (129): 0=MG1+2 1=MG1 2=MG2 3=Blend

    // ZombieVerter VCU frames 0x510/0x511
    int16_t vcu_rpm;       // motor (MG2) RPM (2016)
    int8_t  vcu_dir;       // DIRS: -1=R 0=N 1=D 2=P; INT8_MIN=unknown (2024)
    uint8_t vcu_opmode;    // OPMODES: 0=Off 1=Run 2=Precharge 3=PchFail 4=Charge 5=Preheat
    float   vcu_udc;       // DC bus voltage, V (2006)
    float   vcu_idc;       // DC bus current, A (2012)
    float   vcu_potnom;    // throttle demand, % (2023)
    int8_t  vcu_range;     // Lexus Gear hi/lo (param 27): 0=Low 1=High; -1=unknown
    uint8_t vcu_brake;     // din_brake (2037): 1=brake pressed (0x511 byte7)
    uint8_t vcu_park;      // din_12Vgp (2071): 1=park pawl engaged (0x512 byte3)
    float   can_load_pct;  // CAN bus utilization %, computed on the host
    uint32_t last_ms_0x510;
    uint32_t last_ms_0x511;

    // Cruise control
    uint8_t cruise_state;  // CRUISE_CC_* bitmask from cruisestt
    float   cruise_kph;    // target speed in kph (converted from cruisespeed RPM)
    uint32_t last_ms_cruise;

    // Zombieverter confirmed direction: 1=Forward 0=Neutral -1=Reverse
    // Populated when Zombieverter dir CAN mapping is configured.
    // TODO: add oic mapping for Zombieverter dir signal.
    int8_t  dir_confirmed; // INT8_MIN = unknown/not yet received

    // Frame freshness — ms timestamp of last update per CAN ID
    uint32_t last_ms_0x125;
    uint32_t last_ms_0x312;
    uint32_t last_ms_0x126;
    uint32_t last_ms_0x257;
    uint32_t last_ms_0x355;
    uint32_t last_ms_0x356;
    uint32_t last_ms_0x210;
    uint32_t last_ms_any;    // stamped on EVERY received frame -> true CAN-alive liveness
    uint32_t last_ms_can0;   // per-bus liveness (stamped by the RX loop, by interface)
    uint32_t last_ms_can1;
} DashData;

// Global instance — write locked by g_dash_mutex in main.cpp
extern DashData g_dash;

// Parse one CAN frame and update g_dash.
// id      — 11-bit CAN identifier
// data    — 8-byte frame payload
// now_ms  — current time in milliseconds (for staleness tracking)
void parse_can_frame(uint32_t id, const uint8_t *data, uint32_t now_ms);

// Returns true if the given ID has not been seen within timeout_ms.
bool can_signal_stale(uint32_t last_ms, uint32_t now_ms, uint32_t timeout_ms);
