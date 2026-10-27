// SPDX-FileCopyrightText: 2026 Herb Peyerl
// SPDX-License-Identifier: BSD-3-Clause

// =============================================================
//  http_api.c — tiny HTTP control/status API for the Linux dashboard
//
//  A single background thread, minimal HTTP/1.1. Enough for remote
//  navigation + live state (headless debugging, config later). Reuses
//  dashboard_ui_set_screen() (thread-safe queue) and cJSON.
// =============================================================
#include "http_api.h"
#include "can_parser.h"
#include "dashboard_ui.h"
#include "cJSON.h"

#include <pthread.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>

static void send_resp(int fd, const char *status, const char *ctype, const char *body)
{
    char hdr[256];
    int n = snprintf(hdr, sizeof hdr,
        "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
        "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n",
        status, ctype, strlen(body));
    if (write(fd, hdr, (size_t)n) < 0) return;
    if (write(fd, body, strlen(body)) < 0) return;
}

static void handle_status(int fd)
{
    const DashData *d = &g_dash;
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "soc",          d->soc_pct);
    cJSON_AddNumberToObject(o, "speed",        d->speed);
    cJSON_AddNumberToObject(o, "power_kw",     d->power_kw);
    cJSON_AddNumberToObject(o, "pack_v",       d->pack_volts);
    cJSON_AddNumberToObject(o, "vcu_udc",      d->vcu_udc);
    cJSON_AddNumberToObject(o, "vcu_idc",      d->vcu_idc);
    cJSON_AddNumberToObject(o, "rpm",          d->vcu_rpm);
    cJSON_AddNumberToObject(o, "dir",          d->vcu_dir);
    cJSON_AddNumberToObject(o, "opmode",       d->vcu_opmode);
    cJSON_AddNumberToObject(o, "range",        d->vcu_range);
    cJSON_AddNumberToObject(o, "motor_c",      d->motor_temp_c);
    cJSON_AddNumberToObject(o, "inv_c",        d->inverter_temp_c);
    cJSON_AddNumberToObject(o, "batt_c",       d->batt_temp_c);
    cJSON_AddNumberToObject(o, "aux_v",        d->aux_volts);
    cJSON_AddNumberToObject(o, "potnom",       d->vcu_potnom);
    cJSON_AddBoolToObject  (o, "brake",        d->vcu_brake);
    cJSON_AddBoolToObject  (o, "park",         d->vcu_park);
    cJSON_AddNumberToObject(o, "cruise_state", d->cruise_state);
    cJSON_AddNumberToObject(o, "cruise_kph",   d->cruise_kph);
    cJSON_AddNumberToObject(o, "gear",         d->gear);
    cJSON_AddNumberToObject(o, "hl_mode",      d->hl_mode);
    cJSON_AddNumberToObject(o, "mg_mode",      d->mg_mode);
    cJSON_AddNumberToObject(o, "can_load",     d->can_load_pct);
    cJSON_AddNumberToObject(o, "screen",       dashboard_ui_get_screen());
    char *js = cJSON_PrintUnformatted(o);
    send_resp(fd, "200 OK", "application/json", js ? js : "{}");
    free(js);
    cJSON_Delete(o);
}

static void handle_nav(int fd, const char *path)
{
    const char *q = strstr(path, "screen=");
    dash_screen_t scr;
    int ok = 1;
    if (q) {
        q += 7;
        if      (!strncasecmp(q, "home",     4)) scr = DASH_SCREEN_HOME;
        else if (!strncasecmp(q, "settings", 8)) scr = DASH_SCREEN_SETTINGS;
        else if (!strncasecmp(q, "status",   6)) scr = DASH_SCREEN_VCU;
        else if (!strncasecmp(q, "vcu",      3)) scr = DASH_SCREEN_VCU;
        else if (!strncasecmp(q, "bms",      3)) scr = DASH_SCREEN_BMS;
        else ok = 0;
    } else ok = 0;

    if (ok) {
        dashboard_ui_set_screen(scr);
        send_resp(fd, "200 OK", "application/json", "{\"ok\":true}");
    } else {
        send_resp(fd, "400 Bad Request", "application/json",
                  "{\"ok\":false,\"err\":\"screen=home|settings|status|vcu|bms\"}");
    }
}

static int parse_screen_q(const char *path, dash_screen_t *out)
{
    const char *q = strstr(path, "screen=");
    if (!q) return 0;
    q += 7;
    if      (!strncasecmp(q, "home",     4)) *out = DASH_SCREEN_HOME;
    else if (!strncasecmp(q, "settings", 8)) *out = DASH_SCREEN_SETTINGS;
    else if (!strncasecmp(q, "status",   6)) *out = DASH_SCREEN_VCU;
    else if (!strncasecmp(q, "vcu",      3)) *out = DASH_SCREEN_VCU;
    else if (!strncasecmp(q, "bms",      3)) *out = DASH_SCREEN_BMS;
    else return 0;
    return 1;
}

// GET /api/screenshot[?screen=NAME] — grab /dev/fb0 (720x1920 BGRX), rotate to
// 1920x720 landscape, return as a BMP (no deps; browsers render it fine).
static void handle_screenshot(int fd, const char *path)
{
    dash_screen_t scr;
    if (parse_screen_q(path, &scr)) {
        dashboard_ui_set_screen(scr);
        usleep(150 * 1000);              // let the UI thread render the switch
    }

    const int FBW = 720, FBH = 1920;     // physical framebuffer (portrait)
    size_t fbsz = (size_t)FBW * FBH * 4;
    unsigned char *fb = (unsigned char *)malloc(fbsz);
    if (!fb) { send_resp(fd, "500 Internal Server Error", "text/plain", "oom"); return; }
    int f = open("/dev/fb0", O_RDONLY);
    ssize_t rd = (f >= 0) ? read(f, fb, fbsz) : -1;
    if (f >= 0) close(f);
    if (rd != (ssize_t)fbsz) {
        free(fb);
        send_resp(fd, "500 Internal Server Error", "text/plain", "fb read failed");
        return;
    }

    const int OW = 1920, OH = 720;       // landscape output
    int rowsz = OW * 3;                  // 5760, already 4-byte aligned
    size_t pixsz = (size_t)rowsz * OH;
    size_t filesz = 54 + pixsz;
    unsigned char *bmp = (unsigned char *)calloc(1, filesz);
    if (!bmp) { free(fb); send_resp(fd, "500 Internal Server Error", "text/plain", "oom"); return; }

    bmp[0] = 'B'; bmp[1] = 'M';
    uint32_t v;
    v = (uint32_t)filesz; memcpy(bmp + 2,  &v, 4);
    v = 54;               memcpy(bmp + 10, &v, 4);   // pixel data offset
    v = 40;               memcpy(bmp + 14, &v, 4);   // info header size
    int32_t iv;
    iv = OW;  memcpy(bmp + 18, &iv, 4);
    iv = OH;  memcpy(bmp + 22, &iv, 4);              // positive => bottom-up
    uint16_t s16;
    s16 = 1;  memcpy(bmp + 26, &s16, 2);
    s16 = 24; memcpy(bmp + 28, &s16, 2);            // 24bpp
    v = (uint32_t)pixsz; memcpy(bmp + 34, &v, 4);

    // 90° CCW map: out(xo,yo) = fb(fx=FBW-1-yo, fy=xo). BMP is bottom-up + BGR;
    // fb is BGRX so the first 3 bytes copy straight over.
    for (int r = 0; r < OH; r++) {
        int yo = OH - 1 - r;                          // bottom-up
        unsigned char *drow = bmp + 54 + (size_t)r * rowsz;
        int fx = FBW - 1 - yo;
        for (int xo = 0; xo < OW; xo++) {
            const unsigned char *src = fb + ((size_t)xo * FBW + fx) * 4;
            drow[xo * 3 + 0] = src[0];
            drow[xo * 3 + 1] = src[1];
            drow[xo * 3 + 2] = src[2];
        }
    }
    free(fb);

    char hdr[256];
    int hn = snprintf(hdr, sizeof hdr,
        "HTTP/1.1 200 OK\r\nContent-Type: image/bmp\r\nContent-Length: %zu\r\n"
        "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n", filesz);
    if (write(fd, hdr, (size_t)hn) > 0) {
        size_t off = 0;
        while (off < filesz) {
            ssize_t w = write(fd, bmp + off, filesz - off);
            if (w <= 0) break;
            off += (size_t)w;
        }
    }
    free(bmp);
}

static void *server_thread(void *arg)
{
    int port = (int)(intptr_t)arg;
    struct timeval rcv_to = { 2, 0 };   // idle client can't block the server
    signal(SIGPIPE, SIG_IGN);

    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return NULL;
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);

    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons((uint16_t)port);
    if (bind(s, (struct sockaddr *)&a, sizeof a) < 0) {
        fprintf(stderr, "http_api: bind :%d failed\n", port);
        close(s);
        return NULL;
    }
    listen(s, 8);
    printf("http api: listening on :%d\n", port);

    for (;;) {
        int c = accept(s, NULL, NULL);
        if (c < 0) continue;
        setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof rcv_to);
        char buf[1024];
        ssize_t n = read(c, buf, sizeof buf - 1);
        if (n > 0) {
            buf[n] = 0;
            char method[8] = {0}, path[512] = {0};
            sscanf(buf, "%7s %511s", method, path);
            if      (!strncmp(path, "/api/status",     11)) handle_status(c);
            else if (!strncmp(path, "/api/screenshot", 15)) handle_screenshot(c, path);
            else if (!strncmp(path, "/api/nav",         8)) handle_nav(c, path);
            else send_resp(c, "200 OK", "text/plain",
                    "EV Dashboard API\n"
                    "  GET /api/status                   live values (JSON)\n"
                    "  GET /api/nav?screen=NAME          home|settings|status|vcu|bms\n"
                    "  GET /api/screenshot[?screen=NAME] framebuffer as BMP\n");
        }
        close(c);
    }
    return NULL;
}

void http_api_start(int port)
{
    pthread_t t;
    if (pthread_create(&t, NULL, server_thread, (void *)(intptr_t)port) == 0)
        pthread_detach(t);
}
