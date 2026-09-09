# CM3 GT911 touch fix - driver-owned reset via panel MCU (plan + draft patch)

Status 2026-09-08: root cause PROVEN, fix designed, NOT yet built/deployed.
Prepared offline (box + Herb needed to build/flash/test). Execute with Herb present.

## Proven root cause (live ssh, 2026-09-08)
The GT911 NAKs every read until it gets a hardware RESET. Repro on the box:
```
echo 0-005d | sudo tee /sys/bus/i2c/drivers/Goodix-TS/unbind
sudo i2ctransfer -y -f 0 w2@0x5d 0x81 0x40 r4     # -> "No such device" (chip dead)
sudo i2cset -y -f 0 0x45 0x94 0x01                # assert  touch reset (MCU line 9)
sudo i2cset -y -f 0 0x45 0x94 0x03                # release
sudo i2ctransfer -y -f 0 w2@0x5d 0x81 0x40 r4     # -> 0x39 0x32 0x37 0x31  ("9271")  ✔
```
So: not an interrupt, not a strap, not power-order, not driver-choice. A MISSING RESET.

## Why not the obvious fixes
- **reset-gpio (Pi5 way):** Pi5 DT has `goodix@5d { reset-gpio = <&display_mcu 9 0> }` where
  `display_mcu@45 = "waveshare,touchscreen-panel-regulator"` exposes the MCU as a gpiochip.
  **That driver is NOT in the CM3 BSP** (`gpiodetect` shows only rk3566 banks + rk817; no
  panel-regulator module). So there is no gpio phandle to reference.
- **userspace pulse in ws-panel-power.sh:** works at boot only. On STR **resume** the goodix
  `resume()` runs in kernel space *before* userspace, so a script hook is always too late.
- **modprobe/rmmod dance:** the currently-bound driver is the BSP **gt9xx**, which is BUILT-IN
  (`CONFIG_TOUCHSCREEN_GT9XX=y`, in modules.builtin) - can't rmmod it. AND runtime unbind/REBIND
  of it OOPSES the kernel (seen 2026-09-08, recoverable). Dead end.

## The fix: patch mainline goodix to pulse the MCU reset in probe() AND resume()
Two independent changes:

### 1. DT node -> mainline goodix (loadable `=m`), not the built-in gt9xx
`CONFIG_TOUCHSCREEN_GOODIX=m` (module `goodix_ts.ko` present). Change the touch node so the
mainline goodix binds and the buggy built-in gt9xx does not:
```
goodix@5d {
    compatible = "goodix,gt911";          /* was goodix,gt9xx (pulls the BSP driver) */
    reg = <0x5d>;
    touchscreen-size-x = <720>;
    touchscreen-size-y = <1920>;
    waveshare,panel-mcu-reset;            /* NEW bool: enable the MCU i2c reset below */
    /* NO reset-gpios / NO irq-gpios (MCU is not a gpiochip; driver polls) */
};
```
CAVEAT to verify on-box: confirm the built-in gt9xx `of_match_table` does NOT also match
`goodix,gt911` (if it does, both drivers fight - "Goodix-TS already registered"). gt9xx binds
`goodix,gt9xx`; using `gt911` should keep it away. Check with `modinfo`/source if in doubt.

### 2. goodix.c patch - reset the GT911 over i2c to MCU@0x45
The MCU@0x45 reg 0x94 bit1 = touch reset (line 9): 0 = assert, 1 = release. Add a helper that
pulses it, gated on the `waveshare,panel-mcu-reset` property, and call it in probe (before the
first chip read) and resume (before the IRQ_PIN_ACCESS_NONE early-return that otherwise skips
any reset on wake = the STR-death path). Draft against goodix.c v6.1.84 (matches the box kernel
6.1.84-18-rk2410-nocsf; base copy at `~/cm3-buildroot/.../linux-custom/drivers/input/touchscreen/goodix.c`).
Full draft in `goodix-waveshare-mcu-reset.patch` (this dir). Core helper:
```c
#define WS_PANEL_MCU_ADDR      0x45
#define WS_PANEL_MCU_GPIO_REG  0x94
#define WS_PANEL_TOUCH_RST_BIT BIT(1)          /* MCU gpio line 9 */

static int goodix_ws_mcu_reset(struct goodix_ts_data *ts)
{
    struct i2c_adapter *adap = ts->client->adapter;
    u8 reg = WS_PANEL_MCU_GPIO_REG, val;
    u8 wr[2] = { WS_PANEL_MCU_GPIO_REG, 0 };
    struct i2c_msg rmsg[2] = {
        { .addr = WS_PANEL_MCU_ADDR, .flags = 0,        .len = 1, .buf = &reg },
        { .addr = WS_PANEL_MCU_ADDR, .flags = I2C_M_RD, .len = 1, .buf = &val },
    };
    struct i2c_msg wmsg = { .addr = WS_PANEL_MCU_ADDR, .flags = 0, .len = 2, .buf = wr };
    int err;

    if (i2c_transfer(adap, rmsg, 2) != 2)          /* read so we only toggle line 9 */
        return -EIO;
    wr[1] = val & ~WS_PANEL_TOUCH_RST_BIT;          /* assert  */
    if (i2c_transfer(adap, &wmsg, 1) != 1) return -EIO;
    msleep(20);
    wr[1] = val | WS_PANEL_TOUCH_RST_BIT;           /* release -> GT911 defaults to 0x5d */
    if (i2c_transfer(adap, &wmsg, 1) != 1) return -EIO;
    msleep(60);                                     /* GT911 boot after reset release */
    return 0;
}
```
- **probe:** at the `reset:` label, just before `goodix_i2c_test(client)` (~line 1371):
  `if (device_property_read_bool(&client->dev,"waveshare,panel-mcu-reset")) { err = goodix_ws_mcu_reset(ts); if (err) return err; }`
- **resume:** at the very top of `goodix_resume()`, BEFORE the `IRQ_PIN_ACCESS_NONE` early return
  (~line 1487): same `goodix_ws_mcu_reset(ts)` call (chip is re-booted on every wake). Re-sending
  config after may be needed - test; the poll path may re-read fine once the chip answers.

## Build (out-of-tree module, against the box's headers)
Box HAS the build tree: `/lib/modules/6.1.84-18-rk2410-nocsf/build` (linux-headers pkg installed).
```
mkdir ~/goodix-build && cd ~/goodix-build
cp <patched>/goodix.c .
printf 'obj-m += goodix_ts.o\ngoodix_ts-objs := goodix.o\n' > Makefile   # match installed name
make -C /lib/modules/$(uname -r)/build M=$PWD modules
# test-load (after the DT change is live + a boot, chip powered):
sudo insmod ./goodix_ts.ko     # or install: cp over the packaged .ko(.xz), depmod, modprobe
```
(May need CONFIG_TOUCHSCREEN_GOODIX build deps; goodix.c is fairly self-contained.)

## Test sequence (Herb present; DTB change needs ro-rw + reboot)
1. First clear the current wedged/oopsed BSP driver: **reboot** the box.
2. Apply DT change (compatible->gt911 + the bool), rebuild+restack the merged DTB
   (`~/cm3-dtbo/rebuild-dtb.sh`, `sudo ro-rw`, copy DTB, `ro`), reboot.
3. Load the patched goodix_ts.ko. Expect: NO `<<-GTP-ERROR->>` spam, `goodix-ts` on /dev/input/eventN,
   ID reads OK. Poke screen -> `evtest`/`cat eventN` shows ABS/BTN_TOUCH. (App auto-detects "goodix-ts".)
4. **STR test (the real prize):** `systemctl suspend`, wake, poke screen again -> still reads =
   resume reset works. THAT is the acceptance gate.
5. Rotation: panel is 720x1920 portrait shown as 1920x720 landscape - touch axes will need
   swap/invert (Herb previously saw "works but rotated"). Handle via touchscreen-swapped-x-y /
   -inverted-x/-y DT props once events flow.

## Fallbacks if the patch path stalls
- Port `waveshare-panel-regulator.c` + `panel-waveshare-dsi-v2.c` from raspberrypi/linux as
  modules (makes the MCU a real gpiochip -> stock goodix + reset-gpio, no goodix patch, also
  cleans up panel power). More work, most upstream-correct.
- Userspace: blacklist auto-load, ws-panel-power.sh does power->reset->`modprobe goodix`; suspend
  hook `modprobe -r goodix`, resume hook reset->`modprobe goodix`. Works ONLY with the mainline
  (loadable) goodix, not the built-in gt9xx. Fragile vs the driver patch; keep as plan B.

## Do NOT
- runtime unbind/REBIND the BSP Goodix-TS driver -> kernel Oops.
- remote-reboot without someone at the box (wifi only, no OOB recovery).
See memory [[project_cm3_dashboard_app]] / [[feedback_dont_confabulate]].

## 2026-09-09 - built + tested on the box (reset SOLVED; IRQ open)

- **MCU-reset-in-probe patch WORKS.** With the DT node switched to `goodix,gt911`
  + `waveshare,panel-mcu-reset` and the patched driver loaded, probe pulsed MCU
  line 9 and the driver read **`ID 9271`**. The reset half is proven end-to-end.
- **Blocker (the remaining half):** I compiled `goodix.c` from the **RK BSP tree**
  (`~/cm3-buildroot/.../linux-custom`). That goodix `request_irq`s and ABORTS probe
  with **`-22`** when there is no interrupt -> it is the **WRONG FORK**. The Pi runs
  the **Raspberry-Pi-kernel** goodix, which works with **no interrupt**. NO INT is
  wired and that is SETTLED - do not reopen it.
- **Driver-name collision:** both the built-in gt9xx (`CONFIG_TOUCHSCREEN_GT9XX=y`,
  cannot blacklist) and mainline goodix register the i2c driver name `Goodix-TS`;
  gt9xx wins, so the module must rename `.driver.name` (used `goodix-ws-ts`) to load.
- **REMAINING WORK = a goodix that runs WITHOUT an irq, like the Pi's:** either patch
  the RK-BSP goodix to poll / skip `request_irq` when `irq==0`, or build from the
  Pi-kernel goodix source.
- **Box state:** eMMC boot DTB is now `goodix,gt911` + the bool (backups `*.pre911.*`
  on `/mnt/ro`); the installed eMMC `goodix_ts.ko` is the OLD **colliding** build that
  fails at boot; the working (renamed, reset-patched, irq-failing) build lived only in
  tmpfs `~/goodix-build` and is gone after reboot. Rebuild = `goodix.c` +
  `goodix_fwupload.c` + a two-file Makefile (`goodix_ts-y := goodix.o goodix_fwupload.o`).

## 2026-09-09 - touch works at the driver level; LVGL coordinate mapping remains

Driver DONE (reset + poll patch, persisted to eMMC, auto-loads). Captured real
taps: X 0-1920, Y 0-720 = the GT911 emits in the LANDSCAPE frame. App restart
detected it (`dashboard: touch: /dev/input/event2`), but button presses do not
register - a coordinate-mapping issue, NOT the driver.

ROOT CAUSE (read from vendored LVGL 9.5.0, not guessed):
- `lv_evdev.c _evdev_process_pointer` scales raw input into `disp->hor_res` x
  `disp->ver_res` = the NATIVE panel frame **720x1920 portrait** (those fields keep
  the created resolution; rotation does not change them).
- `lv_indev.c:743` then calls `lv_display_rotate_point()` -> LVGL rotates the point
  270 itself onto the visible 1920x720 landscape UI.
- So LVGL expects raw touch in the 720x1920 PORTRAIT frame; the GT911 gives 1920x720
  LANDSCAPE. Mismatch -> presses miss. (Rendering is already correct; touch only.)

FIX (app-only; no kernel/DTB/reboot): in `src/main_linux.cpp`, right after
`lv_indev_set_display(touch, disp);`
```c
lv_evdev_set_swap_axes(touch, true);              /* landscape raw -> portrait frame */
lv_evdev_set_calibration(touch, 0, 0, 720, 1920); /* rescale swapped axes 1:1 to native */
```
Then rebuild on the box (`cmake --build build -j1`) + `systemctl restart dashboard`.
If mirrored (touch a corner to see which way), invert that axis by swapping its
min/max in the SAME call:
- X mirrored: `lv_evdev_set_calibration(touch, 720, 0, 0, 1920)`
- Y mirrored: `lv_evdev_set_calibration(touch, 0, 1920, 720, 0)`
- both:      `lv_evdev_set_calibration(touch, 720, 1920, 0, 0)`
lv_evdev API: `lv_evdev_set_swap_axes(indev,bool)`, `lv_evdev_set_calibration(indev,
min_x,min_y,max_x,max_y)` - `_evdev_calibrate` inverts when min>max. Iterate: 2-3
rebuilds max to find the correct corner.

## 2026-09-09 - SOLVED, end to end
GT911 touch fully working on the CM3 and persisted. Driver: reset + poll + driver-name-
rename patch, auto-loads from eMMC at cold boot (verified - `goodix-ws-ts` binds, ID 9271,
polled input2, no INT). App: LVGL coordinate fix, one line pair after `lv_indev_set_display`:
`lv_evdev_set_swap_axes(touch, true); lv_evdev_set_calibration(touch, 0, 0, 720, 1920);`
(aligned on the first try, no mirror needed). Binary + source persisted to eMMC
(/mnt/ro, backups `*.pretouch.*`). The whole GT911-on-CM3 saga is closed.
