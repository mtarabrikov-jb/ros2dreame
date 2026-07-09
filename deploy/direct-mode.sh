#!/bin/sh
# ===========================================================================
# Full autonomy, ava OFF - ONE binary, ZERO extra processes. Kills the vendor
# daemon and runs ros2dreame, which then does everything itself:
#   - opens /dev/ttyS4 (MCU) + /dev/ttyS3 (LDS), drives the MCU (MotorCtrl 50Hz
#     + heartbeats + ping/pong, watchdog/clamp/cliff-gate), spins the turret;
#   - drives the RGB camera via libsunxicamera and JPEG-encodes it in-process
#     (no w10-cam / ava_cam_relay / go2rtc);
#   - republishes everything as ROS 2.
# The only runtime dependency is the vendor /usr/lib/libsunxicamera.so (dlopen'd).
#
# Freezes the ava reboot+respawn watchdogs so ava stays off for the session (see
# ava_off). CRITICAL: the vendor /etc/rc.d/monitor.sh REBOOTS the robot (then
# factory-resets it) if ava is not alive - freezing it is what makes ava safe to
# stop at all.
#
#   direct-mode.sh start      ava off, ros2dreame up
#   direct-mode.sh restore    stop, bring ava back
#   direct-mode.sh status
# ===========================================================================
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
R2D="${R2D:-$DIR/ros2dreame}"
CAMD="${CAMD:-$DIR/w10-camd}"   # dynamic camera helper (dlopens libsunxicamera)
AVA_RESTART_MARK=/tmp/restart_ava.mark

# --- keep ava OFF safely -----------------------------------------------------
# The vendor firmware fights an absent ava HARD: /etc/rc.d/monitor.sh probes ava
# with `avacmd media status_get` and, after 3 failed probes, REBOOTS the robot -
# and factory-resets it ("monitor_rescue_brick") if it is still down after the
# reboot. So ava can only be stopped if monitor.sh is stopped too. We freeze
# (kill -STOP) the whole ava reboot+respawn set; each is an init child and init
# does not respawn a *stopped* child, so the freeze holds for the whole session:
#   monitor.sh       - the rebooter/factory-resetter (THE safety-critical freeze)
#   exec_monitor.sh  \ ava launcher chain: exec_monitor restarts exec_proc, which
#   exec_proc        / respawns ava - freeze both so ava does not come back and
#                      grab ttyS4 + video1/video2 out from under ros2dreame
#   sys_monitor.sh   - ava memory/status monitor
# (A "bind a stub over /usr/bin/ava" approach was WORSE: it left monitor.sh
# running, the stub failed the health probe, and it triggered exactly the
# reboot+factory-reset above. Freeze monitor.sh; never stub.)
WD_COMMS='monitor.sh exec_monitor.sh exec_proc sys_monitor.sh'
wd_pids() { for d in /proc/[0-9]*; do for c in $WD_COMMS; do
    [ "$(cat "$d/comm" 2>/dev/null)" = "$c" ] && echo "${d##*/}"; done; done; }
mon_frozen() {   # is monitor.sh (the rebooter) actually stopped (state T)?
    for d in /proc/[0-9]*; do [ "$(cat "$d/comm" 2>/dev/null)" = monitor.sh ] || continue
        [ "$(cut -d' ' -f3 "$d/stat" 2>/dev/null)" = T ] || return 1; done; return 0; }

ava_off() {
    for p in $(wd_pids); do kill -STOP "$p" 2>/dev/null; done
    if ! mon_frozen; then
        echo ">> FATAL: could not freeze monitor.sh (it would REBOOT the robot). Aborting."
        ava_on; exit 1
    fi
    killall -9 ava avacmd 2>/dev/null   # safe now: monitor.sh is stopped -> no reboot
    sleep 1
}
ava_on() {
    touch "$AVA_RESTART_MARK" 2>/dev/null   # tell monitor.sh this downtime was intentional (no reboot)
    for p in $(wd_pids); do kill -CONT "$p" 2>/dev/null; done
    killall -9 ava 2>/dev/null
    [ -x /etc/rc.d/ava.sh ] && /etc/rc.d/ava.sh >/dev/null 2>&1 &
}

case "${1:-status}" in
    start|observe)
        # start  = nav mode: drive MCU, spin lidar -> /scan /odom + IR camera.
        #          RGB stalls here - the spinning LDS turret disrupts the OV8856
        #          MIPI (isp0 frame errors) and wedges it until an ava reprime or
        #          reboot; it is NOT "any active mode" (nav with the turret off
        #          streams RGB fine). See docs/MCU.md.
        # observe = park mode: idle MCU (no drive/nav, turret off) -> RGB camera
        #          + /odom, no /scan.
        [ -x "$R2D" ] || { echo "ERROR: $R2D missing (deploy first)"; exit 1; }
        OBS=""; [ "$1" = observe ] && OBS="W10_OBSERVE=1"
        echo ">> stop any prior stack, force ava OFF (freeze reboot+respawn watchdogs)"
        killall ros2dreame w10-camd avatap-relay ava_cam_relay w10-cam go2rtc 2>/dev/null
        # ava MUST be fully dead before we open a camera: while it lives it holds
        # video1/video2 and runs the RGB pipeline on isp0, which corrupts our
        # capture (kernel logs "video1 open busy" + "isp0 frame error" and the
        # frame is pure noise). This was THE cause of the long "ToF only streams
        # noise" red herring. ava_off freezes monitor.sh first (else it reboots
        # the robot when ava goes away), then the launcher chain, then kills ava.
        ava_off
        sleep 1
        mkdir -p /data/log
        IR=""; [ "$1" = observe ] || IR="W10_CAM_IR=1"
        # ORDER MATTERS in observe: ros2dreame drives the MCU and emits the
        # camera-AI-reset frame 0x1d [0x05,0x00], which un-wedges a (previously
        # turret-wedged) RGB isp0 - but only while the camera is still CLOSED and
        # then re-opened. So start ros2dreame FIRST, let the reset land, THEN open
        # w10-camd on the now-clean isp0. This is what makes RGB recoverable
        # off-dock (no ava/reboot reprime). nav uses the ToF/isp1 path (no reset).
        echo ">> start ros2dreame ($1)"
        setsid env RUST_LOG=info $OBS $IR "$R2D" >/data/log/ros2dreame.log 2>&1 </dev/null &
        if [ "$1" = observe ]; then sleep 3; else sleep 1; fi
        if [ -x "$CAMD" ]; then
            # ONE camera per mode (RGB=isp0, ToF=isp1 - separate ISPs), ava dead:
            # observe -> RGB (/camera), nav -> ToF (/camera_ir, structured-light IR).
            if [ "$1" = observe ]; then CAMMODE=rgb; else CAMMODE=tof; fi
            echo ">> start camera helper w10-camd ($CAMMODE)"
            setsid "$CAMD" "$CAMMODE" >/data/log/camd.log 2>&1 </dev/null &
            sleep 3
        else
            echo "   (no w10-camd at $CAMD; cameras skipped)"
        fi
        if pidof ros2dreame >/dev/null; then
            if [ "$1" = observe ]; then
                echo ">> UP (ava OFF, OBSERVE). /odom /tf /camera (RGB), no /scan. Restore: direct-mode.sh restore"
            else
                echo ">> UP (ava OFF, nav). /scan /odom /tf /camera_ir. Restore: direct-mode.sh restore"
            fi
        else
            echo ">> WARN: ros2dreame not up"; tail -8 /data/log/ros2dreame.log
        fi
        ;;
    auto)
        # Auto mode: ros2dreame switches turret + cameras with motion. Driving
        # (fresh /cmd_vel) -> turret on -> /scan + IR; idle -> turret off, RGB
        # un-wedge reset (0x1d 05 00), both cameras (RGB + IR). ros2dreame OWNS the
        # w10-camd helper here (starts/stops tof<->both), so we do NOT start it -
        # W10_CAMD just tells ros2dreame where the helper binary is.
        [ -x "$R2D" ] || { echo "ERROR: $R2D missing (deploy first)"; exit 1; }
        echo ">> stop any prior stack, force ava OFF (freeze reboot+respawn watchdogs)"
        killall ros2dreame w10-camd avatap-relay ava_cam_relay w10-cam go2rtc 2>/dev/null
        ava_off
        sleep 1
        mkdir -p /data/log
        echo ">> start ros2dreame (auto; owns w10-camd)"
        setsid env RUST_LOG=info W10_AUTO=1 W10_CAMD="$CAMD" "$R2D" >/data/log/ros2dreame.log 2>&1 </dev/null &
        sleep 3
        if pidof ros2dreame >/dev/null; then
            echo ">> UP (ava OFF, AUTO). drive -> /scan + IR; stop -> /camera (RGB) + /camera_ir. Restore: direct-mode.sh restore"
        else
            echo ">> WARN: ros2dreame not up"; tail -8 /data/log/ros2dreame.log
        fi
        ;;
    restore)
        echo ">> stop ros2dreame + camera helper, restore real ava, resume watchdogs"
        killall ros2dreame w10-camd 2>/dev/null
        sleep 1
        ava_on
        sleep 5
        if pidof ava >/dev/null; then echo ">> ava back"; else echo ">> WARN: ava not up yet (retry: direct-mode.sh restore)"; fi
        ;;
    status)
        mon_frozen && echo "monitor.sh : FROZEN (ava reboot-watchdog held off)"
        for p in ava ros2dreame w10-camd; do
            echo -n "$p : "; pidof "$p" || echo none
        done
        ;;
    *) echo "usage: direct-mode.sh start | observe | auto | restore | status"; exit 1 ;;
esac
