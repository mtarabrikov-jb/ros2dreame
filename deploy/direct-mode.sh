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
# Freezes BOTH ava watchdogs (sys_monitor.sh ava + rc.d/monitor.sh) so ava does
# not respawn mid-session.
#
#   direct-mode.sh start      ava off, ros2dreame up
#   direct-mode.sh restore    stop, bring ava back
#   direct-mode.sh status
# ===========================================================================
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
R2D="${R2D:-$DIR/ros2dreame}"
CAMD="${CAMD:-$DIR/w10-camd}"   # dynamic camera helper (dlopens libsunxicamera)

sysmon() { ps 2>/dev/null | grep '[s]ys_monitor.sh ava' | awk '{print $1}'; }
mon()    { ps 2>/dev/null | grep '[r]c.d/monitor.sh'     | awk '{print $1}'; }
freeze() { for p in $(sysmon) $(mon); do kill -STOP "$p" 2>/dev/null; done; }
resume() { for p in $(sysmon) $(mon); do kill -CONT "$p" 2>/dev/null; done; }

case "${1:-status}" in
    start|observe)
        # start  = nav mode: drive MCU, spin lidar -> /scan /odom + IR camera.
        #          RGB is dead (vendor firmware kills OV8856 in any active mode).
        # observe = park mode: idle MCU (no drive/nav, turret off) -> RGB camera
        #          + /odom, no /scan. The only state where the OV8856 streams.
        [ -x "$R2D" ] || { echo "ERROR: $R2D missing (deploy first)"; exit 1; }
        OBS=""; [ "$1" = observe ] && OBS="W10_OBSERVE=1"
        echo ">> stop any prior stack, freeze ava watchdogs, kill relay + ava"
        killall ros2dreame w10-camd avatap-relay ava_cam_relay w10-cam go2rtc 2>/dev/null
        freeze
        killall ava 2>/dev/null
        # ava MUST be fully dead before we open a camera: while it lives it holds
        # video1/video2 and runs the RGB pipeline on isp0, which corrupts our
        # capture (kernel logs "video1 open busy" + "isp0 frame error" and the
        # frame is pure noise). This was THE cause of the long "ToF only streams
        # noise" red herring. Wait for it to exit, then force-kill as a backstop.
        for i in 1 2 3 4 5; do pidof ava >/dev/null 2>&1 || break; sleep 1; done
        killall -9 ava 2>/dev/null
        sleep 1
        mkdir -p /data/log
        IR=""
        if [ -x "$CAMD" ]; then
            # ONE camera per mode (RGB=isp0, ToF=isp1 - separate ISPs). Both give a
            # clean image with ava dead: observe -> RGB (/camera), nav -> ToF
            # (/camera_ir, structured-light IR). RGB additionally needs the OV8856
            # primed (by ava or a reboot) before the first parked capture.
            if [ "$1" = observe ]; then CAMMODE=rgb; else CAMMODE=tof; IR="W10_CAM_IR=1"; fi
            echo ">> start camera helper w10-camd ($CAMMODE)"
            setsid "$CAMD" "$CAMMODE" >/data/log/camd.log 2>&1 </dev/null &
            sleep 2
        else
            echo "   (no w10-camd at $CAMD; cameras skipped)"
        fi
        echo ">> start ros2dreame ($1)"
        setsid env RUST_LOG=info $OBS $IR "$R2D" >/data/log/ros2dreame.log 2>&1 </dev/null &
        sleep 3
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
    restore)
        echo ">> stop ros2dreame + camera helper, restart ava, resume watchdogs"
        killall ros2dreame w10-camd 2>/dev/null
        sleep 1
        /etc/rc.d/ava.sh >/dev/null 2>&1 &
        sleep 3
        resume
        sleep 1
        pidof ava >/dev/null && echo ">> ava back" || echo ">> WARN: ava not up yet"
        ;;
    status)
        for p in ava ros2dreame w10-camd; do
            echo -n "$p : "; pidof "$p" || echo none
        done
        ;;
    *) echo "usage: direct-mode.sh start | observe | restore | status"; exit 1 ;;
esac
