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
    start)
        [ -x "$R2D" ] || { echo "ERROR: $R2D missing (deploy first)"; exit 1; }
        echo ">> stop any prior stack, freeze ava watchdogs, kill relay + ava"
        killall ros2dreame w10-camd avatap-relay ava_cam_relay w10-cam go2rtc 2>/dev/null
        freeze
        killall ava 2>/dev/null
        sleep 1
        mkdir -p /data/log
        IR=""
        if [ -x "$CAMD" ]; then
            echo ">> start camera helper w10-camd (MJPEG :8090 RGB / :8091 IR)"
            setsid "$CAMD" both >/data/log/camd.log 2>&1 </dev/null &
            IR="W10_CAM_IR=1"
            sleep 2
        else
            echo "   (no w10-camd at $CAMD; cameras skipped)"
        fi
        echo ">> start ros2dreame (drives MCU/LDS; reads camera helper)"
        setsid env RUST_LOG=info $IR "$R2D" >/data/log/ros2dreame.log 2>&1 </dev/null &
        sleep 3
        if pidof ros2dreame >/dev/null; then
            echo ">> UP (ava OFF). /scan /odom /tf /camera(_ir). Restore: direct-mode.sh restore"
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
    *) echo "usage: direct-mode.sh start | restore | status"; exit 1 ;;
esac
