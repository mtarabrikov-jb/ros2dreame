#!/bin/sh
# ===========================================================================
# Full autonomy, ava OFF - ONE binary. Kills the vendor daemon and runs
# ros2dreame, which opens /dev/ttyS4 (MCU) + /dev/ttyS3 (LDS) itself, drives the
# MCU (MotorCtrl 50Hz + heartbeats + ping/pong, watchdog/clamp/cliff-gate), spins
# the LDS turret, and republishes everything as ROS 2. The camera stack
# (noava-cam: w10-cam -> relay -> go2rtc) is brought up alongside if present.
#
# Freezes BOTH ava watchdogs (sys_monitor.sh ava + rc.d/monitor.sh) so ava does
# not respawn mid-session.
#
#   direct-mode.sh start [rgb|tof|both]   ava off, ros2dreame + cameras (default both)
#   direct-mode.sh restore                stop, bring ava back
#   direct-mode.sh status
# ===========================================================================
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
R2D="${R2D:-$DIR/ros2dreame}"
CAMSH="${CAMSH:-/data/camstream/noava-cam.sh}"

sysmon() { ps 2>/dev/null | grep '[s]ys_monitor.sh ava' | awk '{print $1}'; }
mon()    { ps 2>/dev/null | grep '[r]c.d/monitor.sh'     | awk '{print $1}'; }
freeze() { for p in $(sysmon) $(mon); do kill -STOP "$p" 2>/dev/null; done; }
resume() { for p in $(sysmon) $(mon); do kill -CONT "$p" 2>/dev/null; done; }

case "${1:-status}" in
    start)
        CAM="${2:-both}"
        [ -x "$R2D" ] || { echo "ERROR: $R2D missing (deploy first)"; exit 1; }
        echo ">> freeze BOTH ava watchdogs, kill relay + ava (frees ttyS4/ttyS3)"
        killall avatap-relay 2>/dev/null
        freeze
        killall ava 2>/dev/null
        sleep 1
        mkdir -p /data/log
        echo ">> start cameras ($CAM)"
        [ -x "$CAMSH" ] && sh "$CAMSH" start "$CAM" 2>&1 | tail -2 || echo "   (no camera stack at $CAMSH; skipping)"
        echo ">> start ros2dreame (drives MCU/LDS directly, turret on)"
        IR=""
        { [ "$CAM" = both ] || [ "$CAM" = tof ]; } && IR="W10_CAM_IR=1"
        setsid env RUST_LOG=info $IR "$R2D" >/data/log/ros2dreame.log 2>&1 </dev/null &
        sleep 2
        if pidof ros2dreame >/dev/null; then
            echo ">> UP (ava OFF). /scan /odom /tf + /camera(_ir). Restore: direct-mode.sh restore"
        else
            echo ">> WARN: ros2dreame not up"; tail -8 /data/log/ros2dreame.log
        fi
        ;;
    restore)
        echo ">> stop ros2dreame + cameras, restart ava, resume watchdogs"
        killall ros2dreame 2>/dev/null
        [ -x "$CAMSH" ] && sh "$CAMSH" stop >/dev/null 2>&1
        sleep 1
        /etc/rc.d/ava.sh >/dev/null 2>&1 &
        sleep 3
        resume
        sleep 1
        pidof ava >/dev/null && echo ">> ava back" || echo ">> WARN: ava not up yet"
        ;;
    status)
        for p in ava ros2dreame w10-cam go2rtc; do
            echo -n "$p : "; pidof "$p" || echo none
        done
        ;;
    *) echo "usage: direct-mode.sh start [rgb|tof|both] | restore | status"; exit 1 ;;
esac
