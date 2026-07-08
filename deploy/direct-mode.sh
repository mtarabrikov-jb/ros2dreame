#!/bin/sh
# ===========================================================================
# Full autonomy, ava OFF. Brings up the whole "no vendor daemon" stack, all
# built from THIS repo (w10-mcud is vendored here):
#
#   w10-mcud    drives /dev/ttyS4 (MotorCtrl 50Hz + heartbeats + ping/pong,
#               own watchdog/clamp/cliff-gate), forwards ttyS3 LDS, serves
#               telemetry on 7701 (mcu-rx) / 7702 (lds-rx) + control on 7705.
#   noava-cam   the standalone w10-cam -> relay -> go2rtc camera stack.
#   ros2dreame  reads 7701/7702 + go2rtc MJPEG -> ROS 2 topics.
#
# Freezes BOTH ava watchdogs (sys_monitor.sh ava + rc.d/monitor.sh) so ava does
# not respawn mid-session, and enables the LDS turret (silent until told).
#
#   direct-mode.sh start [rgb|tof|both]   ava off, full stack up (default both)
#   direct-mode.sh restore                stop stack, bring ava back
#   direct-mode.sh status
# ===========================================================================
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
MCUD="${MCUD:-$DIR/w10-mcud}"
R2D="${R2D:-$DIR/ros2dreame}"
CAMSH="${CAMSH:-/data/camstream/noava-cam.sh}"
CTRL=7705

sysmon() { ps 2>/dev/null | grep '[s]ys_monitor.sh ava' | awk '{print $1}'; }
mon()    { ps 2>/dev/null | grep '[r]c.d/monitor.sh'     | awk '{print $1}'; }
freeze() { for p in $(sysmon) $(mon); do kill -STOP "$p" 2>/dev/null; done; }
resume() { for p in $(sysmon) $(mon); do kill -CONT "$p" 2>/dev/null; done; }
# Send one control line to w10-mcud (text protocol on 7705).
ctrl() {
    if command -v nc >/dev/null 2>&1; then
        printf '%s\n' "$1" | nc -w1 127.0.0.1 "$CTRL" >/dev/null 2>&1
    else
        printf '%s\n' "$1" > "/dev/tcp/127.0.0.1/$CTRL" 2>/dev/null
    fi
}

case "${1:-status}" in
    start)
        CAM="${2:-both}"
        [ -x "$MCUD" ] || { echo "ERROR: $MCUD missing (deploy first)"; exit 1; }
        [ -x "$R2D" ]  || { echo "ERROR: $R2D missing (deploy first)"; exit 1; }
        echo ">> freeze BOTH ava watchdogs, kill relay + ava"
        killall avatap-relay 2>/dev/null
        freeze
        killall ava 2>/dev/null
        sleep 1
        mkdir -p /data/log
        echo ">> start w10-mcud (ttyS4/ttyS3 -> 7701/7702, control 7705)"
        setsid "$MCUD" >/data/log/mcud.log 2>&1 </dev/null &
        sleep 2
        pidof w10-mcud >/dev/null || { echo "ERROR: w10-mcud not up"; tail -5 /data/log/mcud.log; exit 1; }
        echo ">> enable LDS turret (lidar 1 -> :$CTRL)"
        ctrl "lidar 1"
        echo ">> start cameras ($CAM)"
        [ -x "$CAMSH" ] && sh "$CAMSH" start "$CAM" 2>&1 | tail -2 || echo "   (no camera stack at $CAMSH; skipping)"
        echo ">> start ros2dreame"
        IR=""
        { [ "$CAM" = both ] || [ "$CAM" = tof ]; } && IR="W10_CAM_IR=1"
        setsid env RUST_LOG=info $IR "$R2D" >/data/log/ros2dreame.log 2>&1 </dev/null &
        sleep 2
        if pidof ros2dreame >/dev/null; then
            echo ">> UP (ava OFF). /scan /odom /tf + /camera(_ir). Restore: direct-mode.sh restore"
        else
            echo ">> WARN: ros2dreame not up"; tail -6 /data/log/ros2dreame.log
        fi
        ;;
    restore)
        echo ">> stop ros2dreame + cameras + w10-mcud, restart ava, resume watchdogs"
        killall ros2dreame 2>/dev/null
        [ -x "$CAMSH" ] && sh "$CAMSH" stop >/dev/null 2>&1
        killall w10-mcud 2>/dev/null
        sleep 1
        /etc/rc.d/ava.sh >/dev/null 2>&1 &
        sleep 3
        resume
        sleep 1
        pidof ava >/dev/null && echo ">> ava back" || echo ">> WARN: ava not up yet"
        ;;
    status)
        for p in ava w10-mcud ros2dreame w10-cam go2rtc; do
            echo -n "$p : "; pidof "$p" || echo none
        done
        ;;
    *) echo "usage: direct-mode.sh start [rgb|tof|both] | restore | status"; exit 1 ;;
esac
