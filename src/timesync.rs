//! Built-in clock sync. ROS 2 tf2 rejects transforms whose time is outside its
//! buffer, so if the robot clock differs from the companion host by more than ~1 s,
//! the host's tf2 drops the robot's `/scan` + `/tf` ("timestamp earlier than all
//! the data in the transform cache"), SLAM never maps, and Nav2 never activates.
//!
//! The Dreame has no NTP client (busybox only) and its clock drifts (~28 ppm) and
//! boots seconds off. So ros2dreame syncs it itself:
//!   - `initial_step()` runs BEFORE the DDS participant is created and hard-STEPS
//!     the clock (settimeofday). Doing it pre-DDS means RustDDS starts with the
//!     correct time and never sees a jump.
//!   - `spawn_daemon()` then SLEWS the clock (adjtime) every `PERIOD` - the kernel
//!     nudges the tick rate, time never moves backward, so ROS/DDS timestamps stay
//!     monotonic. This is why we do NOT keep calling `date -s`/settimeofday: a
//!     running clock that steps (especially backward) desyncs the RustDDS<->FastDDS
//!     participants and drops robot<->container comms.
//!
//! Time source: the robot has internet by IP but no DNS, so we hit public NTP
//! anycast IPs directly (Cloudflare + Google). The host is NTP-synced to the same
//! UTC, so robot == UTC == host. Offset is the standard 4-timestamp NTP calc (RTT
//! compensated, sub-ms), median across the servers that answer. Disable the whole
//! thing with `W10_NO_TIMESYNC`.

use std::net::UdpSocket;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const NTP_UNIX_DELTA: f64 = 2_208_988_800.0; // seconds between the 1900 and 1970 epochs
// Public NTP anycast IPs (the robot has no DNS): Cloudflare time, Google time.
const SERVERS: &[&str] = &["162.159.200.123", "162.159.200.1", "216.239.35.0", "216.239.35.4"];
const STEP_THRESHOLD: f64 = 0.02; // pre-DDS: step to get within ~20 ms, then slew holds
const PERIOD: Duration = Duration::from_secs(64);

fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64()
}

/// Query one SNTP server; offset in seconds (positive = our clock is behind).
fn query(server: &str, timeout: Duration) -> Option<f64> {
    let sock = UdpSocket::bind("0.0.0.0:0").ok()?;
    sock.set_read_timeout(Some(timeout)).ok()?;
    let mut req = [0u8; 48];
    req[0] = 0x1b; // LI=0, VN=3, Mode=3 (client)
    let t1 = unix_now();
    sock.send_to(&req, (server, 123)).ok()?;
    let mut buf = [0u8; 48];
    let (n, _) = sock.recv_from(&mut buf).ok()?;
    let t4 = unix_now();
    if n < 48 {
        return None;
    }
    let rd = |o: usize| u32::from_be_bytes([buf[o], buf[o + 1], buf[o + 2], buf[o + 3]]) as f64;
    let ntp = |s: usize, f: usize| rd(s) - NTP_UNIX_DELTA + rd(f) / 4_294_967_296.0;
    let t2 = ntp(32, 36); // server receive timestamp
    let t3 = ntp(40, 44); // server transmit timestamp
    if t2 <= 0.0 || t3 <= 0.0 {
        return None; // KoD / unsynced server
    }
    Some(((t2 - t1) + (t3 - t4)) / 2.0)
}

/// Median offset across the servers that answer, or the first if `first_only`.
fn sample(timeout: Duration, first_only: bool) -> Option<f64> {
    if first_only {
        return SERVERS.iter().find_map(|s| query(s, timeout));
    }
    let mut offs: Vec<f64> = SERVERS.iter().filter_map(|s| query(s, timeout)).collect();
    if offs.is_empty() {
        return None;
    }
    offs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Some(offs[offs.len() / 2])
}

fn timeval(sec: f64) -> libc::timeval {
    libc::timeval {
        tv_sec: sec.trunc() as libc::time_t,
        tv_usec: ((sec - sec.trunc()) * 1e6) as libc::suseconds_t,
    }
}

/// Hard step (settimeofday). Only used pre-DDS, where a jump is harmless.
fn step(offset: f64) {
    let tv = timeval(unix_now() + offset); // absolute target time (always positive)
    unsafe {
        libc::settimeofday(&tv, std::ptr::null());
    }
}

/// Gradual slew (adjtime). Never moves time backward - safe with DDS/ROS running.
fn slew(offset: f64) {
    let tv = timeval(offset); // signed delta
    unsafe {
        libc::adjtime(&tv, std::ptr::null_mut());
    }
}

fn enabled() -> bool {
    std::env::var_os("W10_NO_TIMESYNC").is_none()
}

/// Sync the clock to UTC BEFORE the DDS participant exists. Best-effort and fast
/// (short timeout, first responder) so a missing internet only delays startup a
/// little. Steps if off by more than ~20 ms.
pub fn initial_step() {
    if !enabled() {
        return;
    }
    match sample(Duration::from_millis(1200), true) {
        Some(off) if off.abs() > 3600.0 => log::warn!("timesync: absurd offset {off:+.1}s, ignoring"),
        Some(off) if off.abs() > STEP_THRESHOLD => {
            step(off);
            log::info!("timesync: stepped clock {off:+.3}s to UTC (pre-DDS)");
        }
        Some(off) => log::info!("timesync: clock already within {off:+.3}s of UTC"),
        None => log::warn!("timesync: no NTP response at startup - clock left as-is (set W10_NO_TIMESYNC to silence)"),
    }
}

/// Background thread: keep the clock on UTC by slewing (never stepping).
pub fn spawn_daemon() {
    if !enabled() {
        return;
    }
    std::thread::spawn(|| loop {
        std::thread::sleep(PERIOD);
        if let Some(off) = sample(Duration::from_secs(2), false) {
            if off.abs() <= 3600.0 {
                slew(off);
                log::debug!("timesync: slew {off:+.3}s");
            }
        }
    });
}
