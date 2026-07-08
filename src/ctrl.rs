//! Control client to `w10-mcud` (direct mode, ava off): text protocol on TCP
//! 7705. On connect we enable the LDS turret (`lidar 1`) so `/scan` has data -
//! the turret is silent until told. The connection is then held open (future
//! `/cmd_vel` will drive through it); on drop we reconnect and re-enable.
//!
//! Only started when `W10_CTRL_ADDR` is set (i.e. direct mode) - in tap mode the
//! vendor `ava` owns the LDS and there is nothing to enable.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::thread;
use std::time::Duration;

pub fn ctrl_client(addr: String) {
    loop {
        match TcpStream::connect(&addr) {
            Ok(mut s) => {
                log::info!("ctrl: connected to {addr}; enabling LDS turret (lidar 1)");
                if s.write_all(b"lidar 1\n").is_err() {
                    thread::sleep(Duration::from_millis(500));
                    continue;
                }
                // Hold the connection open; read and discard any replies. When it
                // drops (0 bytes or error) we fall through and reconnect, which
                // re-sends `lidar 1`.
                let mut buf = [0u8; 256];
                loop {
                    match s.read(&mut buf) {
                        Ok(0) | Err(_) => break,
                        Ok(_) => {}
                    }
                }
                log::warn!("ctrl: {addr} closed; reconnecting");
            }
            Err(e) => log::warn!("ctrl: connect {addr}: {e}; retry"),
        }
        thread::sleep(Duration::from_millis(500));
    }
}
