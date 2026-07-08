//! Camera source: read MJPEG from the vendored camera helper (`w10-camd`) over
//! loopback and republish each JPEG as `sensor_msgs/CompressedImage`. No go2rtc.
//!
//! Why a helper: driving the camera needs the vendor `libsunxicamera.so`, which
//! must be `dlopen`'d - and a fully static musl binary (like ros2dreame) has no
//! dynamic loader, so it can't. `w10-camd` is a small DYNAMIC binary that does
//! the `dlopen` + capture + JPEG and serves MJPEG on loopback; ros2dreame stays
//! static and just forwards the frames. Source is vendored under `cam-helper/`.
//!
//! Minimal hand-rolled HTTP/1.0 GET + JPEG splitter (loopback, no TLS, no HTTP
//! crate): scan the byte stream for SOI (FF D8) .. EOI (FF D9) and emit each
//! complete JPEG. Robust to the multipart/x-mixed-replace headers (no FF D8).

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::mpsc::Sender;
use std::thread;
use std::time::Duration;

use crate::msg::{self, CompressedImage, Header};
use crate::tap::Tap;

const MAX_ACC: usize = 4 * 1024 * 1024; // bound memory if no EOI ever comes

fn find(hay: &[u8], needle: [u8; 2], from: usize) -> Option<usize> {
    if hay.len() < 2 {
        return None;
    }
    (from..hay.len() - 1).find(|&i| hay[i] == needle[0] && hay[i + 1] == needle[1])
}

/// Pull all complete JPEGs out of `acc`, leaving the remainder for next read.
fn extract_jpegs(acc: &mut Vec<u8>) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    loop {
        let Some(soi) = find(acc, [0xFF, 0xD8], 0) else {
            if acc.len() > 1 {
                acc.drain(0..acc.len() - 1); // keep a trailing split FF
            }
            break;
        };
        let Some(eoi) = find(acc, [0xFF, 0xD9], soi + 2) else {
            if soi > 0 {
                acc.drain(0..soi); // drop junk before the incomplete frame
            }
            if acc.len() > MAX_ACC {
                acc.clear();
            }
            break;
        };
        out.push(acc[soi..eoi + 2].to_vec());
        acc.drain(0..eoi + 2);
    }
    out
}

fn stream_once(addr: &str, frame_id: &str, tx: &Sender<Tap>) -> std::io::Result<()> {
    let mut s = TcpStream::connect(addr)?;
    s.write_all(format!("GET /stream HTTP/1.0\r\nHost: {addr}\r\n\r\n").as_bytes())?;
    log::info!("cam[{frame_id}]: streaming MJPEG from {addr}");
    let mut buf = [0u8; 32768];
    let mut acc: Vec<u8> = Vec::with_capacity(128 * 1024);
    loop {
        let n = s.read(&mut buf)?;
        if n == 0 {
            return Ok(());
        }
        acc.extend_from_slice(&buf[..n]);
        for jpeg in extract_jpegs(&mut acc) {
            let img = CompressedImage {
                header: Header { stamp: msg::now(), frame_id: frame_id.into() },
                format: "jpeg".into(),
                data: jpeg,
            };
            if tx.send(Tap::Image(Box::new(img))).is_err() {
                return Ok(());
            }
        }
    }
}

/// Camera reader thread: reconnecting MJPEG (from `w10-camd`) -> CompressedImage.
pub fn cam_reader(addr: String, frame_id: String, tx: Sender<Tap>) {
    loop {
        if let Err(e) = stream_once(&addr, &frame_id, &tx) {
            log::warn!("cam[{frame_id}]: {e}; reconnect");
        }
        thread::sleep(Duration::from_millis(500));
    }
}
