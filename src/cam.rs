//! Camera source: read an MJPEG stream over HTTP and republish each frame as
//! `sensor_msgs/CompressedImage` (format "jpeg"). Works the same whether the
//! MJPEG is fed by the vendor `ava` stack or the no-ava standalone `w10-cam`
//! stack - it just reads go2rtc, which is up in both. No image decode: we pass
//! the JPEG bytes straight through, so this is cheap.
//!
//! Minimal hand-rolled HTTP/1.0 GET + JPEG splitter (loopback, no TLS, no HTTP
//! crate): scan the byte stream for SOI (FF D8) .. EOI (FF D9) and emit each
//! complete JPEG. Robust to the multipart/x-mixed-replace headers (they contain
//! no FF D8).

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
            // no start-of-image; keep only a trailing byte (a split FF)
            if acc.len() > 1 {
                acc.drain(0..acc.len() - 1);
            }
            break;
        };
        let Some(eoi) = find(acc, [0xFF, 0xD9], soi + 2) else {
            // frame still incomplete; drop junk before it to bound memory
            if soi > 0 {
                acc.drain(0..soi);
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

fn stream_once(addr: &str, src: &str, frame_id: &str, tx: &Sender<Tap>) -> std::io::Result<()> {
    let mut s = TcpStream::connect(addr)?;
    let req = format!(
        "GET /api/stream.mjpeg?src={src} HTTP/1.0\r\nHost: {addr}\r\nAccept: multipart/x-mixed-replace\r\n\r\n"
    );
    s.write_all(req.as_bytes())?;
    log::info!("cam[{frame_id}]: streaming http://{addr}/api/stream.mjpeg?src={src}");
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

/// Camera reader thread: reconnecting MJPEG -> CompressedImage on `frame_id`.
pub fn cam_reader(addr: String, src: String, frame_id: String, tx: Sender<Tap>) {
    loop {
        if let Err(e) = stream_once(&addr, &src, &frame_id, &tx) {
            log::warn!("cam[{frame_id}]: {e}; reconnect");
        }
        thread::sleep(Duration::from_millis(500));
    }
}
