//! Camera source: read JPEG frames from the vendored camera helper (`w10-camd`)
//! over a tmpfs shared-memory ring and republish each as `sensor_msgs/CompressedImage`.
//! No HTTP server - frames go straight into ROS topics.
//!
//! Why a helper: driving the camera needs the vendor `libsunxicamera.so`, which
//! must be `dlopen`'d - and a fully static musl binary (like ros2dreame) has no
//! dynamic loader, so it can't. `w10-camd` is a small DYNAMIC binary that does
//! the `dlopen` + capture + JPEG and writes each frame into the shm; ros2dreame
//! stays static and just forwards the frames. Source is vendored under `cam-helper/`.
//!
//! Transport is a single-writer/single-reader seqlock (see cam-helper/ros2cam_shm.h):
//! the writer bumps `seq` odd before touching the frame and even after; we sample
//! `seq` (even + unchanged across the copy) so we never see a torn frame and the
//! writer never blocks on us.

use std::ffi::CString;
use std::sync::atomic::{fence, Ordering};
use std::sync::mpsc::Sender;
use std::thread;
use std::time::Duration;

use crate::msg::{self, CompressedImage, Header};
use crate::tap::Tap;

// Layout mirrors `struct r2c_shm` in cam-helper/ros2cam_shm.h.
const R2C_MAGIC: u32 = 0x5232_4331;
const R2C_MAX_JPEG: usize = 2 * 1024 * 1024;
const OFF_MAGIC: usize = 0;
const OFF_SEQ: usize = 4;
const OFF_SIZE: usize = 8;
const OFF_FRAMES: usize = 24;
const OFF_DATA: usize = 32;
const SHM_LEN: usize = OFF_DATA + R2C_MAX_JPEG;

struct Shm {
    base: *mut u8,
}
// The pointer is into our own mmap; only this thread touches it.
unsafe impl Send for Shm {}

impl Shm {
    #[inline]
    unsafe fn u32(&self, off: usize) -> u32 {
        std::ptr::read_volatile(self.base.add(off) as *const u32)
    }
    #[inline]
    unsafe fn u64(&self, off: usize) -> u64 {
        std::ptr::read_volatile(self.base.add(off) as *const u64)
    }
    /// Consistent read of the latest frame via the seqlock. Returns the JPEG
    /// bytes only if a NEW frame (frames counter advanced) was read cleanly.
    unsafe fn read(&self, last_frames: &mut u64) -> Option<Vec<u8>> {
        let s0 = self.u32(OFF_SEQ);
        if s0 & 1 != 0 {
            return None; // writer in progress
        }
        fence(Ordering::Acquire);
        let frames = self.u64(OFF_FRAMES);
        if frames == *last_frames {
            return None; // nothing new
        }
        let size = self.u32(OFF_SIZE) as usize;
        if size == 0 || size > R2C_MAX_JPEG {
            return None;
        }
        let mut buf = vec![0u8; size];
        std::ptr::copy_nonoverlapping(self.base.add(OFF_DATA), buf.as_mut_ptr(), size);
        fence(Ordering::Acquire);
        if self.u32(OFF_SEQ) != s0 {
            return None; // torn read - writer touched it mid-copy, retry next tick
        }
        *last_frames = frames;
        Some(buf)
    }
}

/// mmap the shm read-only. Returns None until the writer has created it at full
/// size and stamped the magic (so we never SIGBUS on a short/uninitialized file).
fn map_shm(path: &str) -> Option<Shm> {
    let c = CString::new(path).ok()?;
    unsafe {
        let fd = libc::open(c.as_ptr(), libc::O_RDONLY);
        if fd < 0 {
            return None;
        }
        let mut st: libc::stat = std::mem::zeroed();
        if libc::fstat(fd, &mut st) != 0 || (st.st_size as usize) < SHM_LEN {
            libc::close(fd);
            return None;
        }
        let p = libc::mmap(std::ptr::null_mut(), SHM_LEN, libc::PROT_READ, libc::MAP_SHARED, fd, 0);
        libc::close(fd);
        if p == libc::MAP_FAILED {
            return None;
        }
        let shm = Shm { base: p as *mut u8 };
        if shm.u32(OFF_MAGIC) != R2C_MAGIC {
            libc::munmap(p, SHM_LEN);
            return None;
        }
        Some(shm)
    }
}

/// Camera reader thread: shm JPEG ring (from `w10-camd`) -> CompressedImage.
pub fn cam_reader(shm_path: String, frame_id: String, tx: Sender<Tap>) {
    let mut shm: Option<Shm> = None;
    let mut last = 0u64;
    loop {
        if shm.is_none() {
            shm = map_shm(&shm_path);
            if shm.is_none() {
                thread::sleep(Duration::from_millis(500));
                continue;
            }
            log::info!("cam[{frame_id}]: reading shm {shm_path}");
            last = 0;
        }
        let s = shm.as_ref().unwrap();
        match unsafe { s.read(&mut last) } {
            Some(data) => {
                let img = CompressedImage {
                    header: Header { stamp: msg::now(), frame_id: frame_id.clone() },
                    format: "jpeg".into(),
                    data,
                };
                if tx.send(Tap::Image(Box::new(img))).is_err() {
                    return;
                }
            }
            None => thread::sleep(Duration::from_millis(8)),
        }
    }
}
