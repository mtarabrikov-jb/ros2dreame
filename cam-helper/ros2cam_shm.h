// ---------------------------------------------------------------------------
// Shared JPEG frame buffer between w10-camd (dynamic camera helper, the writer)
// and ros2dreame (static-musl ROS node, the reader). Backed by a file in tmpfs
// (/tmp -> RAM). This replaces the old MJPEG-over-HTTP transport: frames now go
// straight into ROS as sensor_msgs/CompressedImage, no HTTP server.
//
// Single-writer / single-reader seqlock: the writer bumps `seq` to odd before
// touching the frame and to even after; the reader samples `seq` (must be even
// and unchanged across the copy), so the writer NEVER blocks on the reader.
// Offsets are fixed and mirrored in ros2dreame's src/cam.rs.
// ---------------------------------------------------------------------------
#ifndef ROS2CAM_SHM_H
#define ROS2CAM_SHM_H

#include <stdint.h>

#define R2C_MAGIC     0x52324331u        /* sentinel ("R2C1") */
#define R2C_MAX_JPEG  (2 * 1024 * 1024)  /* generous ceiling for one JPEG */

struct r2c_shm {
    volatile uint32_t magic;   // 0   R2C_MAGIC once initialized
    volatile uint32_t seq;     // 4   seqlock: odd = write in progress
    volatile uint32_t size;    // 8   JPEG byte length in data[] (0 = none yet)
    volatile uint32_t width;   // 12  encoded image width  (info only)
    volatile uint32_t height;  // 16  encoded image height (info only)
    volatile uint32_t _pad;    // 20  keep `frames` 8-byte aligned
    volatile uint64_t frames;  // 24  frames published (bumps every new frame)
    uint8_t  data[R2C_MAX_JPEG]; // 32
};

#endif
