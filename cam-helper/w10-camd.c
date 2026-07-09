// ---------------------------------------------------------------------------
// w10-camd - the ros2dreame camera helper (ava OFF). A small DYNAMIC binary
// that drives the Dreame W10 cameras via the vendor libsunxicamera.so (dlopen -
// which the static-musl ros2dreame binary can't do), JPEG-encodes each frame in
// process, and publishes it into a tmpfs shared-memory ring (seqlock) that
// ros2dreame reads and forwards as ROS 2 CompressedImage. No HTTP server, no
// go2rtc - frames go straight into ROS topics.
//
//   RGB   : OV8856   /dev/video2, NV21 672x504  -> JPEG -> /tmp/ros2cam.shm
//   IR/ToF: ofilm0092/dev/video1, BG12 224x1558 -> gray -> JPEG -> /tmp/ros2cam_ir.shm
//
//   w10-camd [rgb|tof|both]     (default both)
//   env CAM_SHM / CAM_SHM_IR override the shm paths.
//
// The only external dependency is /usr/lib/libsunxicamera.so (the Allwinner ISP
// driver, part of the robot OS) - dlopen'd, not vendorable.
// ---------------------------------------------------------------------------
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <dlfcn.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include "ir_process.h"
#include "jpeg_gray.h"
#include "ros2cam_shm.h"

// --- vendor libsunxicamera SunxiCam (mangled C++ symbols; verified by RE) -----
#define SYM_OPEN   "_ZN9sunxi_cam8SunxiCam10OpenCameraEiiiii"
#define SYM_FRAME  "_ZN9sunxi_cam8SunxiCam13GetImageFrameEPNS_10ImageFrameE"
#define SYM_RETURN "_ZN9sunxi_cam8SunxiCam16ReturnImageFrameEPNS_10ImageFrameE"
#define SYM_CLOSE  "_ZN9sunxi_cam8SunxiCam11CloseCameraEv"
#define IMGFRAME_DATA_OFF 0x20
typedef int (*open_fn)(void *self, int idx, int fourcc, int a3, int w, int h);
typedef int (*frame_fn)(void *self, void *imgframe);
typedef int (*close_fn)(void *self);
static open_fn  Open;
static frame_fn Get, Return;
static close_fn Close;

static uint32_t fourcc(const char *s) {
    return (uint32_t)s[0] | ((uint32_t)s[1] << 8) | ((uint32_t)s[2] << 16) | ((uint32_t)s[3] << 24);
}

// --- ToF media-controller pipeline (mirrors noava-cam.sh tof_pipeline) --------
static void media_link(const char *dev, int se, int sp, int de, int dp, int en) {
    int fd = open(dev, O_RDWR);
    if (fd < 0) return;
    struct media_link_desc l;
    memset(&l, 0, sizeof l);
    l.source.entity = se; l.source.index = sp;
    l.sink.entity = de;   l.sink.index = dp;
    l.flags = en ? MEDIA_LNK_FL_ENABLED : 0;
    if (ioctl(fd, MEDIA_IOC_SETUP_LINK, &l) < 0)
        fprintf(stderr, "media_link e%d/%d->e%d/%d: %s\n", se, sp, de, dp, strerror(errno));
    close(fd);
}
static void subdev_fmt(int subdev, int pad, int w, int h, unsigned code) {
    char path[64];
    snprintf(path, sizeof path, "/dev/v4l-subdev%d", subdev);
    int fd = open(path, O_RDWR);
    if (fd < 0) return;
    struct v4l2_subdev_format sf;
    memset(&sf, 0, sizeof sf);
    sf.pad = pad;
    sf.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sf.format.width = w; sf.format.height = h; sf.format.code = code;
    sf.format.field = 1; // V4L2_FIELD_NONE
    if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &sf) < 0)
        fprintf(stderr, "subdev%d pad%d %dx%d/0x%x: %s\n", subdev, pad, w, h, code, strerror(errno));
    close(fd);
}
static void tof_pipeline(void) {
    media_link("/dev/media0", 1, 0, 32, 0, 1);   // ofilm0092 -> mipi.0
    media_link("/dev/media0", 26, 1, 44, 0, 1);  // csi.0 -> isp1
    int sd[8][2] = {{7,0},{7,1},{5,0},{5,1},{11,0},{11,2},{14,0},{14,1}};
    for (int i = 0; i < 8; i++)
        subdev_fmt(sd[i][0], sd[i][1], TOF_W, TOF_H, 0x3011);
}

// --- one camera: the capture loop encodes each frame to JPEG and publishes it
// into the shared-memory seqlock ring that ros2dreame reads (no HTTP server).
struct cam {
    int idx, w, h, tof;
    uint32_t fourcc;
    const char *shm_path;
    struct r2c_shm *shm;
};

// Create/attach the tmpfs JPEG ring this camera writes into.
static struct r2c_shm *open_shm(const char *path) {
    int fd = open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) return NULL;
    if (ftruncate(fd, sizeof(struct r2c_shm)) != 0) { close(fd); return NULL; }
    void *p = mmap(NULL, sizeof(struct r2c_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) return NULL;
    struct r2c_shm *s = p;
    s->seq = 0; s->size = 0; s->frames = 0; s->magic = R2C_MAGIC;
    return s;
}

// --- ToF (video1) raw-V4L2 path -------------------------------------------
// libsunxicamera's OpenCamera is built for the RGB camera (isp0/ov8856); on the
// ToF path (isp1/ofilm0092) it opens video1 but the structured-light VCSEL
// projector stays off -> the frame is pure noise. ava's node_tof.so instead
// drives video1 with raw V4L2 and, crucially, writes 9 registers to the ToF
// sensor on /dev/i2c-2 @0x3d (VCSEL/integration enable) BETWEEN REQBUFS and
// STREAMON. We replay that exact sequence (captured with an LD_PRELOAD i2c snoop
// of ava). Register wire format: [reg16 BE][val16 BE].
#define I2C_RDWR_ 0x0707
struct i2c_msg_ { unsigned short addr, flags, len; unsigned char *buf; };
struct i2c_rdwr_data_ { struct i2c_msg_ *msgs; unsigned int nmsgs; };
static void tof_sensor_enable(void) {
    static unsigned char seq[9][4] = {
        {0x90, 0x02, 0x5e, 0xfb}, {0x90, 0x04, 0x5e, 0xfb}, {0x90, 0x06, 0x5e, 0xfb}, {0x90, 0x08, 0x5e, 0xfb},
        {0x90, 0x0a, 0x57, 0x3c}, {0x90, 0x0c, 0x57, 0x3c}, {0x90, 0x0e, 0x57, 0x3c}, {0x90, 0x10, 0x57, 0x3c},
        {0x94, 0x02, 0x00, 0x01},
    };
    int fd = open("/dev/i2c-2", O_RDWR);
    if (fd < 0) { fprintf(stderr, "camd: tof i2c-2 open: %s\n", strerror(errno)); return; }
    int ok = 0;
    for (int i = 0; i < 9; i++) {
        struct i2c_msg_ m = { .addr = 0x3d, .flags = 0, .len = 4, .buf = seq[i] };
        struct i2c_rdwr_data_ d = { .msgs = &m, .nmsgs = 1 };
        if (ioctl(fd, I2C_RDWR_, &d) == 1) ok++;
    }
    close(fd);
    fprintf(stderr, "camd: tof sensor enable: %d/9 regs ACKed\n", ok);
}

static void publish_jpeg(struct cam *c, const unsigned char *jpg, int n, int ow, int oh,
                         uint64_t *frames, uint64_t misses) {
    if (n <= 0 || (size_t)n > R2C_MAX_JPEG || !c->shm) return;
    struct r2c_shm *s = c->shm;
    s->seq++; __sync_synchronize();
    s->size = (uint32_t)n; s->width = (uint32_t)ow; s->height = (uint32_t)oh;
    memcpy(s->data, jpg, n);
    s->frames++;
    __sync_synchronize(); s->seq++;
    if (++(*frames) % 60 == 0)
        fprintf(stderr, "camd: video%d %llu frames, %llu misses\n",
                c->idx, (unsigned long long)*frames, (unsigned long long)misses);
}

// JPEG quality, env-tunable (W10_JPEG_Q, default 80). Lower = smaller frames =
// less WiFi/RTPS bandwidth so incoming /cmd_vel is not starved by the two
// camera streams (RustDDS has one network thread; big JPEG fragments block it).
static int jpeg_q(void) {
    static int q = 0;
    if (!q) { const char *e = getenv("W10_JPEG_Q"); q = e ? atoi(e) : 80; if (q < 1 || q > 99) q = 80; }
    return q;
}

#define TOF_NBUF 4
static void capture_tof(struct cam *c) {
    // W10_TOF_NOPIPE: skip re-running the media graph setup (links + subdev
    // formats) - ava does NOT re-run it per activation (relies on the persisted
    // graph), and re-running subdev_setfmt may reset the sensor's streaming
    // state. W10_TOF_NOI2C: skip our 9-register sensor write (inherit ava's
    // persisted VCSEL config instead). For A/B testing the ToF bring-up.
    if (!getenv("W10_TOF_NOPIPE")) tof_pipeline();
    int fd = open("/dev/video1", O_RDWR);
    if (fd < 0) { fprintf(stderr, "camd: open video1: %s\n", strerror(errno)); return; }
    // ava's node_tof.so order: S_INPUT(0) then S_PARM then S_FMT. S_INPUT selects
    // and powers the ToF sensor input - without it the sensor stays unpowered
    // (i2c writes NACK) and no frames dequeue.
    int input = 0;
    if (ioctl(fd, VIDIOC_S_INPUT, &input) < 0) fprintf(stderr, "camd: tof S_INPUT(0): %s\n", strerror(errno));
    struct v4l2_streamparm parm; memset(&parm, 0, sizeof parm);
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = 10;   // ava uses 1/10 (snooped)
    ioctl(fd, VIDIOC_S_PARM, &parm);   // best-effort framerate
    struct v4l2_format fmt; memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = TOF_W; fmt.fmt.pix_mp.height = TOF_H;
    fmt.fmt.pix_mp.pixelformat = c->fourcc; fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    fmt.fmt.pix_mp.num_planes = 1;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { fprintf(stderr, "camd: tof S_FMT: %s\n", strerror(errno)); close(fd); return; }
    struct v4l2_requestbuffers req; memset(&req, 0, sizeof req);
    req.count = TOF_NBUF; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) { fprintf(stderr, "camd: tof REQBUFS: %s\n", strerror(errno)); close(fd); return; }
    void *bp[TOF_NBUF];
    for (unsigned i = 0; i < req.count && i < TOF_NBUF; i++) {
        struct v4l2_plane pl; memset(&pl, 0, sizeof pl);
        struct v4l2_buffer b; memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        b.m.planes = &pl; b.length = 1;
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { fprintf(stderr, "camd: tof QUERYBUF: %s\n", strerror(errno)); close(fd); return; }
        bp[i] = mmap(NULL, pl.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pl.m.mem_offset);
        if (bp[i] == MAP_FAILED) { fprintf(stderr, "camd: tof mmap: %s\n", strerror(errno)); close(fd); return; }
        struct v4l2_plane qp; memset(&qp, 0, sizeof qp);
        struct v4l2_buffer qb; memset(&qb, 0, sizeof qb);
        qb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; qb.memory = V4L2_MEMORY_MMAP; qb.index = i;
        qb.m.planes = &qp; qb.length = 1;
        if (ioctl(fd, VIDIOC_QBUF, &qb) < 0) { fprintf(stderr, "camd: tof QBUF: %s\n", strerror(errno)); close(fd); return; }
    }
    if (!getenv("W10_TOF_NOI2C")) tof_sensor_enable();   // VCSEL/integration enable BEFORE streamon
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (getenv("W10_TOF_DBL")) {
        // ava runs the S_INPUT/S_PARM/S_FMT/STREAMON cycle TWICE (snooped); the
        // first streamon may arm the VCSEL<->sensor structured-light sync and the
        // second captures. Mimic with a streamon/streamoff/streamon arm cycle,
        // re-queuing buffers after streamoff (which returns them).
        ioctl(fd, VIDIOC_STREAMON, &type); usleep(120000);
        ioctl(fd, VIDIOC_STREAMOFF, &type); usleep(120000);
        for (unsigned i = 0; i < req.count && i < TOF_NBUF; i++) {
            struct v4l2_plane qp; memset(&qp, 0, sizeof qp);
            struct v4l2_buffer qb; memset(&qb, 0, sizeof qb);
            qb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; qb.memory = V4L2_MEMORY_MMAP; qb.index = i;
            qb.m.planes = &qp; qb.length = 1;
            ioctl(fd, VIDIOC_QBUF, &qb);
        }
    }
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) { fprintf(stderr, "camd: tof STREAMON: %s\n", strerror(errno)); close(fd); return; }
    usleep(50000);
    if (!getenv("W10_TOF_NOI2C")) tof_sensor_enable();   // re-apply AFTER streamon (kernel s_stream may reset pre-streamon writes)
    fprintf(stderr, "camd: video1 raw ToF streaming (%dx%d BG12) -> shm %s\n", TOF_W, TOF_H, c->shm_path);
    unsigned char *jpg = malloc(2 * 1024 * 1024);
    unsigned char *gray = malloc(IR_SUB * TOF_W);
    unsigned char *big = malloc(IR_SUB * TOF_W * 4);
    uint64_t frames = 0, misses = 0;
    for (;;) {
        struct v4l2_plane pl; memset(&pl, 0, sizeof pl);
        struct v4l2_buffer b; memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; b.memory = V4L2_MEMORY_MMAP; b.m.planes = &pl; b.length = 1;
        if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) {
            if (++misses % 200 == 0) fprintf(stderr, "camd: tof DQBUF: %s\n", strerror(errno));
            usleep(5000); continue;
        }
        tof_to_gray((const uint16_t *)bp[b.index], -1, gray);
        ir_upscale(gray, TOF_W, IR_SUB, 2, big);
        int n = jpeg_encode_gray(big, TOF_W * 2, IR_SUB * 2, jpeg_q(), jpg);
        publish_jpeg(c, jpg, n, TOF_W * 2, IR_SUB * 2, &frames, misses);
        ioctl(fd, VIDIOC_QBUF, &b);
    }
}

static void capture(struct cam *c) {
    if (c->tof) { capture_tof(c); return; }
    void *self = calloc(1, 64);
    int r = Open(self, c->idx, (int)c->fourcc, 15, c->w, c->h);
    if (r != 1) { fprintf(stderr, "camd: OpenCamera(video%d) failed ret=%d\n", c->idx, r); return; }
    int ow = c->tof ? TOF_W * 2 : c->w, oh = c->tof ? IR_SUB * 2 : c->h;
    fprintf(stderr, "camd: video%d streaming (%dx%d, %s) -> shm %s\n",
            c->idx, ow, oh, c->tof ? "ToF" : "NV21", c->shm_path);
    uint8_t imgframe[64];
    unsigned char *jpg = malloc(2 * 1024 * 1024);
    unsigned char *gray = malloc(IR_SUB * TOF_W);
    unsigned char *big = malloc(IR_SUB * TOF_W * 4);   // scale 2 -> *4
    uint64_t frames = 0, misses = 0;
    for (;;) {
        memset(imgframe, 0, sizeof imgframe);
        int gr = Get(self, imgframe);
        void *data = *(void **)(imgframe + IMGFRAME_DATA_OFF);
        if (gr != 1 || !data) {
            if (++misses % 200 == 0)
                fprintf(stderr, "camd: video%d %llu frames, %llu misses\n",
                        c->idx, (unsigned long long)frames, (unsigned long long)misses);
            usleep(5000);
            continue;
        }
        int n;
        if (c->tof) {
            tof_to_gray((const uint16_t *)data, -1, gray);   // 224x173 grayscale
            ir_upscale(gray, TOF_W, IR_SUB, 2, big);         // -> 448x346
            n = jpeg_encode_gray(big, TOF_W * 2, IR_SUB * 2, jpeg_q(), jpg);
        } else {
            n = jpeg_encode_nv21((const unsigned char *)data, c->w, c->h, jpeg_q(), jpg);
        }
        Return(self, imgframe);
        if (n > 0 && (size_t)n <= R2C_MAX_JPEG && c->shm) {
            struct r2c_shm *s = c->shm;
            s->seq++; __sync_synchronize();        // odd = write in progress
            s->size = (uint32_t)n; s->width = (uint32_t)ow; s->height = (uint32_t)oh;
            memcpy(s->data, jpg, n);
            s->frames++;
            __sync_synchronize(); s->seq++;         // even = done
            if (++frames % 60 == 0)
                fprintf(stderr, "camd: video%d %llu frames, %llu misses\n",
                        c->idx, (unsigned long long)frames, (unsigned long long)misses);
        }
    }
}

static struct cam rgb = { .idx = 2, .w = 672, .h = 504, .tof = 0, .shm_path = "/tmp/ros2cam.shm" };
static struct cam tof = { .idx = 1, .w = 224, .h = 1558, .tof = 1, .shm_path = "/tmp/ros2cam_ir.shm" };

// Load libsunxicamera into THIS process and drive one camera: open its shm ring
// then run the capture loop (which blocks). Each camera runs in its OWN process
// (see main) - the vendor lib keeps global ISP/sensor state, so two SunxiCam
// instances in one process clobber each other (RGB's isp0 reset loop was
// corrupting the ToF capture into pure noise, and RGB never came up). Separate
// processes = separate lib globals. The reference noava-cam.sh proved this
// works: two w10-cam processes, RGB (isp0) + ToF (isp1) concurrent, 0 misses.
static int run_camera(struct cam *c, const char *lib) {
    // RGB drives the sensor through the vendor lib; ToF uses raw V4L2 (capture_tof)
    // and needs no libsunxicamera - skip the dlopen so its isp0-oriented init
    // can't interfere with the ToF pipeline.
    if (!c->tof) {
        void *h = dlopen(lib, RTLD_NOW | RTLD_GLOBAL);
        if (!h) { fprintf(stderr, "camd: dlopen %s: %s\n", lib, dlerror()); return 1; }
        Open = (open_fn) dlsym(h, SYM_OPEN);
        Get = (frame_fn) dlsym(h, SYM_FRAME);
        Return = (frame_fn) dlsym(h, SYM_RETURN);
        Close = (close_fn) dlsym(h, SYM_CLOSE);
        if (!Open || !Get || !Return || !Close) { fprintf(stderr, "camd: missing SunxiCam symbols\n"); return 1; }
    }
    c->shm = open_shm(c->shm_path);
    if (!c->shm) { fprintf(stderr, "camd: shm %s: %s\n", c->shm_path, strerror(errno)); return 1; }
    capture(c);   // blocks forever (only returns if OpenCamera fails)
    return 0;
}

int main(int argc, char **argv) {
    const char *mode = argc > 1 ? argv[1] : "both";
    const char *lib = getenv("CAM_LIB");
    if (!lib) lib = "/usr/lib/libsunxicamera.so";
    signal(SIGPIPE, SIG_IGN);

    if (getenv("CAM_SHM"))    rgb.shm_path = getenv("CAM_SHM");
    if (getenv("CAM_SHM_IR")) tof.shm_path = getenv("CAM_SHM_IR");
    rgb.fourcc = fourcc("NV21");
    tof.fourcc = fourcc("BG12");

    if (strcmp(mode, "rgb") == 0) return run_camera(&rgb, lib);
    if (strcmp(mode, "tof") == 0) return run_camera(&tof, lib);

    // both: one process per camera (the vendor lib is not multi-camera safe in a
    // single process). Parent drives RGB (isp0), child drives ToF (isp1). Stagger
    // the ToF bring-up so RGB's OpenCamera finishes before ToF touches the shared
    // media graph, mirroring noava-cam.sh (cam_rgb; sleep 1; cam_tof).
    pid_t pid = fork();
    if (pid < 0) { fprintf(stderr, "camd: fork: %s\n", strerror(errno)); return 1; }
    if (pid == 0) { usleep(1500000); run_camera(&tof, lib); for (;;) pause(); }  // child: ToF
    signal(SIGCHLD, SIG_IGN);
    run_camera(&rgb, lib);   // parent: RGB
    for (;;) pause();         // stay up even if RGB failed, so the ToF child keeps serving
    return 0;
}
