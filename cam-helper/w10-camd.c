// ---------------------------------------------------------------------------
// w10-camd - the ros2dreame camera helper (ava OFF). A small DYNAMIC binary
// that drives the Dreame W10 cameras via the vendor libsunxicamera.so (dlopen -
// which the static-musl ros2dreame binary can't do), JPEG-encodes each frame in
// process, and serves MJPEG on loopback for ros2dreame to forward as ROS 2
// CompressedImage. Replaces the old w10-cam + ava_cam_relay + go2rtc chain with
// one process; no go2rtc.
//
//   RGB   : OV8856   /dev/video2, NV21 672x504  -> JPEG -> 127.0.0.1:8090
//   IR/ToF: ofilm0092/dev/video1, BG12 224x1558 -> gray -> JPEG -> 127.0.0.1:8091
//
//   w10-camd [rgb|tof|both]     (default both)
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
#include <pthread.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include "ir_process.h"
#include "jpeg_gray.h"

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

// --- one camera: capture thread fills the latest JPEG; server thread streams it
struct cam {
    int idx, w, h, tof, port;
    uint32_t fourcc;
    pthread_mutex_t lock;
    unsigned char jpg[2 * 1024 * 1024];
    int jlen;
    uint64_t seq;
};

static int write_all(int fd, const void *b, int n) {
    const char *p = b;
    while (n > 0) { int w = write(fd, p, n); if (w <= 0) return -1; p += w; n -= w; }
    return 0;
}

static void *capture(void *arg) {
    struct cam *c = arg;
    if (c->tof) tof_pipeline();
    void *self = calloc(1, 64);
    int r = Open(self, c->idx, (int)c->fourcc, 15, c->w, c->h);
    if (r != 1) { fprintf(stderr, "camd: OpenCamera(video%d) failed ret=%d\n", c->idx, r); return NULL; }
    fprintf(stderr, "camd: video%d streaming (%dx%d, %s) -> :%d\n",
            c->idx, c->w, c->h, c->tof ? "ToF" : "NV21", c->port);
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
            n = jpeg_encode_gray(big, TOF_W * 2, IR_SUB * 2, 80, jpg);
        } else {
            n = jpeg_encode_nv21((const unsigned char *)data, c->w, c->h, 80, jpg);
        }
        Return(self, imgframe);
        if (n > 0) {
            pthread_mutex_lock(&c->lock);
            memcpy(c->jpg, jpg, n); c->jlen = n; c->seq++;
            pthread_mutex_unlock(&c->lock);
            if (++frames % 60 == 0)
                fprintf(stderr, "camd: video%d %llu frames, %llu misses\n",
                        c->idx, (unsigned long long)frames, (unsigned long long)misses);
        }
    }
}

static void *serve(void *arg) {
    struct cam *c = arg;
    int srv = socket(AF_INET, SOCK_STREAM, 0), one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_LOOPBACK); a.sin_port = htons(c->port);
    if (bind(srv, (struct sockaddr *)&a, sizeof a) < 0 || listen(srv, 4) < 0) {
        fprintf(stderr, "camd: bind :%d: %s\n", c->port, strerror(errno));
        return NULL;
    }
    for (;;) {
        int cl = accept(srv, NULL, NULL);
        if (cl < 0) continue;
        char req[512]; ssize_t rn = read(cl, req, sizeof req); (void)rn;
        const char *h = "HTTP/1.0 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=ffcam\r\n"
            "Cache-Control: no-cache\r\nConnection: close\r\n\r\n";
        if (write_all(cl, h, strlen(h)) < 0) { close(cl); continue; }
        unsigned char *jbuf = malloc(2 * 1024 * 1024);
        uint64_t last = 0;
        for (;;) {
            pthread_mutex_lock(&c->lock);
            if (c->seq == last || c->jlen == 0) { pthread_mutex_unlock(&c->lock); usleep(6000); continue; }
            int n = c->jlen; memcpy(jbuf, c->jpg, n); last = c->seq;
            pthread_mutex_unlock(&c->lock);
            char hdr[128];
            int hl = snprintf(hdr, sizeof hdr,
                "--ffcam\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", n);
            if (write_all(cl, hdr, hl) < 0 || write_all(cl, jbuf, n) < 0 || write_all(cl, "\r\n", 2) < 0)
                break;
        }
        free(jbuf);
        close(cl);
    }
}

static struct cam rgb = { .idx = 2, .w = 672, .h = 504, .tof = 0, .port = 8090,
                          .lock = PTHREAD_MUTEX_INITIALIZER };
static struct cam tof = { .idx = 1, .w = 224, .h = 1558, .tof = 1, .port = 8091,
                          .lock = PTHREAD_MUTEX_INITIALIZER };

static void start(struct cam *c) {
    pthread_t t;
    pthread_create(&t, NULL, capture, c);
    pthread_create(&t, NULL, serve, c);
}

int main(int argc, char **argv) {
    const char *mode = argc > 1 ? argv[1] : "both";
    const char *lib = getenv("CAM_LIB");
    if (!lib) lib = "/usr/lib/libsunxicamera.so";
    signal(SIGPIPE, SIG_IGN);

    void *h = dlopen(lib, RTLD_NOW | RTLD_GLOBAL);
    if (!h) { fprintf(stderr, "camd: dlopen %s: %s\n", lib, dlerror()); return 1; }
    Open = (open_fn) dlsym(h, SYM_OPEN);
    Get = (frame_fn) dlsym(h, SYM_FRAME);
    Return = (frame_fn) dlsym(h, SYM_RETURN);
    Close = (close_fn) dlsym(h, SYM_CLOSE);
    if (!Open || !Get || !Return || !Close) { fprintf(stderr, "camd: missing SunxiCam symbols\n"); return 1; }

    rgb.fourcc = fourcc("NV21");
    tof.fourcc = fourcc("BG12");
    if (strcmp(mode, "tof") != 0) start(&rgb);
    if (strcmp(mode, "rgb") != 0) start(&tof);

    for (;;) pause();
    return 0;
}
