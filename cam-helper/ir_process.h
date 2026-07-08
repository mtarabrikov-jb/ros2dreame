// ---------------------------------------------------------------------------
// Shared ToF-frame -> viewable grayscale processing (used by irstream and the
// go2rtc H264 relay). The raw /dev/video1 frame is 224x1558: nine infrared
// sub-frames of 224x173 stacked vertically (verified — see REVERSE_ENGINEERING).
//
//   band <0  : max-project sub-frames 1..8 (skip 0 = dark reference)
//   band 0..8: use that single sub-frame
// Then flat-field (subtract per-column fixed pattern) + percentile stretch.
// Output: SUB*TOF_W bytes of 8-bit grayscale (224x173).
// ---------------------------------------------------------------------------
#ifndef IR_PROCESS_H
#define IR_PROCESS_H
#include <stdint.h>
#include <string.h>

#define TOF_W 224
#define TOF_H 1558
#define IR_SUB 173
#define IR_NSUB 9

static void tof_to_gray(const uint16_t *raw, int band, unsigned char *gray) {
	static int32_t proj[IR_SUB * TOF_W];
	if (band >= 0 && band < IR_NSUB) {
		const uint16_t *b = raw + band * IR_SUB * TOF_W;
		for (int i = 0; i < IR_SUB * TOF_W; i++) proj[i] = b[i];
	} else {                                   // max-project sub-frames 1..8
		for (int i = 0; i < IR_SUB * TOF_W; i++) proj[i] = 0;
		for (int s = 1; s < IR_NSUB; s++) {
			const uint16_t *b = raw + s * IR_SUB * TOF_W;
			for (int i = 0; i < IR_SUB * TOF_W; i++) if (b[i] > proj[i]) proj[i] = b[i];
		}
	}
	int64_t gsum = 0; static int32_t colmean[TOF_W];
	for (int c = 0; c < TOF_W; c++) {
		int64_t s = 0; for (int y = 0; y < IR_SUB; y++) s += proj[y * TOF_W + c];
		colmean[c] = (int32_t)(s / IR_SUB); gsum += s;
	}
	int32_t gm = (int32_t)(gsum / (IR_SUB * TOF_W));
	static int hist[4096]; memset(hist, 0, sizeof(hist));
	for (int i = 0; i < IR_SUB * TOF_W; i++) {
		int v = proj[i] - colmean[i % TOF_W] + gm;
		if (v < 0) v = 0; if (v > 4095) v = 4095;
		proj[i] = v; hist[v]++;
	}
	int total = IR_SUB * TOF_W, lo = 0, hi = 4095, acc = 0;
	for (int i = 0; i < 4096; i++) { acc += hist[i]; if (acc >= total * 2 / 100) { lo = i; break; } }
	acc = 0;
	for (int i = 4095; i >= 0; i--) { acc += hist[i]; if (acc >= total * 2 / 100) { hi = i; break; } }
	if (hi <= lo) hi = lo + 1;
	for (int i = 0; i < IR_SUB * TOF_W; i++) {
		int v = (proj[i] - lo) * 255 / (hi - lo);
		if (v < 0) v = 0; if (v > 255) v = 255;
		gray[i] = (unsigned char)v;
	}
	// The ToF sensor is mounted rotated vs the robot's forward view; rotate the
	// image 180 degrees (reverse the pixel order) so it matches how you'd look.
	for (int i = 0, j = IR_SUB * TOF_W - 1; i < j; i++, j--) {
		unsigned char t = gray[i]; gray[i] = gray[j]; gray[j] = t;
	}
}

static void ir_upscale(const unsigned char *src, int w, int h, int sc, unsigned char *dst) {
	for (int y = 0; y < h * sc; y++)
		for (int x = 0; x < w * sc; x++)
			dst[y * (w * sc) + x] = src[(y / sc) * w + (x / sc)];
}

#endif
