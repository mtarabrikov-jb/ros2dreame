// ---------------------------------------------------------------------------
// jpeg_gray.h — minimal self-contained baseline JPEG encoder, grayscale only.
//
// Stateless (each frame independent), no vendor/CedarX API, no reference frames,
// no SPS/PPS — the opposite of the H264 path. Used to build an MJPEG feed that
// the existing go2rtc consumes. Standard baseline sequential DCT with the JPEG
// Annex-K luminance quantization and Huffman tables.
//
//   int jpeg_encode_gray(img, W, H, quality, out) -> byte length written to out.
//   out must hold at least W*H + 1024.
// ---------------------------------------------------------------------------
#ifndef JPEG_GRAY_H
#define JPEG_GRAY_H
#include <stdint.h>
#include <string.h>
#include <math.h>

static const int JZIG[64] = {
	0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,48,41,34,27,20,13,6,7,14,21,28,
	35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63
};
static const int JQL[64] = {   // Annex K.1 luminance quant table
	16,11,10,16,24,40,51,61,12,12,14,19,26,58,60,55,14,13,16,24,40,57,69,56,14,17,22,29,51,87,80,62,
	18,22,37,56,68,109,103,77,24,35,55,64,81,104,113,92,49,64,78,87,103,121,120,101,72,92,95,98,112,100,103,99
};
// Standard luminance DC Huffman (Annex K.3): bits[1..16], then values.
static const uint8_t DC_BITS[17] = {0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static const uint8_t DC_VAL[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const uint8_t AC_BITS[17] = {0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static const uint8_t AC_VAL[162] = {
	0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,0x22,0x71,0x14,0x32,
	0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,
	0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,
	0x46,0x47,0x48,0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
	0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x92,0x93,0x94,
	0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,
	0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,
	0xd9,0xda,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
	0xf9,0xfa
};

typedef struct { uint16_t code[256]; uint8_t size[256]; } jhuff;
static void jbuild(jhuff *h, const uint8_t *bits, const uint8_t *val) {
	memset(h, 0, sizeof(*h));
	int k = 0; uint16_t code = 0;
	for (int len = 1; len <= 16; len++) {
		for (int i = 0; i < bits[len]; i++) { h->code[val[k]] = code; h->size[val[k]] = len; k++; code++; }
		code <<= 1;
	}
}

typedef struct { uint8_t *o; int n; uint32_t acc; int nb; uint8_t q[64]; } jenc;
static void jput(jenc *e, int v) { e->o[e->n++] = (uint8_t)v; if (v == 0xFF) e->o[e->n++] = 0; }
static void jbits(jenc *e, uint16_t code, int size) {
	e->acc |= (uint32_t)code << (24 - e->nb - size); e->nb += size;
	while (e->nb >= 8) { jput(e, (e->acc >> 16) & 0xFF); e->acc <<= 8; e->nb -= 8; }
}
static void jword(jenc *e, int w) { e->o[e->n++] = (w >> 8) & 0xFF; e->o[e->n++] = w & 0xFF; }

// forward DCT (float, straightforward — resolution is small)
static void fdct(float *b) {
	static float c[8][8]; static int init = 0;
	if (!init) { for (int u=0;u<8;u++) for (int x=0;x<8;x++) c[u][x]=cosf((2*x+1)*u*3.14159265f/16.0f)*(u==0?0.353553391f:0.5f); init=1; }
	float t[64];
	for (int y=0;y<8;y++) for (int u=0;u<8;u++){ float s=0; for(int x=0;x<8;x++) s+=b[y*8+x]*c[u][x]; t[y*8+u]=s; }
	for (int x=0;x<8;x++) for (int v=0;v<8;v++){ float s=0; for(int y=0;y<8;y++) s+=t[y*8+x]*c[v][y]; b[v*8+x]=s; }
}

// Chroma quant table (Annex K.1) + chroma Huffman tables (Annex K.3), for color.
static const int JQC[64] = {
	17,18,24,47,99,99,99,99,18,21,26,66,99,99,99,99,24,26,56,99,99,99,99,99,47,66,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99
};
static const uint8_t DCc_BITS[17]={0,0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static const uint8_t DCc_VAL[12]={0,1,2,3,4,5,6,7,8,9,10,11};
static const uint8_t ACc_BITS[17]={0,0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static const uint8_t ACc_VAL[162]={
	0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,0x13,0x22,0x32,0x81,
	0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,
	0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,
	0x45,0x46,0x47,0x48,0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
	0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x92,
	0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,
	0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,
	0xd7,0xd8,0xd9,0xda,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
	0xf9,0xfa
};

// One 8x8 block -> DC/AC bitstream (huffman tables + quant passed in). prevdc updated.
static void jblock(jenc *e, float *blk, const uint8_t *q, jhuff *hdc, jhuff *hac, int *prevdc) {
	fdct(blk);
	int qz[64];
	for (int i = 0; i < 64; i++) { float v = blk[JZIG[i]] / q[JZIG[i]]; qz[i] = (int)(v < 0 ? v - 0.5f : v + 0.5f); }
	int diff = qz[0] - *prevdc; *prevdc = qz[0];
	int a = diff < 0 ? -diff : diff, s = 0; while (a) { s++; a >>= 1; }
	jbits(e, hdc->code[s], hdc->size[s]);
	if (s) { int v = diff < 0 ? diff - 1 : diff; jbits(e, v & ((1<<s)-1), s); }
	int run = 0;
	for (int i = 1; i < 64; i++) {
		if (qz[i] == 0) { run++; continue; }
		while (run > 15) { jbits(e, hac->code[0xF0], hac->size[0xF0]); run -= 16; }
		int av = qz[i] < 0 ? -qz[i] : qz[i], sz = 0; while (av) { sz++; av >>= 1; }
		int sym = (run << 4) | sz; jbits(e, hac->code[sym], hac->size[sym]);
		int v = qz[i] < 0 ? qz[i] - 1 : qz[i]; jbits(e, v & ((1<<sz)-1), sz); run = 0;
	}
	if (run) jbits(e, hac->code[0x00], hac->size[0x00]);
}

// Baseline 4:2:0 color JPEG from NV21 (Y plane w*h, then interleaved V,U at w/2 x h/2).
static int jpeg_encode_nv21(const unsigned char *nv21, int W, int H, int quality, unsigned char *out) {
	jenc e; memset(&e, 0, sizeof(e)); e.o = out; e.acc = 0; e.nb = 0;
	uint8_t qc[64];
	if (quality < 1) quality = 1; if (quality > 99) quality = 99;
	int scale = quality < 50 ? 5000 / quality : 200 - quality * 2;
	for (int i = 0; i < 64; i++) { int q = (JQL[i]*scale+50)/100; e.q[i]=(uint8_t)(q<1?1:q>255?255:q);
		q = (JQC[i]*scale+50)/100; qc[i]=(uint8_t)(q<1?1:q>255?255:q); }
	jhuff hdcL,hacL,hdcC,hacC;
	jbuild(&hdcL,DC_BITS,DC_VAL); jbuild(&hacL,AC_BITS,AC_VAL);
	jbuild(&hdcC,DCc_BITS,DCc_VAL); jbuild(&hacC,ACc_BITS,ACc_VAL);
	const unsigned char *Yp = nv21, *VU = nv21 + W*H;

	jword(&e,0xFFD8);
	jword(&e,0xFFDB); jword(&e,0x0043); e.o[e.n++]=0; for(int i=0;i<64;i++) e.o[e.n++]=e.q[JZIG[i]];
	jword(&e,0xFFDB); jword(&e,0x0043); e.o[e.n++]=1; for(int i=0;i<64;i++) e.o[e.n++]=qc[JZIG[i]];
	jword(&e,0xFFC0); jword(&e,0x0011); e.o[e.n++]=8; jword(&e,H); jword(&e,W); e.o[e.n++]=3;
	e.o[e.n++]=1; e.o[e.n++]=0x22; e.o[e.n++]=0;   // Y 2x2, quant 0
	e.o[e.n++]=2; e.o[e.n++]=0x11; e.o[e.n++]=1;   // Cb, quant 1
	e.o[e.n++]=3; e.o[e.n++]=0x11; e.o[e.n++]=1;   // Cr, quant 1
	jword(&e,0xFFC4); jword(&e,0x001F); e.o[e.n++]=0x00; for(int i=1;i<=16;i++)e.o[e.n++]=DC_BITS[i]; for(int i=0;i<12;i++)e.o[e.n++]=DC_VAL[i];
	jword(&e,0xFFC4); jword(&e,0x00B5); e.o[e.n++]=0x10; for(int i=1;i<=16;i++)e.o[e.n++]=AC_BITS[i]; for(int i=0;i<162;i++)e.o[e.n++]=AC_VAL[i];
	jword(&e,0xFFC4); jword(&e,0x001F); e.o[e.n++]=0x01; for(int i=1;i<=16;i++)e.o[e.n++]=DCc_BITS[i]; for(int i=0;i<12;i++)e.o[e.n++]=DCc_VAL[i];
	jword(&e,0xFFC4); jword(&e,0x00B5); e.o[e.n++]=0x11; for(int i=1;i<=16;i++)e.o[e.n++]=ACc_BITS[i]; for(int i=0;i<162;i++)e.o[e.n++]=ACc_VAL[i];
	jword(&e,0xFFDA); jword(&e,0x000C); e.o[e.n++]=3;
	e.o[e.n++]=1; e.o[e.n++]=0x00; e.o[e.n++]=2; e.o[e.n++]=0x11; e.o[e.n++]=3; e.o[e.n++]=0x11;
	e.o[e.n++]=0; e.o[e.n++]=63; e.o[e.n++]=0;

	int dcY=0,dcCb=0,dcCr=0;
	for (int my = 0; my < H; my += 16) {
		for (int mx = 0; mx < W; mx += 16) {
			float blk[64];
			for (int j = 0; j < 2; j++) for (int i = 0; i < 2; i++) {   // 4 Y blocks
				for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
					int sx=mx+i*8+x, sy=my+j*8+y; if(sx>=W)sx=W-1; if(sy>=H)sy=H-1;
					blk[y*8+x]=(float)Yp[sy*W+sx]-128.0f;
				}
				jblock(&e,blk,e.q,&hdcL,&hacL,&dcY);
			}
			float cb[64],cr[64];                                        // 1 Cb + 1 Cr (subsampled)
			for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
				int cx=mx/2+x, cy=my/2+y; if(cx>=W/2)cx=W/2-1; if(cy>=H/2)cy=H/2-1;
				int idx=(cy*(W/2)+cx)*2;      // NV21: V then U
				cr[y*8+x]=(float)VU[idx]-128.0f;   // Cr = V
				cb[y*8+x]=(float)VU[idx+1]-128.0f; // Cb = U
			}
			jblock(&e,cb,qc,&hdcC,&hacC,&dcCb);
			jblock(&e,cr,qc,&hdcC,&hacC,&dcCr);
		}
	}
	if (e.nb > 0) jbits(&e, 0x7F, 7);
	jword(&e,0xFFD9);
	return e.n;
}

static int jpeg_encode_gray(const unsigned char *img, int W, int H, int quality, unsigned char *out) {
	jenc e; memset(&e, 0, sizeof(e)); e.o = out; e.acc = 0; e.nb = 0;
	if (quality < 1) quality = 1; if (quality > 99) quality = 99;
	int scale = quality < 50 ? 5000 / quality : 200 - quality * 2;
	for (int i = 0; i < 64; i++) { int q = (JQL[i] * scale + 50) / 100; e.q[i] = (uint8_t)(q < 1 ? 1 : q > 255 ? 255 : q); }
	jhuff hdc, hac; jbuild(&hdc, DC_BITS, DC_VAL); jbuild(&hac, AC_BITS, AC_VAL);

	// --- headers ---
	jword(&e, 0xFFD8);                                   // SOI
	jword(&e, 0xFFDB); jword(&e, 0x0043); e.o[e.n++] = 0; // DQT
	for (int i = 0; i < 64; i++) e.o[e.n++] = e.q[JZIG[i]];
	jword(&e, 0xFFC0); jword(&e, 0x000B); e.o[e.n++] = 8; // SOF0
	jword(&e, H); jword(&e, W); e.o[e.n++] = 1;
	e.o[e.n++] = 1; e.o[e.n++] = 0x11; e.o[e.n++] = 0;
	jword(&e, 0xFFC4); jword(&e, 0x001F); e.o[e.n++] = 0x00; // DHT DC
	for (int i=1;i<=16;i++) e.o[e.n++]=DC_BITS[i]; for (int i=0;i<12;i++) e.o[e.n++]=DC_VAL[i];
	jword(&e, 0xFFC4); jword(&e, 0x00B5); e.o[e.n++] = 0x10; // DHT AC
	for (int i=1;i<=16;i++) e.o[e.n++]=AC_BITS[i]; for (int i=0;i<162;i++) e.o[e.n++]=AC_VAL[i];
	jword(&e, 0xFFDA); jword(&e, 0x0008); e.o[e.n++] = 1;    // SOS
	e.o[e.n++] = 1; e.o[e.n++] = 0x00; e.o[e.n++] = 0; e.o[e.n++] = 63; e.o[e.n++] = 0;

	// --- scan ---
	int prevdc = 0;
	for (int by = 0; by < H; by += 8) {
		for (int bx = 0; bx < W; bx += 8) {
			float blk[64];
			for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
				int sx = bx + x < W ? bx + x : W - 1, sy = by + y < H ? by + y : H - 1;
				blk[y*8+x] = (float)img[sy*W+sx] - 128.0f;
			}
			fdct(blk);
			int qz[64];
			for (int i = 0; i < 64; i++) {
				float v = blk[JZIG[i]] / e.q[JZIG[i]];
				qz[i] = (int)(v < 0 ? v - 0.5f : v + 0.5f);
			}
			// DC
			int diff = qz[0] - prevdc; prevdc = qz[0];
			int a = diff < 0 ? -diff : diff, s = 0; while (a) { s++; a >>= 1; }
			jbits(&e, hdc.code[s], hdc.size[s]);
			if (s) { int v = diff < 0 ? diff - 1 : diff; jbits(&e, v & ((1<<s)-1), s); }
			// AC
			int run = 0;
			for (int i = 1; i < 64; i++) {
				if (qz[i] == 0) { run++; continue; }
				while (run > 15) { jbits(&e, hac.code[0xF0], hac.size[0xF0]); run -= 16; }
				int av = qz[i] < 0 ? -qz[i] : qz[i], sz = 0; while (av) { sz++; av >>= 1; }
				int sym = (run << 4) | sz;
				jbits(&e, hac.code[sym], hac.size[sym]);
				int v = qz[i] < 0 ? qz[i] - 1 : qz[i]; jbits(&e, v & ((1<<sz)-1), sz);
				run = 0;
			}
			if (run) jbits(&e, hac.code[0x00], hac.size[0x00]);  // EOB
		}
	}
	if (e.nb > 0) jbits(&e, 0x7F, 7);                          // flush with 1-bits
	jword(&e, 0xFFD9);                                          // EOI
	return e.n;
}

#endif
