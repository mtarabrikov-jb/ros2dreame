# Build w10-camd - the ros2dreame camera helper - for the robot (aarch64).
#
# DYNAMIC (glibc + libdl): the helper dlopens the vendor libsunxicamera.so, so it
# cannot be static. It uses only basic libc/dl/pthread symbols (<= GLIBC_2.17) so
# it still loads on the robot's old glibc (2.23), same as the original w10-cam.
#
#   make docker   reproducible cross-build (needs docker; no host toolchain)
#   make          if you already have aarch64-linux-gnu-gcc
CC     := aarch64-linux-gnu-gcc
CFLAGS := -O2 -Wall -Wextra

.PHONY: all docker clean
all: w10-camd

w10-camd: w10-camd.c ir_process.h jpeg_gray.h
	$(CC) $(CFLAGS) w10-camd.c -o $@ -ldl -lpthread -lm
	@echo ">> built $@ (aarch64 dynamic camera helper)"

docker:
	docker run --rm -v $(CURDIR):/work -w /work debian:bullseye sh -c '\
		apt-get update -qq >/dev/null && \
		apt-get install -y -qq gcc-aarch64-linux-gnu libc6-dev-arm64-cross make >/dev/null && \
		make all && aarch64-linux-gnu-strip w10-camd'
	@echo ">> docker build done (aarch64 w10-camd)"

clean:
	rm -f w10-camd
