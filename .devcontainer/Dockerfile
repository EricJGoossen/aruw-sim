# aruw-sim development image: everything scripts/build.sh and scripts/run.sh
# need for the hub (hw-emulator + Achilles + MuJoCo), the hosted MCB firmware
# and otto, on Ubuntu 24.04. scripts/dev.sh builds and runs it.
#
# - g++-11: GCC 13 rejects the hosted modm's protothread macros ("jump to case
#   label"); taproot's SConscript picks gcc-11 when it's installed.
# - libgtest-dev/libgmock-dev: the hosted MCB build links them.
# - scons 3.1.2 and otto's Python 3.10 venv are made by build.sh with uv, in
#   build/, so they track the checkout rather than the image.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential g++-11 cmake ninja-build ccache git ca-certificates curl pkg-config \
        libgtest-dev libgmock-dev googletest \
        python3 python3-dev python3-venv python3-pip python3-numpy python3-yaml python3-serial \
        python3-pytest python3-jsonschema \
        libgl1-mesa-dev libegl-dev libegl1 libgles-dev libgl1-mesa-dri libglx-mesa0 mesa-utils \
        libglfw3-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev \
        xvfb xauth \
        gdb gdbserver rr \
        iproute2 procps psmisc less \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:0.9.7 /uv /uvx /usr/local/bin/

ENV LIBGL_ALWAYS_SOFTWARE=1 \
    MESA_GL_VERSION_OVERRIDE=4.5
WORKDIR /work/aruw-sim
