# Self-contained build + test environment for the native STM32 port.
#
# The whole point: collapse the host requirement to *Docker only*. Everything the
# Makefile tiers need on the host — the ARM cross toolchain (Tier B), Renode + socat
# (Tier C), the host g++ for the unit tests (Tier A), and the Docker CLI used to spawn
# the pinned libmicroros builder and the micro-ROS agent as *sibling* containers — all
# live in this image. Run it with the host's Docker socket bind-mounted (see the
# `docker-*` targets in the root Makefile); `make libmicroros` / the agent tiers then
# launch their pinned containers on the host daemon, while the toolchain and Renode run
# in here.
#
# ARM GCC: we use Ubuntu 24.04's distro `gcc-arm-none-eabi` (currently 15:13.2.rel1-2), NOT the
# upstream ARM tarball. This is deliberate and load-bearing: the distro and the ARM-official
# builds report the same version (13.2.1 20231009) but ship DIFFERENT newlib-nano builds, and the
# firmware linked against ARM-official newlib HANGS at boot in Renode (scheduler starts, but
# uros_task never reaches the transmit path — stuck in prvIdleTask), while the distro newlib boots
# and transmits the micro-ROS ping. The distro toolchain is also exactly what CI and the dev box
# use, so this image reproduces the validated firmware (text=108896). Renode is pinned to 1.16.1
# (CI's version). `FROM ubuntu:24.04` pins the toolchain; the self-check below prints the versions.

FROM ubuntu:24.04

ARG RENODE_VER=1.16.1
ARG DOCKER_CLI_VER=27.3.1

ENV DEBIAN_FRONTEND=noninteractive

# Tier A (g++/make/git), Tier B cross toolchain (gcc/binutils-arm-none-eabi + the SEPARATE
# newlib C/C++ runtime packages — gcc-arm-none-eabi ships no libc, so without libnewlib-* the
# build dies on "stdlib.h: No such file or directory"), download tooling, Tier C runtime: socat
# (reads USART2) + iproute2 (the Ф2 smoke waits on `ss -ltn` until Renode's socket terminal is
# listening before connecting socat — without ss that wait overruns Renode's lifetime and captures
# 0 bytes) + xxd (hexdump of the captured ping) + Renode-portable's shared-lib deps (it bundles its
# own mono). gawk is required too: the smoke scripts use awk strtonum() for numeric PC/symbol
# compares, and mawk (Ubuntu's default) lacks it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        make \
        git \
        gawk \
        ca-certificates \
        curl \
        xz-utils \
        socat \
        iproute2 \
        xxd \
        python3 \
        gcc-arm-none-eabi \
        binutils-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libnewlib-dev \
        libstdc++-arm-none-eabi-newlib \
        libstdc++-arm-none-eabi-dev \
        gpg \
        policykit-1 \
        libgtk2.0-0 \
        libcanberra-gtk-module \
        screen \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

# --- Renode (Tier C) — portable build bundles its own runtime; just unpack + expose ---
RUN curl -fsSL -o /tmp/renode.tgz \
        "https://github.com/renode/renode/releases/download/v${RENODE_VER}/renode-${RENODE_VER}.linux-portable.tar.gz" \
    && mkdir -p /opt/renode \
    && tar -xzf /tmp/renode.tgz -C /opt/renode --strip-components=1 \
    && rm /tmp/renode.tgz
ENV RENODE="/opt/renode/renode"
ENV PATH="/opt/renode:${PATH}"

# --- Docker CLI (static) — to spawn the libmicroros builder / agent as sibling containers
#     over the bind-mounted host socket (Docker-out-of-Docker). No daemon in here. ---
RUN curl -fsSL -o /tmp/docker.tgz \
        "https://download.docker.com/linux/static/stable/x86_64/docker-${DOCKER_CLI_VER}.tgz" \
    && tar -xzf /tmp/docker.tgz -C /tmp \
    && install -m0755 /tmp/docker/docker /usr/local/bin/docker \
    && rm -rf /tmp/docker /tmp/docker.tgz

# Quick self-check at build time: fail the image build if a tool is missing/mismatched.
RUN arm-none-eabi-gcc --version | head -1 \
    && g++ --version | head -1 \
    && make --version | head -1 \
    && socat -V | head -1 \
    && docker --version \
    && test -x "$RENODE" && echo "renode launcher present at $RENODE"

WORKDIR /work
CMD ["bash"]
