# Developer entry point for building + testing the STM32 native (GUI-free) port.
# Three tiers -- run `make help` for the list.
#
#   make test-host    # host unit tests (kinematics/pid/odometry) -- fast, no toolchain
#   make libmicroros  # build libmicroros.a via the pinned Docker builder (needs docker)
#   make build-fw     # link the F446RE firmware (needs arm-none-eabi-gcc + libmicroros)
#   make renode          # Renode boot smoke (F2): firmware boots + transmits the ping (needs renode, socat)
#   make control         # F4: encoder/PWM control loop smoke + injected-encoder->odom (needs renode)
#   make imu             # F5: MPU6050 read over real HAL I2C via a Python mock slave (needs renode)
#   make agent-roundtrip # F3: live micro_ros_agent <-> firmware XRCE session in emulation (needs renode, docker, socat)
#   make topics          # F6: full base-node topic round-trip (cmd_vel + odom + imu) (needs renode, docker, socat)
#   make test-all        # test-host + libmicroros + build-fw + renode + control + imu, in order
#                         # (the agent tiers agent-roundtrip/topics are not part of test-all)
#
#   make docker-image    # build the self-contained dev image (toolchain + Renode + socat)
#   make docker-test-all # run the WHOLE suite inside Docker -- host needs ONLY Docker
#   make docker-shell    # interactive shell in the dev image (repo + docker socket mounted)
#   make docker-<target> # run any target above inside the dev image (e.g. make docker-build-fw)
#
# Two ways to run:
#   1) Host has the tools  -> use the bare targets (test-host/build-fw/renode/...).
#      Prereqs: arm-none-eabi-gcc/g++, GNU make, docker, socat, Renode 1.16.1 (set $RENODE
#      or PATH). See README / docs/TESTING.md for the Renode install one-liner.
#   2) Host has ONLY Docker -> use the docker-* targets. The dev image (docker/Dockerfile)
#      carries the toolchain + Renode + socat; the libmicroros builder and the micro-ROS
#      agent run as sibling containers via the bind-mounted host socket. Nothing else to install.
#
# Either way: git submodules first -> git submodule update --init --recursive.
# NO host ROS install needed (libmicroros is built by a pinned Docker image; ros2/the
# agent run only inside the agent container).

DOCKER_IMG = microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
FW = firmware_stm32
LIBMICROROS = $(FW)/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a

# Self-contained dev image (docker/Dockerfile): carries the ARM toolchain + Renode + socat.
# Bind-mount the repo at its host path (so the sibling libmicroros/agent containers see the
# same paths) and the host Docker socket (Docker-out-of-Docker). --network host lets Renode's
# socket terminal and the agent container reach each other on localhost.
DEV_IMG ?= linorobot2-stm32-dev:latest
DOCKER_RUN = docker run --rm --network host \
  -v "$(CURDIR)":"$(CURDIR)" -w "$(CURDIR)" \
  -v /var/run/docker.sock:/var/run/docker.sock \
  $(DEV_IMG)

.PHONY: help test-host libmicroros build-fw renode control imu agent-roundtrip topics test-all clean submodules \
        docker-image docker-test-all docker-shell

help:
	@grep -E '^#   make ' $(MAKEFILE_LIST) | sed 's/^#   /  /'

submodules:
	git submodule update --init --recursive

# --- Tier A: host unit tests (no MCU toolchain, no board) ---
test-host:
	$(MAKE) -C test_host clean
	$(MAKE) -C test_host test

# --- libmicroros.a (built once by the pinned Docker image from the firmware Makefile) ---
libmicroros:
	cd $(FW) && printf 'y' | docker run --rm -i -v "$$PWD":/project \
	  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
	  $(DOCKER_IMG)
	@test -f $(LIBMICROROS) && echo "libmicroros.a OK"

# --- Tier B: link the firmware (F446RE) ---
build-fw:
	@test -f $(LIBMICROROS) || { echo "libmicroros.a missing -> run 'make libmicroros'"; exit 1; }
	$(MAKE) -C $(FW)

# --- Tier C: Renode boot smoke (F2) -- boots + transmits the micro-ROS ping ---
renode:
	bash $(FW)/renode/boot_smoke.sh $(FW)/build/firmware_stm32.elf

# --- Tier C (F4): encoder/PWM control loop smoke + injected-encoder propagation ---
control:
	bash $(FW)/renode/control_smoke.sh $(FW)/build/firmware_stm32.elf

# --- Tier C (F5): MPU6050 IMU read over real HAL I2C (Python mock slave in Renode) ---
imu:
	bash $(FW)/renode/imu_smoke.sh $(FW)/build/firmware_stm32.elf

# --- Tier C+: live agent round-trip (F3) -- real micro_ros_agent <-> firmware in emulation ---
agent-roundtrip:
	bash $(FW)/renode/agent_bridge.sh $(FW)/build/firmware_stm32.elf

# --- Tier C+ (F6): full base-node topic round-trip (cmd_vel + odom + imu) over a live agent ---
topics:
	bash $(FW)/renode/topic_roundtrip.sh $(FW)/build/firmware_stm32.elf

# --- everything ---
test-all: test-host libmicroros build-fw renode control imu
	@echo "================ ALL TIERS GREEN ================"

# --- Run any tier inside the self-contained dev image (host needs only Docker) ---
docker-image:
	docker build -t $(DEV_IMG) docker

docker-test-all: docker-image
	$(DOCKER_RUN) make test-all

docker-shell: docker-image
	docker run --rm -it --network host \
	  -v "$(CURDIR)":"$(CURDIR)" -w "$(CURDIR)" \
	  -v /var/run/docker.sock:/var/run/docker.sock \
	  $(DEV_IMG) bash

# Generic passthrough: `make docker-build-fw`, `make docker-renode`, ... run `make <t>` inside.
# (Explicit docker-image/docker-test-all/docker-shell above take precedence over this pattern.)
docker-%: docker-image
	$(DOCKER_RUN) make $*

clean:
	-$(MAKE) -C test_host clean
	-$(MAKE) -C $(FW) clean
