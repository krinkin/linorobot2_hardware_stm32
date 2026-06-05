# Developer entry point for building + testing the STM32 native (GUI-free) port.
# Three tiers — run `make help` for the list.
#
#   make test-host    # host unit tests (kinematics/pid/odometry) — fast, no toolchain
#   make libmicroros  # build libmicroros.a via the pinned Docker builder (needs docker)
#   make build-fw     # link the F446RE firmware (needs arm-none-eabi-gcc + libmicroros)
#   make renode          # Renode boot smoke (Ф2): firmware boots + transmits the ping (needs renode, socat)
#   make control         # Ф4: encoder/PWM control loop smoke + injected-encoder->odom (needs renode)
#   make imu             # Ф5: MPU6050 read over real HAL I2C via a Python mock slave (needs renode)
#   make agent-roundtrip # Ф3: live micro_ros_agent <-> firmware XRCE session in emulation (needs renode, docker, socat)
#   make test-all        # everything above, in order
#
# Prereqs: git submodules initialised  ->  git submodule update --init --recursive
#          arm-none-eabi-gcc, GNU make, docker, socat; renode for renode/agent-roundtrip.

DOCKER_IMG = microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
FW = firmware_stm32
LIBMICROROS = $(FW)/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a

.PHONY: help test-host libmicroros build-fw renode control imu agent-roundtrip test-all clean submodules

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

# --- Tier B: link the firmware (F0) ---
build-fw:
	@test -f $(LIBMICROROS) || { echo "libmicroros.a missing -> run 'make libmicroros'"; exit 1; }
	$(MAKE) -C $(FW)

# --- Tier C: Renode boot smoke (Ф2) — boots + transmits the micro-ROS ping ---
renode:
	bash $(FW)/renode/boot_smoke.sh $(FW)/build/firmware_stm32.elf

# --- Tier C (Ф4): encoder/PWM control loop smoke + injected-encoder propagation ---
control:
	bash $(FW)/renode/control_smoke.sh $(FW)/build/firmware_stm32.elf

# --- Tier C (Ф5): MPU6050 IMU read over real HAL I2C (Python mock slave in Renode) ---
imu:
	bash $(FW)/renode/imu_smoke.sh $(FW)/build/firmware_stm32.elf

# --- Tier C+: live agent round-trip (Ф3) — real micro_ros_agent <-> firmware in emulation ---
agent-roundtrip:
	bash $(FW)/renode/agent_bridge.sh $(FW)/build/firmware_stm32.elf

# --- everything ---
test-all: test-host libmicroros build-fw renode control imu
	@echo "================ ALL TIERS GREEN ================"

clean:
	-$(MAKE) -C test_host clean
	-$(MAKE) -C $(FW) clean
