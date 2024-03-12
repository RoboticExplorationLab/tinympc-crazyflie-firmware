# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the TinyMPC-integrated firmware used in the Crazyflie 2.1. Nothing like these have existed before in the Crazyflie 2.1.

# TinyMPC-Integrated Firmware

Step 1: `git clone --recursive [this repo]`

Step 2: Modify `Makefile` to avoid compile errors

```BASH
ifeq ($(CONFIG_DEBUG),y)
ARCH_CFLAGS	+= -O0 -Wconversion
else
# ARCH_CFLAGS += -Os -Werror
ARCH_CFLAGS += -Ofast -fsingle-precision-constant -DNDEBUG 
endif
```

Step 3: go to `examples/controller_tinympc_task`

Step 4: `make -j8`

Step 5: `cfloader flash ./build/cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E701`

# Record Data

Step 1: rosbag record -o [PREFIX] -a

