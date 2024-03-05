# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the TinyMPC-integrated firmware used in the Crazyflie 2.1. Nothing like these have existed before in the Crazyflie 2.1.

# TinyMPC

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

### Crazyflie 1.0 support

The 2017.06 release was the last release with Crazyflie 1.0 support. If you want
to play with the Crazyflie 1.0 and modify the code, please clone this repo and
branch off from the 2017.06 tag.

## Building and Flashing
See the [building and flashing instructions](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md) in the github docs folder.
`make && CLOAD_CMDS="-w radio://0/1/2M" make cload`

## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on our website.

## Generated documentation

The easiest way to generate the API documentation is to use the [toolbelt](https://github.com/bitcraze/toolbelt)

```tb build-docs```

and to view it in a web page

```tb docs```

## Contribute
Go to the [contribute page](https://www.bitcraze.io/contribute/) on our website to learn more.

### Test code for contribution

To run the tests please have a look at the [unit test documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/unit_testing/).

## License

The code is licensed under LGPL-3.0

