# TinyMPC-Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

**This repo might be deprecated. Recent TinyMPC-Crazyflie firmware is updated at https://github.com/A2R-Lab/tinympc-crazyflie/tree/main.**

This repo contains the source code for the TinyMPC-integrated firmware used in the Crazyflie 2.1. TinyMPC is publicly available at [tinympc.org](https://tinympc.org/).

Currently, it supports:

* Full-state quaternion-based LQR or LQI controller within the app layer in `examples/controller_lqr` and `examples/controller_lqi`.
* Full-state quaternion-based TinyMPC supporting quadratic objective, linear system, linear and second-order cone constraints on state and control input in `example/controller_mpc*`.
* Some other related stuff.

High-level commands via https://github.com/sschoedel/crazyswarm/tree/demo/socp

You might want to check different branches for the corresponding experiments. **Make sure to build and flash the source code in `examples/`.**

Feel free to reach out to the developers if you have any questions. 

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

