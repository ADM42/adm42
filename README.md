![ADM42](https://adm42.dev/assets/images/social.jpg)

# ADM42 Official Repository

This repository contains both QMK and the [ADM42 source code](/keyboards/adm42/).

The implementation of ADM42 in this repository includes many improvements to QMK features that would not function correctly if combined with pure QMK ones.

In case you want a pure QMK version of the ADM42, please use the [official QMK repository](https://github.com/qmk/qmk_firmware) which already supports the ADM42 and can serve as a starting point for your modifications.

## Build

Make (after setting up your build environment):

    make adm42:default

Flashing:

    make adm42:default:flash

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information.

## Official Website

[adm42.dev](https://adm42.dev) is the official website of the ADM42 keyboard, where you can find links to this repository and all the documentation of the ADM42.
