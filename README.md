Fruitfly
========

A simple alternative debug probe firmware for the RPi debugprobe. The goal of this is to provide an
alternative to the PIO-driven debugprobe firmware which may have too string timing for certain
target devices.

The probe implements both the SWD and JTAG interfaces. SWD is available using the DEBUG port as
usual, while JTAG uses both DEBUG and UART with the following pin assignment:

| SWD   | JTAG   | Pin | Wire         |
| ----- | ------ | --- | ------------ |
| SWDIO | TMS    | P14 | DEBUG yellow |
|       | TDI    | P6  | UART yellow  |
|       | TDO    | P4  | UART orange  |
| SWCLK | TCLK   | P12 | DEBUG orange |

The firmware uses the [bitbang-dap] crate that acts as an
adapter between [dap-rs] and the hardware. The hardware then only needs to implement a bidirectional
GPIO driver, and a cycle-resolution delay method.

Originally based on https://github.com/embassy-rs/eprobe/

[bitbang-dap]: https://github.com/bugadani/bitbang-dap
[dap-rs]: https://crates.io/crates/dap-rs
