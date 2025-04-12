Fruitfly
========

A simple alternative debug probe firmware for the RPi debugprobe. The goal of this is to provide a
cheap and readily available probe for [`dap-rs`]() development, and to hack the debugprobe into
eventually implement JTAG using the debug and UART connectors together.

The firmware uses the [bitbang-dap](https://github.com/bugadani/bitbang-dap) crate that acts as an
adapter between dap-rs and the hardware. The hardware then only needs to implement a bidirectional
GPIO driver, and a cycle-resolution delay method.
