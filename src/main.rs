#![no_std]
#![no_main]

use dap_rs::dap::{self, Dap, DapLeds, DapVersion, DelayNs};
use dap_rs::driver::bitbang::{BitbangAdapter, DelayCycles, InputOutputPin};
use dap_rs::jtag::TapConfig;
use dap_rs::swo::NoSwo;
use defmt::{unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::flash::Flash;
use embassy_rp::gpio::{Flex, Level, Output, Pin};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler};
use embassy_rp::{Peri, bind_interrupts};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::cdc_acm::State;
use embassy_usb::class::cmsis_dap_v2::{CmsisDapV2Class, State as CmsisDapV2State};
use embassy_usb::msos::windows_version;
use embassy_usb::{Builder, Config};
use heapless::String;
use static_cell::{ConstStaticCell, StaticCell};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Configuration options:
    // 1. Pinout
    let t_nrst = p.PIN_29; // Disconnected
    let t_jtdi = p.PIN_6; // UART yellow
    let t_jtms_swdio = p.PIN_14; // DEBUG yellow
    let t_jtck_swclk = p.PIN_12; // DEBUG orange
    let t_jtdo = p.PIN_4; // UART orange
    //let t_swo // Not supported yet

    // 2. Max JTAG scan chain
    const MAX_SCAN_CHAIN_LENGTH: usize = 8;

    // 3. USB configuration
    const MANUFACTURER: &str = "me";
    const PRODUCT: &str = "Fruitfly debug probe CMSIS-DAP";

    // Create the driver, from the HAL.
    let driver = UsbDriver::new(p.USB, Irqs);

    // Get unique id from flash
    let mut flash = Flash::<_, _, 0>::new_blocking(p.FLASH);

    let mut uid = [0; 8];
    flash.blocking_unique_id(&mut uid).unwrap();

    static SERIAL: ConstStaticCell<String<16>> = ConstStaticCell::new(String::<16>::new());
    let serial = SERIAL.take();
    for b in uid {
        let lower = b & 0x0F;
        let upper = (b >> 4) & 0x0F;
        fn hex(nibble: u8) -> char {
            if nibble < 10 {
                (b'0' + nibble) as char
            } else {
                (b'A' + nibble - 10) as char
            }
        }
        unwrap!(serial.push(hex(upper)));
        unwrap!(serial.push(hex(lower)));
    }

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some(MANUFACTURER);
    config.product = Some(PRODUCT);
    config.serial_number = Some(serial);
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 196]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut CONFIG_DESC.init([0; 256])[..],
        &mut BOS_DESC.init([0; 256])[..],
        &mut MSOS_DESC.init([0; 196])[..],
        &mut CONTROL_BUF.init([0; 128])[..],
    );

    builder.msos_descriptor(windows_version::WIN8_1, 0);

    // DAP - Custom Class 0
    static DAP_STATE: ConstStaticCell<CmsisDapV2State> =
        ConstStaticCell::new(CmsisDapV2State::new());
    let mut dap_class = CmsisDapV2Class::new(&mut builder, DAP_STATE.take(), 64, true);

    // CDC - dummy class to get things working for now. Windows needs more than one interface
    // to load usbccgp.sys, which is necessary for nusb to be able to list interfaces.
    static CDC_STATE: ConstStaticCell<State> = ConstStaticCell::new(State::new());
    _ = CdcAcmClass::new(&mut builder, CDC_STATE.take(), 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Now create the CMSIS-DAP handler.

    static SCAN_CHAIN: ConstStaticCell<[TapConfig; MAX_SCAN_CHAIN_LENGTH]> =
        ConstStaticCell::new([TapConfig::INIT; MAX_SCAN_CHAIN_LENGTH]);
    let deps = BitbangAdapter::new(
        IoPin::new(t_nrst),
        IoPin::new(t_jtdi),
        IoPin::new(t_jtms_swdio),
        IoPin::new(t_jtck_swclk),
        IoPin::new(t_jtdo),
        BitDelay,
        SCAN_CHAIN.take(),
    );
    let mut dap = Dap::new(
        deps,
        Leds {
            _power: Output::new(p.PIN_2, Level::High),
            green: Output::new(p.PIN_15, Level::Low),
            yellow: Output::new(p.PIN_16, Level::Low),
        },
        BitDelay,
        NoSwo,
        concat!("2.1.0, Adaptor version ", env!("CARGO_PKG_VERSION")),
    );

    let dap_fut = async {
        let mut req = [0u8; 1024];
        let mut resp = [0u8; 1024];
        loop {
            dap_class.wait_connection().await;

            let Ok(req_len) = dap_class.read_packet(&mut req).await.inspect_err(|e| {
                warn!("failed to read from USB: {:?}", e);
            }) else {
                continue;
            };

            let resp_len = dap.process_command(&req[..req_len], &mut resp, DapVersion::V2);

            if let Err(e) = dap_class.write_packet(&resp[..resp_len]).await {
                warn!("failed to write to USB: {:?}", e);
                continue;
            }
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, dap_fut).await;
}

struct BitDelay;

impl DelayNs for BitDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_cycles((ns as u64 * self.cpu_clock() as u64 / 1_000_000_000_u64) as u32);
    }
}

impl DelayCycles for BitDelay {
    fn delay_cycles(&mut self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }

    fn cpu_clock(&self) -> u32 {
        // This function is used to calculate the number of cycles to wait in a SWD/JTAG clock
        // cycle, so we don't actually have to return the real CPU frequency.
        // cortex_m __delay divides by 2 and Cortex-M0+ needs 4 CPU cycles per delay loop iteration.
        125_000_000 / 2
    }
}

struct IoPin<'a> {
    pin: Flex<'a>,
}

impl<'a> IoPin<'a> {
    fn new(pin: Peri<'a, impl Pin>) -> Self {
        Self {
            pin: Flex::new(pin),
        }
    }
}

impl InputOutputPin for IoPin<'_> {
    fn set_as_output(&mut self) {
        self.pin.set_as_output();
    }

    fn set_high(&mut self, high: bool) {
        match high {
            true => self.pin.set_high(),
            false => self.pin.set_low(),
        }
    }

    fn set_as_input(&mut self) {
        self.pin.set_as_input();
    }

    fn is_high(&mut self) -> bool {
        self.pin.is_high()
    }
}

struct Leds<'a> {
    _power: Output<'a>,
    green: Output<'a>,
    yellow: Output<'a>,
}

impl DapLeds for Leds<'_> {
    fn react_to_host_status(&mut self, host_status: dap::HostStatus) {
        match host_status {
            dap::HostStatus::Connected(c) => self.green.set_level(Level::from(c)),
            dap::HostStatus::Running(r) => self.yellow.set_level(Level::from(r)),
        }
    }
}
