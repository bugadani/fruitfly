#![no_std]
#![no_main]

use bitbang_dap::{BitbangAdapter, DelayCycles, InputOutputPin};
use dap_rs::dap::{self, Dap, DapLeds, DapVersion, DelayNs};
use dap_rs::swo::Swo;
use defmt::{todo, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::gpio::{self, Flex, Output, Pin};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler};
use embassy_rp::{Peri, bind_interrupts};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Config, Handler};
use embassy_usb::{
    class::cdc_acm::State,
    driver::{Endpoint, EndpointError, EndpointIn, EndpointOut},
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create the driver, from the HAL.
    let driver = UsbDriver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xf569, 0x0001);
    config.manufacturer = Some("me");
    config.product = Some("Fruitfly debug probe CMSIS-DAP");
    config.serial_number = Some("12345678");
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
    let iface_string = builder.string();
    let mut function = builder.function(0xFF, 0, 0);
    function.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    function.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        // CMSIS-DAP standard GUID, from https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__ConfigUSB__gr.html
        msos::PropertyData::RegMultiSz(&["{CDB3B5AD-293B-4663-AA36-1AAE46463776}"]),
    ));
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, Some(iface_string));
    let mut read_ep = alt.endpoint_bulk_out(64);
    let mut write_ep = alt.endpoint_bulk_in(64);
    drop(function);

    static CONTROL_HANDLER: StaticCell<Control> = StaticCell::new();
    builder.handler(CONTROL_HANDLER.init(Control { iface_string }));

    // CDC - dummy class to get things working for now. Windows needs more than one interface
    // to load usbccgp.sys, which is necessary for nusb to be able to list interfaces.
    static STATIC_STATE: StaticCell<State> = StaticCell::new();
    let state = STATIC_STATE.init(State::new());
    _ = CdcAcmClass::new(&mut builder, state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Now create the CMSIS-DAP handler.
    let t_nrst = p.PIN_29; // Disconnected
    let t_jtdi = p.PIN_6;
    let t_jtms_swdio = p.PIN_14;
    let t_jtck_swclk = p.PIN_12;
    let t_jtdo = p.PIN_4;

    let deps = BitbangAdapter::new(
        IoPin::new(t_nrst),
        IoPin::new(t_jtdi),
        IoPin::new(t_jtms_swdio),
        IoPin::new(t_jtck_swclk),
        IoPin::new(t_jtdo),
        BitDelay,
    );
    let mut dap = Dap::new(
        deps,
        Leds {
            red: Output::new(p.PIN_2, gpio::Level::High),
            green: Output::new(p.PIN_15, gpio::Level::Low),
            yellow: Output::new(p.PIN_16, gpio::Level::Low),
        },
        BitDelay,
        None::<NoSwo>,
        "Fruitfly CMSIS-DAP",
    );

    // Do stuff with the class!
    let dap_fut = async {
        let mut req = [0u8; 1024];
        let mut resp = [0u8; 1024];
        loop {
            read_ep.wait_enabled().await;

            let req_len = match read_packet(&mut read_ep, &mut req).await {
                Ok(n) => n,
                Err(e) => {
                    warn!("failed to read from USB: {:?}", e);
                    continue;
                }
            };
            let resp_len = dap.process_command(&req[..req_len], &mut resp, DapVersion::V2);

            if let Err(e) = write_packet(&mut write_ep, &resp[..resp_len]).await {
                warn!("failed to write to USB: {:?}", e);
                continue;
            }
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, dap_fut).await;
}

struct Control {
    iface_string: StringIndex,
}

impl Handler for Control {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.iface_string {
            Some("CMSIS-DAP v2 Interface")
        } else {
            warn!("unknown string index requested");
            None
        }
    }
}

async fn read_packet(ep: &mut impl EndpointOut, buf: &mut [u8]) -> Result<usize, EndpointError> {
    let mut n = 0;

    loop {
        let i = ep.read(&mut buf[n..]).await?;
        n += i;
        if i < 64 {
            return Ok(n);
        }
    }
}

async fn write_packet(ep: &mut impl EndpointIn, buf: &[u8]) -> Result<(), EndpointError> {
    for chunk in buf.chunks(64) {
        ep.write(chunk).await?;
    }
    if buf.len() % 64 == 0 {
        ep.write(&[]).await?;
    }
    Ok(())
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
    red: Output<'a>,
    green: Output<'a>,
    yellow: Output<'a>,
}

impl DapLeds for Leds<'_> {
    fn react_to_host_status(&mut self, host_status: dap::HostStatus) {
        match host_status {
            dap::HostStatus::Connected(connected) => {
                if connected {
                    self.green.set_high();
                } else {
                    self.green.set_low();
                }
            }
            dap::HostStatus::Running(running) => {
                if running {
                    self.yellow.set_high();
                } else {
                    self.yellow.set_low();
                }
            }
        }
    }
}

struct NoSwo;

impl Swo for NoSwo {
    fn set_transport(&mut self, transport: dap_rs::swo::SwoTransport) {
        todo!()
    }

    fn set_mode(&mut self, mode: dap_rs::swo::SwoMode) {
        todo!()
    }

    fn set_baudrate(&mut self, baudrate: u32) -> u32 {
        todo!()
    }

    fn set_control(&mut self, control: dap_rs::swo::SwoControl) {
        todo!()
    }

    fn polling_data(&mut self, buf: &mut [u8]) -> u32 {
        todo!()
    }

    fn streaming_data(&mut self) {
        todo!()
    }

    fn is_active(&self) -> bool {
        todo!()
    }

    fn bytes_available(&self) -> u32 {
        todo!()
    }

    fn buffer_size(&self) -> u32 {
        todo!()
    }

    fn support(&self) -> dap_rs::swo::SwoSupport {
        todo!()
    }

    fn status(&mut self) -> dap_rs::swo::SwoStatus {
        todo!()
    }
}
