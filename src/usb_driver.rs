use core::mem::MaybeUninit;

use defmt::warn;
use embassy_usb::driver::Driver;
use embassy_usb::driver::{Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::msos::{self};
use embassy_usb::types::{InterfaceNumber, StringIndex};
use embassy_usb::{Builder, Handler};

pub struct CmsisDapV2State {
    control: MaybeUninit<Control>,
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

impl CmsisDapV2State {
    pub const fn new() -> Self {
        CmsisDapV2State {
            control: MaybeUninit::uninit(),
        }
    }
}

pub struct CmsisDapV2Class<'d, D: Driver<'d>> {
    _interface: InterfaceNumber,
    _name: StringIndex,
    read_ep: D::EndpointOut,
    write_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> CmsisDapV2Class<'d, D> {
    /// Creates a new CdcAcmClass with the provided UsbBus and `max_packet_size` in bytes. For
    /// full-speed devices, `max_packet_size` has to be one of 8, 16, 32 or 64.
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut CmsisDapV2State,
        max_packet_size: u16,
    ) -> Self {
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
        let interface_number = interface.interface_number();
        let mut alt = interface.alt_setting(0xFF, 0, 0, Some(iface_string));
        let read_ep = alt.endpoint_bulk_out(max_packet_size);
        let write_ep = alt.endpoint_bulk_in(max_packet_size);
        drop(function);

        builder.handler(state.control.write(Control { iface_string }));

        CmsisDapV2Class {
            _interface: interface_number,
            _name: iface_string,
            read_ep,
            write_ep,
        }
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        for chunk in data.chunks(64) {
            self.write_ep.write(chunk).await?;
        }
        if data.len() % 64 == 0 {
            self.write_ep.write(&[]).await?;
        }
        Ok(())
    }

    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        let mut n = 0;

        loop {
            let i = self.read_ep.read(&mut data[n..]).await?;
            n += i;
            if i < 64 {
                return Ok(n);
            }
        }
    }
}
