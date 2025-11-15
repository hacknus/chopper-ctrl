use core::cell::RefCell;

// use arrform::{arrform, ArrForm};
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::pac::interrupt;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

// Make USB serial device globally available
pub static G_USB_SERIAL: Mutex<RefCell<Option<SerialPort<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));

// Make USB device globally available
pub static G_USB_DEVICE: Mutex<RefCell<Option<UsbDevice<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));

pub const USB_MESSAGE_LEN: usize = 256; // Length of the USB message buffer

pub static mut EP_MEMORY: [u32; USB_MESSAGE_LEN] = [0; USB_MESSAGE_LEN];
static mut USB_BUS: Option<UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> = None;

#[allow(dead_code)]
pub unsafe fn usb_init(usb: USB) {
    USB_BUS = Some(stm32f4xx_hal::otg_fs::UsbBusType::new(usb, &mut EP_MEMORY));
    let usb_bus = USB_BUS.as_ref().unwrap();
    let serial_port = SerialPort::new(&usb_bus);
    let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("University of Bern")
            .product("Chopper")
            .serial_number("C25")])
        .unwrap()
        .build();

    cortex_m::interrupt::free(|cs| {
        *G_USB_SERIAL.borrow(cs).borrow_mut() = Some(serial_port);
        *G_USB_DEVICE.borrow(cs).borrow_mut() = Some(usb_dev);
    });
}

#[allow(dead_code)]
pub fn usb_read(message: &mut [u8; USB_MESSAGE_LEN]) -> bool {
    cortex_m::interrupt::free(|cs| {
        *message = [0; USB_MESSAGE_LEN];
        return match G_USB_SERIAL.borrow(cs).borrow_mut().as_mut() {
            None => false,
            Some(serial) => match serial.read(message) {
                Ok(a) => {
                    if a < USB_MESSAGE_LEN {
                        true
                    } else {
                        false
                    }
                }
                Err(_err) => {
                    // usb_println(arrform!(128, "Serial read Error: {:?}", err).as_str());
                    // let _ = serial.flush();
                    false
                }
            },
        };
    })
}

#[allow(dead_code)]
pub fn usb_println(string: &str) {
    cortex_m::interrupt::free(|cs| match G_USB_SERIAL.borrow(cs).borrow_mut().as_mut() {
        None => {}
        Some(serial) => {
            let string_bytes = string.as_bytes();
            let mut index = 0;
            let length = 32;
            loop {
                if string_bytes.len() > index + length {
                    let bytes_to_send = &string_bytes[index..index + length];
                    serial.write(bytes_to_send).unwrap_or(0);
                    serial.flush().unwrap_or(());
                } else {
                    let bytes_to_send = &string_bytes[index..];
                    serial.write(bytes_to_send).unwrap_or(0);
                    serial.flush().unwrap_or(());
                    break;
                }
                index += length;
            }
            serial.write(b"\r\n").unwrap_or(0);
            serial.flush().unwrap_or(());
        }
    })
}

#[allow(dead_code)]
pub fn usb_print(string: &str) {
    cortex_m::interrupt::free(|cs| match G_USB_SERIAL.borrow(cs).borrow_mut().as_mut() {
        None => {}
        Some(serial) => {
            serial.write(string.as_bytes()).unwrap_or(0);
            serial.flush().unwrap_or(());
        }
    })
}

#[interrupt]
#[allow(non_snake_case)]
fn OTG_FS() {
    cortex_m::interrupt::free(|cs| {
        match G_USB_DEVICE.borrow(cs).borrow_mut().as_mut() {
            None => {}
            Some(usb_dev) => {
                match G_USB_SERIAL.borrow(cs).borrow_mut().as_mut() {
                    None => {}
                    Some(serial) => {
                        // do this regularly to keep connection to USB host
                        usb_dev.poll(&mut [serial]);
                    }
                }
            }
        }
    });
}
