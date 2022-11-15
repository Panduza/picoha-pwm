#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use pio;
use pio_proc;
// use rp_pico::hal::gpio::dynpin::DynPin;

// USB Device support
use usb_device::class_prelude::*;

use rp_pico::hal::gpio::{FunctionPio0, Pin};
use rp_pico::hal::pio::{PIOExt, PIOBuilder};

// ============================================================================

mod application;
mod platform;

// ============================================================================

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac  = pac::Peripherals::take().unwrap();
    let     core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut usb_serial = platform::init_usb_serial(&usb_bus);
    let mut usb_device = platform::init_usb_device(&usb_bus);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Test PIO stuff
    let _led: Pin<_, FunctionPio0> = pins.led.into_mode();
    let led_pin_id = 25;

    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "set pins, 1 [31]",
        "nop [31]",
        "nop [31]",
        "nop [31]",
        "set pins, 0 [31]",
        "nop [31]",
        "nop [31]",
        "nop [31]",
        ".wrap",
    );

    let(mut pio, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (int,frac) = (0,0); // as slow as possible (0 is interpreted as 65536)
    let (mut sm, _,_) = PIOBuilder::from_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor(65535.0)
        .build(sm0);

    sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
    sm.start();

    // Init. the app
    let mut app = application::PicohaPwm::new(
        cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer()), // Append delay feature to the app
    );

    // Run the app
    let mut ans_buffer = [0u8; 1024];
    loop {
        // Update USB
        if usb_device.poll(&mut [&mut usb_serial]) {
            let mut buf = [0u8; 1024];
            match usb_serial.read(&mut buf) {
                Err(_) => {}
                Ok(0)  => {}

                Ok(count) => {
                    app.feed_cmd_buffer(&buf, count);
                }
            }
        }

        // Update app command process
        match app.update_command_processing() {
            None           => {},
            Some(response) => {
                match serde_json_core::to_slice(&response, &mut ans_buffer) {
                    Ok(size) => {
                        ans_buffer[size] = '\n' as u8;
                        usb_serial.write(&ans_buffer[0..(size+1)]).unwrap();
                    }

                    Err(_) => {} // Ignore errors for now
                }
            }
        }
    }
}

// ============================================================================

// PANIC MANAGEMENT
use core::panic::PanicInfo;
#[panic_handler]
unsafe fn panic(_info: &PanicInfo) -> ! {
    //let mut tmp_buf = [0u8; 20];

    //self.usb_serial.write(b"{\"log\":\"").ok();
    //self.usb_serial.write(b"PANIC! => ").ok();
    //self.usb_serial
    //    .write(_info.location().unwrap().file().as_bytes())
    //    .ok();
    //self.usb_serial.write(b":").ok();
    //self.usb_serial
    //    .write(_info.location().unwrap().line().numtoa(10, &mut tmp_buf))
    //    .ok();
    //self.usb_serial.write(b"\"}\r\n").ok();
    loop {
        // self.led_pin.set_high().ok();
        // self.delay.delay_ms(100);
        // self.led_pin.set_low().ok();
        // self.delay.delay_ms(100);
    }
}

// ============================================================================

// End of file
