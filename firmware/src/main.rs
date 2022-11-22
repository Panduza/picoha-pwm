#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
// Time handling traits
use embedded_time::rate::*;

use core::borrow::BorrowMut;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;

use rp_pico::hal::gpio::PushPullOutput;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use pio;
use pio_proc::pio_file;
// use rp_pico::hal::gpio::dynpin::DynPin;

// USB Device support
use usb_device::class_prelude::*;

use rp_pico::hal::gpio::{FunctionPio0, Pin};
use rp_pico::hal::pio::{PIOExt, PIOBuilder};

use irq::{handler, scope, scoped_interrupts};

use rp_pico::hal::pac::interrupt;

use rp_pico::hal::pio::{StateMachine, StateMachineIndex};

use core::sync::atomic::{AtomicU32, Ordering};

// ============================================================================

mod application;
mod platform;

use application::PicohaPwm;
use application::protocol::Answer;

use application::freqmeter::FrequencyMeter;



// ============================================================================

use cortex_m::interrupt as cmit;
use cortex_m::interrupt::CriticalSection;

use core::cell::RefCell;

use cmit::Mutex;
use cortex_m::asm;

static mut LED_PIN: RefCell<Option<Pin<hal::gpio::bank0::Gpio25, PushPullOutput>>> = RefCell::new(None);

// ============================================================================
scoped_interrupts! {
    #[allow(non_camel_case_types)]
    enum Interrupt {
        PIO0_IRQ_0,
    }

    use #[interrupt];
}
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

    let led = pins.led.into_push_pull_output();
    unsafe {
        LED_PIN.replace(Some(led));
    }

    //===============================================
    // Test for PIO PWM Input
    //===============================================

    // Init. freqmeter
    let freqmeter = Mutex::new(RefCell::new(FrequencyMeter::new(
        &mut pac.RESETS,
        pac.PIO0,

        pins.gpio14.into_mode::<FunctionPio0>(),
        pins.gpio15.into_mode::<FunctionPio0>(),
        pins.gpio18.into_mode::<FunctionPio0>(),
        pins.gpio19.into_mode::<FunctionPio0>(),
    )));

    // Init. the app
    let mut app = application::PicohaPwm::new(
        &mut pac.RESETS,
        || { cmit::free(|cs| {let fm = freqmeter.borrow(cs).borrow(); (fm.port0.highp_us(), fm.port0.lowp_us())}) },
        || { cmit::free(|cs| {let fm = freqmeter.borrow(cs).borrow(); (fm.port1.highp_us(), fm.port1.lowp_us())}) },
        || { cmit::free(|cs| {let fm = freqmeter.borrow(cs).borrow(); (fm.port2.highp_us(), fm.port2.lowp_us())}) },
        || { cmit::free(|cs| {let fm = freqmeter.borrow(cs).borrow(); (fm.port3.highp_us(), fm.port3.lowp_us())}) },
    );

    // Define handlers
    handler!(pio0_isr = || {
        cmit::free(|cs| freqmeter.borrow(cs).borrow_mut().irq_handler(cs));
    });


    scope(|scope| {
        scope.register(Interrupt::PIO0_IRQ_0, pio0_isr);

        // Unmask PIO interrupt
        unsafe {
            pac::NVIC::unmask(hal::pac::Interrupt::PIO0_IRQ_0);
        };

        // Enable interrupt
        cmit::free(|cs| {
            freqmeter.borrow(cs).borrow_mut().irq_enable(cs);
        });

        cmit::free(|cs| {
            let freqmeter  = freqmeter.borrow(cs).borrow_mut();

            freqmeter.port0.start();
            freqmeter.port1.start();
            freqmeter.port2.start();
            freqmeter.port3.start();
        });

        let mut ans_buffer = [0u8; 1024];
        loop {
            // Trigger ports
            cmit::free(|cs| {
                let mut freqmeter = freqmeter.borrow(cs).borrow_mut();

                freqmeter.port0.trigger();
                freqmeter.port1.trigger();
                freqmeter.port2.trigger();
                freqmeter.port3.trigger();
            });

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
    })
}

// ============================================================================

#[inline]
fn us_to_cycles(x: f32) -> u32{
    (x * 125.0) as u32
}

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
        unsafe {
            let mut borrow = LED_PIN.borrow_mut();

            match borrow.as_mut() {
                Some(led) => {
                    led.set_high().ok();
                    cortex_m::asm::delay(us_to_cycles(100_000.0));
                    led.set_low().ok();
                    cortex_m::asm::delay(us_to_cycles(100_000.0));
                },

                None => {}
            }
        }
        // self.led_pin.set_high().ok();
        // self.delay.delay_ms(100);
        // self.led_pin.set_low().ok();
        // self.delay.delay_ms(100);
    }
}

// ============================================================================

// End of file
