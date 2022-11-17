#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;

use rp_pico::hal::gpio::PushPullOutput;
use rp_pico::hal::pio::ValidStateMachine;
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

// ============================================================================

use cortex_m::interrupt as cmit;
use cortex_m::interrupt::Mutex;

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

    //===============================================
    // Test for PIO PWM Input
    //===============================================

    let (mut pio0, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
    let program   = pio_file!("src/application/freqmeter.pio", select_program("freqmeter"),);
    let installed = pio0.install(&program.program).unwrap();

    let _in0: hal::gpio::Pin<_, FunctionPio0> = pins.gpio14.into_mode();
    let _in1: hal::gpio::Pin<_, FunctionPio0> = pins.gpio15.into_mode();
    let _in2: hal::gpio::Pin<_, FunctionPio0> = pins.gpio18.into_mode();
    let _in3: hal::gpio::Pin<_, FunctionPio0> = pins.gpio19.into_mode();

    let in0_id = 14u8;
    //let in1_id = 15u8;
    //let in2_id = 18u8;
    //let in3_id = 19u8;

    let mut led: hal::gpio::Pin<_, PushPullOutput> = pins.led.into_mode();

    let builder = PIOBuilder::from_program(installed);

    let (sm0, mut rx0, _) = builder
        .jmp_pin(in0_id)
        .in_pin_base(in0_id)
        .clock_divisor(1.0)
        .build(sm0);

    let pio_mtx = Mutex::new(pio0);
    
    //===============================================

    // Init. the app
    let mut app = application::PicohaPwm::new(
        cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer()) // Append delay feature to the app
    );

    // Define handlers
    let mut period = AtomicU32::new(0);
    let mut pulsewidth = AtomicU32::new(0);


    handler!(pio0_isr = || {
        let it_state  = cmit::free(|cs| pio_mtx.borrow(cs).interrupts().get(0).unwrap().state());

        if(it_state.sm0()) {
            if let Some(x) = rx0.read() {
                period.store(x, Ordering::Relaxed);
            }

            if let Some(x) = rx0.read() {
                pulsewidth.store(x, Ordering::Relaxed);
            }
        }

        led.toggle().ok();
        cmit::free(|cs| pio_mtx.borrow(cs).clear_irq(0x1));
    });


    scope(|scope| {
        scope.register(Interrupt::PIO0_IRQ_0, pio0_isr);

        // Unmask PIO interrupt
        unsafe {
            pac::NVIC::unmask(hal::pac::Interrupt::PIO0_IRQ_0);
        };

        // Enable interrupt
        cmit::free(|cs| {
            let it = pio_mtx.borrow(cs).interrupts().get(0).unwrap();
            it.enable_sm_interrupt(0)
        });

        sm0.start();

        let mut ans_buffer = [0u8; 1024];
        loop {
            // Poll PIO
            app.period     = period.load(Ordering::Relaxed);
            app.pulsewidth = pulsewidth.load(Ordering::Relaxed);

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
