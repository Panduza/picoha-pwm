/// Simple frequency meter using a PIO program
/// Florian Dupeyron <florian.dupeyron@mugcat.fr>, November 2022
/// See https://github.com/GitJer/Some_RPI-Pico_stuff/blob/main/PwmIn/PwmIn_4pins/

use cortex_m::interrupt::CriticalSection;

use rp_pico::hal::{pio::{
    PIOExt,
    PIOBuilder,

    StateMachine,
    Rx,
    Tx,
    StateMachineIndex,
    ValidStateMachine,
    UninitStateMachine, InstalledProgram,

    Stopped,
    Running
}, gpio::{PinId, ValidPinMode, PinMode}};

use rp_pico::hal;

use pio_proc::pio_file;

use core::sync::atomic::{AtomicU32, Ordering};
use core::cell::Cell;


const PIO_DIVISOR: f32 = 62.5; // Divisor to transform tick count into microseconds

// =========================================================
enum StateMachineVariant<SM: ValidStateMachine> {
    Empty,
    Stopped(StateMachine<SM, Stopped>),
    Running(StateMachine<SM, Running>),
}

impl<SM: ValidStateMachine> Default for StateMachineVariant<SM> {
    fn default() -> Self {
        Self::Empty
    }
}

// =========================================================

//pub struct FrequencyMeterPort<P: PIOExt, SM: StateMachineIndex, PinFunction: FunctionConfig> {
pub struct FrequencyMeterPort<P, SM, Index, Mode> 
where
    P: PIOExt,
    SM: StateMachineIndex,
    Index: PinId,
    Mode: PinMode + ValidPinMode<Index>
{
    /// Pin
    pin: rp_pico::hal::gpio::Pin<Index, Mode>,
    
    /// PIO State machine instance
    //sm: StateMachine<(P, SM), Stopped>,
    sm: Cell<StateMachineVariant<(P,SM)>>,

    /// RX Fifo instance
    rx: Rx<(P,SM)>,

    /// TX Fifo instance
    tx: Tx<(P,SM)>,

    /// High period tick count
    highp: AtomicU32,

    /// Low period tick count
    lowp: AtomicU32,
}

impl<P, SM, Index, Mode> FrequencyMeterPort<P, SM, Index, Mode>
where
    P: PIOExt,
    SM: StateMachineIndex,
    Index: PinId,
    Mode: PinMode + ValidPinMode<Index>,
{
    pub fn new(
        installed: &InstalledProgram<P>,
        pin: hal::gpio::Pin<Index, Mode>,
        sm: UninitStateMachine<(P,SM)>
    ) -> Self
    {
        unsafe {
            // Build program in state machine
            let (sm, rx, tx) = PIOBuilder::from_program(installed.share())
                .jmp_pin(Index::DYN.num)
                .in_pin_base(Index::DYN.num)
                .clock_divisor(1.0)
                .build(sm);

            Self {
                pin,
                sm: Cell::new(StateMachineVariant::Stopped(sm)),
                rx,
                tx,
                highp: AtomicU32::new(0),
                lowp: AtomicU32::new(0),
            }
        }
    }

    pub fn start(&self) {
        let sm = self.sm.take();
        
        if let StateMachineVariant::Stopped(sm) = sm {
            self.sm.replace(StateMachineVariant::Running(sm.start()));
        }
    }

    pub fn stop(&self) {
        let sm = self.sm.take();

        if let StateMachineVariant::Running(sm) = sm {
            self.sm.replace(StateMachineVariant::Stopped(sm.stop()));
        }
    }

    pub fn trigger(&mut self) {
        self.tx.write(0xdeabeefu32);
    }

    pub fn irq_handler(&mut self) {
        if let Some(x) = self.rx.read() {
            self.highp.store(x, Ordering::Relaxed);
        }

        if let Some(x) = self.rx.read() {
            self.lowp.store(x, Ordering::Relaxed);
        }
    }

    pub fn highp_us(&self) -> f32 {
        (self.highp.load(Ordering::Relaxed) as f32) / PIO_DIVISOR
    }

    pub fn lowp_us(&self) -> f32 {
        (self.lowp.load(Ordering::Relaxed) as f32) / PIO_DIVISOR
    }

    pub fn highp_tick(&self) -> u32 {
        self.highp.load(Ordering::Relaxed)
    }

    pub fn lowp_tick(&self) -> u32 {
        self.lowp.load(Ordering::Relaxed)
    }

}

trait CheckPortInterrupt {
    fn check_it(&self, it: &hal::pio::InterruptState) -> bool;
}

macro_rules! impl_check_it {
    ($smi: path, $smf: ident) => {
        impl<P: PIOExt, I: PinId, M: PinMode + ValidPinMode<I>> CheckPortInterrupt for FrequencyMeterPort<P, $smi, I, M> {
            fn check_it(&self, it: &hal::pio::InterruptState) -> bool {
                it.$smf()
            }
        }
    }
}

impl_check_it!(hal::pio::SM0, sm0);
impl_check_it!(hal::pio::SM1, sm1);
impl_check_it!(hal::pio::SM2, sm2);
impl_check_it!(hal::pio::SM3, sm3);

// =========================================================

pub struct FrequencyMeter<P,I0,I1,I2,I3,M0,M1,M2,M3>
where
    P: PIOExt,

    I0: PinId,
    I1: PinId,
    I2: PinId,
    I3: PinId,

    M0: PinMode + ValidPinMode<I0>,
    M1: PinMode + ValidPinMode<I1>,
    M2: PinMode + ValidPinMode<I2>,
    M3: PinMode + ValidPinMode<I3>,
{
    /// Instance of PIO periperhal
    pio: cortex_m::interrupt::Mutex<hal::pio::PIO<P>>,

    /// Instance of the installed program
    installed: InstalledProgram<P>,

    /// First port instance
    pub port0: FrequencyMeterPort<P,hal::pio::SM0,I0,M0>,

    /// Second port instance
    pub port1: FrequencyMeterPort<P,hal::pio::SM1,I1,M1>,

    /// Third port instance
    pub port2: FrequencyMeterPort<P,hal::pio::SM2,I2,M2>,

    /// Fourth port instance
    pub port3: FrequencyMeterPort<P,hal::pio::SM3,I3,M3>,
}

impl<P,I0,I1,I2,I3,M0,M1,M2,M3> FrequencyMeter<P,I0,I1,I2,I3,M0,M1,M2,M3>
where
    P: PIOExt,

    I0: PinId,
    I1: PinId,
    I2: PinId,
    I3: PinId,

    M0: PinMode + ValidPinMode<I0>,
    M1: PinMode + ValidPinMode<I1>,
    M2: PinMode + ValidPinMode<I2>,
    M3: PinMode + ValidPinMode<I3>,
{
    pub fn new(
        resets: &mut hal::pac::RESETS,
        pio: P,

        pin0: hal::gpio::Pin<I0, M0>,
        pin1: hal::gpio::Pin<I1, M1>,
        pin2: hal::gpio::Pin<I2, M2>,
        pin3: hal::gpio::Pin<I3, M3>,
    ) -> Self {
        let (mut pio, sm0, sm1, sm2, sm3) = pio.split(resets);

        // Install PIO program
        let program = pio_file!("./src/application/freqmeter.pio", select_program("freqmeter"),);
        let installed = pio.install(&program.program).unwrap();

        let port0 = FrequencyMeterPort::new(&installed, pin0, sm0);
        let port1 = FrequencyMeterPort::new(&installed, pin1, sm1);
        let port2 = FrequencyMeterPort::new(&installed, pin2, sm2);
        let port3 = FrequencyMeterPort::new(&installed, pin3, sm3);

        // Return structure
        Self {
            pio: cortex_m::interrupt::Mutex::new(pio),
            installed,
            port0,
            port1,
            port2,
            port3,
        }
    }

    pub fn irq_handler(&mut self, cs: &CriticalSection) {
        let pio = self.pio.borrow(cs);
        let state = pio.interrupts().get(0).unwrap().state();

        if self.port0.check_it(&state) {
            self.port0.irq_handler();
        }

        if self.port1.check_it(&state) {
            self.port1.irq_handler();
        }

        if self.port2.check_it(&state) {
            self.port2.irq_handler();
        }

        if self.port3.check_it(&state) {
            self.port3.irq_handler();
        }

        pio.clear_irq(pio.get_irq_raw());
    }

    pub fn irq_enable(&self, cs: &CriticalSection) {
        let pio = self.pio.borrow(cs);
        let it  = pio.interrupts().get(0).unwrap();

        it.enable_sm_interrupt(0);
        it.enable_sm_interrupt(1);
        it.enable_sm_interrupt(2);
        it.enable_sm_interrupt(3);
    }
}