/// Simple frequency meter using a PIO program
/// See https://github.com/GitJer/Some_RPI-Pico_stuff/blob/main/PwmIn/PwmIn_4pins/

use irq::{handler, scope, Handler, Scope};
use rp_pico::hal::{pio::{
    PIOExt,
    PIOBuilder,

    StateMachine,
    Rx,
    StateMachineIndex,
    ValidStateMachine,
    UninitStateMachine, InstalledProgram,

    Stopped,
    Running
}, gpio::{FunctionConfig, PinId, ValidPinMode, PinMode}};

use rp_pico::hal;

use pio_proc::pio_file;

use rp_pico::hal::gpio::pin::Function;
use serde::de::value::UnitDeserializer;

use core::sync::atomic::AtomicU32;

// =========================================================
enum StateMachineVariant<SM: ValidStateMachine> {
    Stopped(StateMachine<SM, Stopped>),
    Running(StateMachine<SM, Running>),
}

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
    sm: StateMachineVariant<(P,SM)>,

    /// RX Fifo instance
    rx: Rx<(P,SM)>,

    /// Read pulsewidth value from PIO
    pulsewidth: AtomicU32,

    /// Read period value from PIO
    period: AtomicU32,
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
        mut pin: hal::gpio::Pin<Index, Mode>,
        idx: u8,
        mut sm: UninitStateMachine<(P,SM)>
    ) -> Self
    {
        unsafe {
            // Build program in state machine
            let (sm, mut rx, _) = PIOBuilder::from_program(installed.share())
                .clock_divisor(0.0)
                .jmp_pin(idx)
                .in_pin_base(idx)
                .build(sm);

            Self {
                pin,
                sm: StateMachineVariant::Stopped(sm),
                rx,
                pulsewidth: AtomicU32::new(0),
                period: AtomicU32::new(0),
            }
        }
    }

    pub fn start(&mut self) {
        if let StateMachineVariant::Stopped(sm) = self.sm {
            self.sm = StateMachineVariant::Running(sm.start());
        }
    }

    pub fn stop(&mut self) {
        if let StateMachineVariant::Running(sm) = self.sm {
            self.sm = StateMachineVariant::Stopped(sm.stop());
        }
    }
}

trait CheckPortInterrupt {
    fn check_it(it: &hal::pio::InterruptState) -> bool;
}

macro_rules! impl_check_it {
    ($smi: path, $smf: ident) => {
        impl<P: PIOExt, I: PinId, M: PinMode + ValidPinMode<I>> CheckPortInterrupt for FrequencyMeterPort<P, $smi, I, M> {
            fn check_it(it: &hal::pio::InterruptState) -> bool {
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

pub trait IndexedPin<I: PinId, M: ValidPinMode> {

}

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
    pio: P,

    /// Instance of the installed program
    installed: InstalledProgram<P>,

    /// First port instance
    port0: FrequencyMeterPort<P,hal::pio::SM0,I0,M0>,

    /// Second port instance
    port1: FrequencyMeterPort<P,hal::pio::SM1,I1,M1>,

    /// Third port instance
    port2: FrequencyMeterPort<P,hal::pio::SM2,I2,M2>,

    /// Fourth port instance
    port3: FrequencyMeterPort<P,hal::pio::SM3,I3,M3>,
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

        pin_id0: u8,
        pin_id1: u8,
        pin_id2: u8,
        pin_id3: u8,
    ) -> Self {
        let (mut pio0, sm0, sm1, sm2, sm3) = pio.split(resets);

        // Install PIO program
        let program = pio_file!("./src/application/freqmeter.pio", select_program("freqmeter"),);
        let installed = pio.install(&program.program).unwrap();

        // Return structure
        Self {
            pio,
            installed,

            port0: FrequencyMeterPort::new()
        }
    }

    fn irq_handler(&mut self) {
        // TODO //
    }
}