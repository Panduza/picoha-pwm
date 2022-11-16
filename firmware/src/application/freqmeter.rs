/// Simple frequency meter using a PIO program
/// See https://github.com/GitJer/Some_RPI-Pico_stuff/blob/main/PwmIn/PwmIn_4pins/

use irq::{handler, scope, Handler, Scope};
use rp_pico::hal::pio::{
    PIOExt,
    PIOBuilder,

    StateMachineIndex,
    UninitStateMachine,
};

use pio::Program;
use pio_proc::pio_file;

// =========================================================

pub struct FrequencyMeterPort<P: PIOExt, SM: StateMachineIndex> {
    /// PIO State machine instance
    sm: StateMachine<(P,SM)>,

    /// RX Fifo instance
    rx: Rx<(P,SM)>,

    /// Read pulsewidth value from PIO
    pulsewidth: u32,

    /// Read period value from PIO
    period: u32,
}

impl FrequencymeterPort {
    pub fn new<P: PIOExt, SM: StateMachineIndex>(
        builder: &PIOBuilder,
        mut sm: UninitStateMachine<(P,SM)>
    ) -> Self
    {
        // Build program in state machine
        let (sm, mut rx, _) = builder.clock_divisor(0.0)
            .jmp_pin(TODO)
            .set_pins(TODO)
            .build(sm);

        Self {
            sm,
            rx,
            pulsewidth: 0u32,
            period: 0u32,
        }
    }

    pub fn start(&self) {
        self.sm.start();
    }
}


// =========================================================

pub struct FrequencyMeter {
    /// Instance of PIO periperhal
    pio: PIOExt,

    /// Port instances
    ports: [FrequencyMeterPort; 4],

    /// IRQ Handler callback
    handler: Handler,
}

impl FrequencyMeter {
    pub fn new(
        pio: PIOExt,
        sm: [StateMachine ; 4],
    ) -> Self {

        // Install PIO program
        let program   = pio_file!("freqmeter.pio", select_program("freqmeter"),);
        let installed = pio.install(&program.program).unwrap();

        // Return structure
        Self {
            pio,
            ports: [
                FrequencyMeterPort::new(installed, sm[0]),
                FrequencyMeterPort::new(installed, sm[1]),
                FrequencyMeterPort::new(installed, sm[2]),
                FrequencyMeterPort::new(installed, sm[3]),
            ]
        }
    }

    fn irq_handler(&mut self) {
        // TODO //
    }

    pub fn register_irq(&mut self, scope: &mut irq::Scope) {
        handler!(handler = || self.irq_handler());
        self.handler = handler;


    }
}