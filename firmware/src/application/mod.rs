// ============================================================================

// HAL
use rp_pico::hal;

// Algos
use core::str::FromStr;
use core::write;
use core::fmt::Write;

// Frequency meter
pub mod freqmeter;

// Protocol
pub mod protocol;
use protocol::{Answer, AnswerText, Command, CommandCode, Target};

use cortex_m::interrupt::CriticalSection;

// ============================================================================

enum CmdError {
    /// Arg value is invalid
    ArgError(u8),

    /// A HAL error occured
    HalError(hal::gpio::Error),
}

// ============================================================================

mod buffer;
use buffer::UsbBuffer;

// ============================================================================

/// Store all the usefull objects for the application
pub struct PicohaPwm<FuncReadPort0, FuncReadPort1, FuncReadPort2, FuncReadPort3>
where
    FuncReadPort0: Fn() -> (f32, f32),
    FuncReadPort1: Fn() -> (f32, f32),
    FuncReadPort2: Fn() -> (f32, f32),
    FuncReadPort3: Fn() -> (f32, f32),
{
    /// Buffer to hold incomnig data
    usb_buffer: UsbBuffer<512>,

    /// Callbacks to read ports
    read_port0: FuncReadPort0,
    read_port1: FuncReadPort1,
    read_port2: FuncReadPort2,
    read_port3: FuncReadPort3,
}

// ============================================================================

/// Implementation of the App
impl<FuncReadPort0, FuncReadPort1, FuncReadPort2, FuncReadPort3> PicohaPwm<FuncReadPort0, FuncReadPort1, FuncReadPort2, FuncReadPort3> where
    FuncReadPort0: Fn() -> (f32, f32,),
    FuncReadPort1: Fn() -> (f32, f32,),
    FuncReadPort2: Fn() -> (f32, f32,),
    FuncReadPort3: Fn() -> (f32, f32,),
{
    // ------------------------------------------------------------------------

    /// Application intialization
    pub fn new(
        resets: &mut hal::pac::RESETS,

        read_port0: FuncReadPort0,
        read_port1: FuncReadPort1,
        read_port2: FuncReadPort2,
        read_port3: FuncReadPort3,
    ) -> Self {

        Self {
            usb_buffer: UsbBuffer::new(),

            read_port0,
            read_port1,
            read_port2,
            read_port3,
        }
    }

    // -----------------------------------------------------------------------

    /// Process PWM read command
    ///
    fn process_readpwm(&self, cmd: &Command, cs: &CriticalSection) -> Answer {
        match protocol::Target::from_u8(cmd.target) {
            Some(x) => match x {
                Target::Input0 => { let (highp, lowp) = (self.read_port0)(); Answer::mes_answer(x, highp, lowp) },
                Target::Input1 => { let (highp, lowp) = (self.read_port1)(); Answer::mes_answer(x, highp, lowp) },
                Target::Input2 => { let (highp, lowp) = (self.read_port2)(); Answer::mes_answer(x, highp, lowp) },
                Target::Input3 => { let (highp, lowp) = (self.read_port3)(); Answer::mes_answer(x, highp, lowp) },

                other => Answer::error(other, AnswerText::from_str("Expected input target").unwrap()),
            }

            None => {
                let mut txt = AnswerText::new();
                write!(txt, "Uknown target {}", cmd.target).unwrap();
                Answer::error(Target::Unknown, txt)
            }
        }
    }

    // -----------------------------------------------------------------------

    /// Process incoming commands
    ///
    pub fn update_command_processing(&mut self) -> Option<Answer> {
        let mut cmd_buffer = [0u8; 512];

        match self.usb_buffer.get_command(&mut cmd_buffer) {
            None => None,
            Some(cmd_end_index) => {
                let cmd_slice_ref = &cmd_buffer[0..cmd_end_index];

                match serde_json_core::de::from_slice::<Command>(cmd_slice_ref) {
                    // Process parsing error
                    Err(_e) => {
                        let mut txt = AnswerText::new();
                        write!(txt, "Error: {}", _e).unwrap();

                        Some(Answer::error(Target::Unknown, txt))
                    },

                    // Process received command
                    Ok(cmd) => {
                        let data = &cmd.0;

                        match CommandCode::from_u8(data.cod) {
                            Some(x) => match x {
                                CommandCode::Test         => Some(Answer::ok(Target::Unknown, AnswerText::from_str("").unwrap())),
                                CommandCode::ReadPwm      => Some(cortex_m::interrupt::free(|cs| self.process_readpwm(&data, cs))),

                                _  => {
                                    let mut txt = AnswerText::new();
                                    write!(txt, "Unimplemented code: {}", data.cod).unwrap();
                                    Some(Answer::error(Target::Unknown, txt))
                                }
                            },

                            None => {
                                let mut txt = AnswerText::new();
                                write!(txt, "Unknown command code: {}", data.cod).unwrap();
                                Some(Answer::error(Target::Unknown, txt))
                            },
                        }
                    },
                }
            }
        }
    }

    // ------------------------------------------------------------------------

    /// Feed input buffer
    ///
    pub fn feed_cmd_buffer(&mut self, buf: &[u8], count: usize) {
        self.usb_buffer.load(buf, count);
    }
}