// ============================================================================

// HAL
use embedded_hal::digital::v2::OutputPin;

use rp_pico::hal;

// Algos
use core::str::FromStr;
use core::write;
use core::fmt::Write;

// Protocol
mod protocol;
use protocol::{Answer, AnswerText, Command, CommandCode, Target};

// GPIO Control
mod gpio_ctrl;

const PIO_DIVISOR: f32 = 62.5; // Divisor to convert count cycles -> us

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

mod freqmeter;
use freqmeter::{FrequencyMeter, FrequencyMeterPort};

// ============================================================================

/// Store all the usefull objects for the application
pub struct PicohaPwm {
    /// To manage delay
    delay: cortex_m::delay::Delay,

    /// Buffer to hold incomnig data
    usb_buffer: UsbBuffer<512>,


    /// Period value
    pub period: u32,

    /// Pulsewidth value
    pub pulsewidth: u32,
}

// ============================================================================

/// Implementation of the App
impl PicohaPwm {

    // ------------------------------------------------------------------------

    /// Application intialization
    pub fn new(
        delay: cortex_m::delay::Delay,
        //pins: rp_pico::Pins,
    ) -> Self {
        Self {
            delay:      delay,
            usb_buffer: UsbBuffer::new(),

            period: 0,
            pulsewidth: 0,
        }
    }

    // -----------------------------------------------------------------------

    /// Process PWM read command
    ///
    fn process_readpwm(&self, _cmd: &Command) -> Answer {
        let highp = (self.period as f32) / PIO_DIVISOR;
        let lowp = (self.pulsewidth as f32) / PIO_DIVISOR;

        Answer::mes_answer(Target::Unknown, highp, lowp)
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
                                CommandCode::ReadPwm      => Some(self.process_readpwm(&data)),

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