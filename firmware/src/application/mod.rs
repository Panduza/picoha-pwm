// ============================================================================

// HAL
use rp_pico::hal;

// Algos
use core::str::FromStr;
use core::write;
use core::fmt::Write;

// Frequency meter
mod freqmeter;
use freqmeter::FrequencyMeter;

// Protocol
mod protocol;
use protocol::{Answer, AnswerText, Command, CommandCode, Target};

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

// ============================================================================

/// Store all the usefull objects for the application
pub struct PicohaPwm
{
    /// Buffer to hold incomnig data
    usb_buffer: UsbBuffer<512>,

    /// Period value
    pub period: u32,

    /// Pulsewidth value
    pub pulsewidth: u32,

    /// Frequency meter instance
    pub freqmeter: FrequencyMeter<
        hal::pac::PIO0,
        hal::gpio::bank0::Gpio14,
        hal::gpio::bank0::Gpio15,
        hal::gpio::bank0::Gpio18,
        hal::gpio::bank0::Gpio19,
        hal::gpio::FunctionPio0,
        hal::gpio::FunctionPio0,
        hal::gpio::FunctionPio0,
        hal::gpio::FunctionPio0>,
}

// ============================================================================

/// Implementation of the App
impl PicohaPwm
{
    // ------------------------------------------------------------------------

    /// Application intialization
    pub fn new(
        resets: &mut hal::pac::RESETS,
        pio: hal::pac::PIO0,
        pins: rp_pico::Pins,
    ) -> Self {

        let freqmeter = FrequencyMeter::new(
            resets, pio, 
            pins.gpio14.into_mode(),
            pins.gpio15.into_mode(),
            pins.gpio18.into_mode(),
            pins.gpio19.into_mode()
        );

        Self {
            usb_buffer: UsbBuffer::new(),
            freqmeter,

            period: 0,
            pulsewidth: 0,
        }
    }

    // -----------------------------------------------------------------------

    /// Process PWM read command
    ///
    fn process_readpwm(&self, cmd: &Command) -> Answer {
        match protocol::Target::from_u8(cmd.target) {
            Some(x) => match x {
                Target::Input0 => Answer::mes_answer(x, self.freqmeter.port0.highp_us(), self.freqmeter.port0.lowp_us()),
                Target::Input1 => Answer::mes_answer(x, self.freqmeter.port1.highp_us(), self.freqmeter.port1.lowp_us()),
                Target::Input2 => Answer::mes_answer(x, self.freqmeter.port2.highp_us(), self.freqmeter.port2.lowp_us()),
                Target::Input3 => Answer::mes_answer(x, self.freqmeter.port3.highp_us(), self.freqmeter.port3.lowp_us()),

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

unsafe impl Send for PicohaPwm {}