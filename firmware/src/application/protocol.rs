use serde::{Deserialize, Serialize};
use serde_repr::{Serialize_repr};
use heapless::String;

// ============================================================================

/// Max message string length in answer
pub const MAX_MSG_SIZE: usize = 128;

// ============================================================================

/// Represents the command codes as an enum
//#[derive(Deserialize_repr, Debug)]
pub enum CommandCode {
    SetDirection,
    WriteValue,
    ReadValue,
    Test,
}

impl CommandCode {
    pub fn from_u8(x: u8) -> Option<Self> {
        match x {
            0  => Some(Self::SetDirection),
            1  => Some(Self::WriteValue),
            2  => Some(Self::ReadValue),
            10 => Some(Self::Test),
            _  => None
        }
    }
}

/// Represents a command from the host
#[derive(Deserialize, Debug)]
pub struct Command {
    /// Command code as u8
    pub cod: u8,

    /// id of targetted pin (X => gpioX)
    pub pin: u8,

    /// argument value
    pub arg: u8,
}

// ============================================================================

/// Type for anwser text
pub type AnswerText = String<MAX_MSG_SIZE>;

/// Answer status code
#[derive(Serialize_repr, Debug)]
#[repr(u8)]
pub enum AnswerStatus {
    Ok    = 0u8,
    Error = 1u8
}

/// Represenattion of an answer
#[derive(Serialize, Debug)]
pub struct Answer {
    /// Status code
    pub sts: AnswerStatus,

    /// ID of target pin (X => gpioX)
    pub pin: u8,

    /// Argument value
    pub arg: u8,

    /// Text message
    pub msg: AnswerText,
}

impl Answer {
    pub fn ok(pin: u8, arg: u8, msg: AnswerText) -> Self {
        Self {
            sts: AnswerStatus::Ok,
            pin: pin,
            arg: arg,
            msg: msg,
        }
    }

    pub fn error(pin: u8, arg: u8, msg: AnswerText) -> Self {
        Self {
            sts: AnswerStatus::Error,
            pin: pin,
            arg: arg,
            msg: msg,
        }
    }
}

// ============================================================================

/// Possible argument values for pin output value
pub enum CmdPinWriteValue {
    Low,
    High
}

impl CmdPinWriteValue {
    pub fn from_u8(x: u8) -> Option<Self> {
        match x {
            0 => Some(Self::Low),
            1 => Some(Self::High),
            _ => None,
        }
    }
}


/// Possible argument values for pin direction value
pub enum CmdPinDirValue {
    PullUpInput,
    PullDownInput,
    ReadableOutput,
}

impl CmdPinDirValue {
    pub fn from_u8(x: u8) -> Option<Self> {
        match x {
            0 => Some(Self::PullUpInput),
            1 => Some(Self::PullDownInput),
            2 => Some(Self::ReadableOutput),
            _ => None
        }
    }
}