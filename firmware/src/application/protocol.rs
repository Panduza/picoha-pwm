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
    ReadPwm,
    WritePwm,
    Test,
}

impl CommandCode {
    pub fn from_u8(x: u8) -> Option<Self> {
        match x {
            0  => Some(Self::ReadPwm),
            1  => Some(Self::WritePwm),
            10 => Some(Self::Test),

            _  => None
        }
    }
}

/// Represents the target PWM block
#[derive(Copy, Clone)]
pub enum Target {
    Unknown,

    Input0,
    Input1,
    Input2,
    Input3,

    Output0,
    Output1,
    Output2,
    Output3,
}

impl Target {
    pub fn from_u8(x: u8) -> Option<Self> {
        match x {
            0u8  => Some(Self::Input0 ),
            1u8  => Some(Self::Input1 ),
            2u8  => Some(Self::Input2 ),
            3u8  => Some(Self::Input3 ),

            10u8 => Some(Self::Output0),
            11u8 => Some(Self::Output1),
            12u8 => Some(Self::Output2),
            13u8 => Some(Self::Output3),

            _ => None,
        }
    }

    pub fn to_u8(&self) -> u8 {
        match self {
            Self::Input0  => 0u8,
            Self::Input1  => 1u8,
            Self::Input2  => 2u8,
            Self::Input3  => 3u8,

            Self::Output0 => 10u8,
            Self::Output1 => 11u8,
            Self::Output2 => 12u8,
            Self::Output3 => 13u8,

            Self::Unknown => 255u8,
        }
    }
}

/// Represents a command from the host
#[derive(Deserialize, Debug)]
pub struct Command {
    /// Command code as u8
    pub cod: u8,

    /// id of target PWM block
    pub target: u8,

    /// argument value
    pub arg: f32,
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

    /// ID of target PWM block
    pub target: u8,

    /// Text message
    pub msg: Option<AnswerText>,

    /// Period value
    pub period: Option<u32>,

    /// Pulsewidth value
    pub pulsewidth: Option<u32>,
}

impl Answer {
    pub fn ok(target: Target, msg: AnswerText) -> Self {
        Self {
            sts: AnswerStatus::Ok,
            target: target.to_u8(),
            msg: Some(msg),
            period: None,
            pulsewidth: None
        }
    }

    pub fn error(target: Target, msg: AnswerText) -> Self {
        Self {
            sts: AnswerStatus::Error,
            target: target.to_u8(),
            msg: Some(msg),
            period: None,
            pulsewidth: None,
        }
    }

    pub fn mes_answer(target: Target, period: u32, pulsewidth: u32) -> Self {
        Self {
            sts: AnswerStatus::Ok,
            target: target.to_u8(),
            msg: None,
            period: Some(period),
            pulsewidth: Some(pulsewidth),
        }
    }
}