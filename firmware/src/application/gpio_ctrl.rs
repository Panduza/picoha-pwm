use rp_pico::hal::gpio::dynpin::DynPin;
use rp_pico::Pins;

/// Aliases the pins into DynPin
pub struct GpioController {
    // TODO // implement declaration using a macro?
    gpio0: DynPin,
    gpio1: DynPin,
    gpio2: DynPin,
    gpio3: DynPin,
    gpio4: DynPin,
    gpio5: DynPin,
    gpio6: DynPin,
    gpio7: DynPin,
    gpio8: DynPin,
    gpio9: DynPin,
    gpio10: DynPin,
    gpio11: DynPin,
    gpio12: DynPin,
    gpio13: DynPin,
    gpio14: DynPin,
    gpio15: DynPin,
    gpio16: DynPin,
    gpio17: DynPin,
    gpio18: DynPin,
    gpio19: DynPin,
    gpio20: DynPin,
    gpio21: DynPin,
    gpio22: DynPin,
    led: DynPin,
}

impl GpioController {
    pub fn new(pins: Pins) -> Self {
        Self {
            gpio0:  pins.gpio0.into(),
            gpio1:  pins.gpio1.into(),
            gpio2:  pins.gpio2.into(),
            gpio3:  pins.gpio3.into(),
            gpio4:  pins.gpio4.into(),
            gpio5:  pins.gpio5.into(),
            gpio6:  pins.gpio6.into(),
            gpio7:  pins.gpio7.into(),
            gpio8:  pins.gpio8.into(),
            gpio9:  pins.gpio9.into(),
            gpio10: pins.gpio10.into(),
            gpio11: pins.gpio11.into(),
            gpio12: pins.gpio12.into(),
            gpio13: pins.gpio13.into(),
            gpio14: pins.gpio14.into(),
            gpio15: pins.gpio15.into(),
            gpio16: pins.gpio16.into(),
            gpio17: pins.gpio17.into(),
            gpio18: pins.gpio18.into(),
            gpio19: pins.gpio19.into(),
            gpio20: pins.gpio20.into(),
            gpio21: pins.gpio21.into(),
            gpio22: pins.gpio22.into(),
            led:    pins.led.into(),
        }
    }

    pub fn borrow(&mut self, idx: u8) -> Option<&mut DynPin> {
        match idx {
            0  => Some(&mut self.gpio0 ),
            1  => Some(&mut self.gpio1 ),
            2  => Some(&mut self.gpio2 ),
            3  => Some(&mut self.gpio3 ),
            4  => Some(&mut self.gpio4 ),
            5  => Some(&mut self.gpio5 ),
            6  => Some(&mut self.gpio6 ),
            7  => Some(&mut self.gpio7 ),
            8  => Some(&mut self.gpio8 ),
            9  => Some(&mut self.gpio9 ),
            10 => Some(&mut self.gpio10),
            11 => Some(&mut self.gpio11),
            12 => Some(&mut self.gpio12),
            13 => Some(&mut self.gpio13),
            14 => Some(&mut self.gpio14),
            15 => Some(&mut self.gpio15),
            16 => Some(&mut self.gpio16),
            17 => Some(&mut self.gpio17),
            18 => Some(&mut self.gpio18),
            19 => Some(&mut self.gpio19),
            20 => Some(&mut self.gpio20),
            21 => Some(&mut self.gpio21),
            22 => Some(&mut self.gpio22),
            25 => Some(&mut self.led   ),
            _  => None,
        }
    }
}