use core::option::Option;
use embedded_hal::digital::v2::InputPin;
use rp2040_hal::gpio::{bank0, Pin, PullUpInput};

#[derive(PartialEq)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}

impl TryFrom<u8> for Direction {
    fn try_from(s: u8) -> Result<Self, ()> {
        match s {
            0b0001 | 0b0111 | 0b1000 | 0b1110 => Ok(Direction::Clockwise),
            0b0010 | 0b0100 | 0b1011 | 0b1101 => Ok(Direction::CounterClockwise),
            _ => Err(()),
        }
    }

    type Error = ();
}

pub struct RotaryEncoder {
    pin_a: Pin<bank0::Gpio8, PullUpInput>,
    pin_b: Pin<bank0::Gpio9, PullUpInput>,
    state: u8,
}

impl RotaryEncoder {
    pub fn new(
        pin_a: Pin<bank0::Gpio8, PullUpInput>,
        pin_b: Pin<bank0::Gpio9, PullUpInput>,
    ) -> Self {
        Self {
            pin_a,
            pin_b,
            state: 0,
        }
    }

    pub fn update(&mut self) -> Option<Direction> {
        // turn existing state into old state
        self.state <<= 2;

        // move to new state
        self.state |= self.pin_a.is_high().unwrap() as u8;
        self.state |= (self.pin_b.is_high().unwrap() as u8) << 1;

        self.state.try_into().ok()
    }
}
