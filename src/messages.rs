use crate::encoder::Direction;

pub trait Codable<T> {
    fn serialize(&self) -> T;
    fn deserialize(data: T) -> Self;
}

pub enum Message {
    // Secondary -> Main
    KeyEvent(keyberon::layout::Event),
    EncoderMoved(Direction),

    // Main -> Secondary
    RgbState(bool),
    RgbHue(u8),
    RgbBright(u8),
    Awake(bool),
}

impl Codable<[u8; 2]> for Message {
    fn serialize(&self) -> [u8; 2] {
        match self {
            Self::KeyEvent(event) => [0, event.serialize()],
            Self::EncoderMoved(dir) => [1, (*dir == Direction::Clockwise) as u8],
            Self::RgbState(state) => [2, *state as u8],
            Self::RgbHue(hue) => [3, *hue],
            Self::RgbBright(bright) => [4, *bright],
            Self::Awake(state) => [5, *state as u8],
        }
    }

    fn deserialize(data: [u8; 2]) -> Self {
        match data[0] {
            0 => Self::KeyEvent(keyberon::layout::Event::deserialize(data[1])),
            1 => Self::EncoderMoved(if data[1] != 0 {
                Direction::Clockwise
            } else {
                Direction::CounterClockwise
            }),
            2 => Self::RgbState(data[1] != 0),
            3 => Self::RgbHue(data[1]),
            4 => Self::RgbBright(data[1]),
            5 => Self::Awake(data[1] != 0),
            _ => unreachable!(),
        }
    }
}

impl Codable<u8> for keyberon::layout::Event {
    // my keypress event encoding:
    // ┬
    // │ [0] unused
    // ┼
    // │ [1] press (0) / release (1)
    // ┼
    // │
    // │ [2..4] x val (max 7)
    // │
    // ┼
    // │
    // │ [5..7] y val (max 7)
    // │
    // ┴

    fn serialize(&self) -> u8 {
        let mut data = 0u8;

        if self.is_release() {
            data |= 0b01000000
        }

        data |= self.coord().0 << 3;
        data |= self.coord().1;

        data
    }

    fn deserialize(data: u8) -> Self {
        let i = (data >> 3) & 0b0000_0111;
        let j = data & 0b0000_0111;
        let release = (data >> 6) & 0b01 == 1;
        if release {
            Self::Release(i, j)
        } else {
            Self::Press(i, j)
        }
    }
}
