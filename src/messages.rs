use crate::encoder::Direction;

pub trait Codable<T>: Sized {
    fn serialize(&self) -> T;
    fn deserialize(data: T) -> Option<Self>;
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

    // Startup
    QueryMain,
    AnswerMain,
}

impl Codable<[u8; 2]> for Message {
    fn serialize(&self) -> [u8; 2] {
        // start at 1 so reading all 0s is not valid
        match self {
            Self::KeyEvent(event) => [1, event.serialize()],
            Self::EncoderMoved(dir) => [2, (*dir == Direction::Clockwise) as u8],
            Self::RgbState(state) => [3, *state as u8],
            Self::RgbHue(hue) => [4, *hue],
            Self::RgbBright(bright) => [5, *bright],
            Self::Awake(state) => [6, *state as u8],
            Self::QueryMain => [7, 0],
            Self::AnswerMain => [8, 0],
        }
    }

    fn deserialize(data: [u8; 2]) -> Option<Self> {
        match data[0] {
            1 => Some(Self::KeyEvent(keyberon::layout::Event::deserialize(data[1]).unwrap())),
            2 => {
                let dir = if data[1] != 0 {
                    Direction::Clockwise
                } else {
                    Direction::CounterClockwise
                };
                Some(Self::EncoderMoved(dir))
            },
            3 => Some(Self::RgbState(data[1] != 0)),
            4 => Some(Self::RgbHue(data[1])),
            5 => Some(Self::RgbBright(data[1])),
            6 => Some(Self::Awake(data[1] != 0)),
            7 => Some(Self::QueryMain),
            8 => Some(Self::AnswerMain),
            _ => None,
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

    fn deserialize(data: u8) -> Option<Self> {
        let i = (data >> 3) & 0b0000_0111;
        let j = data & 0b0000_0111;
        let release = (data >> 6) & 0b01 == 1;
        if release {
            Some(Self::Release(i, j))
        } else {
            Some(Self::Press(i, j))
        }
    }
}
