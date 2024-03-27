use keyberon::action::Action;
use keyberon::key_code::{KbHidReport, KeyCode};
use keyberon::layout::{layout, Layers};

use crate::app;
use crate::encoder::Direction;
use crate::Side;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum CustomActions {
    Uf2,
    TogRGB,
    TogWinMode,
}

const UF2: Action<CustomActions> = Action::Custom(CustomActions::Uf2);
const TOG_RGB: Action<CustomActions> = Action::Custom(CustomActions::TogRGB);
const TOG_WIN: Action<CustomActions> = Action::Custom(CustomActions::TogWinMode);

#[rustfmt::skip]
pub static LAYERS: Layers<12, 5, 4, CustomActions> = layout! {
    {
        [Escape  Kb1      Kb2      Kb3      Kb4      Kb5          Kb6      Kb7      Kb8      Kb9      Kb0   Equal],
        [Tab      Q        W        F        P        G            J        L        U        Y    SColon   Minus],
        [BSpace   A        R        S        T        D            H        N        E        I        O    Quote],
        [LShift   Z        X        C        V        B            K        M      Comma     Dot    Slash  RShift],
        [n       LCtrl    LAlt    LGui     Enter    Mute       {TOG_RGB}  Space     (1)      (2)      (3)       n]
    }
    {
        [n        n        n        n        n        n            n        n        n        n        n        n],
        [n        n      Bslash     ^        &        n            n       '{'      '}'       n        n        n],
        [BSpace   !        @        #        $        %            n       '('      ')'       <        >        n],
        [LShift   n      Grave      |        ~        n            n       '['      ']'       n        n        n],
        [n       LCtrl    LAlt    LGui     Enter      t            t        t        t        t        t        n]
    }
    {
        [n        n        n        n        n        n            n        n        n        n        n        n],
        [n        n        7        8        9        -            n    CapsLock     n        n        n        n],
        [BSpace   0        4        5        6        +            n      Left      Down      Up     Right      n],
        [LShift   n        1        2        3        E            n        n        n        n        n        n],
        [n       LCtrl    LAlt    LGui     Enter      t            t        t        t        t        t        n]
    }
    {
        [n        n        n        n        n        n            n        n        n        n        n    {UF2}],
        [Delete   F1       F2       F3       F4       n            n    {TOG_WIN}    n        n        n        n],
        [BSpace   F5       F6       F7       F7       n            n        n        n        n        n        n],
        [LShift   F8       F10      F11      F12      n            n        n        n        n        n        n],
        [n       LCtrl    LAlt    LGui     Enter      t            t        t        t        t        t        n]
    }
};

#[inline(always)]
pub fn handle_encoder(side: Side, dir: Direction, layer: usize, report: &mut KbHidReport) {
    match side {
        Side::Left => match dir {
            Direction::Clockwise => report.pressed(KeyCode::VolUp),
            Direction::CounterClockwise => report.pressed(KeyCode::VolDown),
        },
        Side::Right => {
            let up = dir == Direction::Clockwise;
            if layer == 3 {
                app::shift_bright::spawn(up).unwrap()
            } else {
                app::shift_hue::spawn(up).unwrap()
            }
        }
    }
}

#[inline(always)]
pub fn handle_custom_action(action: CustomActions) {
    match action {
        CustomActions::Uf2 => rp2040_hal::rom_data::reset_to_usb_boot(0, 0),
        CustomActions::TogRGB => app::toggle_underglow::spawn().unwrap(),
        CustomActions::TogWinMode => app::toggle_windows_mode::spawn().unwrap(),
    }
}
