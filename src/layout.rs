use keyberon::action::Action;
use keyberon::layout::{layout, Layers};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum CustomActions {
    Uf2,
    TogRGB,
}

const UF2: Action<CustomActions> = Action::Custom(CustomActions::Uf2);
const TOG_RGB: Action<CustomActions> = Action::Custom(CustomActions::TogRGB);

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
        [n        n      Grave      |        ~        n            n       '['      ']'       n        n        n],
        [n        t        t        t        t        t            t        t        t        t        t        n]
    }
    {
        [n        n        n        n        n        n            n        n        n        n        n        n],
        [n        n        7        8        9        -            n    CapsLock     n        n        n        n],
        [BSpace   0        4        5        6        +            n      Left      Down      Up     Right      n],
        [n        n        1        2        3        E            n        n        n        n        n        n],
        [n        t        t        t        t        t            t        t        t        t        t        n]
    }
    {
        [n        n        n        n        n        n            n        n        n        n        n    {UF2}],
        [n        F1       F2       F3       F4       n            n        n        n        n        n        n],
        [n        F5       F6       F7       F7       n            n        n        n        n        n        n],
        [n        F8      F10      F11      F12       n            n        n        n        n        n        n],
        [n        t        t        t        t        t            t        t        t        t        t        n]
    }
};
