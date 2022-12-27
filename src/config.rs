use smart_leds::hsv::Hsv;
use crate::Side;

pub const MAIN_SIDE: Side = Side::Left;

pub const SCAN_TIME_US: u32 = 1000;
pub const SLEEP_TIMEOUT_SEC: u32 = 150;
pub const WATCHDOG_MS: u32 = 50;
pub const DEBOUNCE_SCANS: u16 = 5;

pub const DEFAULT_UNDERGLOW: Hsv = Hsv {
    hue: 180,
    sat: 255,
    val: 50,
};
