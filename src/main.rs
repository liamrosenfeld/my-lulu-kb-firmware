#![no_std]
#![no_main]

use panic_halt as _;

mod config;
mod encoder;
mod images;
mod layout;
mod messages;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(
    device = rp2040_hal::pac,
    peripherals = true,
    dispatchers = [PIO1_IRQ_0, PIO1_IRQ_1, I2C0_IRQ]
)]
mod app {
    use cortex_m::prelude::*;
    use fugit::{ExtU32, RateExtU32};
    use keyberon::{
        debounce::Debouncer,
        key_code::KbHidReport,
        layout::{CustomEvent, Event as LayoutEvent, Layout},
        matrix::Matrix,
    };
    use rp2040_hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{
            bank0, DynPinId, FunctionI2C, FunctionPio0, FunctionSio, FunctionUart, Pin, Pins,
            PullDown, PullUp, SioInput, SioOutput,
        },
        pac::UART0,
        pac::{I2C1, PIO0},
        pio::{PIOExt, SM0},
        timer::{Alarm, Alarm0, Alarm1, Alarm2, Timer},
        typelevel::{OptionTNone, OptionTSome},
        uart::{self, DataBits, Enabled, StopBits, UartConfig, UartPeripheral},
        usb::UsbBus,
        Sio, Watchdog, I2C,
    };
    use smart_leds::{
        hsv::{hsv2rgb, Hsv},
        SmartLedsWrite, RGB8,
    };
    use ssd1306::{
        mode::{BasicMode, DisplayConfig},
        prelude::I2CInterface,
        rotation::DisplayRotation,
        size::DisplaySize128x32,
        I2CDisplayInterface, Ssd1306,
    };
    use usb_device::{
        class_prelude::{UsbBusAllocator, UsbClass},
        device::{StringDescriptors, UsbDevice, UsbDeviceState},
        prelude::{UsbDeviceBuilder, UsbVidPid},
    };
    use ws2812_pio::Ws2812Direct;

    use crate::{
        config,
        encoder::{Direction, RotaryEncoder},
        images,
        layout::{self, CustomActions, LAYERS},
        messages::{Codable, Message, Modes},
        LedHandler, Side,
    };

    type Tx = Pin<bank0::Gpio0, FunctionUart, PullDown>;
    type Rx = Pin<bank0::Gpio1, FunctionUart, PullDown>;
    type UartPeriph = UartPeripheral<
        Enabled,
        UART0,
        uart::Pins<OptionTSome<Tx>, OptionTSome<Rx>, OptionTNone, OptionTNone>,
    >;

    type Sda = Pin<bank0::Gpio22, FunctionI2C, PullUp>;
    type Scl = Pin<bank0::Gpio23, FunctionI2C, PullUp>;
    type DisplayI2C = I2CInterface<I2C<I2C1, (Sda, Scl)>>;

    const CLOCK_FREQ_HZ: u32 = 12_000_000;

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus>> = None;

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, UsbBus>,
        usb_class:
            keyberon::hid::HidClass<'static, UsbBus, keyberon::keyboard::Keyboard<LedHandler>>,
        uart: UartPeriph,
        layout: Layout<12, 5, 4, CustomActions>,
        this_encoder_dir: Option<Direction>,
        other_encoder_dir: Option<Direction>,
        underglow_color: Hsv,
        underglow_state: bool,
        awake: bool,
        sleep_alarm: Alarm1,
        layer: usize,
        modes: Modes,
        #[lock_free]
        is_main: bool,
        #[lock_free]
        scan_alarm: Alarm0,
        #[lock_free]
        watchdog: Watchdog,
    }

    type ColPin = Pin<DynPinId, FunctionSio<SioInput>, PullUp>;
    type RowPin = Pin<DynPinId, FunctionSio<SioOutput>, PullDown>;

    #[local]
    struct Local {
        matrix: Matrix<ColPin, RowPin, 6, 5>,
        debouncer: Debouncer<[[bool; 6]; 5]>,
        underglow: Ws2812Direct<PIO0, SM0, Pin<bank0::Gpio29, FunctionPio0, PullDown>>,
        display: Ssd1306<DisplayI2C, DisplaySize128x32, BasicMode>,
        encoder: RotaryEncoder,
        start_alarm: Alarm2,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after watchdog reset
        unsafe {
            rp2040_hal::sio::spinlock_reset();
        }

        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);

        // ---- get clocks ----
        let clocks = init_clocks_and_plls(
            CLOCK_FREQ_HZ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // ---- get pins ----
        let sio = Sio::new(c.device.SIO);
        let pins = Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // ---- delay for power on ----
        cortex_m::asm::delay(1000);

        // ---- set up uart ----
        let uart_pins = uart::Pins::default()
            .tx(pins.gpio0.into_function())
            .rx(pins.gpio1.into_function());

        let uart = UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable(
                UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        // ---- event handling stuff ----
        let matrix = Matrix::new(
            [
                pins.gpio2.into_pull_up_input().into_dyn_pin(),
                pins.gpio3.into_pull_up_input().into_dyn_pin(),
                pins.gpio4.into_pull_up_input().into_dyn_pin(),
                pins.gpio5.into_pull_up_input().into_dyn_pin(),
                pins.gpio6.into_pull_up_input().into_dyn_pin(),
                pins.gpio7.into_pull_up_input().into_dyn_pin(),
            ],
            [
                pins.gpio14.into_push_pull_output().into_dyn_pin(),
                pins.gpio15.into_push_pull_output().into_dyn_pin(),
                pins.gpio16.into_push_pull_output().into_dyn_pin(),
                pins.gpio17.into_push_pull_output().into_dyn_pin(),
                pins.gpio18.into_push_pull_output().into_dyn_pin(),
            ],
        )
        .unwrap();

        let layout = Layout::new(&LAYERS);
        let debouncer = Debouncer::new([[false; 6]; 5], [[false; 6]; 5], config::DEBOUNCE_SCANS);

        // ---- create alarms ----
        let mut timer = Timer::new(c.device.TIMER, &mut resets, &clocks);

        let mut scan_alarm = timer.alarm_0().unwrap();
        scan_alarm.enable_interrupt();

        let mut sleep_alarm = timer.alarm_1().unwrap();
        sleep_alarm.enable_interrupt();

        let mut start_alarm = timer.alarm_2().unwrap();
        start_alarm.enable_interrupt();
        start_alarm.schedule(1.secs()).unwrap();

        // ---- set up usb ----
        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        let usb_class =
            keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, LedHandler::new());
        let usb_device = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27db),
        )
        .strings(&[StringDescriptors::default()
            .manufacturer("Boardsource")
            .product("Lulu")
            .serial_number("42")])
        .unwrap()
        .build();

        // ---- set up underglow ----
        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);
        let underglow = Ws2812Direct::new(
            pins.gpio29.into_function(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        // ---- set up display ----
        let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio22.reconfigure();
        let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio23.reconfigure();
        let i2c = I2C::i2c1(
            c.device.I2C1,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut resets,
            clocks.system_clock.freq(),
        );
        let disp_interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(disp_interface, DisplaySize128x32, DisplayRotation::Rotate0);
        display.init().unwrap();
        display.clear().unwrap();

        // ---- set up rotary encoder ----
        let encoder = RotaryEncoder::new(
            pins.gpio8.into_pull_up_input(),
            pins.gpio9.into_pull_up_input(),
        );

        // ---- set up state ---
        (
            Shared {
                usb_device,
                usb_class,
                uart,
                layout,
                this_encoder_dir: None,
                other_encoder_dir: None,
                underglow_color: config::DEFAULT_UNDERGLOW,
                underglow_state: true,
                awake: true,
                sleep_alarm,
                layer: 0,
                modes: Modes::new(),
                is_main: false,
                scan_alarm,
                watchdog,
            },
            Local {
                matrix,
                debouncer,
                underglow,
                display,
                encoder,
                start_alarm,
            },
            init::Monotonics(),
        )
    }

    /* --------------------------------- shared --------------------------------- */
    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [uart, layout, usb_device, this_encoder_dir, is_main, modes, scan_alarm, watchdog],
        local = [matrix, debouncer, encoder],
    )]
    fn scan(mut c: scan::Context) {
        // schedule the next scan
        let scan_alarm = c.shared.scan_alarm;
        scan_alarm.clear_interrupt();
        scan_alarm.schedule(config::SCAN_TIME_US.micros()).unwrap();

        // feed watchdog so it knows this did not freeze
        c.shared.watchdog.feed();

        // get keys pressed
        let keys_pressed = c
            .local
            .matrix
            .get_with_delay(|| cortex_m::asm::delay(100))
            .unwrap();
        let events = c.local.debouncer.events(keys_pressed);

        if *c.shared.is_main {
            // register the key-presses with the layout
            let windows_mode = c.shared.modes.lock(|m| m.windows);
            c.shared.layout.lock(|l| {
                for mut event in events {
                    // mirror main side events if main side is the right
                    if config::MAIN_SIDE == Side::Right {
                        event = event.transform(|i, j| (i, 11 - j));
                    }

                    // if windows mode, swap command and control
                    if windows_mode {
                        match event.coord() {
                            (4, 3) => event = event.transform(|_, _| (4, 1)),
                            (4, 1) => event = event.transform(|_, _| (4, 3)),
                            _ => {}
                        }
                    }

                    l.event(event)
                }
            });

            // read encoder
            if let Some(dir) = c.local.encoder.update() {
                c.shared.this_encoder_dir.lock(|e| *e = Some(dir));
            }

            // handle all events
            handle_events::spawn().unwrap();
        } else {
            // if not main side, just forward over events
            c.shared.uart.lock(|u| {
                // send matrix events
                for event in events {
                    let data = Message::KeyEvent(event).serialize();
                    u.write_full_blocking(&data);
                }

                // send encoder
                if let Some(dir) = c.local.encoder.update() {
                    let data = Message::EncoderMoved(dir).serialize();
                    u.write_full_blocking(&data);
                }
            });
        }
    }

    #[task(
        binds = TIMER_IRQ_2,
        priority = 1,
        shared = [is_main, usb_device, sleep_alarm, uart, scan_alarm, watchdog],
        local = [start_alarm]
    )]
    fn startup(mut c: startup::Context) {
        c.local.start_alarm.clear_interrupt();

        // set main side
        let is_main = c
            .shared
            .usb_device
            .lock(|d| d.state() == UsbDeviceState::Configured);

        // if not main, keep checking if main until confirming the other side is main
        // prevents timing issues causing two secondary sides
        if !is_main {
            // read and then write so the other half has time to respond during the start_alarm
            let is_other_main: bool = c.shared.uart.lock(|u| {
                if u.uart_is_readable() {
                    // decode
                    let mut buff = [0u8; 2];
                    let Ok(()) = u.read_full_blocking(&mut buff) else {
                        return false;
                    };
                    let msg = Message::deserialize(buff);

                    // handle
                    if let Some(Message::AnswerMain) = msg {
                        return true;
                    }
                }
                false
            });

            if !is_other_main {
                c.shared.uart.lock(|u| {
                    if u.uart_is_writable() {
                        u.write_full_blocking(&Message::QueryMain.serialize());
                    }
                });

                c.local.start_alarm.schedule(100.millis()).unwrap();
                return;
            }
        }

        // set is_main after confirming
        *c.shared.is_main = is_main;

        // start scan and watchdog
        c.shared
            .scan_alarm
            .schedule(config::SCAN_TIME_US.micros())
            .unwrap();
        c.shared.watchdog.start(config::WATCHDOG_MS.millis());

        // since manual uart reading is over, enable interrupts to enable read_msg
        c.shared.uart.lock(|u| u.enable_rx_interrupt());

        // set to default appearance
        update_display::spawn().unwrap();
        update_underglow::spawn().unwrap();

        // sleep managed by main side
        if is_main {
            c.shared
                .sleep_alarm
                .lock(|a| a.schedule(config::SLEEP_TIMEOUT_SEC.secs()).unwrap());
        }
    }

    /* -------------------------- split communications -------------------------- */

    #[task(
        binds = UART0_IRQ,
        priority = 3,
        shared = [uart, other_encoder_dir, awake, underglow_state, underglow_color, modes]
    )]
    fn read_msg(mut c: read_msg::Context) {
        if c.shared.uart.lock(|u| u.uart_is_readable()) {
            // decode message
            let mut buff = [0u8; 2];
            let Ok(()) = c.shared.uart.lock(|u| u.read_full_blocking(&mut buff)) else {
                return;
            };
            let Some(msg) = Message::deserialize(buff) else {
                return;
            };

            // execute
            match msg {
                Message::KeyEvent(mut event) => {
                    // mirror if secondary side is right
                    if config::MAIN_SIDE == Side::Left {
                        event = event.transform(|i, j| (i, 11 - j));
                    }
                    register_keyboard_event::spawn(event).unwrap();
                }
                Message::EncoderMoved(dir) => c.shared.other_encoder_dir.lock(|e| *e = Some(dir)),
                Message::RgbState(state) => {
                    c.shared.underglow_state.lock(|s| *s = state);
                    update_underglow::spawn().unwrap();
                }
                Message::RgbHue(hue) => {
                    c.shared.underglow_color.lock(|hsv| hsv.hue = hue);
                    update_underglow::spawn().unwrap()
                }
                Message::RgbBright(bright) => {
                    c.shared.underglow_color.lock(|hsv| hsv.val = bright);
                    update_underglow::spawn().unwrap()
                }
                Message::Awake(state) => {
                    c.shared.awake.lock(|a| *a = state);
                    update_underglow::spawn().unwrap();
                    update_display::spawn().unwrap();
                }
                Message::SetModes(modes) => {
                    c.shared.modes.lock(|m| *m = modes);
                    update_display::spawn().unwrap();
                }
                Message::QueryMain => {
                    // discard capacity error because we only need to send one of these total
                    let _ = answer_if_main::spawn();
                }
                Message::AnswerMain => {}
            };
        }
    }

    #[task(priority = 1, shared = [is_main])]
    fn answer_if_main(c: answer_if_main::Context) {
        if *c.shared.is_main {
            let _ = send_msg::spawn(Message::AnswerMain);
        }
    }

    #[task(priority = 2, capacity = 8, shared = [uart])]
    fn send_msg(mut c: send_msg::Context, msg: Message) {
        c.shared
            .uart
            .lock(|u| u.write_full_blocking(&msg.serialize()));
    }

    /* ------------------------------ primary side ------------------------------ */
    #[task(
        priority = 2,
        capacity = 8,
        shared = [usb_class, layout, layer, this_encoder_dir, other_encoder_dir, sleep_alarm, awake],
    )]
    fn handle_events(mut c: handle_events::Context) {
        let mut had_event = false;

        // handle custom events
        if let CustomEvent::Press(event) = c.shared.layout.lock(|l| l.tick()) {
            layout::handle_custom_action(*event);
            had_event = true;
        };

        // generate report (layout + encoder)
        let mut report: KbHidReport = c.shared.layout.lock(|l| {
            let mut keycodes = l.keycodes().peekable();
            had_event |= keycodes.peek().is_some(); // check if at least one keypress
            keycodes.collect()
        });

        // react to layer
        let current_layer = c.shared.layout.lock(|l| l.current_layer());
        let layer_changed = c.shared.layer.lock(|l| {
            let changed = *l != current_layer;
            *l = current_layer;
            changed
        });
        let already_awake = c.shared.awake.lock(|a| *a);
        if layer_changed {
            if already_awake {
                update_display::spawn().unwrap();
            }
            had_event = true;
        }

        // handle encoder
        if let Some(dir) = c.shared.this_encoder_dir.lock(|e| e.take()) {
            layout::handle_encoder(config::MAIN_SIDE, dir, current_layer, &mut report);
            had_event = true;
        }
        if let Some(dir) = c.shared.other_encoder_dir.lock(|e| e.take()) {
            layout::handle_encoder(config::MAIN_SIDE.other(), dir, current_layer, &mut report);
            had_event = true;
        }

        // send usb keyboard report
        if c.shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
        }

        // wake up if had event
        if had_event {
            // set new sleep alarm
            c.shared
                .sleep_alarm
                .lock(|a| a.schedule(config::SLEEP_TIMEOUT_SEC.secs()).unwrap());

            // don't need to do anything more if already awake
            if already_awake {
                return;
            }

            // wake up this side
            c.shared.awake.lock(|a| *a = true);
            update_underglow::spawn().unwrap();
            update_display::spawn().unwrap();

            // notify other side
            let _ = send_msg::spawn(Message::Awake(true));
        }
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_device, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let mut usb_d = c.shared.usb_device;
        let mut usb_c = c.shared.usb_class;
        usb_d.lock(|d| {
            usb_c.lock(|c| {
                if d.poll(&mut [c]) {
                    c.poll();
                }
            })
        });
    }

    #[task(
        binds = TIMER_IRQ_1,
        priority = 1,
        shared = [awake, sleep_alarm]
    )]
    fn bedtime(mut c: bedtime::Context) {
        c.shared.sleep_alarm.lock(|a| a.clear_interrupt());

        // shut down this side
        c.shared.awake.lock(|a| *a = false);
        update_display::spawn().unwrap();
        update_underglow::spawn().unwrap();

        // notify the other side
        let _ = send_msg::spawn(Message::Awake(false));
    }

    #[task(priority = 1, shared = [modes])]
    fn toggle_windows_mode(mut c: toggle_windows_mode::Context) {
        let new_state = c.shared.modes.lock(|m| {
            m.windows = !m.windows;
            *m
        });
        let _ = send_msg::spawn(Message::SetModes(new_state));
    }

    #[task(priority = 1, capacity=8, shared = [modes])]
    fn set_caps_lock(mut c: set_caps_lock::Context, state: bool) {
        let new_state = c.shared.modes.lock(|m| {
            if state == m.caps_lock {
                return None;
            }

            m.caps_lock = state;
            Some(*m)
        });

        if let Some(new_state) = new_state {
            let _ = send_msg::spawn(Message::SetModes(new_state));
        }
    }

    /* ----------------------------- secondary side ----------------------------- */

    /// Register a key event with the layout to be later processed
    #[task(priority=1, capacity=8, shared=[layout])]
    fn register_keyboard_event(mut c: register_keyboard_event::Context, event: LayoutEvent) {
        c.shared.layout.lock(|l| l.event(event));
    }

    /* -------------------------------- underglow ------------------------------- */
    #[task(priority = 1, shared = [underglow_state])]
    fn toggle_underglow(mut c: toggle_underglow::Context) {
        // update this side
        let state = c.shared.underglow_state.lock(|s| {
            *s = !*s;
            *s
        });
        update_underglow::spawn().unwrap();

        // notify secondary side
        let _ = send_msg::spawn(Message::RgbState(state));
    }

    #[task(priority = 1, shared = [underglow_color])]
    fn shift_hue(mut c: shift_hue::Context, up: bool) {
        // update this side
        let new_hue = c.shared.underglow_color.lock(|hsv| {
            hsv.hue = if up {
                hsv.hue.wrapping_add(3)
            } else {
                hsv.hue.wrapping_sub(3)
            };
            hsv.hue
        });
        update_underglow::spawn().unwrap();

        // notify other side
        let _ = send_msg::spawn(Message::RgbHue(new_hue));
    }

    #[task(priority = 1, shared = [underglow_color])]
    fn shift_bright(mut c: shift_bright::Context, up: bool) {
        // update this side
        let new_bright = c.shared.underglow_color.lock(|hsv| {
            hsv.val = if up {
                hsv.val.saturating_add(10)
            } else {
                hsv.val.saturating_sub(10)
            };
            hsv.val
        });
        update_underglow::spawn().unwrap();

        // notify other side
        let _ = send_msg::spawn(Message::RgbBright(new_bright));
    }

    #[task(priority = 1, shared=[underglow_state, underglow_color, awake], local = [underglow])]
    fn update_underglow(mut c: update_underglow::Context) {
        // set to new state
        let state = c.shared.underglow_state.lock(|s| *s);
        let awake = c.shared.awake.lock(|a| *a);
        let color = match state && awake {
            false => RGB8::default(),
            true => hsv2rgb(c.shared.underglow_color.lock(|x| *x)),
        };
        c.local.underglow.write([color; 35].into_iter()).unwrap();
    }

    /* --------------------------------- display -------------------------------- */
    #[task(priority = 1, shared = [is_main, layer, awake, modes], local = [display])]
    fn update_display(mut c: update_display::Context) {
        // toggle display depending on asleep or not
        if c.shared.awake.lock(|a| *a) {
            c.local.display.set_display_on(true).unwrap();
        } else {
            c.local.display.set_display_on(false).unwrap();
            return;
        }

        // draw to display
        let data = if *c.shared.is_main {
            let layer = c.shared.layer.lock(|l| *l);
            match layer {
                0 => images::LAYER1,
                1 => images::LAYER2,
                2 => images::LAYER3,
                3 => images::LAYER4,
                _ => images::LOGO,
            }
        } else {
            let modes = c.shared.modes.lock(|m| *m);
            if modes.caps_lock {
                images::CAPS_LOCK
            } else if modes.windows {
                images::WINDOWS_MODE
            } else {
                images::LOGO
            }
        };
        c.local.display.draw(data).unwrap();
    }
}

#[derive(PartialEq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    const fn other(&self) -> Side {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        }
    }
}

pub struct LedHandler;

impl LedHandler {
    fn new() -> Self {
        Self
    }
}

impl keyberon::keyboard::Leds for LedHandler {
    fn caps_lock(&mut self, state: bool) {
        app::set_caps_lock::spawn(state).unwrap();
    }
}
