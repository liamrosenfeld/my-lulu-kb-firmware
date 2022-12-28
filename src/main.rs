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
        gpio::{self, bank0, DynPin, FunctionUart, Pin, Pins},
        pac::UART0,
        pac::{I2C1, PIO0},
        pio::{PIOExt, SM0},
        timer::{Alarm, Alarm0, Alarm1, Alarm2, Timer},
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
        device::{UsbDevice, UsbDeviceState},
        prelude::{UsbDeviceBuilder, UsbVidPid},
    };
    use ws2812_pio::Ws2812Direct as Ws2812Pio;

    use crate::{
        config,
        encoder::{Direction, RotaryEncoder},
        images,
        layout::{self, CustomActions, LAYERS},
        messages::{Codable, Message},
        Side,
    };

    type UartPeriph = UartPeripheral<
        Enabled,
        UART0,
        uart::Pins<Pin<bank0::Gpio0, FunctionUart>, Pin<bank0::Gpio1, FunctionUart>, (), ()>,
    >;

    type Sda = Pin<bank0::Gpio22, gpio::Function<gpio::I2C>>;
    type Scl = Pin<bank0::Gpio23, gpio::Function<gpio::I2C>>;
    type DisplayI2C = I2CInterface<I2C<I2C1, (Sda, Scl)>>;

    const CLOCK_FREQ_HZ: u32 = 12_000_000;

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus>> = None;

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, UsbBus>,
        usb_class: keyberon::hid::HidClass<'static, UsbBus, keyberon::keyboard::Keyboard<()>>,
        uart: UartPeriph,
        layout: Layout<12, 5, 4, CustomActions>,
        this_encoder_dir: Option<Direction>,
        other_encoder_dir: Option<Direction>,
        underglow_color: Hsv,
        underglow_state: bool,
        awake: bool,
        sleep_alarm: Alarm1,
        layer: usize,
        #[lock_free]
        is_main: bool,
    }

    #[local]
    struct Local {
        matrix: Matrix<DynPin, DynPin, 6, 5>,
        debouncer: Debouncer<[[bool; 6]; 5]>,
        scan_alarm: Alarm0,
        underglow: Ws2812Pio<PIO0, SM0, bank0::Gpio29>,
        display: Ssd1306<DisplayI2C, DisplaySize128x32, BasicMode>,
        encoder: RotaryEncoder,
        start_alarm: Alarm2,
        watchdog: Watchdog,
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
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }

        // ---- set up uart ----
        let uart_pins = uart::Pins::default()
            .tx(pins.gpio0.into_mode())
            .rx(pins.gpio1.into_mode());

        let mut uart = UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable(
                UartConfig::new(9600u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        uart.enable_rx_interrupt();

        // ---- event handling stuff ----
        let matrix = Matrix::new(
            [
                pins.gpio2.into_pull_up_input().into(),
                pins.gpio3.into_pull_up_input().into(),
                pins.gpio4.into_pull_up_input().into(),
                pins.gpio5.into_pull_up_input().into(),
                pins.gpio6.into_pull_up_input().into(),
                pins.gpio7.into_pull_up_input().into(),
            ],
            [
                pins.gpio14.into_push_pull_output().into(),
                pins.gpio15.into_push_pull_output().into(),
                pins.gpio16.into_push_pull_output().into(),
                pins.gpio17.into_push_pull_output().into(),
                pins.gpio18.into_push_pull_output().into(),
            ],
        )
        .unwrap();

        let layout = Layout::new(&LAYERS);
        let debouncer = Debouncer::new([[false; 6]; 5], [[false; 6]; 5], config::DEBOUNCE_SCANS);

        // ---- create alarms ----
        let mut timer = Timer::new(c.device.TIMER, &mut resets);

        let mut scan_alarm = timer.alarm_0().unwrap();
        scan_alarm.enable_interrupt();

        let mut sleep_alarm = timer.alarm_1().unwrap();
        sleep_alarm.enable_interrupt();

        let mut start_alarm = timer.alarm_2().unwrap();
        start_alarm.enable_interrupt();

        // start alarms
        scan_alarm.schedule(config::SCAN_TIME_US.micros()).unwrap();
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
        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_device = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27db),
        )
        .manufacturer("Boardsource")
        .product("Lulu")
        .serial_number("42")
        .build();

        // ---- set up underglow ----
        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);
        let underglow = Ws2812Pio::new(
            pins.gpio29.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        // ---- set up display ----
        let i2c = I2C::i2c1(
            c.device.I2C1,
            pins.gpio22.into_mode(),
            pins.gpio23.into_mode(),
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

        // ---- start watchdog ----
        watchdog.start(config::WATCHDOG_MS.millis());

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
                is_main: false,
            },
            Local {
                scan_alarm,
                matrix,
                debouncer,
                underglow,
                display,
                encoder,
                start_alarm,
                watchdog,
            },
            init::Monotonics(),
        )
    }

    /* --------------------------------- shared --------------------------------- */
    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [uart, layout, usb_device, this_encoder_dir, is_main],
        local = [matrix, debouncer, scan_alarm, encoder, watchdog],
    )]
    fn scan(mut c: scan::Context) {
        // schedule the next scan
        let scan_alarm = c.local.scan_alarm;
        scan_alarm.clear_interrupt();
        scan_alarm.schedule(config::SCAN_TIME_US.micros()).unwrap();

        // feed watchdog so it knows this did not freeze
        c.local.watchdog.feed();

        // get keys pressed
        let keys_pressed = c.local.matrix.get().unwrap();
        let events = c.local.debouncer.events(keys_pressed);

        if *c.shared.is_main {
            // register the key-presses with the layout
            c.shared.layout.lock(|l| {
                for mut event in events {
                    // mirror main side events if main side is the right
                    if config::MAIN_SIDE == Side::Right {
                        event = event.transform(|i, j| (i, 11 - j));
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
        shared = [is_main, usb_device, sleep_alarm],
        local = [start_alarm])
    ]
    fn startup(mut c: startup::Context) {
        c.local.start_alarm.clear_interrupt();

        // set main side
        let is_main = c
            .shared
            .usb_device
            .lock(|d| d.state() == UsbDeviceState::Configured);
        *c.shared.is_main = is_main;

        // set to default
        update_display::spawn().unwrap();
        update_underglow::spawn().unwrap();

        // sleep managed my main side
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
        shared = [uart, other_encoder_dir, awake, underglow_state, underglow_color])
    ]
    fn read_msg(mut c: read_msg::Context) {
        if c.shared.uart.lock(|u| u.uart_is_readable()) {
            // decode message
            let mut buff = [0u8; 2];
            let Ok(()) = c.shared.uart.lock(|u| u.read_full_blocking(&mut buff)) else {
                return;
            };
            let msg = Message::deserialize(buff);

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
            };
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
    #[task(priority = 1, shared = [is_main, layer, awake], local = [display])]
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
            images::LOGO
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
