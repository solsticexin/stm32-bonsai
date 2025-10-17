#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Print panic message to probe console
use panic_probe as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use core::fmt::Write;

    use heapless::String;
    use nb::block;
    use stm32f1xx_hal::{
        gpio::{gpioa::PA1, GpioExt, Output, PushPull},
        pac::{self, USART1},
        prelude::*,
        rcc::Config as RccConfig,
        serial::{Config as SerialConfig, Event as SerialEvent, Rx, Serial, Tx},
        timer::{CounterHz, Event as TimerEvent, Timer},
    };

    use cortex_m::asm;
    use serde::Deserialize;
    use serde_json_core::de::from_str;

    const LINE_BUFFER_CAPACITY: usize = 192;
    const TX_BUFFER_CAPACITY: usize = 192;
    const TELEMETRY_INTERVAL_MS: u32 = 5_000;
    const MIN_PULSE_MS: u32 = 1;
    const MAX_PULSE_MS: u32 = 10_000;

    #[derive(Clone, Copy, Default)]
    pub struct ActuatorState {
        water: bool,
        light: bool,
        fan: bool,
        buzzer: bool,
    }

    impl ActuatorState {
        fn water(&mut self) -> &mut bool {
            &mut self.water
        }

        fn light(&mut self) -> &mut bool {
            &mut self.light
        }

        fn fan(&mut self) -> &mut bool {
            &mut self.fan
        }
    }

    #[derive(Deserialize)]
    struct CommandFrame<'a> {
        #[serde(rename = "type")]
        kind: &'a str,
        #[serde(default)]
        target: Option<&'a str>,
        #[serde(default)]
        action: Option<&'a str>,
        #[serde(default)]
        time: Option<u32>,
    }

    #[shared]
    struct Shared {
        tx: Tx<USART1>,
        actuators: ActuatorState,
        buzzer_pin: PA1<Output<PushPull>>,
        buzzer_pulse_ms: Option<u32>,
    }

    #[local]
    struct Local {
        rx: Rx<USART1>,
        line_buf: String<LINE_BUFFER_CAPACITY>,
        telemetry_timer: CounterHz<pac::TIM2>,
        telemetry_ticks: u32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx
            .device
            .RCC
            .freeze(
                RccConfig::hse(8.MHz())
                    .sysclk(72.MHz())
                    .pclk1(36.MHz())
                    .pclk2(72.MHz()),
                &mut flash.acr,
            );

        let mut gpioa = ctx.device.GPIOA.split(&mut rcc);
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc);

        let _water_pin = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
        let _light_pin = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        let _fan_pin = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
        let mut buzzer_pin = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
        buzzer_pin.set_high();

        let _soil_pin = gpioa.pa0.into_analog(&mut gpioa.crl);
        let _dht11_pin = gpiob.pb5.into_open_drain_output(&mut gpiob.crl);
        let _i2c_scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let _i2c_sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let tx_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx_pin = gpioa.pa10;

        let mut serial = Serial::new(
            ctx.device.USART1,
            (tx_pin, rx_pin),
            SerialConfig::default().baudrate(115_200.bps()),
            &mut rcc,
        );
        serial.listen(SerialEvent::Rxne);
        let (tx, rx) = serial.split();

        let mut telemetry_timer = Timer::new(ctx.device.TIM2, &mut rcc).counter_hz();
        telemetry_timer.listen(TimerEvent::Update);
        let _ = telemetry_timer.start(1_000.Hz());

        (
            Shared {
                tx,
                actuators: ActuatorState::default(),
                buzzer_pin,
                buzzer_pulse_ms: None,
            },
            Local {
                rx,
                line_buf: String::new(),
                telemetry_timer,
                telemetry_ticks: 0,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    #[task(
        binds = USART1,
        priority = 3,
        shared = [tx, actuators, buzzer_pin, buzzer_pulse_ms],
        local = [rx, line_buf]
    )]
    fn on_usart1(mut ctx: on_usart1::Context) {
        match ctx.local.rx.read() {
            Ok(byte) => match byte {
                b'\n' => {
                    if !ctx.local.line_buf.is_empty() {
                        let line = ctx.local.line_buf.as_str();
                        if let Ok((frame, _)) = from_str::<CommandFrame>(line) {
                            if frame.kind == "cmd" {
                                if let (Some(target), Some(action)) =
                                    (frame.target, frame.action)
                                {
                                    let (result, message) =
                                        if target == "buzzer" {
                                            let duration = frame.time;
                                            let outcome = (
                                                &mut ctx.shared.actuators,
                                                &mut ctx.shared.buzzer_pin,
                                                &mut ctx.shared.buzzer_pulse_ms,
                                            )
                                                .lock(|actuators, buzzer_pin, pulse_ms| {
                                                    handle_buzzer_command(
                                                        actuators,
                                                        buzzer_pin,
                                                        pulse_ms,
                                                        action,
                                                        duration,
                                                    )
                                                });

                                            match outcome {
                                                CommandOutcome::Applied(msg) => ("ok", msg),
                                                CommandOutcome::Error(msg) => {
                                                    ("error", Some(msg))
                                                }
                                            }
                                        } else {
                                            let outcome = ctx.shared.actuators.lock(|actuators| {
                                                apply_action(actuators, target, action)
                                            });

                                            match outcome {
                                                CommandOutcome::Applied(msg) => ("ok", msg),
                                                CommandOutcome::Error(msg) => {
                                                    ("error", Some(msg))
                                                }
                                            }
                                        };

                                    ctx.shared.tx.lock(|tx| {
                                        let _ = send_ack(
                                            tx,
                                            Some(target),
                                            Some(action),
                                            result,
                                            message,
                                        );
                                    });
                                } else {
                                    ctx.shared.tx.lock(|tx| {
                                        let _ = send_ack(
                                            tx,
                                            frame.target,
                                            frame.action,
                                            "error",
                                            Some("missing target/action"),
                                        );
                                    });
                                }
                            }
                        }
                    }
                    ctx.local.line_buf.clear();
                }
                b'\r' => {}
                b => {
                    if ctx.local.line_buf.push(b as char).is_err() {
                        ctx.local.line_buf.clear();
                    }
                }
            },
            Err(nb::Error::WouldBlock) => {}
            Err(_) => {}
        }
    }

    enum CommandOutcome {
        Applied(Option<&'static str>),
        Error(&'static str),
    }

    fn apply_action(state: &mut ActuatorState, target: &str, action: &str) -> CommandOutcome {
        match target {
            "water" => apply_binary_action(state.water(), action),
            "light" => apply_binary_action(state.light(), action),
            "fan" => apply_binary_action(state.fan(), action),
            _ => CommandOutcome::Error("unknown target"),
        }
    }

    fn apply_binary_action(flag: &mut bool, action: &str) -> CommandOutcome {
        match action {
            "on" => {
                if *flag {
                    CommandOutcome::Applied(Some("already on"))
                } else {
                    *flag = true;
                    CommandOutcome::Applied(None)
                }
            }
            "off" => {
                if !*flag {
                    CommandOutcome::Applied(Some("already off"))
                } else {
                    *flag = false;
                    CommandOutcome::Applied(None)
                }
            }
            "pulse" => CommandOutcome::Error("pulse action not implemented"),
            _ => CommandOutcome::Error("unsupported action"),
        }
    }

    fn handle_buzzer_command(
        state: &mut ActuatorState,
        buzzer: &mut PA1<Output<PushPull>>,
        pulse_ms: &mut Option<u32>,
        action: &str,
        duration_ms: Option<u32>,
    ) -> CommandOutcome {
        match action {
            "on" => {
                if state.buzzer {
                    CommandOutcome::Applied(Some("already on"))
                } else {
                    state.buzzer = true;
                    *pulse_ms = None;
                    buzzer.set_low();
                    CommandOutcome::Applied(None)
                }
            }
            "off" => {
                if !state.buzzer {
                    CommandOutcome::Applied(Some("already off"))
                } else {
                    state.buzzer = false;
                    *pulse_ms = None;
                    buzzer.set_high();
                    CommandOutcome::Applied(None)
                }
            }
            "pulse" => {
                let ms = match duration_ms {
                    Some(value) if (MIN_PULSE_MS..=MAX_PULSE_MS).contains(&value) => value,
                    Some(_) => return CommandOutcome::Error("pulse time out of range"),
                    None => return CommandOutcome::Error("missing pulse time"),
                };

                state.buzzer = true;
                *pulse_ms = Some(ms);
                buzzer.set_low();
                CommandOutcome::Applied(Some("pulsing"))
            }
            _ => CommandOutcome::Error("unsupported action"),
        }
    }

    #[task(
        binds = TIM2,
        priority = 2,
        shared = [tx, actuators, buzzer_pin, buzzer_pulse_ms],
        local = [telemetry_timer, telemetry_ticks]
    )]
    fn on_tim2(mut ctx: on_tim2::Context) {
        ctx.local
            .telemetry_timer
            .clear_interrupt(TimerEvent::Update);

        *ctx.local.telemetry_ticks = ctx.local.telemetry_ticks.wrapping_add(1);

        (&mut ctx.shared.buzzer_pin, &mut ctx.shared.buzzer_pulse_ms, &mut ctx.shared.actuators)
            .lock(|buzzer_pin, pulse_ms, actuators| {
                if let Some(remaining) = pulse_ms.as_mut() {
                    if *remaining > 0 {
                        *remaining -= 1;
                        if *remaining == 0 {
                            buzzer_pin.set_high();
                            actuators.buzzer = false;
                            *pulse_ms = None;
                        }
                    }
                }
            });

        if *ctx.local.telemetry_ticks >= TELEMETRY_INTERVAL_MS {
            *ctx.local.telemetry_ticks = 0;
            (&mut ctx.shared.tx, &mut ctx.shared.actuators).lock(|tx, actuators| {
                let state = *actuators;
                let _ = send_data(tx, state);
            });
        }
    }

    fn send_ack(
        tx: &mut Tx<USART1>,
        target: Option<&str>,
        action: Option<&str>,
        result: &'static str,
        message: Option<&'static str>,
    ) -> Result<(), ()> {
        let mut buf: String<TX_BUFFER_CAPACITY> = String::new();
        write!(buf, "{{\"type\":\"ack\",\"target\":").map_err(|_| ())?;
        write_option_string(&mut buf, target)?;
        write!(buf, ",\"action\":").map_err(|_| ())?;
        write_option_string(&mut buf, action)?;
        write!(buf, ",\"result\":\"{}\"", result).map_err(|_| ())?;
        write!(buf, ",\"message\":").map_err(|_| ())?;
        write_option_string(&mut buf, message)?;
        buf.push('}').map_err(|_| ())?;

        transmit_line(tx, buf.as_bytes());
        Ok(())
    }

    fn send_data(tx: &mut Tx<USART1>, state: ActuatorState) -> Result<(), ()> {
        let mut buf: String<TX_BUFFER_CAPACITY> = String::new();
        // Sensor fields remain null until their drivers are wired up.
        write!(
            buf,
            "{{\"type\":\"data\",\"temp\":null,\"humi\":null,\"soil\":null,\"lux\":null,\
             \"water\":{},\"light\":{},\"fan\":{},\"buzzer\":{}}}",
            bool_to_flag(state.water),
            bool_to_flag(state.light),
            bool_to_flag(state.fan),
            bool_to_flag(state.buzzer)
        )
        .map_err(|_| ())?;
        transmit_line(tx, buf.as_bytes());
        Ok(())
    }

    fn write_option_string<const N: usize>(
        buf: &mut String<N>,
        value: Option<&str>,
    ) -> Result<(), ()> {
        match value {
            Some(text) => {
                write!(buf, "\"{}\"", text).map_err(|_| ())
            }
            None => {
                buf.push_str("null").map_err(|_| ())
            }
        }
    }

    fn transmit_line(tx: &mut Tx<USART1>, payload: &[u8]) {
        for byte in payload {
            let _ = block!(tx.write_u8(*byte));
        }
        let _ = block!(tx.write_u8(b'\n'));
        let _ = block!(tx.flush());
    }

    const fn bool_to_flag(value: bool) -> u8 {
        if value {
            1
        } else {
            0
        }
    }
}
