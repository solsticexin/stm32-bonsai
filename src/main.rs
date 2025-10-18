#![deny(unsafe_code)]
#![no_main]
#![no_std]

// 将 panic 输出到调试串口
use panic_probe as _;

// 模块化拆分：执行器控制逻辑、通信协议与传感器采集
mod actuators;
mod display;
mod protocol;
mod sensors;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {
    use cortex_m::{asm, delay::Delay};
    use heapless::String;
    use stm32f1xx_hal::{
        adc::Adc,
        gpio::{
            gpioa::{PA0, PA1, PA2, PA3, PA4},
            gpiob::{PB0, PB1, PB10},
            Analog, Floating, GpioExt, Output, PushPull,
        },
        i2c::{I2c, Mode},
        pac::{self, USART1},
        prelude::*,
        rcc::Config as RccConfig,
        serial::{Config as SerialConfig, Event as SerialEvent, Rx, Serial, Tx},
        spi::{Mode as SpiMode, Phase, Polarity, Spi, SpiExt},
        timer::{CounterHz, Event as TimerEvent, Timer},
    };

    use crate::{
        actuators::{
            apply_action, handle_buzzer_command, ActuatorState, CommandOutcome, MAX_PULSE_MS,
            MIN_PULSE_MS,
        },
        display::St7735Display,
        protocol::{self, LINE_BUFFER_CAPACITY, TELEMETRY_INTERVAL_MS},
        sensors::{bh1750::Bh1750, dht11::Dht11, soil::SoilSensor, Environment},
    };

    /// 共享资源：串口发送端、执行器状态以及蜂鸣器控制
    type LightI2c = I2c<pac::I2C1>;
    type DisplaySpi = Spi<pac::SPI1, u8, Floating>;
    type DisplayDriver = St7735Display<
        DisplaySpi,
        PA4<Output<PushPull>>,
        PA2<Output<PushPull>>,
        PA3<Output<PushPull>>,
    >;

    #[shared]
    struct Shared {
        tx: Tx<USART1>,
        actuators: ActuatorState,
        environment: Environment,
        buzzer_pin: PA1<Output<PushPull>>,
        buzzer_pulse_ms: Option<u32>,
        water_pin: PB0<Output<PushPull>>,
        light_pin: PB1<Output<PushPull>>,
        fan_pin: PB10<Output<PushPull>>,
        water_pulse_ms: Option<u32>,
        light_pulse_ms: Option<u32>,
        fan_pulse_ms: Option<u32>,
    }

    /// 本地资源：串口接收端、接收缓冲以及定时器
    #[local]
    struct Local {
        rx: Rx<USART1>,
        line_buf: String<LINE_BUFFER_CAPACITY>,
        telemetry_timer: CounterHz<pac::TIM2>,
        telemetry_ticks: u32,
        dht11: Dht11,
        delay: Delay,
        adc: Adc<pac::ADC1>,
        soil_pin: PA0<Analog>,
        i2c: LightI2c,
        display: DisplayDriver,
    }

    /// 系统初始化：配置时钟、外设与定时器
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // 配置外部 8MHz 晶振，并将系统时钟提升到 72MHz
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

        // 拆分 GPIO 寄存器，准备配置所需引脚
        let mut gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let mut gpiob_crl = gpiob.crl;
        let mut gpiob_crh = gpiob.crh;

        // 执行器输出引脚，其中蜂鸣器为低电平触发，因此默认拉高关闭
        let mut water_pin = gpiob.pb0.into_push_pull_output(&mut gpiob_crl);
        water_pin.set_low();
        let mut light_pin = gpiob.pb1.into_push_pull_output(&mut gpiob_crl);
        light_pin.set_low();
        let mut fan_pin = gpiob.pb10.into_push_pull_output(&mut gpiob_crh);
        fan_pin.set_low();
        let mut buzzer_pin = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
        buzzer_pin.set_high();

        // 传感器相关引脚保持占位状态，后续接入真实驱动
        let soil_pin = gpioa.pa0.into_analog(&mut gpioa.crl);
        let dht_pin = gpiob.pb5.into_floating_input(&mut gpiob_crl);
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob_crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob_crl);
        let dht11 = Dht11::new(dht_pin, gpiob_crl);
        let mut i2c = I2c::new(
            ctx.device.I2C1,
            (scl, sda),
            Mode::standard(100.kHz()),
            &mut rcc,
        );
        let _ = Bh1750::init(&mut i2c);

        // USART1：PA9 / PA10 作为 TX / RX
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

        // TIM2 作为 1kHz 基准计时器，用于遥测与蜂鸣器脉冲计时
        let mut telemetry_timer = Timer::new(ctx.device.TIM2, &mut rcc).counter_hz();
       telemetry_timer.listen(TimerEvent::Update);
       let _ = telemetry_timer.start(1_000.Hz());

        // 创建基于 SYST 的延迟，用于 DHT11 微秒级时序及显示屏初始化
        let mut delay = Delay::new(ctx.core.SYST, rcc.clocks.sysclk().raw());

        // SPI 显示屏：ST7735
        let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
        let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
        let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
        let mut dc = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let mut rst = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        let mut cs = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
        cs.set_high();
        rst.set_high();
        dc.set_high();
        let spi_mode = SpiMode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi = ctx
            .device
            .SPI1
            .spi((Some(sck), Some(miso), Some(mosi)), spi_mode, 8.MHz(), &mut rcc);
        let mut display = St7735Display::new(spi, dc, rst, cs);
        let _ = display.init(&mut delay);
        let _ = display.render_environment(&Environment::default());

        let adc = Adc::new(ctx.device.ADC1, &mut rcc);

        (
            Shared {
                tx,
                actuators: ActuatorState::default(),
                environment: Environment::default(),
                buzzer_pin,
                buzzer_pulse_ms: None,
                water_pin,
                light_pin,
                fan_pin,
                water_pulse_ms: None,
                light_pulse_ms: None,
                fan_pulse_ms: None,
            },
            Local {
                rx,
                line_buf: String::new(),
                telemetry_timer,
                telemetry_ticks: 0,
                dht11,
                delay,
                adc,
                soil_pin,
                i2c,
                display,
            },
            init::Monotonics(),
        )
    }

    /// 空闲任务保持低功耗等待
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    /// 串口中断任务：接收并解析来自 ESP 的指令
    #[task(
        binds = USART1,
        priority = 3,
        shared = [
            tx,
            actuators,
            buzzer_pin,
            buzzer_pulse_ms,
            water_pin,
            light_pin,
            fan_pin,
            water_pulse_ms,
            light_pulse_ms,
            fan_pulse_ms
        ],
        local = [rx, line_buf]
    )]
    fn on_usart1(mut ctx: on_usart1::Context) {
        match ctx.local.rx.read() {
            Ok(byte) => match byte {
                b'\n' => {
                    if !ctx.local.line_buf.is_empty() {
                        let line = ctx.local.line_buf.as_str();
                        if let Ok(frame) = protocol::parse_command_line(line) {
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
                                        } else if matches!(target, "water" | "light" | "fan")
                                            && action == "pulse"
                                        {
                                            let duration = match frame.time {
                                                Some(value)
                                                    if (MIN_PULSE_MS..=MAX_PULSE_MS)
                                                        .contains(&value) =>
                                                {
                                                    Ok(value)
                                                }
                                                Some(_) => Err("pulse time out of range"),
                                                None => Err("missing pulse time"),
                                            };

                                            match duration {
                                                Ok(ms) => {
                                                    match target {
                                                        "water" => (
                                                            &mut ctx.shared.water_pin,
                                                            &mut ctx.shared.water_pulse_ms,
                                                            &mut ctx.shared.actuators,
                                                        )
                                                            .lock(|pin, pulse_ms, actuators| {
                                                                pin.set_high();
                                                                *pulse_ms = Some(ms);
                                                                *actuators.water() = true;
                                                            }),
                                                        "light" => (
                                                            &mut ctx.shared.light_pin,
                                                            &mut ctx.shared.light_pulse_ms,
                                                            &mut ctx.shared.actuators,
                                                        )
                                                            .lock(|pin, pulse_ms, actuators| {
                                                                pin.set_high();
                                                                *pulse_ms = Some(ms);
                                                                *actuators.light() = true;
                                                            }),
                                                        "fan" => (
                                                            &mut ctx.shared.fan_pin,
                                                            &mut ctx.shared.fan_pulse_ms,
                                                            &mut ctx.shared.actuators,
                                                        )
                                                            .lock(|pin, pulse_ms, actuators| {
                                                                pin.set_high();
                                                                *pulse_ms = Some(ms);
                                                                *actuators.fan() = true;
                                                            }),
                                                        _ => {}
                                                    }
                                                    ("ok", Some("pulsing"))
                                                }
                                                Err(msg) => ("error", Some(msg)),
                                            }
                                        } else {
                                            let mut new_state = None;
                                            let outcome = ctx.shared.actuators.lock(|actuators| {
                                                let out = apply_action(actuators, target, action);
                                                new_state = actuators.state(target);
                                                out
                                            });

                                            if let Some(state) = new_state {
                                                match target {
                                                    "water" => (
                                                        &mut ctx.shared.water_pin,
                                                        &mut ctx.shared.water_pulse_ms,
                                                    )
                                                        .lock(|pin, pulse_ms| {
                                                            if state {
                                                                pin.set_high();
                                                            } else {
                                                                pin.set_low();
                                                            }
                                                            *pulse_ms = None;
                                                        }),
                                                    "light" => (
                                                        &mut ctx.shared.light_pin,
                                                        &mut ctx.shared.light_pulse_ms,
                                                    )
                                                        .lock(|pin, pulse_ms| {
                                                            if state {
                                                                pin.set_high();
                                                            } else {
                                                                pin.set_low();
                                                            }
                                                            *pulse_ms = None;
                                                        }),
                                                    "fan" => (
                                                        &mut ctx.shared.fan_pin,
                                                        &mut ctx.shared.fan_pulse_ms,
                                                    )
                                                        .lock(|pin, pulse_ms| {
                                                            if state {
                                                                pin.set_high();
                                                            } else {
                                                                pin.set_low();
                                                            }
                                                            *pulse_ms = None;
                                                        }),
                                                    _ => {}
                                                }
                                            }

                                            match outcome {
                                                CommandOutcome::Applied(msg) => ("ok", msg),
                                                CommandOutcome::Error(msg) => {
                                                    ("error", Some(msg))
                                                }
                                            }
                                        };

                                    ctx.shared.tx.lock(|tx| {
                                        let _ = protocol::send_ack(
                                            tx,
                                            Some(target),
                                            Some(action),
                                            result,
                                            message,
                                        );
                                    });
                                } else {
                                    ctx.shared.tx.lock(|tx| {
                                        let _ = protocol::send_ack(
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

    /// TIM2 中断：负责蜂鸣器脉冲计时及周期性遥测输出
    #[task(
        binds = TIM2,
        priority = 2,
        shared = [
            tx,
            actuators,
            buzzer_pin,
            buzzer_pulse_ms,
            environment,
            water_pin,
            light_pin,
            fan_pin,
            water_pulse_ms,
            light_pulse_ms,
            fan_pulse_ms
        ],
        local = [telemetry_timer, telemetry_ticks, dht11, delay, adc, soil_pin, i2c, display]
    )]
    fn on_tim2(mut ctx: on_tim2::Context) {
        ctx.local
            .telemetry_timer
            .clear_interrupt(TimerEvent::Update);

        *ctx.local.telemetry_ticks = ctx.local.telemetry_ticks.wrapping_add(1);

        // 蜂鸣器脉冲倒计时：计数归零后自动释放
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

        // 水泵脉冲控制（高电平触发）
        (&mut ctx.shared.water_pin, &mut ctx.shared.water_pulse_ms, &mut ctx.shared.actuators)
            .lock(|pin, pulse_ms, actuators| {
                if let Some(remaining) = pulse_ms.as_mut() {
                    if *remaining > 0 {
                        *remaining -= 1;
                        if *remaining == 0 {
                            pin.set_low();
                            *actuators.water() = false;
                            *pulse_ms = None;
                        }
                    }
                }
            });

        // 补光灯脉冲控制
        (&mut ctx.shared.light_pin, &mut ctx.shared.light_pulse_ms, &mut ctx.shared.actuators)
            .lock(|pin, pulse_ms, actuators| {
                if let Some(remaining) = pulse_ms.as_mut() {
                    if *remaining > 0 {
                        *remaining -= 1;
                        if *remaining == 0 {
                            pin.set_low();
                            *actuators.light() = false;
                            *pulse_ms = None;
                        }
                    }
                }
            });

        // 风扇脉冲控制
        (&mut ctx.shared.fan_pin, &mut ctx.shared.fan_pulse_ms, &mut ctx.shared.actuators)
            .lock(|pin, pulse_ms, actuators| {
                if let Some(remaining) = pulse_ms.as_mut() {
                    if *remaining > 0 {
                        *remaining -= 1;
                        if *remaining == 0 {
                            pin.set_low();
                            *actuators.fan() = false;
                            *pulse_ms = None;
                        }
                    }
                }
            });

        // 每 TELEMETRY_INTERVAL_MS 毫秒上报一次执行器状态
        if *ctx.local.telemetry_ticks >= TELEMETRY_INTERVAL_MS {
            *ctx.local.telemetry_ticks = 0;

            // 读取 DHT11，更新环境数据
            let reading = ctx.local.dht11.read(ctx.local.delay);
            let soil_percent =
                SoilSensor::read_percent(&mut ctx.local.adc, &mut ctx.local.soil_pin);
            let lux_result = Bh1750::read_lux(&mut ctx.local.i2c, ctx.local.delay);

            ctx.shared.environment.lock(|env| {
                match reading {
                    Ok(data) => {
                        env.temperature = Some(data.temperature);
                        env.humidity = Some(data.humidity);
                    }
                    Err(_) => {
                        env.temperature = None;
                        env.humidity = None;
                    }
                }
                env.soil = Some(soil_percent);
                env.lux = lux_result.ok();
            });

            let env_snapshot = ctx.shared.environment.lock(|env| *env);
            let _ = ctx.local.display.render_environment(&env_snapshot);

            (&mut ctx.shared.tx, &mut ctx.shared.actuators, &mut ctx.shared.environment)
                .lock(|tx, actuators, env| {
                    let _ = protocol::send_data(tx, env, actuators);
                });
        }
    }
}
