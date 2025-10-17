//! 执行器模块：负责执行器状态管理以及蜂鸣器的控制策略

use stm32f1xx_hal::gpio::gpioa::PA1;
use stm32f1xx_hal::gpio::{Output, PushPull};

/// 蜂鸣器脉冲的最短持续时间（毫秒）
pub const MIN_PULSE_MS: u32 = 1;
/// 蜂鸣器脉冲的最长持续时间（毫秒）
pub const MAX_PULSE_MS: u32 = 10_000;

/// 执行器状态结构体，记录当前的各通道开关状态
#[derive(Clone, Copy, Default)]
pub struct ActuatorState {
    pub(crate) water: bool,
    pub(crate) light: bool,
    pub(crate) fan: bool,
    pub(crate) buzzer: bool,
}

impl ActuatorState {
    /// 返回水泵状态的可变引用，便于复用统一的布尔开关逻辑
    #[inline]
    pub fn water(&mut self) -> &mut bool {
        &mut self.water
    }

    /// 返回补光灯状态的可变引用
    #[inline]
    pub fn light(&mut self) -> &mut bool {
        &mut self.light
    }

    /// 返回风扇状态的可变引用
    #[inline]
    pub fn fan(&mut self) -> &mut bool {
        &mut self.fan
    }
}

/// 指令执行结果，用于生成 ACK 信息
pub enum CommandOutcome {
    Applied(Option<&'static str>),
    Error(&'static str),
}

/// 处理除蜂鸣器外的通用开关型执行器
pub fn apply_action(state: &mut ActuatorState, target: &str, action: &str) -> CommandOutcome {
    match target {
        "water" => apply_binary_action(state.water(), action),
        "light" => apply_binary_action(state.light(), action),
        "fan" => apply_binary_action(state.fan(), action),
        _ => CommandOutcome::Error("unknown target"),
    }
}

/// 处理蜂鸣器指令，支持 on/off/pulse 动作
pub fn handle_buzzer_command(
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
