//! 通信协议模块：负责 JSON 命令解析与串口报文生成

use core::fmt::Write;
use heapless::String;
use nb::block;
use serde::Deserialize;
use serde_json_core::de::{self, from_str};
use stm32f1xx_hal::{
    pac::USART1,
    serial::Tx,
};

use crate::actuators::ActuatorState;

/// 串口接收缓冲区容量（按行缓存 NDJSON）
pub const LINE_BUFFER_CAPACITY: usize = 192;
/// 串口发送时的临时字符串缓冲区容量
pub const TX_BUFFER_CAPACITY: usize = 192;
/// 遥测数据的发送周期（毫秒）
pub const TELEMETRY_INTERVAL_MS: u32 = 5_000;

/// 与前端约定的命令帧结构
#[derive(Deserialize)]
pub struct CommandFrame<'a> {
    #[serde(rename = "type")]
    pub kind: &'a str,
    #[serde(default)]
    pub target: Option<&'a str>,
    #[serde(default)]
    pub action: Option<&'a str>,
    #[serde(default)]
    pub time: Option<u32>,
}

/// 将一行 NDJSON 文本解析为命令帧
pub fn parse_command_line<'a>(line: &'a str) -> Result<CommandFrame<'a>, de::Error> {
    from_str::<CommandFrame>(line).map(|(frame, _)| frame)
}

/// 发送 ACK，反馈指令执行结果
pub fn send_ack(
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

/// 发送 data 帧，报告执行器状态
pub fn send_data(tx: &mut Tx<USART1>, state: &ActuatorState) -> Result<(), ()> {
    let mut buf: String<TX_BUFFER_CAPACITY> = String::new();
    // 传感器数据占位，后续接入真实传感器后替换
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
        Some(text) => write!(buf, "\"{}\"", text).map_err(|_| ()),
        None => buf.push_str("null").map_err(|_| ()),
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
