//! 传感器模块集合，包含环境传感器相关实现

pub mod bh1750;
pub mod dht11;
pub mod soil;

/// 环境传感器读数，使用 `Option` 表示是否成功采样
#[derive(Clone, Copy, Default)]
pub struct Environment {
    pub temperature: Option<u8>,
    pub humidity: Option<u8>,
    pub soil: Option<u8>,
    pub lux: Option<u16>,
}
