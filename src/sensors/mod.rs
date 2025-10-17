//! 传感器模块集合，目前仅包含 DHT11 相关实现

pub mod dht11;

/// 环境传感器读数，使用 `Option` 表示是否成功采样
#[derive(Clone, Copy, Default)]
pub struct Environment {
    pub temperature: Option<u8>,
    pub humidity: Option<u8>,
}
