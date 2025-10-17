//! BH1750 光照强度传感器驱动

use cortex_m::delay::Delay;
use embedded_hal::i2c::I2c as HalI2c;

/// BH1750 传感器命令定义
pub struct Bh1750;

impl Bh1750 {
    const ADDRESS: u8 = 0x23;
    const CMD_POWER_ON: u8 = 0x01;
    const CMD_RESET: u8 = 0x07;
    const CMD_ONE_TIME_HIGH_RES: u8 = 0x20;

    /// 初始化 BH1750，确保处于正常工作状态
    pub fn init<I2C>(i2c: &mut I2C) -> Result<(), I2C::Error>
    where
        I2C: HalI2c,
    {
        i2c.write(Self::ADDRESS, &[Self::CMD_POWER_ON])?;
        i2c.write(Self::ADDRESS, &[Self::CMD_RESET])
    }

    /// 读取一次光照强度（Lux），返回整数值
    pub fn read_lux<I2C>(i2c: &mut I2C, delay: &mut Delay) -> Result<u16, I2C::Error>
    where
        I2C: HalI2c,
    {
        i2c.write(Self::ADDRESS, &[Self::CMD_ONE_TIME_HIGH_RES])?;
        // 典型转换时间 120ms，取 180ms 留裕量
        delay.delay_ms(180_u32);
        let mut buf = [0u8; 2];
        i2c.read(Self::ADDRESS, &mut buf)?;
        let raw = u16::from_be_bytes(buf);
        // BH1750 输出值需除以 1.2 才等于 Lux，以整数形式近似
        let lux = ((raw as u32) * 10 / 12) as u16;
        Ok(lux)
    }
}
