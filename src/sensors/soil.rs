//! 土壤湿度传感器（模拟量）读取与转换工具

use nb::block;
use stm32f1xx_hal::{
    adc::{Adc, SampleTime},
    gpio::{gpioa::PA0, Analog},
    pac::ADC1,
    prelude::_embedded_hal_adc_OneShot,
};

/// 简单的模拟量土壤湿度读取器
pub struct SoilSensor;

impl SoilSensor {
    /// 采集一次 ADC，并将原始值转换为 0-100% 的百分比
    pub fn read_percent(adc: &mut Adc<ADC1>, pin: &mut PA0<Analog>) -> u8 {
        // 较长采样时间可以提升信号稳定性
        adc.set_sample_time(SampleTime::T_239);
        let raw: u16 = block!(adc.read(pin)).unwrap_or(0u16);
        // STM32F1 ADC 12bit：0-4095，对应 0-100%
        ((raw as u32 * 100) / 4095) as u8
    }
}
