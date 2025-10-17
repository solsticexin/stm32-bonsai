//! DHT11 温湿度传感器驱动，采用阻塞式读取流程

use cortex_m::delay::Delay;
use stm32f1xx_hal::gpio::{gpiob::PB5, Cr, Floating, Input};

/// DHT11 读取过程中可能出现的错误
#[derive(Debug)]
pub enum DhtError {
    Timeout,
    Checksum,
}

/// DHT11 读取结果，温湿度单位均为整数
#[derive(Clone, Copy, Debug)]
pub struct DhtReading {
    pub temperature: u8,
    pub humidity: u8,
}

/// DHT11 传感器抽象，内部保存引脚与配置寄存器
pub struct Dht11 {
    pin: Option<PB5<Input<Floating>>>,
    crl: Cr<'B', false>,
}

impl Dht11 {
    /// 新建 DHT11 实例，需传入浮空输入模式的 PB5 与 CRL 寄存器所有权
    pub fn new(pin: PB5<Input<Floating>>, crl: Cr<'B', false>) -> Self {
        Self {
            pin: Some(pin),
            crl,
        }
    }

    /// 执行一次阻塞式读取，约耗时 4~5 ms
    pub fn read(&mut self, delay: &mut Delay) -> Result<DhtReading, DhtError> {
        self.start_signal(delay)?;
        self.expect_response(delay)?;
        let data = self.read_payload(delay)?;

        let checksum = data[0]
            .wrapping_add(data[1])
            .wrapping_add(data[2])
            .wrapping_add(data[3]);
        if checksum != data[4] {
            return Err(DhtError::Checksum);
        }

        Ok(DhtReading {
            humidity: data[0],
            temperature: data[2],
        })
    }

    /// 向 DHT11 发送启动信号：拉低 18 ms 后释放
    fn start_signal(&mut self, delay: &mut Delay) -> Result<(), DhtError> {
        let Some(pin_in) = self.pin.take() else {
            return Err(DhtError::Timeout);
        };
        let mut pin = pin_in.into_open_drain_output(&mut self.crl);
        pin.set_low();
        delay.delay_ms(20_u32);
        pin.set_high();
        delay.delay_us(30_u32);
        self.pin = Some(pin.into_floating_input(&mut self.crl));
        Ok(())
    }

    /// 等待 DHT11 的响应序列（80us 低 + 80us 高）
    fn expect_response(&mut self, delay: &mut Delay) -> Result<(), DhtError> {
        self.wait_for_level(false, 100, delay)?;
        self.wait_for_level(true, 100, delay)?;
        self.wait_for_level(false, 100, delay)?;
        Ok(())
    }

    /// 读取 40 位数据帧，每位以高电平持续时间判定 0/1
    fn read_payload(&mut self, delay: &mut Delay) -> Result<[u8; 5], DhtError> {
        let mut bytes = [0u8; 5];
        for bit_index in 0..40 {
            self.wait_for_level(true, 70, delay)?;
            delay.delay_us(40_u32);
            let Some(pin) = self.pin.as_ref() else {
                return Err(DhtError::Timeout);
            };
            let bit = if pin.is_high() {
                1
            } else {
                0
            };
            bytes[(bit_index / 8) as usize] <<= 1;
            bytes[(bit_index / 8) as usize] |= bit;
            self.wait_for_level(false, 70, delay)?;
        }
        Ok(bytes)
    }

    /// 等待引脚达到目标电平，超时则返回错误
    fn wait_for_level(
        &mut self,
        target_high: bool,
        timeout_us: u32,
        delay: &mut Delay,
    ) -> Result<(), DhtError> {
        for _ in 0..timeout_us {
            let Some(pin) = self.pin.as_mut() else {
                return Err(DhtError::Timeout);
            };
            let level = pin.is_high();
            if level == target_high {
                return Ok(());
            }
            delay.delay_us(1_u32);
        }
        Err(DhtError::Timeout)
    }
}
