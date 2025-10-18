//! ST7735 TFT 显示屏驱动，基于 SPI 接口。
//!
//! 实现最基本的初始化、区域填充以及 8x8 点阵字体的文本绘制，
//! 用于将传感器采集的数据呈现在屏幕上。

use core::fmt::Write;

use cortex_m::delay::Delay;
use embedded_hal::{
    digital::OutputPin,
    spi::{ErrorType, SpiBus},
};
use font8x8::UnicodeFonts;
use heapless::String;

use crate::sensors::Environment;

/// 像素颜色（RGB565）
#[derive(Clone, Copy)]
pub struct Rgb565(pub u16);

impl Rgb565 {
    pub const BLACK: Self = Self(0x0000);
    pub const WHITE: Self = Self(0xFFFF);
    pub const GREEN: Self = Self::from_rgb(0, 180, 0);

    pub const fn from_rgb(r: u8, g: u8, b: u8) -> Self {
        let r = (r as u16 >> 3) & 0x1F;
        let g = (g as u16 >> 2) & 0x3F;
        let b = (b as u16 >> 3) & 0x1F;
        Self((r << 11) | (g << 5) | b)
    }

    #[inline]
    pub const fn to_be_bytes(self) -> [u8; 2] {
        self.0.to_be_bytes()
    }
}

/// ST7735 驱动器
pub struct St7735Display<SPI, DC, RST, CS> {
    spi: SPI,
    dc: DC,
    rst: RST,
    cs: CS,
    width: u16,
    height: u16,
    x_offset: u16,
    y_offset: u16,
}

impl<SPI, DC, RST, CS> St7735Display<SPI, DC, RST, CS>
where
    SPI: SpiBus<u8>,
    DC: OutputPin,
    RST: OutputPin,
    CS: OutputPin,
{
    const CMD_SWRESET: u8 = 0x01;
    const CMD_SLPOUT: u8 = 0x11;
    const CMD_DISPON: u8 = 0x29;
    const CMD_NORON: u8 = 0x13;
    const CMD_MADCTL: u8 = 0x36;
    const CMD_COLMOD: u8 = 0x3A;
    const CMD_CASET: u8 = 0x2A;
    const CMD_RASET: u8 = 0x2B;
    const CMD_RAMWR: u8 = 0x2C;
    const CMD_INVCTR: u8 = 0xB4;
    const CMD_PWCTR1: u8 = 0xC0;
    const CMD_PWCTR2: u8 = 0xC1;
    const CMD_PWCTR3: u8 = 0xC2;
    const CMD_PWCTR4: u8 = 0xC3;
    const CMD_PWCTR5: u8 = 0xC4;
    const CMD_VMCTR1: u8 = 0xC5;
    const CMD_FRMCTR1: u8 = 0xB1;
    const CMD_FRMCTR2: u8 = 0xB2;
    const CMD_FRMCTR3: u8 = 0xB3;
    const CMD_GMCTRP1: u8 = 0xE0;
    const CMD_GMCTRN1: u8 = 0xE1;

    pub fn new(spi: SPI, dc: DC, rst: RST, cs: CS) -> Self {
        Self {
            spi,
            dc,
            rst,
            cs,
            width: 160,
            height: 128,
            x_offset: 0,
            y_offset: 0,
        }
    }

    /// 初始化屏幕到默认配置
    pub fn init(&mut self, delay: &mut Delay) -> Result<(), SPI::Error> {
        self.cs.set_high().ok();
        self.dc.set_high().ok();
        self.rst.set_high().ok();
        delay.delay_ms(10);
        self.rst.set_low().ok();
        delay.delay_ms(20);
        self.rst.set_high().ok();
        delay.delay_ms(120);

        self.command(Self::CMD_SWRESET, &[])?;
        delay.delay_ms(150);
        self.command(Self::CMD_SLPOUT, &[])?;
        delay.delay_ms(120);

        self.command(Self::CMD_FRMCTR1, &[0x01, 0x2C, 0x2D])?;
        self.command(Self::CMD_FRMCTR2, &[0x01, 0x2C, 0x2D])?;
        self.command(Self::CMD_FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])?;
        self.command(Self::CMD_INVCTR, &[0x07])?;

        self.command(Self::CMD_PWCTR1, &[0xA2, 0x02, 0x84])?;
        self.command(Self::CMD_PWCTR2, &[0xC5])?;
        self.command(Self::CMD_PWCTR3, &[0x0A, 0x00])?;
        self.command(Self::CMD_PWCTR4, &[0x8A, 0x2A])?;
        self.command(Self::CMD_PWCTR5, &[0x8A, 0xEE])?;
        self.command(Self::CMD_VMCTR1, &[0x0E])?;

        // Memory access control：横屏显示，RGB 顺序
        self.command(Self::CMD_MADCTL, &[0xA0])?;
        // 16-bit 565
        self.command(Self::CMD_COLMOD, &[0x05])?;

        self.command(
            Self::CMD_GMCTRP1,
            &[
                0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01,
                0x03, 0x10,
            ],
        )?;
        self.command(
            Self::CMD_GMCTRN1,
            &[
                0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00,
                0x02, 0x10,
            ],
        )?;

        self.command(Self::CMD_NORON, &[])?;
        delay.delay_ms(10);
        self.command(Self::CMD_DISPON, &[])?;
        delay.delay_ms(100);

        self.clear(Rgb565::BLACK)?;
        self.draw_header()?;

        Ok(())
    }

    /// 根据当前环境数据刷新四行文本
    pub fn render_environment(&mut self, env: &Environment) -> Result<(), SPI::Error> {
        const DATA_SCALE: u16 = 1;
        let char_height = 8 * DATA_SCALE;
        let line_height = char_height + 6;
        let mut y = 24u16;

        let bg = Rgb565::BLACK;
        let fg = Rgb565::WHITE;
        let mut line: String<40> = String::new();

        // 温度
        line.clear();
        let _ = line.push_str("Temp:");
        match env.temperature {
            Some(value) => {
                let _ = write!(line, " {}C", value);
            }
            None => {
                let _ = line.push_str(" --");
            }
        }
        self.draw_line(y, line.as_str(), fg, bg, DATA_SCALE)?;
        y += line_height;

        // 湿度
        line.clear();
        let _ = line.push_str("Humi:");
        match env.humidity {
            Some(value) => {
                let _ = write!(line, " {}%", value);
            }
            None => {
                let _ = line.push_str(" --");
            }
        }
        self.draw_line(y, line.as_str(), fg, bg, DATA_SCALE)?;
        y += line_height;

        // 土壤湿度
        line.clear();
        let _ = line.push_str("Soil:");
        match env.soil {
            Some(value) => {
                let _ = write!(line, " {}%", value);
            }
            None => {
                let _ = line.push_str(" --");
            }
        }
        self.draw_line(y, line.as_str(), fg, bg, DATA_SCALE)?;
        y += line_height;

        // 光照
        line.clear();
        let _ = line.push_str("Light:");
        match env.lux {
            Some(value) => {
                let _ = write!(line, "{} lux", value);
            }
            None => {
                let _ = line.push_str(" --");
            }
        }
        self.draw_line(y, line.as_str(), fg, bg, DATA_SCALE)?;

        Ok(())
    }

    fn draw_header(&mut self) -> Result<(), SPI::Error> {
        let header_bg = Rgb565::GREEN;
        let header_fg = Rgb565::BLACK;
        self.fill_rect(0, 0, self.width, 22, header_bg)?;
        self.draw_text_scaled(6, 4, "Bonsai Monitor", header_fg, header_bg, 1)
    }

    fn draw_line(
        &mut self,
        y: u16,
        text: &str,
        fg: Rgb565,
        bg: Rgb565,
        scale: u16,
    ) -> Result<(), SPI::Error> {
        let scale = scale.max(1);
        let char_height = 8 * scale;
        let padding = 4;
        self.fill_rect(0, y, self.width, char_height + padding, bg)?;
        self.draw_text_scaled(6, y + 2, text, fg, bg, scale)
    }

    fn draw_text_scaled(
        &mut self,
        mut x: u16,
        y: u16,
        text: &str,
        fg: Rgb565,
        bg: Rgb565,
        scale: u16,
    ) -> Result<(), SPI::Error> {
        let scale = scale.max(1);
        let char_width = 8 * scale;
        let char_height = 8 * scale;

        for ch in text.chars() {
            if x + char_width > self.width || y + char_height > self.height {
                break;
            }
            let glyph = font8x8::BASIC_FONTS.get(ch).unwrap_or([0; 8]);
            self.draw_char(x, y, &glyph, fg, bg, scale)?;
            x = x.saturating_add(char_width + scale);
        }
        Ok(())
    }

    fn draw_char(
        &mut self,
        x: u16,
        y: u16,
        glyph: &[u8; 8],
        fg: Rgb565,
        bg: Rgb565,
        scale: u16,
    ) -> Result<(), SPI::Error> {
        let scale = scale.max(1);
        let char_width = 8 * scale;
        let char_height = 8 * scale;
        let max_pixels = (char_width as usize) * (char_height as usize) * 2;
        let mut pixels = [0u8; 512];
        let mut offset = 0usize;
        for &bits in glyph.iter() {
            for _ in 0..scale {
                for col in 0..8 {
                    let bit = (bits >> col) & 0x01;
                    let color = if bit != 0 { fg } else { bg };
                    let bytes = color.to_be_bytes();
                    for _ in 0..scale {
                        if offset + 1 < pixels.len() {
                            pixels[offset] = bytes[0];
                            pixels[offset + 1] = bytes[1];
                        }
                        offset += 2;
                    }
                }
            }
        }
        let offset = offset.min(max_pixels);
        self.set_window(
            x,
            y,
            x + char_width - 1,
            y + char_height - 1,
        )?;
        self.write_data(&pixels[..offset])?;
        Ok(())
    }

    fn fill_rect(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        color: Rgb565,
    ) -> Result<(), SPI::Error> {
        if w == 0 || h == 0 {
            return Ok(());
        }
        let x1 = x + w - 1;
        let y1 = y + h - 1;
        self.set_window(x, y, x1, y1)?;

        const CHUNK_PIXELS: usize = 32;
        let color_bytes = color.to_be_bytes();
        let mut buf = [0u8; CHUNK_PIXELS * 2];
        let mut remaining = (w as usize) * (h as usize);
        while remaining > 0 {
            let batch = remaining.min(CHUNK_PIXELS);
            for i in 0..batch {
                let idx = i * 2;
                buf[idx] = color_bytes[0];
                buf[idx + 1] = color_bytes[1];
            }
            self.write_data(&buf[..batch * 2])?;
            remaining -= batch;
        }
        Ok(())
    }

    fn clear(&mut self, color: Rgb565) -> Result<(), SPI::Error> {
        self.fill_rect(0, 0, self.width, self.height, color)
    }

    fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) -> Result<(), SPI::Error> {
        let xs = (x0 + self.x_offset, x1 + self.x_offset);
        let ys = (y0 + self.y_offset, y1 + self.y_offset);
        self.command(
            Self::CMD_CASET,
            &[
                (xs.0 >> 8) as u8,
                (xs.0 & 0xFF) as u8,
                (xs.1 >> 8) as u8,
                (xs.1 & 0xFF) as u8,
            ],
        )?;
        self.command(
            Self::CMD_RASET,
            &[
                (ys.0 >> 8) as u8,
                (ys.0 & 0xFF) as u8,
                (ys.1 >> 8) as u8,
                (ys.1 & 0xFF) as u8,
            ],
        )?;
        self.command(Self::CMD_RAMWR, &[])
    }

    fn command(&mut self, cmd: u8, data: &[u8]) -> Result<(), SPI::Error> {
        self.cs.set_low().ok();
        self.dc.set_low().ok();
        self.spi.write(&[cmd])?;
        if !data.is_empty() {
            self.dc.set_high().ok();
            self.spi.write(data)?;
        }
        self.cs.set_high().ok();
        Ok(())
    }

    fn write_data(&mut self, data: &[u8]) -> Result<(), SPI::Error> {
        if data.is_empty() {
            return Ok(());
        }
        self.cs.set_low().ok();
        self.dc.set_high().ok();
        self.spi.write(data)?;
        self.cs.set_high().ok();
        Ok(())
    }
}

impl<SPI, DC, RST, CS> ErrorType for St7735Display<SPI, DC, RST, CS>
where
    SPI: SpiBus<u8>,
    DC: OutputPin,
    RST: OutputPin,
    CS: OutputPin,
{
    type Error = SPI::Error;
}
