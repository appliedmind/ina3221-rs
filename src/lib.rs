#![no_std]

extern crate byteorder;
extern crate embedded_hal as hal;

use byteorder::{ByteOrder, BigEndian};
use hal::blocking::i2c;

pub const INA3221_ADDR: u8 = 0x40;

/// Different options for the A0 address selection pin
pub enum A0 {
    Gnd = 0x40,
    Vs = 0x41,
    Sda = 0x42,
    Scl = 0x43
}

impl A0 {
    /// INA3221 address corresponding to the A0 option
    pub fn address(self) -> u8 {
        self as u8
    }
}

pub enum Register {
    Configuration = 0x00,
    ShuntVoltage1 = 0x01,
    BusVoltage1 = 0x02,
    ShuntVoltage2 = 0x03,
    BusVoltage2 = 0x04,
    ShuntVoltage3 = 0x05,
    BusVoltage3 = 0x06,
    CriticalLimit1 = 0x07,
    WarningLimit1 = 0x08,
    CriticalLimit2 = 0x09,
    WarningLimit2 = 0x0A,
    CriticalLimit3 = 0x0B,
    WarningLimit3 = 0x0C,
    ShuntVoltageSum = 0x0D,
    ShuntVoltageSumLimit = 0x0E,
    MaskEnable = 0x0F,
    PowerValidUpper = 0x10,
    PowerValidLower = 0x11,
    ManufacturerId = 0xFE,
    DieId = 0xFF
}

/// Set number of samples for average
#[derive(Copy, Clone)]
pub enum AveragingMode {
    /// 1 Sample (default)
    N1 = 0,
    /// 4 Samples
    N4 = 1,
    /// 16 Samples
    N16 = 2,
    /// 64 Samples
    N64 = 3,
    /// 128 Samples
    N128 = 4,
    /// 256 Samples
    N256 = 5,
    /// 512 Samples
    N512 = 6,
    /// 1024 Samples
    N1024 = 7
}

/// Set the ADC conversion time
#[derive(Copy, Clone)]
pub enum ConversionTime {
    /// 140 μs
    T140Us = 0,
    /// 204 μs
    T204Us = 1,
    /// 332 μs
    T332Us = 2,
    /// 588 μs
    T588Us = 3,
    /// 1.1 ms (default)
    T1100Us = 4,
    /// 2.116 ms
    T2116Us = 5,
    /// 4.156 ms
    T4156Us = 6,
    /// 8.244 ms
    T8244Us = 7
}

/// Operation mode
#[derive(Copy, Clone)]
pub enum OperationMode {
    /// Power-down
    PowerDown = 0,
    /// Shunt voltage, single-shot (triggered)
    ShuntSingle = 1,
    /// Bus voltage, single-shot (triggered)
    BusSingle = 2,
    /// Shunt and bus, single-shot (triggered)
    ShuntBusSingle = 3,
    /// Shunt voltage, continuous
    ShuntContinuous = 5,
    /// Bus voltage, continuous
    BusContinuous = 6,
    /// Shunt and bus, continuous (default)
    ShuntBusContinuous = 7
}

pub struct Config {
    /// Enable channel 1
    ch1_en: bool,
    /// Enable channel 2
    ch2_en: bool,
    /// Enable channel 3
    ch3_en: bool,
    /// Averaging mode
    avg: AveragingMode,
    /// Bus-Voltage conversion time
    vbus_ct: ConversionTime,
    /// Shunt-Voltage conversion time
    vsh_ct: ConversionTime,
    /// Operating mode
    mode: OperationMode
}

impl Default for Config {
    fn default() -> Self {
        Config {
            ch1_en: true,
            ch2_en: true,
            ch3_en: true,
            avg: AveragingMode::N1,
            vbus_ct: ConversionTime::T1100Us,
            vsh_ct: ConversionTime::T1100Us,
            mode: OperationMode::ShuntBusContinuous
        }
    }
}

impl Config {
    pub fn average(self, avg: AveragingMode) -> Self {
        Self {
            avg,
            .. self
        }
    }

    pub fn vbus_ct(self, ct: ConversionTime) -> Self {
        Self {
            vbus_ct: ct,
            .. self
        }
    }

    pub fn vsh_ct(self, ct: ConversionTime) -> Self {
        Self {
            vsh_ct: ct,
            .. self
        }
    }

    pub fn mode(self, mode: OperationMode) -> Self {
        Self {
            mode,
            .. self
        }
    }

    pub fn ch1_en(self, en: bool) -> Self {
        Self {
            ch1_en: en,
            .. self
        }
    }

    pub fn ch2_en(self, en: bool) -> Self {
        Self {
            ch2_en: en,
            .. self
        }
    }

    pub fn ch3_en(self, en: bool) -> Self {
        Self {
            ch3_en: en,
            .. self
        }
    }


}

impl Config {
    pub(crate) fn to_register(&self) -> u16 {
        let mut conf = self.mode as u16
            | (self.vsh_ct as u16) << 3
            | (self.vbus_ct as u16) << 6
            | (self.avg as u16) << 9;
        if self.ch1_en {
            conf |= 1 << 12;
        }
        if self.ch2_en {
            conf |= 1 << 13;
        }
        if self.ch3_en {
            conf |= 1 << 14;
        }
        conf
    }
}

pub struct INA3221<I2C> {
    i2c: I2C,
    address: u8
}

/// Channel measurements
/// Note: Raw values, see datasheet for conversion
pub struct Channel {
    /// Bus voltage
    voltage: u16,
    /// Shunt voltage
    shunt_voltage: i16
}

impl Channel {
    /// Convert raw shunt-voltage to a current by scaling with the shunt resistance
    pub fn current(&self, shunt: f32) -> f32 {
        // 40 uV / LSB, I = V/R
        ((self.shunt_voltage >> 3) as f32 * 40e-6f32) / shunt
    }

    /// Convert raw bus-voltage
    pub fn voltage(&self) -> f32 {
        // 8mV / LSB
        (self.voltage >> 3) as f32 * 8e-3f32
    }
}

impl<I2C, E> INA3221<I2C>
    where I2C: i2c::Write<Error = E> + i2c::Read<Error = E> {

    /// Create new INA3221
    pub fn new(i2c: I2C, address: u8) -> INA3221<I2C> {
        INA3221 {
            i2c,
            address
        }
    }

    /// Create new INA3221 and change its configuration (might fail)
    pub fn new_config(i2c: I2C, address: u8, conf: Config) -> Result<INA3221<I2C>, E>{
        let mut ina = INA3221 {
            i2c,
            address
        };
        ina.write_config(Some(conf))?;
        Ok(ina)
    }

    /// Read a channel's bus and shunt voltage
    pub fn read_channel(&mut self, index: u8) -> Result<Channel, E> {
        let mut buf: [u8; 4] = [0x00; 4];
        self.i2c.write(self.address, &[Register::BusVoltage1 as u8 + (index % 3u8)*2u8])?;
        self.i2c.read(self.address, &mut buf)?;
        Ok(Channel {
            voltage: BigEndian::read_u16(&buf[0..=1]),
            shunt_voltage: BigEndian::read_i16(&buf[2..=3])
        })
    }

    /// Read channel 1-3
    pub fn read_all_channels(&mut self) -> Result<[Channel; 3], E> {
        let mut buf: [u8; 4*3] = [0x00; 4*3];
        self.i2c.write(self.address, &[Register::BusVoltage1 as u8])?;
        self.i2c.read(self.address, &mut buf)?;
        Ok([Channel {
            voltage: BigEndian::read_u16(&buf[0..=1]),
            shunt_voltage: BigEndian::read_i16(&buf[2..=3])
        },Channel {
            voltage: BigEndian::read_u16(&buf[4..=5]),
            shunt_voltage: BigEndian::read_i16(&buf[6..=7])
        },Channel {
            voltage: BigEndian::read_u16(&buf[8..=9]),
            shunt_voltage: BigEndian::read_i16(&buf[10..=11])
        }])
    }

    /// Read Shunt-Voltage Sum
    pub fn read_sum(&mut self) -> Result<u16, E> {
        self.read(Register::ShuntVoltageSum)
    }

    /// Read CVRF (Conversion ready flag) from Mask/Enable register
    /// Useful for single-shot conversion
    pub fn read_cvrf(&mut self) -> Result<bool, E> {
        let mask = self.read(Register::MaskEnable)?;
        Ok(mask & 0x0001u16 != 0)
    }

    /// Read a register
    pub fn read(&mut self, register: Register) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        self.i2c.write(self.address, &[register as u8])?;
        self.i2c.read(self.address, &mut buf)?;
        Ok(BigEndian::read_u16(&buf))
    }

    /// Set the alert and warning limit for a channel
    pub fn write_channel_limit(&mut self, index: u8, alert: u16, warning: u16) -> Result<(), E>{
        let mut buf = [0x00u8; 5];
        buf[0] = Register::CriticalLimit1 as u8 + (index % 3u8)*2u8;
        BigEndian::write_u16_into(&[alert, warning], &mut buf[1..]);
        self.i2c.write(self.address, &buf)
    }

    /// Write upper/lower power valid registers
    pub fn write_power_valid(&mut self, upper: u16, lower: u16) -> Result<(), E>{
        let mut buf = [0x00u8; 5];
        buf[0] = Register::PowerValidUpper as u8;
        BigEndian::write_u16_into(&[upper, lower], &mut buf[1..]);
        self.i2c.write(self.address, &buf)
    }

    /// Write config register
    /// Using None will reset config to default by setting the RST bit
    pub fn write_config(&mut self, conf: Option<Config>) -> Result<(), E>{
        self.write(Register::Configuration, match conf {
            Some(cfg) => cfg.to_register(),
            None => 1u16 << 15
        })
    }

    /// Write Shunt-Voltage Sum-Limit register
    pub fn write_sum_limit(&mut self, limit: i16) -> Result<(), E>{
        self.write(Register::ShuntVoltageSumLimit, limit as u16)
    }

    /// Write a register
    pub fn write(&mut self, register: Register, value: u16) -> Result<(), E> {
        let mut buf: [u8; 3] = [0x00; 3];
        buf[0] = register as u8;
        BigEndian::write_u16(&mut buf, value);
        self.i2c.write(self.address, &buf)
    }

}