extern crate i2cdev;
extern crate byteorder;

use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use i2cdev::core::I2CDevice;
use std::path::Path;
use byteorder::{LittleEndian, BigEndian, WriteBytesExt, ReadBytesExt};
use std::io::Cursor;

const DEFAULT_I2C_ADDRESS: u16 = 0x77;
const DEFAULT_I2C_PATH: &'static str = "/dev/i2c-1";

type Result<T> = std::result::Result<T, Error>;
type Endiness = BigEndian;

enum Error {
    I2cError(LinuxI2CError),
    IoError(std::io::Error),
    Other(()),
}

impl From<LinuxI2CError> for Error {
    fn from(f: LinuxI2CError) -> Self {
        Error::I2cError(f)
    }
}

impl From<std::io::Error> for Error {
    fn from(f: std::io::Error) -> Self {
        Error::IoError(f)
    }
}

impl From<()> for Error {
    fn from(f: ()) -> Self {
        Error::Other(f)
    }
}

enum Register {
    DigT1,
    DigT2,
    DigT3,

    DigP1,
    DigP2,
    DigP3,
    DigP4,
    DigP5,
    DigP6,
    DigP7,
    DigP8,
    DigP9,

    ChipId,
    Version,
    SoftReset,

    /// R calibration stored in 0xE1-0xF0
    Cal26,

    Control,
    Config,
    PressureData,
    TempData,
}

impl<'a> std::convert::From<&'a Register> for u8 {
    fn from(frm: &'a Register) -> u8 {
        use Register::*;
        match *frm {
            DigT1 => 0x88,
            DigT2 => 0x8A,
            DigT3 => 0x8C,

            DigP1 => 0x8E,
            DigP2 => 0x90,
            DigP3 => 0x92,
            DigP4 => 0x94,
            DigP5 => 0x96,
            DigP6 => 0x98,
            DigP7 => 0x9A,
            DigP8 => 0x9C,
            DigP9 => 0x9E,

            ChipId => 0xD0,
            Version => 0xD1,
            SoftReset => 0xE0,

            Cal26 => 0xE1,

            Control => 0xF4,
            Config => 0xF5,
            PressureData => 0xF7,
            TempData => 0xFA,
        }
    }
}

struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,

    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,

    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

struct Bmp280 {
    sensor_id: i32,
    fine: i32,
    calibration: Calibration,
    i2c_device: LinuxI2CDevice,
}

pub struct Builder {
    i2c_address: u16,
    i2c_device: Option<LinuxI2CDevice>,
    i2c_path: String,
}

impl Builder {
    pub fn new() -> Self {
        Builder {
            i2c_address: DEFAULT_I2C_ADDRESS,
            i2c_device: Option::None,
            i2c_path: DEFAULT_I2C_PATH.to_string(),
        }
    }

    pub fn address(&mut self, address: u16) -> &mut Self {
        self.i2c_address = address;
        self
    }

    pub fn device(&mut self, device: LinuxI2CDevice) -> &mut Self {
        self.i2c_device = Some(device);
        self
    }

    pub fn path(&mut self, path: String) -> &mut Self {
        self.i2c_path = path;
        self
    }

    pub fn build(self) -> Result<Bmp280> {
        match self.i2c_device {
            Some(dev) => Ok(Bmp280 { i2c_device: dev }),
            None => {
                let dev = try!(LinuxI2CDevice::new(self.i2c_path, self.i2c_address));
                Ok(Bmp280 { i2c_device: dev })
            }
        }
    }
}

impl Bmp280 {
    fn write8(&self, reg: &Register, value: u8) -> Result<()> {
        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.write(&[value]));
        Ok(())
    }

    fn read8(&self, reg: &Register) -> Result<u8> {
        let mut buf = [0u8; 1];

        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.read(&mut buf));

        let mut curs = Cursor::new(buf);

        let val = try!(curs.read_u8());

        Ok(val)
    }

    fn write16(&self, reg: &Register, value: u16) -> Result<()> {
        let mut buf = vec![0u8, 0u8];
        try!(buf.write_u16::<Endiness>(value));

        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.write(&buf));

        Ok(())
    }

    fn read16(&self, reg: &Register) -> Result<u16> {
        let mut buf = [0u8; 2];

        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.read(&mut buf));

        let mut curs = Cursor::new(buf);

        let val = try!(curs.read_u16::<Endiness>());

        Ok(val)
    }

    fn read16s(&self, reg: &Register) -> Result<i16> {
        let mut buf = [0u8; 2];

        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.read(&mut buf));

        let mut curs = Cursor::new(buf);

        let val = try!(curs.read_i16::<Endiness>());

        Ok(val)
    }

    fn read16le(&self, reg: &Register) -> Result<u16> {
        let mut buf = [0u8; 2];

        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.read(&mut buf));

        let mut curs = Cursor::new(buf);

        let val = try!(curs.read_u16::<LittleEndian>());

        Ok(val)
    }

    fn read16les(&self, reg: &Register) -> Result<i16> {
        let mut buf = [0u8; 2];

        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.read(&mut buf));

        let mut curs = Cursor::new(buf);

        let val = try!(curs.read_i16::<LittleEndian>());

        Ok(val)
    }

    fn read24(&self, reg: &Register) -> Result<u32> {
        let mut buf = [0u8; 3];

        try!(self.i2c_device.write(&[reg.into()]));
        try!(self.i2c_device.read(&mut buf));

        let mut curs = Cursor::new(buf);

        let val = try!(curs.read_uint::<Endiness>(3));

        Ok(val as u32)
    }

    fn read_coefficients(&mut self) -> Result<()> {
        self.calibration.dig_t1 = try!(self.read16le(&Register::DigT1));
        self.calibration.dig_t2 = try!(self.read16les(&Register::DigT2));
        self.calibration.dig_t3 = try!(self.read16les(&Register::DigT3));

        self.calibration.dig_p1 = try!(self.read16le(&Register::DigP1));
        self.calibration.dig_p2 = try!(self.read16les(&Register::DigP2));
        self.calibration.dig_p3 = try!(self.read16les(&Register::DigP3));
        self.calibration.dig_p4 = try!(self.read16les(&Register::DigP4));
        self.calibration.dig_p5 = try!(self.read16les(&Register::DigP5));
        self.calibration.dig_p6 = try!(self.read16les(&Register::DigP6));
        self.calibration.dig_p7 = try!(self.read16les(&Register::DigP7));
        self.calibration.dig_p8 = try!(self.read16les(&Register::DigP8));
        self.calibration.dig_p9 = try!(self.read16les(&Register::DigP9));

        Ok(())
    }

    fn begin(&mut self) -> Result<()> {
        if try!(self.read8(&Register::ChipId)) != 0x58 {
            return Err(Error::Other(()));
        }

        try!(self.read_coefficients());
        try!(self.write8(&Register::Control, 0x3F));

        Ok(())
    }

    pub fn read_temperature(&mut self) -> Result<f32> {
        let mut adc_T = try!(self.read24(&Register::TempData)) as i32;
        adc_T >>= 4;

        let t1 = self.calibration.dig_t1 as i32;
        let t2 = self.calibration.dig_t2 as i32;
        let t3 = self.calibration.dig_t3 as i32;

        let var1 = ((((adc_T >> 3) - (t1 << 1))) * t2) >> 11;
        let var2 = (((((adc_T >> 4) - t1) * ((adc_T >> 4) - t1)) >> 12) * t3) >> 14;

        self.fine = var1 + var2;

        let T = ((self.fine * 5 + 128) >> 8) as f32;
        Ok(T / 100.)
    }

    pub fn read_pressure(&mut self) -> Result<f32> {
        // This is done to initialize the self.fine value.
        try!(self.read_temperature());

        let adc_P = (try!(self.read24(&Register::PressureData)) as i32) >> 4;

        let p1 = self.calibration.dig_p1 as i64;
        let p2 = self.calibration.dig_p2 as i64;
        let p3 = self.calibration.dig_p3 as i64;
        let p4 = self.calibration.dig_p4 as i64;
        let p5 = self.calibration.dig_p5 as i64;
        let p6 = self.calibration.dig_p6 as i64;
        let p7 = self.calibration.dig_p7 as i64;
        let p8 = self.calibration.dig_p8 as i64;
        let p9 = self.calibration.dig_p9 as i64;

        let var1 = (self.fine as i64) - 128000;

        let var2 = var1 * var1 * p6;
        let var2 = var2 + ((var1 * p5) << 17);
        let var2 = var2 + (p4 << 35);

        let var1 = ((var1 * var1 * p3) >> 8) + ((var1 * p2) << 12);
        let var1 = ((((1i64) << 47) + var1)) * (p1) >> 33;

        if var1 == 0 {
            return Err(Error::Other(()));
        }


        let p: i64 = 1048576 - adc_P as i64;
        let p = (((p << 31) - var2) * 3125) / var1;

        let var1 = (p9 * (p >> 13) * (p >> 13)) >> 25;
        let var2 = (p8 * p) >> 19;

        let p = ((p + var1 + var2) >> 8) + (p7 << 4);

        Ok(p as f32 / 256.)
    }


    fn read_altitude(&mut self, sea_level_hpa: f32) -> Result<f32> {
        let pressure = try!(self.read_pressure()) as f32 / 100.;

        let altitude = 44330. * (1. - (pressure / sea_level_hpa).powf(0.1903));
        Ok(altitude)
    }
}
