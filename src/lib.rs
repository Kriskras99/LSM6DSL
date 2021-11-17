//! Platform-agnostic LSM6DSL accelerometer/gyroscope driver which uses I2C via
//! [embedded-hal] and implements the [`Accelerometer`][accelerometer] and
//! [`Gyroscope`][gyroscope] traits.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [accelerometer]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//! [gyroscope]: https://docs.rs/gyroscope/latest/accelerometer/trait.Gyroscope.html

#![no_std]

extern crate embedded_hal as hal;

mod register;
use register::*;

use core::fmt::Debug;
use hal::blocking::i2c::{Write, WriteRead, Read};

/// LSM6DSL I2C address.
/// Assumes SDA/SA0 address pin low
pub const ADDRESS: u8 = 0x6A;

/// LSM6DSL device ID
pub const DEVICE_ID: u8 = 0x6A;

/// LSM6DSL driver
pub struct Lsm6dsl<I2C> {
    /// Underlying I2C device
    i2c: I2C,
}

impl<I2C, E> Lsm6dsl<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
    E: Debug,
{
    /// Create a new Lsm6dsl driver from the given I2C peripheral
    ///
    /// Default tap detection level: 2G, 31.25ms duration, single tap only
    pub fn new(i2c: I2C) -> Result<Self, E> {
        let mut lsm6dsl = Lsm6dsl {
            i2c
        };

        // Ensure we have the correct device ID for the LSM6DSL
        if lsm6dsl.get_device_id()? != DEVICE_ID {
            panic!("{}", "Device ID does not match LSM6DSL");
        }

        // Configure accelerometer
        lsm6dsl.write_register(Register::CTRL1_XL, AccelerometerControlFlags1::default().bits())?;

        // Configure gyroscope
        lsm6dsl.write_register(Register::CTRL2_G, GyroscopeControlFlags2::default().bits())?;

        // General configuration
        lsm6dsl.write_register(Register::CTRL3_C, ControlFlags3::default().bits())?;

        // Low pass filter config for accelerometer
        lsm6dsl.write_register(Register::CTRL8_XL, AccelerometerControlFlags8::default().bits())?;

        Ok(lsm6dsl)
    }

    pub fn set_accelerometer_control_flags_1(&mut self, flags: AccelerometerControlFlags1) -> Result<(), E> {
        self.write_register(Register::CTRL1_XL, flags.bits())
    }

    pub fn set_gyroscope_control_flags_2(&mut self, flags: GyroscopeControlFlags2) -> Result<(), E> {
        self.write_register(Register::CTRL2_G, flags.bits())
    }

    pub fn set_control_flags_3(&mut self, flags: ControlFlags3) -> Result<(), E> {
        self.write_register(Register::CTRL3_C, flags.bits())
    }

    pub fn set_control_flags_4(&mut self, flags: ControlFlags4) -> Result<(), E> {
        self.write_register(Register::CTRL4_C, flags.bits())
    }

    pub fn set_control_flags_5(&mut self, flags: ControlFlags5) -> Result<(), E> {
        self.write_register(Register::CTRL5_C, flags.bits())
    }

    pub fn set_control_flags_6(&mut self, flags: ControlFlags6) -> Result<(), E> {
        self.write_register(Register::CTRL6_C, flags.bits())
    }

    pub fn set_gyroscope_control_flags_7(&mut self, flags: GyroscopeControlFlags7) -> Result<(), E> {
        self.write_register(Register::CTRL7_G, flags.bits())
    }

    pub fn set_accelerometer_control_flags_8(&mut self, flags: AccelerometerControlFlags8) -> Result<(), E> {
        self.write_register(Register::CTRL8_XL, flags.bits())
    }

    /// Get the raw acceleration in the X-axis
    pub fn read_raw_acceleration_x(&mut self) -> Result<i16, E> {
        self.read_register_i16(Register::OUTX_L_XL, Register::OUTX_H_XL)
    }

    /// Get the raw acceleration in the Y-axis
    pub fn read_raw_acceleration_y(&mut self) -> Result<i16, E> {
        self.read_register_i16(Register::OUTY_L_XL, Register::OUTY_H_XL)
    }

    /// Get the raw acceleration in the Z-axis
    pub fn read_raw_acceleration_z(&mut self) -> Result<i16, E> {
        self.read_register_i16(Register::OUTZ_L_XL, Register::OUTZ_H_XL)
    }

    /// Get the raw angular velocity in the X-axis
    pub fn read_raw_angular_velocity_x(&mut self) -> Result<i16, E> {
        self.read_register_i16(Register::OUTX_L_G, Register::OUTX_H_G)
    }

    /// Get the raw angular velocity in the Y-axis
    pub fn read_raw_angular_velocity_y(&mut self) -> Result<i16, E> {
        self.read_register_i16(Register::OUTY_L_G, Register::OUTY_H_G)
    }

    /// Get the raw angular velocity in the Z-axis
    pub fn read_raw_angular_velocity_z(&mut self) -> Result<i16, E> {
        self.read_register_i16(Register::OUTZ_L_G, Register::OUTZ_H_G)
    }

    /// Get the raw temperature
    pub fn read_raw_temperature(&mut self) -> Result<i16, E> {
        self.read_register_i16(Register::OUT_TEMP_L, Register::OUT_TEMP_H)
    }

    /// Write to the given register
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), E> {
        self.i2c.write(ADDRESS, &[register.addr(), value])?;
        Ok(())
    }

    /// Read the given register
    fn read_register(&mut self, register: Register) -> Result<u8, E> {
        let mut buffer: [u8; 1] = [0; 1];
        self.i2c.write_read(ADDRESS, &[register.addr()], &mut buffer).map(|_| buffer[0])
    }

    fn read_register_i16(&mut self, reg_low: Register, reg_high: Register) -> Result<i16, E> {
        let low = self.read_register(reg_low)?;
        let high = self.read_register(reg_high)?;

        // Convert the low and high bytes to signed 16-bit integer
        let signed = i16::from_le_bytes([high, low]);
        Ok(signed)
    }

    /// Get the device ID
    fn get_device_id(&mut self) -> Result<u8, E> {
        self.read_register(Register::WHO_AM_I)
    }

}
