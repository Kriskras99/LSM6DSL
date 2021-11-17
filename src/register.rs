//! LSM6DSL register addresses
#![allow(non_camel_case_types, clippy::unreadable_literal)]

use bitflags::bitflags;

/// Register addresses
/// Taken from the LSM6DSL data sheet (Register mapping, pages 48-51)
/// <https://www.st.com/resource/en/datasheet/lsm6dsl.pdf>
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    /// Embedded functions configuration register (Read/Write)
    FUNC_CFG_ACCESS = 0x01,

    /// Sensor sync configuration register (Read/Write)
    SENSOR_SYNC_TIME_FRAME = 0x04,
    SENSOR_SYNC_RES_RATIO = 0x05,
    MASTER_CMD_CODE = 0x60,
    SENS_SYNC_SPI_ERROR_CODE = 0x61,

    /// FIFO configuration registers (Read/Write)
    FIFO_CTRL1 = 0x06,
    FIFO_CTRL2 = 0x07,
    FIFO_CTRL3 = 0x08,
    FIFO_CTRL4 = 0x09,
    FIFO_CTRL5 = 0x0A,

    /// DataReady configuration register (Read/Write)
    DRDY_PULSE_CFG_G = 0x0B,

    /// Interrupt pins control (Read/Write)
    INT1_CTRL = 0x0D,
    INT2_CTRL = 0x0E,

    /// Who I am ID (Read Only)
    WHO_AM_I = 0x0F,

    /// Accelerometer and gyroscope control registers (Read/Write)
    CTRL1_XL = 0x10,
    CTRL2_G = 0x11,
    CTRL3_C = 0x12,
    CTRL4_C = 0x13,
    CTRL5_C = 0x14,
    CTRL6_C = 0x15,
    CTRL7_G = 0x16,
    CTRL8_XL = 0x17,
    CTRL9_XL = 0x18,
    CTRL10_C = 0x19,

    /// I2C master configuration register (Read/Write)
    MASTER_CONFIG = 0x1A,

    /// Status data register for user interface (Read Only)
    STATUS_REG = 0x1E,

    /// Temperature output data registers (Read Only)
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,

    /// Gyroscope output registers for user interface (Read Only)
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,

    /// Accelerometer output registers (Read Only)
    OUTX_L_XL = 0x28,
    OUTX_H_XL = 0x29,
    OUTY_L_XL = 0x2A,
    OUTY_H_XL = 0x2B,
    OUTZ_L_XL = 0x2C,
    OUTZ_H_XL = 0x2D,

    /// Sensor hub output registers (Read Only)
    SENSORHUB1_REG = 0x2E,
    SENSORHUB2_REG = 0x2F,
    SENSORHUB3_REG = 0x30,
    SENSORHUB4_REG = 0x31,
    SENSORHUB5_REG = 0x32,
    SENSORHUB6_REG = 0x33,
    SENSORHUB7_REG = 0x34,
    SENSORHUB8_REG = 0x35,
    SENSORHUB9_REG = 0x36,
    SENSORHUB10_REG = 0x37,
    SENSORHUB11_REG = 0x38,
    SENSORHUB12_REG = 0x39,
    SENSORHUB13_REG = 0x4D,
    SENSORHUB14_REG = 0x4E,
    SENSORHUB15_REG = 0x4F,
    SENSORHUB16_REG = 0x50,
    SENSORHUB17_REG = 0x51,
    SENSORHUB18_REG = 0x52,

    /// FIFO status registers (Read Only)
    FIFO_STATUS1 = 0x3A,
    FIFO_STATUS2 = 0x3B,
    FIFO_STATUS3 = 0x3C,
    FIFO_STATUS4 = 0x3D,

    /// FIFO data output registers (Read Only)
    FIFO_DATA_OUT_L = 0x3E,
    FIFO_DATA_OUT_H = 0x3F,

    /// Timestamp output registers (Read Only)
    TIMESTAMP0_REG = 0x40,
    TIMESTAMP1_REG = 0x41,
    TIMESTAMP2_REG = 0x42,

    /// Step counter timestamp registers (Read Only)
    STEP_TIMESTAMP_L = 0x49,
    STEP_TIMESTAMP_H = 0x4A,

    /// Step counter output registers (Read Only)
    STEP_COUNTER_L = 0x4B,
    STEP_COUNTER_H = 0x4C,

    /// Interrupt registers (Read Only)
    WAKE_UP_SRC = 0x1B,
    TAP_SRC = 0x1C,
    D6D_SRC = 0x1D,
    FUNC_SRC1 = 0x53,
    FUNC_SRC2 = 0x54,
    WRIST_TILT_IA = 0x55,

    /// Interrupt registers (Read/Write)
    TAP_CFG = 0x58,
    TAP_THS_6D = 0x59,
    INT_DUR2 = 0x5A,
    WAKE_UP_THS = 0x5B,
    WAKE_UP_DUR = 0x5C,
    FREE_FALL = 0x5D,
    MD1_CFG = 0x5E,
    MD2_CFG = 0x5F,

    /// External magnetometer raw data output registers (Read Only)
    OUT_MAG_RAW_X_L = 0x66,
    OUT_MAG_RAW_X_H = 0x67,
    OUT_MAG_RAW_Y_L = 0x68,
    OUT_MAG_RAW_Y_H = 0x69,
    OUT_MAG_RAW_Z_L = 0x6A,
    OUT_MAG_RAW_Z_H = 0x6B,

    /// Accelerometer user offset correction  (Read/Write)
    X_OFS_USR = 0x73,
    Y_OFS_USR = 0x74,
    Z_OFS_USR = 0x75,

}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}

bitflags! {
    /// Flags passed as operands to `Register::CTRL1_XL`
    pub struct AccelerometerControlFlags1: u8 {
        /// Output data rate and power mode selection (see `AccelerometerODR`).
        const ODR_XL3 = 0b10000000;
        const ODR_XL2 = 0b01000000;
        const ODR_XL1 = 0b00100000;
        const ODR_XL0 = 0b00010000;

        /// Accelerometer full-scale selection (see `AccelerometerFS`).
        const FS_XL1 = 0b00001000;
        const FS_XL0 = 0b00000100;

        /// Accelerometer digital LPF (LPF1) bandwidth selection. For bandwidth selection refer to `CTRL8_XL_FLAGS`.
        const LPF1_BW_SEL = 0b00000010;

        /// Accelerometer analog chain bandwidth selection (only for accelerometer ODR ≥ 1.67 kHz).
        /// (0: BW @ 1.5 kHz; 1: BW @ 400 Hz)
        const BW0_XL = 0b00000001;
    }

    /// Flags passed as operands to `Register::CTRL2_G`
    pub struct GyroscopeControlFlags2: u8 {
        /// Gyroscope output data rate selection (see `GyroscopeODR`).
        const ODR_G3 = 0b10000000;
        const ODR_G2 = 0b01000000;
        const ODR_G1 = 0b00100000;
        const ODR_G0 = 0b00010000;

        /// Gyroscope full-scale selection (see `GyroscopeFS`).
        const FS_G1 = 0b00001000;
        const FS_G0 = 0b00000100;
        const FS_125 = 0b00000010;
    }

    /// Flags passed as operands to `Register::CTRL3_C`
    pub struct ControlFlags3: u8 {
        /// Reboot memory content (0: normal mode; 1: reboot memory content).
        const BOOT = 0b10000000;

        /// Block Data Update (0: continuous update; 1: output registers not updated until MSB and LSB have been read).
        const BDU = 0b0100000;

        /// Interrupt activation level (0: interrupt output pads active high; 1: interrupt output pads active low).
        const H_LACTIVE = 0b00100000;

        /// Push-pull/open-drain selection on INT1 and INT2 pads (0: push-pull mode; 1: open-drain mode).
        const PP_OD = 0b00010000;

        /// SPI Serial Interface Mode selection (0: 4-wire interface; 1: 3-wire interface).
        const SIM = 0b00001000;

        /// Register address automatically incremented during a multiple byte access with a serial interface (I2C or SPI) (0: disabled; 1: enabled).
        const IF_INC = 0b00000100;

        /// Big/Little Endian Data selection (0: data LSB @ lower address; 1: data MSB @ lower address).
        const BLE = 0b00000010;

        /// Software reset (0: normal mode; 1: reset device).
        const SW_RESET = 0b00000001;
    }

    /// Flags passed as operands to `Register::CTRL4_C`
    pub struct ControlFlags4: u8 {
        /// Extend DEN functionality to accelerometer sensor (0: disabled; 1: enabled).
        const DEN_XL_EN = 0b10000000;

        /// Gyroscope sleep mode enable (0: disabled; 1: enabled).
        const SLEEP = 0b01000000;

        /// DEN DRDY signal on INT1 pad (0: disabled; 1: enabled).
        const DEN_DRDY_INT1 = 0b00100000;

        /// All interrupt signals available on INT1 pad enable (0: interrupt signals divided between INT1 and INT2 pads; 1: all interrupt signals in logic or on INT1 pad).
        const INT2_on_INT1 = 0b00010000;

        /// Configuration 1 data available enable bit (0: DA timer disabled; 1: DA timer enabled).
        const DRDY_MASK = 0b00001000;

        /// Disable I2C interface (0: both I2C and SPI enabled; 1: I2C disabled, SPI only enabled).
        const I2C_disable = 0b00000100;

        /// Enable gyroscope digital LPF1. The bandwidth can be selected through FTYPE_1..0 in ControlFlags6 (0: disabled; 1: enabled).
        const LPF1_SEL_G = 0b00000010;
    }

    /// Flags passed as operands to `Register::CTRL5_C`
    pub struct ControlFlags5: u8 {
        /// Circular burst-mode (rounding) read from the output registers (see `Rounding`).
        const ROUNDING2 = 0b10000000;
        const ROUNDING1 = 0b01000000;
        const ROUNDING0 = 0b00100000;

        /// DEN active level configuration.
        const DEN_LH = 0b00010000;

        /// Angular rate sensor self-test enable.
        const ST1_G = 0b00001000;
        const ST0_G = 0b00000100;

        /// Linear acceleration sensor self-test enable.
        const ST1_XL = 0b00000010;
        const ST0_XL = 0b00000001;
    }

    /// Flags passed as operands to `Register::CTRL6_C`
    pub struct ControlFlags6: u8 {
        /// DEN data edge-sensitive trigger enable (see `TriggerModeSelection`).
        const TRIG_EN = 0b10000000;

        /// DEN data level-sensitive trigger enable (see `TriggerModeSelection`).
        const LVL_EN = 0b01000000;

        /// DEN level-sensitive latched enable (see `TriggerModeSelection`).
        const LVL2_EN = 0b00100000;

        /// High-performance operating mode disable for accelerometer (0: high-performance operating mode enabled; 1: high-performance operating mode disabled).
        const XL_HM_MODE = 0b00010000;

        /// Weight of XL user offset bits of registers `Register::X_OFS_USR`, `Register::Y_OFS_USR`, `Register::Z_OFS_USR` (0 = 2E-10 g/LSB 1 = 2E-6 g/LSB).
        const USR_OFF_W = 0b00001000;

        /// Gyroscope's low-pass filter (LPF1) bandwidth selection (see `GyroscopeLPF1BandwidthSelection`).
        const FTYPE_1 = 0b00000010;
        const FTYPE_0 = 0b00000001;
    }

    /// Flags passed as operands to `Register::CTRL7_G`
    pub struct GyroscopeControlFlags7: u8 {
        /// High-performance operating mode disable for gyroscope (0: high-performance operating mode enabled; 1: high-performance operating mode disabled).
        const G_HM_MODE = 0b10000000;

        /// Gyroscope digital high-pass filter enable. The filter is enabled only if the gyro is in HP mode (0: HPF disabled; 1: HPF enabled).
        const HP_EN_G = 0b01000000;

        /// Gyroscope digital HP filter cutoff selection (see `GyroscopeHPFilterCutoff`).
        const HPM1_G = 0b00100000;
        const HPM0_G = 0b00010000;

        /// Source register rounding function (0: Rounding disabled; 1: Rounding enabled).
        const ROUNDING_STATUS = 0b00000100;
    }

    /// Flags passed as operands to `Register::CTRL8_XL`
    pub struct AccelerometerControlFlags8: u8 {
        /// Accelerometer low-pass filter LPF2 selection.
        const LPF2_XL_EN = 0b10000000;
        
        ///Accelerometer LPF2 and high-pass filter configuration and cutoff setting (see `AccelerometerbandwidthDivisionSelection`).
        const HPCF_XL1 = 0b01000000;
        const HPCF_XL0 = 0b00100000;
        
        /// Enable HP filter reference mode (0: disabled; 1: enabled).
        const HP_REF_MODE = 0b00010000;
        
        /// Composite filter input selection (0: ODR/2 low pass filtered sent to composite filter; 1: ODR/4 low pass filtered sent to composite filter).
        const INPUT_COMPOSITE = 0b00001000;
        
        /// Accelerometer slope filter / high-pass filter selection.
        const HP_SLOPE_XL_EN = 0b00000100;
        
        /// LPF2 on 6D function selection.
        const LOW_PASS_ON_6D = 0b00000001;
    }
}

/// Default `AccelerometerControlFlags1` settings:
///
/// - Output data rate: 3.33 kHz
/// - Full-scale range: ±8g
/// - Bandwidth: 400 Hz
impl Default for AccelerometerControlFlags1 {
    fn default() -> Self {
        AccelerometerODR::ODR_3_33_KHZ.bits() | AccelerometerFS::PLUSMINUS_8G.bits() | AccelerometerControlFlags1::LPF1_BW_SEL | AccelerometerControlFlags1::BW0_XL
    }
}

/// g-Range setting flags which can be OR'd with `AccelerometerControlFlags1` and passed as
/// operands to `Register::CTRL1_XL`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AccelerometerFS {
    /// ±2g
    PLUSMINUS_2G = 0b0000,

    /// ±4g
    PLUSMINUS_4G = 0b1000,

    /// ±8g
    PLUSMINUS_8G = 0b1100,

    /// ±16g
    PLUSMINUS_16G = 0b0100,
}

impl AccelerometerFS {
    /// Get `AccelerometerFS` representation
    pub fn bits(self) -> AccelerometerControlFlags1 {
        match self {
            AccelerometerFS::PLUSMINUS_2G => AccelerometerControlFlags1::empty(),
            AccelerometerFS::PLUSMINUS_4G => AccelerometerControlFlags1::FS_XL1,
            AccelerometerFS::PLUSMINUS_16G => AccelerometerControlFlags1::FS_XL0,
            AccelerometerFS::PLUSMINUS_8G => AccelerometerControlFlags1::FS_XL1 | AccelerometerControlFlags1::FS_XL0,
        }
    }
}

/// Output data rate setting flags which can be OR'd with `AccelerometerControlFlags1` and passed as
/// operands to `Register::CTRL1_XL`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AccelerometerODR {
    /// Power-down
    POWER_DOWN = 0b00000000,

    /// 1.6 Hz (low power only) / 12.5 Hz (high performance)
    ODR_1_6_HZ = 0b10110000,

    /// 12.5 Hz (low power) / 12.5 Hz (high performance)
    ODR_12_5_HZ = 0b00010000,

    /// 26 Hz (low power) / 26 Hz (high performance)
    ODR_26_HZ = 0b00100000,

    /// 52 Hz (low power) / 52 Hz (high performance)
    ODR_52_HZ = 0b00110000,

    /// 104 Hz (normal mode) / 104 Hz (high performance)
    ODR_104_HZ = 0b01000000,

    /// 208 Hz (normal mode) / 208 Hz (high performance)
    ODR_208_HZ = 0b01010000,

    /// 416 Hz (high performance)
    ODR_416_HZ = 0b01100000,

    /// 833 Hz (high performance)
    ODR_833_HZ = 0b01110000,

    /// 1.66 kHz Hz (high performance)
    ODR_1_66_KHZ = 0b10000000,

    /// 3.33 kHz (high performance)
    ODR_3_33_KHZ = 0b10010000,

    /// 6.66 kHz (high performance)
    ODR_6_66_KHZ = 0b10100000
}

impl AccelerometerODR {
    /// Get `AccelerometerODR` representation
    pub fn bits(self) -> AccelerometerControlFlags1 {
        match self {
            AccelerometerODR::POWER_DOWN => AccelerometerControlFlags1::empty(),
            AccelerometerODR::ODR_1_6_HZ => AccelerometerControlFlags1::ODR_XL3 | AccelerometerControlFlags1::ODR_XL1 | AccelerometerControlFlags1::ODR_XL0,
            AccelerometerODR::ODR_12_5_HZ =>  AccelerometerControlFlags1::ODR_XL0,
            AccelerometerODR::ODR_26_HZ => AccelerometerControlFlags1::ODR_XL1,
            AccelerometerODR::ODR_52_HZ => AccelerometerControlFlags1::ODR_XL1 | AccelerometerControlFlags1::ODR_XL0,
            AccelerometerODR::ODR_104_HZ => AccelerometerControlFlags1::ODR_XL2,
            AccelerometerODR::ODR_208_HZ => AccelerometerControlFlags1::ODR_XL2 | AccelerometerControlFlags1::ODR_XL0,
            AccelerometerODR::ODR_416_HZ => AccelerometerControlFlags1::ODR_XL2 | AccelerometerControlFlags1::ODR_XL1,
            AccelerometerODR::ODR_833_HZ => AccelerometerControlFlags1::ODR_XL2 | AccelerometerControlFlags1::ODR_XL1 | AccelerometerControlFlags1::ODR_XL0,
            AccelerometerODR::ODR_1_66_KHZ => AccelerometerControlFlags1::ODR_XL3,
            AccelerometerODR::ODR_3_33_KHZ => AccelerometerControlFlags1::ODR_XL3 | AccelerometerControlFlags1::ODR_XL0,
            AccelerometerODR::ODR_6_66_KHZ => AccelerometerControlFlags1::ODR_XL3 | AccelerometerControlFlags1::ODR_XL1,
        }
    }

    pub fn from_u8(bits: u8) -> AccelerometerODR {
        let cleaned = bits & 0b11110000;
        match cleaned {
            0u8 => AccelerometerODR::POWER_DOWN,
            0b10110000 => AccelerometerODR::ODR_1_6_HZ,
            0b00010000 => AccelerometerODR::ODR_12_5_HZ,
            0b00100000 => AccelerometerODR::ODR_26_HZ,
            0b00110000 => AccelerometerODR::ODR_52_HZ,
            0b01000000 => AccelerometerODR::ODR_104_HZ,
            0b01010000 => AccelerometerODR::ODR_208_HZ,
            0b01100000 => AccelerometerODR::ODR_416_HZ,
            0b01110000 => AccelerometerODR::ODR_833_HZ,
            0b10000000 => AccelerometerODR::ODR_1_66_KHZ,
            0b10010000 => AccelerometerODR::ODR_3_33_KHZ,
            0b10100000 => AccelerometerODR::ODR_6_66_KHZ,
            _ => panic!("Invalid input!")
        }
    }

    pub fn to_hz(self) -> f32 {
        match self {
            AccelerometerODR::POWER_DOWN => 0f32,
            AccelerometerODR::ODR_1_6_HZ => 1.6f32,
            AccelerometerODR::ODR_12_5_HZ => 12.5f32,
            AccelerometerODR::ODR_26_HZ => 26f32,
            AccelerometerODR::ODR_52_HZ => 52f32,
            AccelerometerODR::ODR_104_HZ => 104f32,
            AccelerometerODR::ODR_208_HZ => 208f32,
            AccelerometerODR::ODR_416_HZ => 416f32,
            AccelerometerODR::ODR_833_HZ => 833f32,
            AccelerometerODR::ODR_1_66_KHZ => 1660f32,
            AccelerometerODR::ODR_3_33_KHZ => 3330f32,
            AccelerometerODR::ODR_6_66_KHZ => 6660f32,
        }
    }
}

/// Default `GyroscopeControlFlags2` settings:
///
/// - Output data rate: 6.66 kHz
/// - Full-scale range: 250 dps
impl Default for GyroscopeControlFlags2 {
    fn default() -> Self {
        GyroscopeODR::ODR_3_33_KHZ.bits() | GyroscopeFS::FS_2000_DPS.bits()
    }
}

/// g-Range setting flags which can be OR'd with `GyroscopeControlFlags2` and passed as
/// operands to `Register::CTRL2_G`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GyroscopeFS {
    /// 125 dps
    FS_125_DPS = 0b0010,

    /// 250 dps
    FS_250_DPS = 0b0000,

    /// 500 dps
    FS_500_DPS = 0b0100,

    /// 1000 dps
    FS_1000_DPS = 0b1000,

    /// 2000 dps
    FS_2000_DPS = 0b1100,
}

impl GyroscopeFS {
    /// Get `GyroscopeFS` representation
    pub fn bits(self) -> GyroscopeControlFlags2 {
        match self {
            GyroscopeFS::FS_125_DPS => GyroscopeControlFlags2::FS_125,
            GyroscopeFS::FS_250_DPS => GyroscopeControlFlags2::empty(),
            GyroscopeFS::FS_500_DPS => GyroscopeControlFlags2::FS_G0,
            GyroscopeFS::FS_1000_DPS => GyroscopeControlFlags2::FS_G1,
            GyroscopeFS::FS_2000_DPS => GyroscopeControlFlags2::FS_G1 | GyroscopeControlFlags2::FS_G0,
        }
    }
}

/// Output data rate setting flags which can be OR'd with `GyroscopeControlFlags2` and passed as
/// operands to `Register::CTRL2_G`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GyroscopeODR {
    /// Power-down
    POWER_DOWN = 0b00000000,

    /// 1.6 Hz (low power only) / 12.5 Hz (high performance)
    ODR_1_6_HZ = 0b10110000,

    /// 12.5 Hz (low power) / 12.5 Hz (high performance)
    ODR_12_5_HZ = 0b00010000,

    /// 26 Hz (low power) / 26 Hz (high performance)
    ODR_26_HZ = 0b00100000,

    /// 52 Hz (low power) / 52 Hz (high performance)
    ODR_52_HZ = 0b00110000,

    /// 104 Hz (normal mode) / 104 Hz (high performance)
    ODR_104_HZ = 0b01000000,

    /// 208 Hz (normal mode) / 208 Hz (high performance)
    ODR_208_HZ = 0b01010000,

    /// 416 Hz (high performance)
    ODR_416_HZ = 0b01100000,

    /// 833 Hz (high performance)
    ODR_833_HZ = 0b01110000,

    /// 1.66 kHz Hz (high performance)
    ODR_1_66_KHZ = 0b10000000,

    /// 3.33 kHz (high performance)
    ODR_3_33_KHZ = 0b10010000,

    /// 6.66 kHz (high performance)
    ODR_6_66_KHZ = 0b10100000
}

impl GyroscopeODR {
    /// Get `GyroscopeODR` representation
    pub fn bits(self) -> GyroscopeControlFlags2 {
        match self {
            GyroscopeODR::POWER_DOWN => GyroscopeControlFlags2::empty(),
            GyroscopeODR::ODR_1_6_HZ => GyroscopeControlFlags2::ODR_G3 | GyroscopeControlFlags2::ODR_G1 | GyroscopeControlFlags2::ODR_G0,
            GyroscopeODR::ODR_12_5_HZ =>  GyroscopeControlFlags2::ODR_G0,
            GyroscopeODR::ODR_26_HZ => GyroscopeControlFlags2::ODR_G1,
            GyroscopeODR::ODR_52_HZ => GyroscopeControlFlags2::ODR_G1 | GyroscopeControlFlags2::ODR_G0,
            GyroscopeODR::ODR_104_HZ => GyroscopeControlFlags2::ODR_G2,
            GyroscopeODR::ODR_208_HZ => GyroscopeControlFlags2::ODR_G2 | GyroscopeControlFlags2::ODR_G0,
            GyroscopeODR::ODR_416_HZ => GyroscopeControlFlags2::ODR_G2 | GyroscopeControlFlags2::ODR_G1,
            GyroscopeODR::ODR_833_HZ =>GyroscopeControlFlags2::ODR_G2 | GyroscopeControlFlags2::ODR_G1 | GyroscopeControlFlags2::ODR_G0,
            GyroscopeODR::ODR_1_66_KHZ => GyroscopeControlFlags2::ODR_G3,
            GyroscopeODR::ODR_3_33_KHZ => GyroscopeControlFlags2::ODR_G3 | GyroscopeControlFlags2::ODR_G0,
            GyroscopeODR::ODR_6_66_KHZ => GyroscopeControlFlags2::ODR_G3 | GyroscopeControlFlags2::ODR_G1,
        }
    }
}

/// Default `ControlFlags3` settings:
impl Default for ControlFlags3 {
    fn default() -> Self {
        ControlFlags3::BDU | ControlFlags3::IF_INC
    }
}

/// Default `ControlFlags4` settings:
impl Default for ControlFlags4 {
    fn default() -> Self {
        ControlFlags4::empty()
    }
}

/// Default `ControlFlags5` settings:
impl Default for ControlFlags5 {
    fn default() -> Self {
        ControlFlags5::empty()
    }
}

/// Rounding setting flags which can be OR'd with `ControlFlags5` and passed as
/// operands to `Register::CTRL5_C`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Rounding {
    NO_ROUNDING = 0b00000000,
    ACCELEROMETER_ONLY = 0b00100000,
    GYROSCOPE_ONLY = 0b01000000,
    ACCELEROMETER_AND_GYROSCOPE = 0b01100000,
}

impl Rounding {
    /// Get `Rounding` representation
    pub fn bits(self) -> ControlFlags5 {
        match self {
            Rounding::NO_ROUNDING => ControlFlags5::empty(),
            Rounding::ACCELEROMETER_ONLY => ControlFlags5::ROUNDING0,
            Rounding::GYROSCOPE_ONLY =>  ControlFlags5::ROUNDING1,
            Rounding::ACCELEROMETER_AND_GYROSCOPE => ControlFlags5::ROUNDING1 | ControlFlags5::ROUNDING0,
        }
    }
}

/// Gyroscope self-test setting flags which can be OR'd with `ControlFlags5` and passed as
/// operands to `Register::CTRL5_C`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GyroscopeSelfTest {
    NORMAL_MODE = 0b00000000,
    POSITIVE_SIGN_SELF_TEST = 0b00000100,
    NEGATIVE_SIGN_SELF_TEST = 0b00001000,
}

impl GyroscopeSelfTest {
    /// Get `GyroscopeSelfTest` representation
    pub fn bits(self) -> ControlFlags5 {
        match self {
            GyroscopeSelfTest::NORMAL_MODE => ControlFlags5::empty(),
            GyroscopeSelfTest::POSITIVE_SIGN_SELF_TEST => ControlFlags5::ST0_G,
            GyroscopeSelfTest::NEGATIVE_SIGN_SELF_TEST => ControlFlags5::ST1_G,
        }
    }
}

/// Accelerometer self-test setting flags which can be OR'd with `ControlFlags5` and passed as
/// operands to `Register::CTRL5_C`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AccelerometerSelfTest {
    NORMAL_MODE = 0b00000000,
    POSITIVE_SIGN_SELF_TEST = 0b00000100,
    NEGATIVE_SIGN_SELF_TEST = 0b00001000,
}

impl AccelerometerSelfTest {
    /// Get `AccelerometerSelfTest` representation
    pub fn bits(self) -> ControlFlags5 {
        match self {
            AccelerometerSelfTest::NORMAL_MODE => ControlFlags5::empty(),
            AccelerometerSelfTest::POSITIVE_SIGN_SELF_TEST => ControlFlags5::ST0_XL,
            AccelerometerSelfTest::NEGATIVE_SIGN_SELF_TEST => ControlFlags5::ST1_XL,
        }
    }
}

/// Default `ControlFlags6` settings:
impl Default for ControlFlags6 {
    fn default() -> Self {
        ControlFlags6::empty()
    }
}

/// Trigger mode selection flags which can be OR'd with `ControlFlags6` and passed as
/// operands to `Register::CTRL6_C`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TriggerModeSelection {
    EDGE_SENSITIVE_TRIGGER_MODE = 0b10000000,
    LEVEL_SENSITIVE_TRIGGER_MODE = 0b01000000,
    LEVEL_SENSITIVE_LATCHED_MODE = 0b01100000,
    LEVEL_SENSITIVE_FIFO_ENABLE_MODE = 0b11000000,
}

impl TriggerModeSelection {
    /// Get `TriggerModeSelection` representation
    pub fn bits(self) -> ControlFlags6 {
        match self {
            TriggerModeSelection::EDGE_SENSITIVE_TRIGGER_MODE => ControlFlags6::TRIG_EN,
            TriggerModeSelection::LEVEL_SENSITIVE_TRIGGER_MODE => ControlFlags6::LVL_EN,
            TriggerModeSelection::LEVEL_SENSITIVE_LATCHED_MODE => ControlFlags6::LVL_EN | ControlFlags6::LVL2_EN,
            TriggerModeSelection::LEVEL_SENSITIVE_FIFO_ENABLE_MODE => ControlFlags6::TRIG_EN | ControlFlags6::LVL_EN
        }
    }
}

/// Gyroscope LPF1 bandwidth selection flags which can be OR'd with `ControlFlags6` and passed as
/// operands to `Register::CTRL6_C`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GyroscopeLPF1BandwidthSelection {
    BANDWIDTH_LOW = 0b10,
    BANDWIDTH_MEDIUM = 0b01,
    BANDWIDTH_HIGH = 0b00,
    BANDWIDTH_HIGHEST = 0b11,
}

impl GyroscopeLPF1BandwidthSelection {
    /// Get `GyroscopeLPF1BandwidthSelection` representation
    pub fn bits(self) -> ControlFlags6 {
        match self {
            GyroscopeLPF1BandwidthSelection::BANDWIDTH_LOW => ControlFlags6::FTYPE_1,
            GyroscopeLPF1BandwidthSelection::BANDWIDTH_MEDIUM => ControlFlags6::FTYPE_0,
            GyroscopeLPF1BandwidthSelection::BANDWIDTH_HIGH => ControlFlags6::empty(),
            GyroscopeLPF1BandwidthSelection::BANDWIDTH_HIGHEST => ControlFlags6::FTYPE_1 | ControlFlags6::FTYPE_0
        }
    }
}

/// Default `GyroscopeControlFlags7` settings:
impl Default for GyroscopeControlFlags7 {
    fn default() -> Self {
        GyroscopeControlFlags7::empty()
    }
}

/// Gyroscope HP filter cutoff selection flags which can be OR'd with `GyroscopeControlFlags7` and passed as
/// operands to `Register::CTRL7_G`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum GyroscopeHPFilterCutoff {
    CUTOFF_16_MHZ = 0b000000,
    CUTOFF_65_MHZ = 0b010000,
    CUTOFF_260_MHZ = 0b100000,
    CUTOFF_1_04_MHZ = 0b110000,
}

impl GyroscopeHPFilterCutoff {
    /// Get `GyroscopeHPFilterCutoff` representation
    pub fn bits(self) -> GyroscopeControlFlags7 {
        match self {
            GyroscopeHPFilterCutoff::CUTOFF_16_MHZ => GyroscopeControlFlags7::empty(),
            GyroscopeHPFilterCutoff::CUTOFF_65_MHZ => GyroscopeControlFlags7::HPM0_G,
            GyroscopeHPFilterCutoff::CUTOFF_260_MHZ => GyroscopeControlFlags7::HPM1_G,
            GyroscopeHPFilterCutoff::CUTOFF_1_04_MHZ => GyroscopeControlFlags7::HPM1_G | GyroscopeControlFlags7::HPM0_G
        }
    }
}

/// Default `AccelerometerControlFlags8` settings:
impl Default for AccelerometerControlFlags8 {
    fn default() -> Self {
        AccelerometerControlFlags8::LPF2_XL_EN | AccelerometerBandwidthDivisionSelection::DIVIDE_9.bits() | AccelerometerControlFlags8::INPUT_COMPOSITE
    }
}

/// Accelerometer bandwidth selection flags which can be OR'd with `AccelerometerControlFlags8` and passed as
/// operands to `Register::CTRL8_XL`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AccelerometerBandwidthDivisionSelection {
    DIVIDE_4_OR_50 = 0b0000000,
    DIVIDE_100 = 0b0100000,
    DIVIDE_9 = 0b1000000,
    DIVIDE_400 = 0b1100000,
}

impl AccelerometerBandwidthDivisionSelection {
    /// Get `AccelerometerBandwidthDivisionSelection` representation
    pub fn bits(self) -> AccelerometerControlFlags8 {
        match self {
            AccelerometerBandwidthDivisionSelection::DIVIDE_4_OR_50 => AccelerometerControlFlags8::empty(),
            AccelerometerBandwidthDivisionSelection::DIVIDE_100 => AccelerometerControlFlags8::HPCF_XL0,
            AccelerometerBandwidthDivisionSelection::DIVIDE_9 => AccelerometerControlFlags8::HPCF_XL1,
            AccelerometerBandwidthDivisionSelection::DIVIDE_400 => AccelerometerControlFlags8::HPCF_XL1 | AccelerometerControlFlags8::HPCF_XL0
        }
    }
}
