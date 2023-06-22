//! This module contains code that prepares a payload and ID in accordance with the DroneCAN
//! and Cyphal specifications.

//! [Relevant section of DroneCAN specification](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
//! [Cyphal specification](https://opencyphal.org/specification/Cyphal_Specification.pdf)

//! Todo: Protocol version detection using toggle bit polarity.

#![no_std]

use core::{mem, sync::atomic::AtomicBool};

use num_enum::TryFromPrimitive;

#[cfg(feature = "hal")]
pub mod broadcast;

mod crc;
pub mod dsdl;
#[cfg(feature = "hal")]
pub mod gnss;
pub mod messages;
pub mod protocol;

pub use dsdl::*;
pub use messages::*;
pub use protocol::*;

#[cfg(feature = "hal")]
pub use broadcast::*;

use crate::crc::TransferCrc;

pub const PAYLOAD_SIZE_CONFIG_COMMON: usize = 4;

// Unfortunately, it seems we can't satisfy static allocation using const *methods*.
pub const PAYLOAD_SIZE_NODE_STATUS: usize = 7;

// todo: Protocol enum instead?
static USING_CYPHAL: AtomicBool = AtomicBool::new(false);

/// Calculate the size in bytes needed to store a certain number of bits.
pub fn bit_size_to_byte_size(len_bits: usize) -> usize {
    let base_size = len_bits / 8;

    if base_size % 8 > 0 {
        base_size + 1
    } else {
        base_size
    }
}

#[derive(Clone, Copy)]
pub enum CanError {
    CanHardware,
    PayloadSize,
    FrameSize,
    PayloadData,
}

/// Using fixed values makes configuring the clocks more deterministic and reliable.
#[derive(Clone, Copy, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum CanBitrate {
    B250k = 0,
    B500k = 1,
    B1m = 2,
    B2m = 3,
    B4m = 4,
    B5m = 5,
    B8m = 6,
}

impl Default for CanBitrate {
    fn default() -> Self {
        Self::B1m
    }
}

impl CanBitrate {
    /// Get timings for devcies that have a 160Mhz CAN clock. Note that timings are available for
    /// most of these using a 170Mhz clock as well.
    /// Returns (prescaler, segment 1, segment 2)
    /// http://www.bittiming.can-wiki.info/
    pub fn timings_160_mhz(&self) -> (u16, u8, u8) {
        match self {
            Self::B250k => (40, 13, 2),
            Self::B500k => (20, 13, 2),
            Self::B1m => (10, 13, 2),
            Self::B2m => (5, 13, 2),
            Self::B4m => (4, 8, 1),
            Self::B5m => (2, 13, 2),
            Self::B8m => (2, 8, 1),
        }
    }

    pub fn timings_170_mhz(&self) -> (u16, u8, u8) {
        match self {
            Self::B250k => (40, 14, 2),
            Self::B500k => (20, 14, 2),
            Self::B1m => (10, 14, 2),
            Self::B2m => (5, 14, 2),
            Self::B4m => unimplemented!(),
            Self::B5m => (2, 14, 2),
            Self::B8m => unimplemented!(),
        }
    }
}

/// 16-bit floating point
/// Alternative to `half` lib, without bringing in a dep.
///
#[allow(non_camel_case_types)]
#[derive(Debug)]
/// We use this instead of the `half` lib due to binary size issues when using the lib.
pub struct f16 {
    bits: u16,
}

impl f16 {
    pub const fn from_f32(value: f32) -> Self {
        // half's implementation
        // https://github.com/starkat99/half-rs/blob/main/src/binary16/arch.rs
        // TODO: Replace mem::transmute with to_bits() once to_bits is const-stabilized
        // Convert to raw bytes
        let x: u32 = unsafe { mem::transmute(value) };

        // Extract IEEE754 components
        let sign = x & 0x8000_0000u32;
        let exp = x & 0x7F80_0000u32;
        let man = x & 0x007F_FFFFu32;

        // Check for all exponent bits being set, which is Infinity or NaN
        if exp == 0x7F80_0000u32 {
            // Set mantissa MSB for NaN (and also keep shifted mantissa bits)
            let nan_bit = if man == 0 { 0 } else { 0x0200u32 };
            return Self {
                bits: ((sign >> 16) | 0x7C00u32 | nan_bit | (man >> 13)) as u16,
            };
        }

        // The number is normalized, start assembling half precision version
        let half_sign = sign >> 16;
        // Unbias the exponent, then bias for half precision
        let unbiased_exp = ((exp >> 23) as i32) - 127;
        let half_exp = unbiased_exp + 15;

        // Check for exponent overflow, return +infinity
        if half_exp >= 0x1F {
            return Self {
                bits: (half_sign | 0x7C00u32) as u16,
            };
        }

        // Check for underflow
        if half_exp <= 0 {
            // Check mantissa for what we can do
            if 14 - half_exp > 24 {
                // No rounding possibility, so this is a full underflow, return signed zero
                return return Self {
                    bits: half_sign as u16,
                };
            }
            // Don't forget about hidden leading mantissa bit when assembling mantissa
            let man = man | 0x0080_0000u32;
            let mut half_man = man >> (14 - half_exp);
            // Check for rounding (see comment above functions)
            let round_bit = 1 << (13 - half_exp);
            if (man & round_bit) != 0 && (man & (3 * round_bit - 1)) != 0 {
                half_man += 1;
            }
            // No exponent for subnormals
            return return Self {
                bits: (half_sign | half_man) as u16,
            };
        }

        // Rebias the exponent
        let half_exp = (half_exp as u32) << 10;
        let half_man = man >> 13;
        // Check for rounding (see comment above functions)
        let round_bit = 0x0000_1000u32;

        let bits = if (man & round_bit) != 0 && (man & (3 * round_bit - 1)) != 0 {
            // Round it
            ((half_sign | half_exp | half_man) + 1) as u16
        } else {
            (half_sign | half_exp | half_man) as u16
        };

        Self { bits }
    }

    pub fn to_le_bytes(&self) -> [u8; 2] {
        self.bits.to_le_bytes()
    }

    pub fn to_be_bytes(&self) -> [u8; 2] {
        self.bits.to_be_bytes()
    }

    pub fn as_u16(&self) -> u16 {
        self.bits
    }
}
