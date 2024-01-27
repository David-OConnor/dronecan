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
#[cfg(feature = "hal")]
pub mod hardware;
pub mod messages;
pub mod protocol;

#[cfg(feature = "hal")]
pub use broadcast::*;
pub use dsdl::*;
#[cfg(feature = "hal")]
pub use hardware::{setup_protocol_filters, ALLOC_STAGE, NODE_ID, init_id_alloc_request};
pub use messages::*;
pub use protocol::*;

pub const PAYLOAD_SIZE_CONFIG_COMMON: usize = 4;

// Unfortunately, it seems we can't satisfy static allocation using const *methods*.
pub const PAYLOAD_SIZE_NODE_STATUS: usize = 7;

pub const NODE_STATUS_BROADCAST_PERIOD: f32 = 1.; // In s. Between 2 and 1000.

// todo: Protocol enum instead?
static USING_CYPHAL: AtomicBool = AtomicBool::new(false);

/// Calculate the size in bytes needed to store a certain number of bits.
pub fn bit_size_to_byte_size(len_bits: usize) -> usize {
    let base_size = len_bits / 8;

    if len_bits % 8 > 0 {
        base_size + 1
    } else {
        base_size
    }
}

#[derive(Clone, Copy)]
pub enum CanError {
    Hardware,
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
    /// Get timings for devcies that have a 160Mhz CAN clock. Note that not all timings are available
    /// at all input clock speeds. Choosing a CAN input clock of 80 or 160Mhz is the most flexible,
    /// of the speeds available here.
    /// Returns (prescaler, segment 1, segment 2)
    /// http://www.bittiming.can-wiki.info/
    pub fn timings_80_mhz(&self) -> (u16, u8, u8) {
        match self {
            Self::B250k => (20, 13, 2),
            Self::B500k => (10, 13, 2),
            Self::B1m => (5, 13, 2),
            Self::B2m => (4, 8, 1),
            Self::B4m => (2, 8, 1),
            Self::B5m => (1, 13, 2),
            Self::B8m => (1, 8, 1),
        }
    }

    pub fn timings_100_mhz(&self) -> (u16, u8, u8) {
        match self {
            Self::B250k => (40, 8, 1),
            Self::B500k => (20, 8, 1),
            Self::B1m => (10, 8, 1),
            Self::B2m => (5, 8, 1),
            Self::B4m => unimplemented!(),
            Self::B5m => unimplemented!(),
            Self::B8m => unimplemented!(),
        }
    }

    pub fn timings_120_mhz(&self) -> (u16, u8, u8) {
        match self {
            Self::B250k => (30, 13, 2),
            Self::B500k => (15, 13, 2),
            Self::B1m => (8, 12, 2),
            Self::B2m => (4, 12, 2),
            Self::B4m => (2, 12, 2),
            Self::B5m => unimplemented!(),
            Self::B8m => (1, 12, 2),
        }
    }

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

/// from `half`'s impl: https://github.com/starkat99/half-rs/blob/main/src/leading_zeros.rs
#[inline]
const fn leading_zeros_u16(mut x: u16) -> u32 {
    use crunchy::unroll;
    let mut c = 0;
    let msb = 1 << 15;
    unroll! { for i in 0 .. 16 {
        if x & msb == 0 {
            c += 1;
        } else {
            return c;
        }
        #[allow(unused_assignments)]
        if i < 15 {
            x <<= 1;
        }
    }}
    c
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
                return Self {
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
            return Self {
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

    pub fn to_f32(self) -> f32 {
        // half's implementation
        // https://github.com/starkat99/half-rs/blob/main/src/binary16/arch.rs

        let i = self.bits;

        // Check for signed zero
        // TODO: Replace mem::transmute with from_bits() once from_bits is const-stabilized
        if i & 0x7FFFu16 == 0 {
            return unsafe { mem::transmute((i as u32) << 16) };
        }

        let half_sign = (i & 0x8000u16) as u32;
        let half_exp = (i & 0x7C00u16) as u32;
        let half_man = (i & 0x03FFu16) as u32;

        // Check for an infinity or NaN when all exponent bits set
        if half_exp == 0x7C00u32 {
            // Check for signed infinity if mantissa is zero
            if half_man == 0 {
                return unsafe { mem::transmute((half_sign << 16) | 0x7F80_0000u32) };
            } else {
                // NaN, keep current mantissa but also set most significiant mantissa bit
                return unsafe {
                    mem::transmute((half_sign << 16) | 0x7FC0_0000u32 | (half_man << 13))
                };
            }
        }

        // Calculate single-precision components with adjusted exponent
        let sign = half_sign << 16;
        // Unbias exponent
        let unbiased_exp = ((half_exp as i32) >> 10) - 15;

        // Check for subnormals, which will be normalized by adjusting exponent
        if half_exp == 0 {
            // Calculate how much to adjust the exponent by
            let e = leading_zeros_u16(half_man as u16) - 6;

            // Rebias and adjust exponent
            let exp = (127 - 15 - e) << 23;
            let man = (half_man << (14 + e)) & 0x7F_FF_FFu32;
            return unsafe { mem::transmute(sign | exp | man) };
        }

        // Rebias exponent for a normalized normal
        let exp = ((unbiased_exp + 127) as u32) << 23;
        let man = (half_man & 0x03FFu32) << 13;

        unsafe { mem::transmute(sign | exp | man) }
    }

    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        Self {
            bits: u16::from_le_bytes(bytes),
        }
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
