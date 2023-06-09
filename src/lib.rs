//! This module contains code that prepares a payload and ID in accordance with the DroneCAN
//! and Cyphal specifications.

//! [Relevant section of DroneCAN specification](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
//! [Cyphal specification](https://opencyphal.org/specification/Cyphal_Specification.pdf)

//! Todo: Protocol version detection using toggle bit polarity.

#![no_std]

use core::sync::atomic::AtomicBool;

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
