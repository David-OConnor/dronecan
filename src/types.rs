//! This module contains types associated with Dronecan messages.

use bitvec::prelude::*;

use crate::{CanError, PAYLOAD_SIZE_NODE_STATUS};

// pub const PARAM_NAME_NODE_ID: [u8; 14] = *b"uavcan.node_id";
pub const PARAM_NAME_NODE_ID: &'static [u8] = "uavcan.node_id".as_bytes();
pub const PARAM_NAME_BIT_RATE: &'static [u8] = "uavcan.bit_rate".as_bytes();

// Must be postfixed with full data type name, eg `uavcan.pubp-uavcan.protocol.NodeStatus`
pub const PARAM_NAME_PUBLICATION_PERIOD: &'static str = "uavcan.pubp-";

/// Reference: https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum NodeMode {
    Operational = 0,
    Initialization = 1,
    Maintenance = 2,
    SoftwareUpdate = 3,
    Offline = 7,
}

/// Broadcast periodically, and sent as part of the Node Status message.
pub struct NodeStatus {
    pub uptime_sec: u32,
    pub health: NodeHealth,
    pub mode: NodeMode,
    pub vendor_specific_status_code: u16,
}

impl NodeStatus {
    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_NODE_STATUS] {
        let mut result = [0; PAYLOAD_SIZE_NODE_STATUS];

        result[..4].clone_from_slice(&self.uptime_sec.to_le_bytes());

        // Health and mode. Submode is reserved by the spec for future use,
        // but is currently not used.
        result[4] = ((self.health as u8) << 6) | ((self.mode as u8) << 3);

        result[5..7].clone_from_slice(&self.vendor_specific_status_code.to_le_bytes());

        result
    }
}

/// Reference: https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum NodeHealth {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Critical = 3,
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/10.ExecuteOpcode.uavcan
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum OpcodeType {
    Save = 0,
    Erase = 1,
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/10.ExecuteOpcode.uavcan
/// Pertains to saving or erasing config to/from flash
pub struct ExecuteOpcode {
    pub opcode: OpcodeType,
    pub error_code: Option<i64>, // 48 bits.
    pub ok: bool,
}

impl ExecuteOpcode {
    pub fn from_bytes(buf: &[u8]) -> Self {
        let opcode = if buf[0] == 1 {
            OpcodeType::Erase
        } else {
            OpcodeType::Save
        };

        let error_code_val = i64::from_le_bytes(buf[56..104].try_into().unwrap());
        let error_code = match error_code_val {
            0 => None,
            _ => Some(error_code_val),
        };

        Self {
            opcode,
            error_code,
            ok: buf[104] != 0,
        }
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/NumericValue.uavcan
/// `uavcan.protocol.param.NumericValue`
/// 2-bit tag.
#[derive(Clone, Copy)]
pub enum NumericValue {
    Integer(i64),
    Real(f32),
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/Value.uavcan
/// `uavcan.protocol.param.Value`
/// 3-bit tag with 5-bit prefix for alignment.
#[derive(Clone, Copy)]
pub enum Value<'a> {
    Integer(i64),
    Real(f32),
    Boolean(bool), // u8 repr
    /// Max length of 128 bytes.
    String(&'a [u8]),
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
pub struct GetSet<'a> {
    pub index: u16, // 13 bits
    /// If set - parameter will be assigned this value, then the new value will be returned.
    /// If not set - current parameter value will be returned.
    pub value1: Value<'a>,
    // pub name: &'a [u8], // up to 92 bytes.
    pub name: [u8; 30], // large enough for many uses
    /// For set requests, it should contain the actual parameter value after the set request was
    /// executed. The objective is to let the client know if the value could not be updated, e.g.
    /// due to its range violation, etc.
    /// Empty value (and/or empty name) indicates that there is no such parameter.
    pub value2: Value<'a>,
    pub default_value: Option<Value<'a>>,
    pub max_value: Option<NumericValue>,
    pub min_value: Option<NumericValue>,
}

impl<'a> GetSet<'a> {
    // pub fn to_bytes(buf: &[u8]) -> Self {
    //     let index
    //     Self {
    //         index,
    //         value,
    //         name,
    //         value2,
    //         default_value,
    //         max_value,
    //         min_value,
    //     }
    // }

    pub fn from_bytes(buf: &[u8]) -> Result<Self, CanError> {
        const NAME_LEN_BIT_SIZE: usize = 7; // round_up(log2(92+1));
        let bits = buf.view_bits::<Lsb0>();

        let index = bits[0..13].load_le::<u16>();

        let value_start_i = 13;

        let value1 = match bits[value_start_i..value_start_i + 8].load_le::<u8>() {
            0 => Value::Integer(bits[value_start_i + 8..value_start_i + 8 + 64].load_le::<i64>()),
            1 => {
                // No support for floats?
                let as_u32 = bits[value_start_i + 8..value_start_i + 8 + 32].load_le::<u32>();
                Value::Real(f32::from_le_bytes(as_u32.to_le_bytes()))
            }
            2 => {
                Value::Boolean(bits[value_start_i + 8..value_start_i + 8 + 8].load_le::<u8>() != 0)
            }
            // todo: Impl string.
            // 3 => Value::String(bits[21..85].load_le::<i64>()),
            _ => return Err(CanError::PayloadData),
        };

        let name_len_i = match value1 {
            Value::Integer(_) => value_start_i + 8 + 64,
            Value::Real(_) => value_start_i + 8 + 32,
            Value::Boolean(_) => value_start_i + 8 + 8,
            Value::String(_) => 0, // todo
        };

        let name_len = bits[name_len_i..name_len_i + NAME_LEN_BIT_SIZE].load_le::<u8>() as usize;

        let name_start_i = name_len_i + NAME_LEN_BIT_SIZE;

        // todo: COnvert name to a byte array or str.
        // let name = bits[name_start_i..name_start_i + name_len];
        let mut name = [0; 30]; // todo!

        // let name = bits[name_start_i..name_start_i + name_len].load_le::<[u8; 30]>();

        // Includes a `void5` spacer.
        let value2_start_i = name_start_i + name_len + 5;

        // todo: DRY. Put in a sep fn etc.
        let value2 = match bits[value2_start_i..value2_start_i + 8].load_le::<u8>() {
            0 => Value::Integer(bits[value2_start_i + 8..value2_start_i + 8 + 64].load_le::<i64>()),
            1 => {
                // No support for floats?
                let as_u32 = bits[value2_start_i + 8..value2_start_i + 8 + 32].load_le::<u32>();
                Value::Real(f32::from_le_bytes(as_u32.to_le_bytes()))
            }
            2 => Value::Boolean(
                bits[value2_start_i + 8..value2_start_i + 8 + 8].load_le::<u8>() != 0,
            ),
            // todo: Impl string.
            // 3 => Value::String(bits[21..85].load_le::<i64>()),
            _ => return Err(CanError::PayloadData),
        };

        // todo: Max, min and default values
        let default_value = None;
        let max_value = None;
        let min_value = None;

        Ok(Self {
            index,
            value1,
            name,
            value2,
            default_value,
            max_value,
            min_value,
        })
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/HardwareVersion.uavcan
/// Generic hardware version information.
/// These values should remain unchanged for the device's lifetime.
// pub struct HardwareVersion<'a> {
pub struct HardwareVersion {
    pub major: u8,
    pub minor: u8,
    /// Unique ID is a 128 bit long sequence that is globally unique for each node.
    /// All zeros is not a valid UID.
    /// If filled with zeros, assume that the value is undefined.
    pub unique_id: [u8; 16],
    // /// Certificate of authenticity (COA) of the hardware, 255 bytes max.
    // pub certificate_of_authority: &'a [u8],
    // pub certificate_of_authority: u8, // todo: Hardcoded as 1 byte.
}

impl HardwareVersion {
    pub fn to_bytes(&self) -> [u8; 18] {
        let mut buf = [0; 18];

        buf[0] = self.major;
        buf[1] = self.minor;
        buf[2..18].clone_from_slice(&self.unique_id);
        // buf[18..self.certificate_of_authority.len() + 18]
        //     .clone_frosm_slice(self.certificate_of_authority);
        // buf[18] = self.certificate_of_authority;

        buf
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/SoftwareVersion.uavcan
/// Generic software version information.
pub struct SoftwareVersion {
    pub major: u8,
    pub minor: u8,
    ///This mask indicates which optional fields (see below) are set.
    /// uint8 OPTIONAL_FIELD_FLAG_VCS_COMMIT = 1
    ///uint8 OPTIONAL_FIELD_FLAG_IMAGE_CRC  = 2
    pub optional_field_flags: u8,
    /// VCS commit hash or revision number, e.g. git short commit hash. Optional.
    pub vcs_commit: u32,
    /// The value of an arbitrary hash function applied to the firmware image.
    pub image_crc: u64,
}

impl SoftwareVersion {
    pub fn to_bytes(&self) -> [u8; 15] {
        let mut result = [0; 15];

        result[0] = self.major;
        result[1] = self.minor;
        result[2] = self.optional_field_flags;
        result[3..7].clone_from_slice(&self.vcs_commit.to_le_bytes());
        result[7..15].clone_from_slice(&self.image_crc.to_le_bytes());

        result
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/DataTypeKind.uavcan
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum DataTypeKind {
    Service = 0,
    Message = 1,
}
