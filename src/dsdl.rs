//! This module contains types associated with specific Dronecan messages.

use bitvec::prelude::*;

#[cfg(feature = "hal")]
use defmt::println;

use crate::{protocol::ConfigCommon, CanError, MsgType, PAYLOAD_SIZE_NODE_STATUS};

// pub const PARAM_NAME_NODE_ID: [u8; 14] = *b"uavcan.node_id";
// pub const PARAM_NAME_NODE_ID: &'static [u8] = "uavcan.node_id".as_bytes();
// pub const PARAM_NAME_BIT_RATE: &'static [u8] = "uavcan.bit_rate".as_bytes();

// todo: QC if you need all lower case.
pub const PARAM_NAME_NODE_ID: &'static [u8] =
    "Node ID (desired if dynamic allocation is set)".as_bytes();
pub const PARAM_NAME_DYNAMIC_ID: &'static [u8] = "Dynamic ID allocation".as_bytes();
pub const PARAM_NAME_FD_MODE: &'static [u8] = "CAN FD enabled".as_bytes();
pub const PARAM_NAME_BITRATE: &'static [u8] = "CAN bitrate (see datasheet)".as_bytes();

// Used to determine which enum (union) variant is used.
// "Tag is 3 bit long, so outer structure has 5-bit prefix to ensure proper alignment"
const VALUE_TAG_BIT_LEN: usize = 3;
const VALUE_NUMERIC_TAG_BIT_LEN: usize = 2;
// For use in `GetSet`
const NAME_LEN_BIT_SIZE: usize = 7; // round_up(log2(92+1));

const MAX_GET_SET_NAME_LEN: usize = 50; // not overal max; max we use to keep buf size down

// Size in bits of the value string's size byte (leading byte)
const VALUE_STRING_LEN_SIZE: usize = 8; // round_up(log2(128+1));

/// Reference: https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum NodeHealth {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Critical = 3,
}

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
    Empty,
    Integer(i64),
    Real(f32),
}

impl Default for NumericValue {
    fn default() -> Self {
        Self::Empty
    }
}

impl NumericValue {
    fn tag(&self) -> u8 {
        match self {
            Self::Empty => 0,
            Self::Integer(_) => 1,
            Self::Real(_) => 2,
        }
    }

    /// Similar to `Value.to_bits()`.
    pub fn to_bits(&self, bits: &mut BitSlice<u8, Msb0>, tag_start_i: usize) -> usize {
        let val_start_i = tag_start_i + VALUE_NUMERIC_TAG_BIT_LEN; // bits
        bits[tag_start_i..val_start_i].store(self.tag());

        match self {
            Self::Empty => val_start_i,
            Self::Integer(v) => {
                bits[val_start_i..val_start_i + 64].store_le(*v);
                val_start_i + 64
            }
            Self::Real(v) => {
                // bitvec doesn't support floats.
                let v_u32 = u32::from_le_bytes(v.to_le_bytes());
                bits[val_start_i..val_start_i + 32].store_le(v_u32);
                val_start_i + 32
            }
        }
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/Value.uavcan
/// `uavcan.protocol.param.Value`
/// 3-bit tag with 5-bit prefix for alignment.
#[derive(Clone, Copy)]
pub enum Value<'a> {
    Empty,
    Integer(i64),
    Real(f32),
    Boolean(bool), // u8 repr
    /// Max length of 128 bytes.
    String(&'a [u8]),
}

impl<'a> Default for Value<'a> {
    fn default() -> Self {
        Self::Empty
    }
}

impl<'a> Value<'a> {
    fn tag(&self) -> u8 {
        match self {
            Self::Empty => 0,
            Self::Integer(_) => 1,
            Self::Real(_) => 2,
            Self::Boolean(_) => 3,
            Self::String(_) => 4,
        }
    }

    /// Modifies a bit array in place, with content from this value.
    /// Returns current bit index.
    pub fn to_bits(&self, bits: &mut BitSlice<u8, Msb0>, tag_start_i: usize) -> usize {
        let val_start_i = tag_start_i + VALUE_TAG_BIT_LEN; // bits
        bits[tag_start_i..val_start_i].store(self.tag());

        // success value of pdc msg for node id w min and max
        // [78, 111, 100, 101, 32, 73, 68, 67]
        // or is it this: [31, 15, 1, 70, 0, 0, 0, 131, 0]

        match self {
            Self::Empty => val_start_i,
            Self::Integer(v) => {
                bits[val_start_i..val_start_i + 64].store_le(*v);
                val_start_i + 64
            }
            Self::Real(v) => {
                // bitvec doesn't support floats.
                let v_u32 = u32::from_le_bytes(v.to_le_bytes());
                bits[val_start_i..val_start_i + 32].store_le(v_u32);
                val_start_i + 32
            }
            Self::Boolean(v) => {
                bits[val_start_i..val_start_i + 8].store_le(*v as u8);
                val_start_i + 8
            }
            Self::String(v) => {
                let mut i = val_start_i;
                bits[i..VALUE_STRING_LEN_SIZE].store_le(v.len());
                i += VALUE_STRING_LEN_SIZE;
                for char in *v {
                    bits[i..i + 8].store_le(*char);
                    i += 8;
                }
                i
            }
        }
    }

    /// Converts from a bit array, eg one of a larger message. Anchors using bit indexes
    /// passed as arguments.
    /// Returns self, and current bit index.
    pub fn from_bits(
        bits: &BitSlice<u8, Msb0>,
        tag_start_i: usize,
    ) -> Result<(Self, usize), CanError> {
        let val_start_i = tag_start_i + VALUE_TAG_BIT_LEN;

        Ok(match bits[tag_start_i..val_start_i].load_le::<u8>() {
            0 => (Self::Empty, val_start_i),
            1 => (
                Self::Integer(bits[val_start_i..val_start_i + 64].load_le::<i64>()),
                val_start_i + 64,
            ),
            2 => {
                // No support for floats in bitvec.
                let as_u32 = bits[val_start_i..val_start_i + 32].load_le::<u32>();
                (
                    Self::Real(f32::from_le_bytes(as_u32.to_le_bytes())),
                    val_start_i + 32,
                )
            }
            3 => (
                Self::Boolean(bits[val_start_i..val_start_i + 8].load_le::<u8>() != 0),
                val_start_i + 8,
            ),
            4 => {
                // todo: Handle non-FD mode that uses TCO
                let current_i = val_start_i + VALUE_STRING_LEN_SIZE;
                let str_len: u8 = bits[val_start_i..current_i].load_le();

                // todo: WTH?
                (Self::Integer(69), val_start_i + 64)
                // unimplemented!()
                // todo: Need to convert bitslice to byte slice.
                // (Self::String(bits[current_i..current_i + str_len as usize * 8]), current_i)
            }
            _ => return Err(CanError::PayloadData),
        })
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
pub struct GetSetRequest<'a> {
    pub index: u16, // 13 bits
    /// If set - parameter will be assigned this value, then the new value will be returned.
    /// If not set - current parameter value will be returned.
    pub value: Value<'a>,
    // pub name: &'a [u8], // up to 92 bytes.
    /// Name of the parameter; always preferred over index if nonempty.
    pub name: [u8; MAX_GET_SET_NAME_LEN], // large enough for many uses
    pub name_len: usize,
}

impl<'a> GetSetRequest<'a> {
    // pub fn to_bytes(buf: &mut [u8]) -> Self {
    //     let index
    //     Self {
    //         index,
    //         value,
    //         name,
    //     }
    // }

    pub fn from_bytes(buf: &[u8], fd_mode: bool) -> Result<Self, CanError> {
        let bits = buf.view_bits::<Msb0>();

        let tag_start_i = 13;
        let index: u16 = bits[0..tag_start_i].load_le();

        // `i` in this function is in bits, not bytes.
        let (value, mut current_i) = Value::from_bits(bits, tag_start_i)?;

        let name_len = if fd_mode {
            let v = bits[current_i..current_i + NAME_LEN_BIT_SIZE].load_le::<u8>() as usize;
            current_i += NAME_LEN_BIT_SIZE;
            v
        } else {
            // todo: Figure it out from message len, or infer from 0s.
            MAX_GET_SET_NAME_LEN // max name len we use
        };

        // println!("Name len: {:?}", name_len);

        let mut name = [0; MAX_GET_SET_NAME_LEN];

        if name_len as usize > name.len() {
            return Err(CanError::PayloadData);
        }

        for char_i in 0..name_len {
            name[char_i] = bits[current_i..current_i + 8].load_le::<u8>();
            current_i += 8;
        }

        Ok(Self {
            index,
            value,
            name,
            name_len,
        })
    }
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
pub struct GetSetResponse<'a> {
    /// For set requests, it should contain the actual parameter value after the set request was
    /// executed. The objective is to let the client know if the value could not be updated, e.g.
    /// due to its range violation, etc.
    pub value: Value<'a>,
    pub default_value: Value<'a>,
    pub max_value: NumericValue,
    pub min_value: NumericValue,
    /// Empty name (and/or empty value) in response indicates that there is no such parameter.
    // pub name: [u8; MAX_GET_SET_NAME_LEN], // large enough for many uses
    pub name: [u8; MAX_GET_SET_NAME_LEN], // large enough for many uses
    pub name_len: usize,
    // pub name: &'a [u8], // up to 92 bytes.
}

impl<'a> GetSetResponse<'a> {
    /// Returns array length in bytes.
    pub fn to_bytes(&self, buf: &mut [u8], fd_mode: bool) -> usize {
        let bits = buf.view_bits_mut::<Msb0>();

        let val_tag_start_i = 5; // bits.

        let current_i = self.value.to_bits(bits, val_tag_start_i);

        // 5 is the pad between `value` and `default_value`.
        let default_value_i = current_i + 5;

        let current_i = self.default_value.to_bits(bits, default_value_i);
        let max_value_i = current_i + 6;

        let current_i = self.max_value.to_bits(bits, max_value_i);
        let min_value_i = current_i + 6;

        let current_i = self.min_value.to_bits(bits, min_value_i);

        // In FD mode, we need the len field of name.
        let mut i_bit = if fd_mode {
            let mut i_bit = current_i; // bits.

            bits[i_bit..i_bit + NAME_LEN_BIT_SIZE].store_le(self.name_len);
            i_bit += NAME_LEN_BIT_SIZE;

            i_bit
        } else {
            current_i
        };

        for char in self.name {
            bits[i_bit..i_bit + 8].store_le(char);
            i_bit += 8;
        }

        crate::bit_size_to_byte_size(i_bit)
    }

    pub fn from_bytes(buf: &[u8]) -> Result<Self, CanError> {
        let bits = buf.view_bits::<Msb0>();

        return unimplemented!(); // todo: You just need to work thorugh it like with related.
                                 //
                                 // let val_tag_start_i = 5;
                                 // let (value, current_i) = Value::from_bits(bits, val_tag_start_i, &mut [])?; // todo: t str buf
                                 //
                                 //
                                 // // todo: Max, min and default values
                                 // let default_value = Value::Empty;
                                 //
                                 // let max_value_i = default_value_i + VALUE_TAG_BIT_LEN + 6; // Includes pad.
                                 //
                                 // let max_value = NumericValue::Empty;
                                 // let max_value_size = 0; // todo
                                 //
                                 // let min_value = NumericValue::Empty;
                                 // let min_value_size = 0; // todo
                                 // // 2 is tag size of numeric value.
                                 // let min_value_i = max_value_i + 2 + max_value_size + 6;
                                 //
                                 // // todo: Update once you include default values.
                                 // let name_len_i = min_value_i + 2 + min_value_size + 6;
                                 //
                                 // // todo: Name section here is DRY with request.
                                 // let name_start_i = name_len_i + NAME_LEN_BIT_SIZE;
                                 //
                                 // let name_len = bits[name_len_i..name_start_i].load_le::<u8>() as usize;
                                 //
                                 // let mut name = [0; MAX_GET_SET_NAME_LEN];
                                 //
                                 // let mut i = name_start_i; // bits.
                                 //
                                 // i += VALUE_STRING_LEN_SIZE;
                                 //
                                 // if name_len as usize > name.len() {
                                 //     return Err(CanError::PayloadData);
                                 // }
                                 //
                                 // for char_i in 0..name_len {
                                 //     name[char_i] = bits[i..i + 8].load_le::<u8>();
                                 //     i += 8;
                                 // }
                                 //
                                 // Ok(Self {
                                 //     value,
                                 //     default_value,
                                 //     max_value,
                                 //     min_value,
                                 //     name,
                                 //     name_len,
                                 // })
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
    // We currently don't use certificate of authority.
    // /// Certificate of authenticity (COA) of the hardware, 255 bytes max.
    // pub certificate_of_authority: &'a [u8],
}

impl HardwareVersion {
    pub fn to_bytes(&self) -> [u8; 19] {
        let mut buf = [0; 19];

        buf[0] = self.major;
        buf[1] = self.minor;
        buf[2..18].clone_from_slice(&self.unique_id);
        // The final index is our 8-bit length field for COA, which we hard-set to 0.
        buf[18] = 0;
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

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/dynamic_node_id/1.Allocation.uavcan
pub struct IdAllocationData {
    pub node_id: u8, // 7 bytes
    pub stage: u8,   // 0, 1, or 3.
    pub unique_id: [u8; 16],
}

impl IdAllocationData {
    pub fn to_bytes(&self, fd_mode: bool) -> [u8; MsgType::IdAllocation.payload_size() as usize] {
        let mut result = [0; MsgType::IdAllocation.payload_size() as usize];

        result[0] = (self.node_id << 1) | ((self.stage == 0) as u8);

        // unique id. Split across payloads if not on FD mode.
        if fd_mode {
            // Must include the 5-bit unique_id len field in FD mode.
            let bits = result.view_bits_mut::<Msb0>();

            let mut i_bit = 1 * 8;
            bits[i_bit..i_bit + 5].store_le(16_u8);

            i_bit += 5;

            for val in self.unique_id {
                bits[i_bit..i_bit + 8].store_le(val);
                i_bit += 8;
            }
        } else {
            match self.stage {
                0 => {
                    result[1..7].copy_from_slice(&self.unique_id[0..6]);
                }
                1 => {
                    result[1..7].copy_from_slice(&self.unique_id[6..12]);
                }
                2 => {
                    result[1..5].copy_from_slice(&self.unique_id[12..16]);
                }
                _ => (),
            };
        }

        result
    }

    pub fn from_bytes(buf: &[u8; MsgType::IdAllocation.payload_size() as usize]) -> Self {
        let stage = if (buf[0] & 1) != 0 { 1 } else { 0 };

        Self {
            // todo: QC order
            node_id: (buf[0] << 1) & 0b111_1111,
            stage,
            unique_id: buf[1..17].try_into().unwrap(),
        }
    }
}

/// Make a GetSet response from common config items. This reduces repetition in node firmware.
pub fn make_getset_response_common<'a>(
    config: &ConfigCommon,
    index: u8,
) -> Option<GetSetResponse<'a>> {
    // We load the default config to determine default values.
    let cfg_default = ConfigCommon::default();

    let mut name = [0; MAX_GET_SET_NAME_LEN];

    match index {
        0 => {
            let text = PARAM_NAME_NODE_ID;
            name[0..text.len()].copy_from_slice(text);

            Some(GetSetResponse {
                value: Value::Integer(config.node_id as i64),
                default_value: Value::Integer(cfg_default.node_id as i64),
                max_value: NumericValue::Integer(127),
                min_value: NumericValue::Integer(0),
                name,
                name_len: text.len(),
            })
        }
        1 => {
            let text = PARAM_NAME_DYNAMIC_ID;
            name[0..text.len()].copy_from_slice(text);

            Some(GetSetResponse {
                value: Value::Boolean(config.dynamic_id_allocation),
                default_value: Value::Boolean(cfg_default.dynamic_id_allocation),
                max_value: NumericValue::Empty,
                min_value: NumericValue::Empty,
                name,
                name_len: text.len(),
            })
        }
        2 => {
            let text = PARAM_NAME_FD_MODE;
            name[0..text.len()].copy_from_slice(text);

            Some(GetSetResponse {
                value: Value::Boolean(config.fd_mode),
                default_value: Value::Boolean(cfg_default.fd_mode),
                max_value: NumericValue::Empty,
                min_value: NumericValue::Empty,
                name,
                name_len: text.len(),
            })
        }
        3 => {
            let text = PARAM_NAME_BITRATE;
            name[0..text.len()].copy_from_slice(text);

            Some(GetSetResponse {
                value: Value::Integer(config.can_bitrate as i64),
                default_value: Value::Integer(cfg_default.can_bitrate as i64),
                max_value: NumericValue::Integer(6),
                min_value: NumericValue::Integer(0),
                name,
                name_len: text.len(),
            })
        }
        _ => None,
    }
}

#[derive(Default)]
/// https://www.expresslrs.org/3.0/info/signal-health/
/// Currently AnyLeaf only.
pub struct LinkStats {
    // /// Timestamp these stats were recorded. (TBD format; processed locally; not part of packet from tx).
    // pub timestamp: u32,
    /// Uplink - received signal strength antenna 1 (RSSI). RSSI dBm as reported by the RX. Values
    /// vary depending on mode, antenna quality, output power and distance. Ranges from -128 to 0.
    pub uplink_rssi_1: u8,
    /// Uplink - received signal strength antenna 2 (RSSI). Second antenna RSSI, used in diversity mode
    /// (Same range as rssi_1)
    pub uplink_rssi_2: u8,
    /// Uplink - link quality (valid packets). The number of successful packets out of the last
    /// 100 from TX → RX
    pub uplink_link_quality: u8,
    /// Uplink - signal-to-noise ratio. SNR reported by the RX. Value varies mostly by radio chip
    /// and gets lower with distance (once the agc hits its limit)
    pub uplink_snr: i8,
    /// Active antenna for diversity RX (0 - 1)
    pub active_antenna: u8,
    pub rf_mode: u8,
    /// Uplink - transmitting power. See the `ElrsTxPower` enum and its docs for details.
    pub uplink_tx_power: u8,
    /// Downlink - received signal strength (RSSI). RSSI dBm of telemetry packets received by TX.
    pub downlink_rssi: u8,
    /// Downlink - link quality (valid packets). An LQ indicator of telemetry packets received RX → TX
    /// (0 - 100)
    pub downlink_link_quality: u8,
    /// Downlink - signal-to-noise ratio. SNR reported by the TX for telemetry packets
    pub downlink_snr: i8,
}

impl LinkStats {
    pub fn to_bytes(&self) -> [u8; MsgType::LinkStats.payload_size() as usize] {
        let mut result = [0; MsgType::LinkStats.payload_size() as usize];

        result[0] = self.uplink_rssi_1;
        result[1] = self.uplink_rssi_2;
        result[2] = self.uplink_link_quality;
        result[3] = self.uplink_snr as u8;
        result[4] = self.active_antenna;
        result[5] = self.rf_mode;
        result[6] = self.uplink_tx_power;
        result[7] = self.downlink_rssi;
        result[8] = self.downlink_link_quality;
        result[9] = self.downlink_snr as u8;

        result
    }
}
