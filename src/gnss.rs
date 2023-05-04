//! This module includes code related to the FIX2 Dronecan standard.

use packed_struct::{PackedStruct, prelude::*};
use half::f16;
use crate::PAYLOAD_SIZE_GLOBAL_NAVIGATION_SOLUTION;

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1063.Fix2.uavcan
pub enum GnssTimeStandard {
    None = 0,
    Tai = 1,
    Utc = 2,
    Gps = 3,
}

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See DroneCan ref.
pub enum FixStatus {
    NoFix = 0,
    TimeOnly = 1,
    Fix2d = 2,
    Fix3d = 3,
}

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See DroneCan ref.
pub enum GnssMode {
    Single = 0,
    Dgps = 1,
    Rtk = 2,
    Ppp = 3,
}

#[derive(Clone, Copy, PrimitiveEnum_u8)]
#[repr(u8)]
/// See DroneCan ref.
pub enum GnssSubMode {
    DgpsOtherRtkFloat = 0,
    DgpsSbasRtkFixed = 1,
}

/// Optional subdata for Fix
pub struct EcefPositionVelocity {
    pub velocity_xyz: [f32; 3],
    pub position_xyz_mm: [i64; 3], // [i36; 3]
    // todo: Tail optimization (no len field) since this is the last field?
    pub covariance: [Option<f32>; 36], // todo: [f16; <=36?]
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1063.Fix2.uavcan
/// See `Fix2` data type. Contains the whole packet.
/// See this for field descriptions.
#[derive(PackedStruct)]
#[packed_struct(bit_numbering = "msb0", endian = "lsb")]
pub struct FixDronecan {
    #[packed_field(bytes = "0..7")]
    pub timestamp: u64, // 56 bits
    #[packed_field(bytes = "7..14")]
    pub gnss_timestamp: u64, // 56 bits
    #[packed_field(bits = "112..115", ty = "enum")]
    pub gnss_time_standard: GnssTimeStandard, // 3 bits
    // 13-bit pad
    #[packed_field(bytes = "16..17")]
    pub num_leap_seconds: u8, // 0 for unknown
    #[packed_field(bits = "137..174")]
    pub longitude_deg_1e8: i64, // 37 bits
    #[packed_field(bits = "174..211")]
    pub latitude_deg_1e8: i64, // 37 bits
    #[packed_field(bits = "211..238")]
    pub height_ellipsoid_mm: i32, // 27 bits
    #[packed_field(bits = "238..265")]
    pub height_msl_mm: i32, // 27 bits
    #[packed_field(bits = "265..361")]
    // pub ned_velocity: [f32; 3],
    pub ned_velocity: [u32; 3], // todo: packed_struct currently doesn't support float.
    #[packed_field(bits = "361..367")]
    pub sats_used: u8, // 6 bits.
    #[packed_field(bits = "367..369", ty = "enum")]
    pub fix_status: FixStatus, // 2 bits.
    #[packed_field(bits = "369..373", ty = "enum")]
    pub mode: GnssMode, // 4 bits.
    #[packed_field(bits = "373..379", ty = "enum")]
    pub sub_mode: GnssSubMode, // 6 bits. todo: Why 6 bits?
    /// Note re variable-length arrays in DroneCAN:
    /// "Normally, a dynamic array will be encoded as a sequence of encoded items,
    /// prepended with an unsigned integer field representing the number of contained items
    /// - the length field. The bit width of the length field is a function of the maximum number
    /// of items in the array: ⌈log2(X + 1)⌉, where X is the maximum number of items in the array.
    /// For example, if the maximum number of items is 251, the length field bit width must be 8
    /// bits, or if the maximum number of items is 1, the length field bit width will be just a
    /// single bit.
    ///
    /// For len of 36, we get 5.2. So, 6-bits len field?
    // pub covariance: [Option<f32>; 36], // todo: [f16; <=36?] // todo: Currently unused.
    #[packed_field(bits = "379..385")]
    pub covariance: u8, // This is a single 0 value to indicate we're not using it. 0 is the length.
    #[packed_field(bits = "385..401")]
    // pub pdop: f32, // 16 bits
    pub pdop: u16, // 16 bits  // todo: packed_struct currently doesn't support float.
    // pub ecef_position_velocity: Option<EcefPositionVelocity>, // 0 or 1.  // todo: Currently unused.
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/navigation/2000.GlobalNavigationSolution.uavcan
/// Note: We need packed-struct etc due to `post_covariance` needing a 6-bit len field.
#[derive(PackedStruct)]
#[packed_struct(bit_numbering = "msb0", endian = "lsb")]
pub struct GlobalNavSolution {
    #[packed_field(bytes = "0..7")]
    pub timestamp: u64,
    #[packed_field(bytes = "7..15")]
    pub longitude: f64,
    #[packed_field(bytes = "15..23")]
    pub latitude: f64,
    #[packed_field(bytes = "23..27")]
    pub height_ellipsoid: f32,
    #[packed_field(bytes = "27..31")]
    pub height_msl: f32,
    #[packed_field(bytes = "31..35")]
    pub height_agl: f32,
    #[packed_field(bytes = "35..39")]
    pub height_baro: f32,
    #[packed_field(bytes = "39..41")] // todo: How does this work with options?
    // pub qnh_hpa: Option<f16>,
    pub qnh_hpa: u16, // todo: Currently unused.
    #[packed_field(bytes = "41..57")]
    pub orientation_xyzw: [f32; 4],
    // We just use a 0 for this for the mandatory len; see notes in `Fix2`,
    // and chapter 3 of the dronecan spec.
    #[packed_field(bits = "456..462")]
    pub pose_covariance: u8,
    // (skipping pose covariance)
    #[packed_field(bits = "462..558")]
    pub linear_velocity_body: [f32; 3],
    #[packed_field(bits = "558..654")]
    pub angular_velocity_body: [f32; 3],
    #[packed_field(bits = "654..700")]
    pub linear_acceleration_body: [u16; 3], // f16: Convert prior to using.
    // (skipping velocity covariance)
}
//
// impl GlobalNavSolution {
//     pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_GLOBAL_NAVIGATION_SOLUTION] {
//
//         // todo: Watch out! There are variable-len fields in the middle and end!
//         // "Normally, a dynamic array will be encoded as a sequence of encoded items, prepended with an unsigned integer field representing the number of contained items - the length field. The bit width of the length field is a function of the maximum number of items in the array: ⌈log2(X + 1)⌉, where X is the maximum number of items in the array. For example, if the maximum number of items is 251, the length field bit width must be 8 bits, or if the maximum number of items is 1, the length field bit width will be just a single bit.
//         //
//         // The transport layer provides a data length for every received data transfer (with an 8-bit resolution); thus, in some cases, the array length information would be redundant as it can be inferred from the overall transfer length reported by the transport layer. Elimination of the dynamic array length field is called tail array optimization, and it can be done if all the conditions below are satisfied:
//         //
//         //     The minimum bit length of an item type is not less than 8 bits - because the transport layer reports the transfer length with an 8-bit resolution.
//         //     The array is the last field in the top-level data structure - because, otherwise, a much
//         // more complicated logic would be required to derive the length."
//
//         // We are using a single `0` value as the pose_covariance index, which is a dynamic-len
//         // field in the middle; and we are not using it.
//
//         let mut result = [0; PAYLOAD_SIZE_GLOBAL_NAVIGATION_SOLUTION];
//
//         result[..7].copy_from_slice(&(self.timestamp & 0b111_1111).to_le_bytes());
//         result[7..15].copy_from_slice(&self.longitude.to_le_bytes());
//         result[15..23].copy_from_slice(&self.latitude.to_le_bytes());
//
//         result[23..27].copy_from_slice(&self.height_ellipsoid.to_le_bytes());
//         result[27..31].copy_from_slice(&self.height_msl.to_le_bytes());
//         result[31..35].copy_from_slice(&self.height_agl.to_le_bytes());
//         result[35..39].copy_from_slice(&self.height_baro.to_le_bytes());
//
//         if let Some(q) = self.qnh_hpa {
//             result[39..41].copy_from_slice(&f16::from_f32(q).to_le_bytes());
//         }
//
//         result[41..45].copy_from_slice(&self.orientation_xyzw[0].to_le_bytes());
//         result[45..49].copy_from_slice(&self.orientation_xyzw[1].to_le_bytes());
//         result[49..53].copy_from_slice(&self.orientation_xyzw[2].to_le_bytes());
//         result[53..57].copy_from_slice(&self.orientation_xyzw[3].to_le_bytes());
//
//         // We insert a 0 at index 57 to indicate pose_covariance len is 0.
//
//         result[58..62].copy_from_slice(&self.linear_velocity_body[0].to_le_bytes());
//         result[62..66].copy_from_slice(&self.linear_velocity_body[1].to_le_bytes());
//         result[66..70].copy_from_slice(&self.linear_velocity_body[2].to_le_bytes());
//
//         result[70..74].copy_from_slice(&self.angular_velocity_body[0].to_le_bytes());
//         result[74..78].copy_from_slice(&self.angular_velocity_body[1].to_le_bytes());
//         result[78..82].copy_from_slice(&self.angular_velocity_body[2].to_le_bytes());
//
//         result[82..84].copy_from_slice(&f16::from_f32(self.angular_velocity_body[0]).to_le_bytes());
//         result[84..86].copy_from_slice(&f16::from_f32(self.angular_velocity_body[1]).to_le_bytes());
//         result[86..88].copy_from_slice(&f16::from_f32(self.angular_velocity_body[1]).to_le_bytes());
//
//         result
//     }
// }
