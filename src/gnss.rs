//! This module includes code related to the FIX2 Dronecan standard.

use packed_struct::{prelude::*, PackedStruct};

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

https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1063.Fix2.uavcan
/// See `Fix2` data type. Contains the whole packet.
/// See this for field descriptions.
#[derive(PackedStruct)]
#[packed_struct(bit_numbering = "msb0", endian = "lsb")]
pub struct FixDronecan {
    #[packed_field(bits = "0..56")]
    pub timestamp: u64, // 56 bits
    #[packed_field(bits = "56..112")]
    pub gnss_timestamp: u64, // 56 bits
    #[packed_field(bits = "112..115", ty = "enum")]
    pub gnss_time_standard: GnssTimeStandard, // 3 bits
    // 13-bit pad
    #[packed_field(bits = "128..136")]
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
    /// For len of 36, we get 5.2. So, 6-bits len field, or 5?
    // pub covariance: [Option<f32>; 36], // todo: [f16; <=36?] // todo: Currently unused.
    #[packed_field(bits = "379..395")]
    // pub pdop: f32, // 16 bits
    pub pdop: u16, // 16 bits  // todo: packed_struct currently doesn't support float.
                   // pub ecef_position_velocity: Option<EcefPositionVelocity>, // 0 or 1.  // todo: Currently unused.
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/navigation/2000.GlobalNavigationSolution.uavcan
#[derive(PackedStruct)]
#[packed_struct(bit_numbering = "msb0", endian = "lsb")]
pub struct GlobalNavigationSoln {
    #[packed_field(bits = "0..56")]
    pub timestamp: u64, // 56 bits
    #[packed_field(bits = "56..112")]
    pub gnss_timestamp: u64, // 56 bits
    #[packed_field(bits = "112..115", ty = "enum")]
    pub gnss_time_standard: GnssTimeStandard, // 3 bits
    // 13-bit pad
    #[packed_field(bits = "128..136")]
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
    /// For len of 36, we get 5.2. So, 6-bits len field, or 5?
    // pub covariance: [Option<f32>; 36], // todo: [f16; <=36?] // todo: Currently unused.
    #[packed_field(bits = "379..395")]
    // pub pdop: f32, // 16 bits
    pub pdop: u16, // 16 bits  // todo: packed_struct currently doesn't support float.
                   // pub ecef_position_velocity: Option<EcefPositionVelocity>, // 0 or 1.  // todo: Currently unused.
}
