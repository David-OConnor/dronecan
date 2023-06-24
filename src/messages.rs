//! This module contains standard messages all nodes must
//! or can implement. It's described in the [DSDL repo, protcols page]
//! (https://github.com/dronecan/DSDL/tree/master/uavcan/protocol)

use crate::{MsgPriority, PAYLOAD_SIZE_CONFIG_COMMON, PAYLOAD_SIZE_NODE_STATUS};

#[cfg(feature = "hal")]
use defmt::println;

// #[repr(u16)]
#[derive(Clone, Copy, PartialEq)]
pub enum MsgType {
    IdAllocation,
    GetNodeInfo,
    GlobalTimeSync,
    TransportStats,
    Panic,
    Restart,
    ExecuteOpcode,
    GetSet,
    NodeStatus,
    AhrsSolution,
    MagneticFieldStrength2,
    RawImu,
    RawAirData,
    StaticPressure,
    StaticTemperature,
    GnssAux,
    Fix2,
    GlobalNavigationSolution,
    RcInput, // WIP
    // ChData,    // AnyLeaf custom for now
    LinkStats, // AnyLeaf custom for now.
    ArdupilotGnssStatus,
    SetConfig, // Anyleaf custom for now.
    // Custom types here we use on multiple projects, but aren't (yet?) part of the DC spec.
    ConfigGnssGet,
    ConfigGnss,
    ConfigRxGet,
    ConfigRx,
    PositFusedAnyleaf,
}

impl MsgType {
    /// Get the data type id.
    pub const fn id(&self) -> u16 {
        match self {
            Self::IdAllocation => 1,
            Self::GetNodeInfo => 1,
            Self::GlobalTimeSync => 4,
            Self::TransportStats => 4, // todo: Duplicate id?
            Self::Panic => 5,
            Self::Restart => 5,
            Self::ExecuteOpcode => 10,
            Self::GetSet => 11,
            Self::NodeStatus => 341,
            Self::AhrsSolution => 1_000,
            Self::MagneticFieldStrength2 => 1_002,
            Self::RawImu => 1_003,
            Self::RawAirData => 1_027,
            Self::StaticPressure => 1_028,
            Self::StaticTemperature => 1_029,
            Self::GnssAux => 1_061,
            Self::Fix2 => 1_063,
            Self::GlobalNavigationSolution => 2_000,
            Self::RcInput => 1_140,
            // Self::ChData => 3_103,    // AnyLeaf custom for now
            Self::LinkStats => 1_141, // AnyLeaf custom for now.
            Self::ArdupilotGnssStatus => 20_003,
            Self::SetConfig => 3_105, // Anyleaf custom for now.
            // Custom types here we use on multiple projects, but aren't (yet?) part of the DC spec.
            Self::ConfigGnssGet => 3_110,
            Self::ConfigGnss => 3_111,
            Self::ConfigRxGet => 3_112,
            Self::ConfigRx => 3_113,
            Self::PositFusedAnyleaf => 3_115,
            // todo: Anyleaf config sizes?
        }
    }

    pub fn priority(&self) -> MsgPriority {
        match self {
            Self::RawImu => MsgPriority::Immediate,
            Self::AhrsSolution => MsgPriority::Immediate,
            Self::RcInput => MsgPriority::Immediate,
            Self::RawAirData => MsgPriority::Fast,
            Self::MagneticFieldStrength2 => MsgPriority::Fast,
            Self::GlobalNavigationSolution => MsgPriority::High,
            Self::StaticPressure => MsgPriority::Nominal,
            Self::Fix2 => MsgPriority::Nominal,
            Self::LinkStats => MsgPriority::Low,
            Self::PositFusedAnyleaf => MsgPriority::Nominal,
            _ => MsgPriority::Slow,
        }
    }

    /// Get the payload size. Does not include padding for a tail byte.
    pub const fn payload_size(&self) -> u8 {
        // todo: 0 values are ones we haven't implemented yet.
        // todo: Handle when response has a diff payload size!
        match self {
            Self::IdAllocation => 19, // Includes 16 bytes of unique id. (Needs a len field for FD mode)
            // This includes no name; add name len to it after. Assumes no hardware certificate of authority.
            Self::GetNodeInfo => 41,
            Self::GlobalTimeSync => 7,
            Self::TransportStats => 18,
            Self::Panic => 7, // todo?
            Self::Restart => 5,
            Self::ExecuteOpcode => 0,
            // We override GetSet's size based on the specific payload
            Self::GetSet => 0,
            Self::NodeStatus => PAYLOAD_SIZE_NODE_STATUS as u8,
            // AHRS solution: Assumes no covariance values. The first 2 cov fields have a 4-bit
            // lenght; the last doesn't use one. Includes the void fields.
            Self::AhrsSolution => 23,
            Self::MagneticFieldStrength2 => 7,
            Self::RawImu => 47,
            Self::RawAirData => 0,
            Self::StaticPressure => 6,
            Self::StaticTemperature => 4,
            Self::GnssAux => 16,
            Self::Fix2 => 62, // 50 without covariance, plus 12 with.
            // This assumes we are not using either dynamic-len fields `pose_covariance` or `velocity_covariance`.
            Self::GlobalNavigationSolution => 88,
            // This is the rssi, status, and id items; add 12 bits for every channel
            Self::RcInput => 4,
            // Self::ChData => 38,
            Self::LinkStats => 10,
            Self::ArdupilotGnssStatus => 7, // Should be 8 from DSDL, but 7 seems to work.
            Self::SetConfig => 20,          // todo
            Self::ConfigGnssGet => 0,
            Self::ConfigGnss => PAYLOAD_SIZE_CONFIG_COMMON as u8 + 8 + 4 * 15, // 4*15 = size of cal data.
            Self::ConfigRxGet => 0,
            Self::ConfigRx => PAYLOAD_SIZE_CONFIG_COMMON as u8 + 2,
            Self::PositFusedAnyleaf => 36,
        }
    }

    /// Includes payload size, padded for a tail byte.
    pub const fn buf_size(&self) -> usize {
        crate::find_tail_byte_index(self.payload_size()) + 1
    }

    /// .base_crc in Pydronecan
    /// def get_info(a, b):
    ///     val = dronecan.DATATYPES[(a, b)]
    ///     print(val)
    ///     print(val.base_crc)
    pub fn base_crc(&self) -> u16 {
        match self {
            Self::IdAllocation => 62_040,
            Self::GetNodeInfo => 55_719,
            Self::GlobalTimeSync => 30_984,
            Self::TransportStats => 19_827,
            Self::Panic => 64_606,
            Self::Restart => 8_063,
            Self::ExecuteOpcode => 55_252,
            Self::GetSet => 64_272,
            Self::NodeStatus => 48_735,
            Self::AhrsSolution => 8_277,
            Self::MagneticFieldStrength2 => 47653,
            Self::RawImu => 20_030,
            Self::RawAirData => 20_590,
            Self::StaticPressure => 60_404,
            Self::StaticTemperature => 6_052,
            Self::GnssAux => 9_390,
            Self::Fix2 => 51_096,
            Self::GlobalNavigationSolution => 7_536,
            Self::RcInput => 22_801, // todo: Update this once ID field PR is merged
            // Self::ChData => 0,
            Self::LinkStats => 0,
            Self::ArdupilotGnssStatus => 47_609,
            Self::SetConfig => 0,
            Self::ConfigGnssGet => 0,
            Self::ConfigGnss => 0,
            Self::ConfigRxGet => 0,
            Self::ConfigRx => 0,
            Self::PositFusedAnyleaf => 0,
        }
    }
}
