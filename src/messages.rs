//! This module contains standard messages all nodes must
//! or can implement. It's described in the [DSDL repo, protcols page]
//! (https://github.com/dronecan/DSDL/tree/master/uavcan/protocol)

use core::sync::atomic::{self, AtomicUsize, Ordering};

use packed_struct::PackedStruct;

use crate::{
    broadcast, get_tail_byte,
    gnss::{GlobalNavSolution, GnssAuxiliary},
    messages::{self},
    types::{
        self, GetSetResponse, HardwareVersion, NodeHealth, NodeMode, NodeStatus, NumericValue,
        SoftwareVersion, Value, PARAM_NAME_NODE_ID,
    },
    Can, CanError, FrameType, MsgPriority, RequestResponse, ServiceData,
};

use half::f16;

use cortex_m;

// #[repr(u16)]
#[derive(Clone, Copy)]
pub enum MsgType {
    GetNodeInfo,
    NodeInfo,
    GlobalTimeSync,
    TransportStats,
    Panic,
    Restart,
    RestartResp,
    ExecuteOpcode,
    GetSet,
    NodeStatus,
    MagneticFieldStrength2,
    RawImu,
    RawAirData,
    StaticPressure,
    StaticTemperature,
    GnssAux,
    Fix2,
    GlobalNavigationSolution,
    ChData,    // AnyLeaf custom for now
    LinkStats, // AnyLeaf custom for now.
    SetConfig, // Anyleaf custom for now.
    // Custom types here we use on multiple projects, but aren't (yet?) part of the DC spec.
    Ack, // AnyLeaf custom for now.
    ConfigGnssGet,
    ConfigGnss,
    ConfigRxGet,
    ConfigRx,
}

impl MsgType {
    /// Get the data type id.
    pub const fn id(&self) -> u16 {
        match self {
            Self::GetNodeInfo => 1,
            Self::NodeInfo => 1,
            Self::GlobalTimeSync => 4,
            Self::TransportStats => 4, // todo: Duplicate id?
            Self::Panic => 5,
            Self::Restart => 5,     // todo: Duplicate id?
            Self::RestartResp => 5, // todo: Duplicate id?
            Self::ExecuteOpcode => 10,
            Self::GetSet => 11,
            Self::NodeStatus => 341,
            Self::MagneticFieldStrength2 => 1_002,
            Self::RawImu => 1_003,
            Self::RawAirData => 1_027,
            Self::StaticPressure => 1_028,
            Self::StaticTemperature => 1_029,
            Self::GnssAux => 1_061,
            Self::Fix2 => 1_063,
            Self::GlobalNavigationSolution => 2_000,
            Self::ChData => 3_103,    // AnyLeaf custom for now
            Self::LinkStats => 3_104, // AnyLeaf custom for now.
            Self::SetConfig => 3_105, // Anyleaf custom for now.
            // Custom types here we use on multiple projects, but aren't (yet?) part of the DC spec.
            Self::Ack => 3_106, // AnyLeaf custom for now.
            Self::ConfigGnssGet => 3_110,
            Self::ConfigGnss => 3_111,
            Self::ConfigRxGet => 3_112,
            Self::ConfigRx => 3_113,
            // todo: Anyleaf config sizes?
        }
    }

    pub fn priority(&self) -> MsgPriority {
        match self {
            Self::RawImu => MsgPriority::Immediate,
            Self::ChData => MsgPriority::Immediate,
            Self::RawAirData => MsgPriority::Fast,
            Self::MagneticFieldStrength2 => MsgPriority::Fast,
            Self::GlobalNavigationSolution => MsgPriority::High,
            Self::StaticPressure => MsgPriority::Nominal,
            Self::Fix2 => MsgPriority::Nominal,
            Self::LinkStats => MsgPriority::Low,
            _ => MsgPriority::Slow,
        }
    }

    /// Get the payload size. Does not include padding for a tail byte.
    pub const fn payload_size(&self) -> u8 {
        // todo: 0 values are ones we haven't implemented yet.
        // todo: Handle when response has a diff payload size!
        match self {
            // This includes no name; add name len to it after. Assumes no hardware certificate of authority.
            Self::GetNodeInfo => 40,
            Self::NodeInfo => 48, // hard-coded for name len of 8.
            Self::GlobalTimeSync => 7,
            Self::TransportStats => 18,
            Self::Panic => 7, // todo?
            Self::Restart => 5,
            Self::RestartResp => 1,
            Self::ExecuteOpcode => 0,
            Self::GetSet => 0,
            Self::NodeStatus => PAYLOAD_SIZE_NODE_STATUS as u8,
            Self::MagneticFieldStrength2 => 7,
            Self::RawImu => 47,
            Self::RawAirData => 0,
            Self::StaticPressure => 6,
            Self::StaticTemperature => 4,
            Self::GnssAux => 16,
            Self::Fix2 => 51,
            // This assumes we are not using either dynamic-len fields `pose_covariance` or `velocity_covariance`.
            Self::GlobalNavigationSolution => 88, // todo: QC this by checking the unpacked size!!
            Self::ChData => 38,                   // todo
            Self::LinkStats => 38,                // todo
            Self::SetConfig => 20,                // todo
            Self::Ack => 20,                      // todo
            Self::ConfigGnssGet => 0,
            Self::ConfigGnss => PAYLOAD_SIZE_CONFIG_COMMON as u8 + 10,
            Self::ConfigRxGet => 0,                                 // todo
            Self::ConfigRx => PAYLOAD_SIZE_CONFIG_COMMON as u8 + 4, // todo
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
            Self::GetNodeInfo => 55_719,
            Self::NodeInfo => 55_719, // todo: Same as GetNodeInfo?
            Self::GlobalTimeSync => 30_984,
            Self::TransportStats => 19_827,
            Self::Panic => 64_606,
            Self::Restart => 8_063,
            Self::RestartResp => 8_063, // todo: Same as restart?
            Self::ExecuteOpcode => 55_252,
            Self::GetSet => 64_272,
            Self::NodeStatus => 48_735,
            Self::MagneticFieldStrength2 => 47653,
            Self::RawImu => 20_030,
            Self::RawAirData => 20_590,
            Self::StaticPressure => 60_404,
            Self::StaticTemperature => 6_052,
            Self::GnssAux => 9_390,
            Self::Fix2 => 51_096,
            Self::GlobalNavigationSolution => 7_536,
            Self::ChData => 0,
            Self::LinkStats => 0,
            Self::SetConfig => 0,
            Self::Ack => 0,
            Self::ConfigGnssGet => 0,
            Self::ConfigGnss => 0,
            Self::ConfigRxGet => 0,
            Self::ConfigRx => 0,
        }
    }
}

// This is a GetSet response. Very messy due to variable-size fields in the middle.
// pads: 5 + 5 + 6 + 6 = 22
// values (Integer + empty): 3 + 64 + 3 + 0 = 70
// Default values (empty): 2 + 2 = 4
// name: 14
//
// 110/8 = 13.75
pub const PAYLOAD_SIZE_CAN_ID_RESP: usize = 14;

pub const PAYLOAD_SIZE_CONFIG_COMMON: usize = 4;

// Unfortunately, it seems we can't satisfy static allocation using const *methods*.
pub const PAYLOAD_SIZE_NODE_STATUS: usize = 7;

pub static TRANSFER_ID_NODE_INFO: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_NODE_STATUS: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GLOBAL_TIME_SYNC: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_TRANSPORT_STATS: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_PANIC: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_RESTART: AtomicUsize = AtomicUsize::new(0);

pub static TRANSFER_ID_MAGNETIC_FIELD_STRENGTH2: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_RAW_IMU: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_AIR_DATA: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_STATIC_PRESSURE: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_STATIC_TEMPERATURE: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GNSS_AUX: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_FIX2: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GLOBAL_NAVIGATION_SOLUTION: AtomicUsize = AtomicUsize::new(0);

pub static TRANSFER_ID_CH_DATA: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_LINK_STATS: AtomicUsize = AtomicUsize::new(0);

// todo: Impl these Arudpilot-specific types:
// https://github.com/dronecan/DSDL/tree/master/ardupilot/gnss

pub static TRANSFER_ID_GET_SET: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_ACK: AtomicUsize = AtomicUsize::new(0);

// Static buffers, to ensure they live long enough through transmission. Note; This all include room for a tail byte,
// based on payload len. We also assume no `certificate_of_authority` for hardware size.
// static mut BUF_NODE_INFO: [u8; 64] = [0; MsgType::GetNodeInfo.buf_size()]; // todo: This approach would be better, but not working.
static mut BUF_NODE_INFO: [u8; 48] = [0; 48];
static mut BUF_NODE_STATUS: [u8; 8] = [0; 8];
static mut BUF_TIME_SYNC: [u8; 8] = [0; 8];
static mut BUF_TRANSPORT_STATS: [u8; 20] = [0; 20];

static mut BUF_MAGNETIC_FIELD_STRENGTH2: [u8; 8] = [0; 8]; // Note: No covariance.
static mut BUF_RAW_IMU: [u8; 48] = [0; 48]; // Note: No covariance.
static mut BUF_PRESSURE: [u8; 8] = [0; 8];
static mut BUF_TEMPERATURE: [u8; 8] = [0; 8];
static mut BUF_GNSS_AUX: [u8; 20] = [0; 20]; // 16 bytes, but needs a tail byte, so 20.
static mut BUF_FIX2: [u8; 64] = [0; 64]; // 48-byte payload; pad to 64.
static mut BUF_GLOBAL_NAVIGATION_SOLUTION: [u8; 64] = [0; 64]; // todo: Size

static mut BUF_CAN_ID_RESP: [u8; PAYLOAD_SIZE_CAN_ID_RESP] = [0; PAYLOAD_SIZE_CAN_ID_RESP];

// Per DC spec.
pub const NODE_ID_MIN_VALUE: u8 = 1;
pub const NODE_ID_MAX_VALUE: u8 = 127;

use crate::gnss::FixDronecan;
use defmt::println;

// todo t
// use crate::{
//     gnss::GlobalNavSolution,
//     types::{HardwareVersion, NodeHealth, NodeMode, NodeStatus, SoftwareVersion},
// };
// use fdcan::{
//     frame::{FrameFormat, RxFrameInfo, TxFrameHeader},
//     id::{ExtendedId, Id},
//     FdCan, Mailbox, NormalOperationMode, ReceiveOverrun,
// };

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_node_status(
    can: &mut crate::Can_,
    health: NodeHealth,
    mode: NodeMode,
    vendor_specific_status_code: u16,
    uptime_sec: u32,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let status = NodeStatus {
        uptime_sec,
        health,
        mode,
        vendor_specific_status_code,
    };

    let m_type = MsgType::NodeStatus;

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_NODE_STATUS as u8) + 1];
    let mut buf = unsafe { &mut BUF_NODE_STATUS };

    buf[..m_type.payload_size() as usize].clone_from_slice(&status.to_bytes());

    let transfer_id = TRANSFER_ID_NODE_STATUS.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/1.GetNodeInfo.uavcan
/// A composite type sent in response to a request.
pub fn publish_node_info(
    can: &mut crate::Can_,
    health: NodeHealth,
    mode: NodeMode,
    vendor_specific_status_code: u16,
    uptime_sec: u32,
    software_version: &SoftwareVersion,
    hardware_version: &HardwareVersion,
    node_name: &[u8],
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::NodeInfo;

    // todo: We have temporarily hardcoded this buffer fo a name len of 8.
    let mut buf = unsafe { &mut BUF_NODE_INFO };

    let status = NodeStatus {
        uptime_sec,
        health,
        mode,
        vendor_specific_status_code,
    };

    buf[..7].clone_from_slice(&status.to_bytes());
    buf[7..22].clone_from_slice(&software_version.to_bytes());
    // Assumes no hardware cert of authority.
    buf[22..40].clone_from_slice(&hardware_version.to_bytes());
    buf[40..node_name.len()].clone_from_slice(node_name);

    let transfer_id = TRANSFER_ID_NODE_INFO.fetch_add(1, Ordering::Relaxed);

    let frame_type = FrameType::Service(ServiceData {
        dest_node_id: requester_node_id,
        req_or_resp: RequestResponse::Response,
    });

    broadcast(
        can,
        frame_type,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GetTransportStats.uavcan
/// Standard data type: uavcan.protocol.GetTransportStats
/// This is published in response to a requested.
/// todo: What is the data type ID? 4 is in conflict.
pub fn publish_transport_stats(
    can: &mut crate::Can_,
    num_transmitted: u64,
    num_received: u64,
    num_errors: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::TransportStats;

    let mut buf = unsafe { &mut BUF_TRANSPORT_STATS };

    buf[..6].clone_from_slice(&num_transmitted.to_le_bytes()[..6]);
    buf[6..12].clone_from_slice(&num_received.to_le_bytes()[..6]);
    buf[12..18].clone_from_slice(&num_errors.to_le_bytes()[..6]);

    // Not used: Can interface stats.

    let transfer_id = TRANSFER_ID_TRANSPORT_STATS.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.Panic.uavcan
/// todo: What is the data type ID? 5 is in conflict.
/// "Nodes that are expected to react to this message should wait for at least MIN_MESSAGES subsequent messages
/// with any reason text from any sender published with the interval no higher than MAX_INTERVAL_MS before
/// undertaking any emergency actions." (Min messages: 3. Max interval: 500ms)
pub fn publish_panic(
    can: &mut crate::Can_,
    reason_text: &mut [u8],
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    if reason_text.len() > 7 {
        return Err(CanError::PayloadSize);
    }

    let m_type = MsgType::Panic;

    let transfer_id = TRANSFER_ID_PANIC.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        reason_text,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn publish_time_sync(
    can: &mut crate::Can_,
    previous_transmission_timestamp_usec: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::GlobalTimeSync;

    let mut buf = unsafe { &mut BUF_TIME_SYNC };

    buf[..7].clone_from_slice(
        &(previous_transmission_timestamp_usec & 0xff_ffff_ffff_ffff).to_le_bytes(),
    );

    let transfer_id = TRANSFER_ID_GLOBAL_TIME_SYNC.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )?;

    Ok(())
}

// todo: Publish air data

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1028.StaticPressure.uavcan
pub fn publish_static_pressure(
    can: &mut crate::Can_,
    pressure: f32,          // Pascal
    pressure_variance: f32, // Pascal^2. 16-bit
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::StaticPressure;

    let mut buf = unsafe { &mut BUF_PRESSURE };

    buf[..4].copy_from_slice(&pressure.to_le_bytes());

    let pressure_variance = f16::from_f32(pressure_variance);
    buf[4..6].copy_from_slice(&pressure_variance.to_le_bytes());

    let transfer_id = TRANSFER_ID_STATIC_PRESSURE.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1029.StaticTemperature.uavcan
pub fn publish_temperature(
    can: &mut crate::Can_,
    temperature: f32,          // Kelvin. 16-bit.
    temperature_variance: f32, // Kelvin^2. 16-bit
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::StaticTemperature;

    let mut buf = unsafe { &mut BUF_TEMPERATURE };

    let temperature = f16::from_f32(temperature);
    buf[..2].clone_from_slice(&temperature.to_le_bytes());

    let temperature_variance = f16::from_f32(temperature_variance);
    buf[2..4].clone_from_slice(&temperature_variance.to_le_bytes());

    let transfer_id = TRANSFER_ID_STATIC_TEMPERATURE.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1002.MagneticFieldStrength2.uavcan
pub fn publish_mag_field_strength(
    can: &mut crate::Can_,
    sensor_id: u8,
    magnetic_field: &[f32; 3],          // f16. Gauss; X, Y, Z.
    _magnetic_field_covariance: &[f32], // f16
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::MagneticFieldStrength2;

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let mut buf = unsafe { &mut BUF_MAGNETIC_FIELD_STRENGTH2 };

    let field_x = f16::from_f32(magnetic_field[0]);
    let field_y = f16::from_f32(magnetic_field[1]);
    let field_z = f16::from_f32(magnetic_field[2]);

    buf[0] = sensor_id;
    buf[1..3].clone_from_slice(&field_x.to_le_bytes());
    buf[3..5].clone_from_slice(&field_y.to_le_bytes());
    buf[5..7].clone_from_slice(&field_z.to_le_bytes());

    // todo: Covariance as-required.

    let transfer_id = TRANSFER_ID_MAGNETIC_FIELD_STRENGTH2.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1003.RawIMU.uavcan
pub fn publish_raw_imu(
    can: &mut crate::Can_,
    timestamp: u64,  // 7-bytes
    gyro: [f32; 3],  // f16. x, y, z. rad/s
    accel: [f32; 3], // f16. x, y, z. rad/s
    // todo: integration?
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::RawImu;

    let mut buf = unsafe { &mut BUF_RAW_IMU };

    buf[..7].clone_from_slice(&(timestamp & 0b111_1111).to_le_bytes());
    // integration interval: 0 here.

    buf[11..13].clone_from_slice(&f16::from_f32(gyro[0]).to_le_bytes());
    buf[13..15].clone_from_slice(&f16::from_f32(gyro[1]).to_le_bytes());
    buf[15..17].clone_from_slice(&f16::from_f32(gyro[2]).to_le_bytes());

    // 0s for integral of gyro and accel.

    buf[29..31].clone_from_slice(&f16::from_f32(accel[0]).to_le_bytes());
    buf[31..33].clone_from_slice(&f16::from_f32(accel[1]).to_le_bytes());
    buf[33..35].clone_from_slice(&f16::from_f32(accel[2]).to_le_bytes());

    // todo: Covariance, and integration as-required.

    let transfer_id = TRANSFER_ID_RAW_IMU.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/navigation/2000.GlobalNavigationSolution.uavcan
pub fn publish_global_navigation_solution(
    can: &mut crate::Can_,
    data: &GlobalNavSolution,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::GlobalNavigationSolution;

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let mut buf = unsafe { &mut BUF_GLOBAL_NAVIGATION_SOLUTION };

    println!("UNPACKED SIZE FOR NAV SOL: {}. Modify MSG_TYPE.payload_size, and buf size as required with this \
    / 8!", data.pack().unwrap().len());

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    let transfer_id = TRANSFER_ID_MAGNETIC_FIELD_STRENGTH2.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1061.Auxiliary.uavcan
pub fn publish_gnss_aux(
    can: &mut crate::Can_,
    data: &GnssAuxiliary,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let mut buf = unsafe { &mut BUF_GNSS_AUX };

    let m_type = MsgType::GnssAux;

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.to_bytes());

    let transfer_id = TRANSFER_ID_GNSS_AUX.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1063.Fix2.uavcan
pub fn publish_fix2(
    can: &mut crate::Can_,
    data: &FixDronecan,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let mut buf = unsafe { &mut BUF_FIX2 };

    let m_type = MsgType::Fix2;

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    buf[47] = 2; // todo: Why?

    let transfer_id = TRANSFER_ID_FIX2.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn handle_time_sync(
    can: &mut crate::Can_,
    payload: &[u8],
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let mut buf = [0; 8];
    buf[..7].clone_from_slice(payload);
    let previous_transmission_timestamp_usec = u64::from_le_bytes(buf);

    // todo: Handle.

    Ok(())
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.RestartNode.uavcan
pub fn handle_restart_request(
    can: &mut crate::Can_,
    payload: &[u8],
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::RestartResp;

    let mut num_bytes = [0; 8];
    num_bytes[..5].clone_from_slice(payload);
    let magic_number = u64::from_le_bytes(num_bytes);

    let transfer_id = TRANSFER_ID_RESTART.fetch_add(1, Ordering::Relaxed);

    if magic_number != 0xAC_CE55_1B1E {
        broadcast(
            can,
            FrameType::Message,
            m_type,
            node_id,
            transfer_id as u8,
            &mut [0], // ie false; error
            fd_mode,
        )?;

        return Err(CanError::PayloadData);
    }

    let frame_type = FrameType::Service(ServiceData {
        dest_node_id: requester_node_id,
        req_or_resp: RequestResponse::Response,
    });

    broadcast(
        can,
        frame_type,
        m_type,
        node_id,
        transfer_id as u8,
        &mut [1], // ie true; success
        // 1,
        fd_mode,
    )?;

    let cp = unsafe { cortex_m::Peripherals::steal() };
    // todo: Not working.
    // cp.SCB.sys_reset();

    Ok(())
}

/// Acknowledge that node ID was successfully changed.
/// In firmware, only apply the change if this returns Ok, since it performs checks,
/// as well as the value to set.
pub fn ack_can_id_change(
    can: &mut crate::Can_,
    node_id_requested: &Value,
    fd_mode: bool,
    node_id_current: u8,
) -> Result<u8, CanError> {
    let m_type = MsgType::Ack;

    let requested_val = match node_id_requested {
        Value::Integer(node_id_requested) => *node_id_requested as u8,
        _ => {
            // todo: Reply back with something over CAN?
            println!("Non-integer value requested as node id");
            return Err(CanError::PayloadData);
        }
    };

    if !(NODE_ID_MIN_VALUE < requested_val && requested_val < NODE_ID_MAX_VALUE) {
        return Err(CanError::PayloadData);
    }

    let mut buf = unsafe { &mut BUF_CAN_ID_RESP };

    let mut name = [0; 20]; // todo: Is this ok?
    name[0..PARAM_NAME_NODE_ID.len()].copy_from_slice(crate::types::PARAM_NAME_NODE_ID);

    let resp = GetSetResponse {
        value: *node_id_requested,
        default_value: None,
        max_value: Some(NumericValue::Integer(NODE_ID_MAX_VALUE as i64)),
        min_value: Some(NumericValue::Integer(NODE_ID_MIN_VALUE as i64)),
        name,
        name_len: PARAM_NAME_NODE_ID.len(),
    };

    resp.to_bytes(buf);

    let transfer_id = TRANSFER_ID_GET_SET.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id_current,
        transfer_id as u8,
        buf,
        fd_mode,
    )?;

    Ok(requested_val)
}
