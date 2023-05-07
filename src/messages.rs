//! This module contains standard messages all nodes must
//! or can implement. It's described in the [DSDL repo, protcols page]
//! (https://github.com/dronecan/DSDL/tree/master/uavcan/protocol)

use core::sync::atomic::{self, AtomicUsize, Ordering};

use packed_struct::PackedStruct;

use crate::{broadcast, Can, CanError, MsgPriority, types::{Value, NumericValue, GetSetResponse, PARAM_NAME_NODE_ID}};

use half::f16;

use cortex_m;

pub const DATA_TYPE_ID_GET_NODE_INFO: u16 = 1;
pub const DATA_TYPE_ID_NODE_STATUS: u16 = 341;
pub const DATA_TYPE_ID_GLOBAL_TIME_SYNC: u16 = 4; // todo?
pub const DATA_TYPE_ID_TRANSPORT_STATS: u16 = 4; // todo?
pub const DATA_TYPE_ID_PANIC: u16 = 5; // todo?
pub const DATA_TYPE_ID_RESTART: u16 = 5; // todo?
pub const DATA_TYPE_ID_EXECUTE_OPCODE: u16 = 10;
pub const DATA_TYPE_ID_GET_SET: u16 = 11;

pub const DATA_TYPE_ID_MAGNETIC_FIELD_STRENGTH2: u16 = 1002;
pub const DATA_TYPE_ID_RAW_IMU: u16 = 1_003;
pub const DATA_TYPE_ID_RAW_AIR_DATA: u16 = 1027;
pub const DATA_TYPE_ID_STATIC_PRESSURE: u16 = 1028;
pub const DATA_TYPE_ID_STATIC_TEMPERATURE: u16 = 1029;
pub const DATA_TYPE_ID_FIX2: u16 = 1_063;
pub const DATA_TYPE_ID_GLOBAL_NAVIGATION_SOLUTION: u16 = 2_000;

pub const DATA_TYPE_ID_CH_DATA: u16 = 2_103;
pub const DATA_TYPE_ID_LINK_STATS: u16 = 2_104;

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
pub static TRANSFER_ID_FIX2: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GLOBAL_NAVIGATION_SOLUTION: AtomicUsize = AtomicUsize::new(0);

pub static TRANSFER_ID_CH_DATA: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_LINK_STATS: AtomicUsize = AtomicUsize::new(0);

// This includes no name; add name len to it after. Assumes no hardware certificate of authority.
pub const PAYLOAD_SIZE_NODE_INFO_WITHOUT_NAME: usize = 40;

pub const PAYLOAD_SIZE_NODE_STATUS: usize = 7;
pub const PAYLOAD_SIZE_TIME_SYNC: usize = 7;
pub const PAYLOAD_SIZE_TRANSPORT_STATS: usize = 18;
pub const PAYLOAD_SIZE_RESTART: usize = 5;

pub const PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2: usize = 7;
pub const PAYLOAD_SIZE_RAW_IMU: usize = 47; // Does not include covariance.
pub const PAYLOAD_SIZE_STATIC_PRESSURE: usize = 6;
pub const PAYLOAD_SIZE_STATIC_TEMPERATURE: usize = 4;
// pub const PAYLOAD_SIZE_FIX2: usize = 51;
pub const PAYLOAD_SIZE_FIX2: usize = 48; // todo?

// This assumes we are not using either dynamic-len fields `pose_covariance` or `velocity_covariance`.
pub const PAYLOAD_SIZE_GLOBAL_NAVIGATION_SOLUTION: usize = 88;

pub const PAYLOAD_SIZE_CONFIG_COMMON: usize = 4;

// Custom types here we use on multiple projects, but aren't (yet?) part of the DC spec.
pub const DATA_TYPE_ID_ACK: u16 = 2_000;

pub static TRANSFER_ID_ACK: AtomicUsize = AtomicUsize::new(0);

// Static buffers, to ensure they live long enough through transmission. Note; This all include room for a tail byte,
// based on payload len. We also assume no `certificate_of_authority` for hardware size.
static mut BUF_NODE_INFO: [u8; 64] = [0; 64];
static mut BUF_NODE_STATUS: [u8; 8] = [0; 8];
static mut BUF_TIME_SYNC: [u8; 8] = [0; 8];
static mut BUF_TRANSPORT_STATS: [u8; 20] = [0; 20];

static mut BUF_MAGNETIC_FIELD_STRENGTH2: [u8; 8] = [0; 8]; // Note: No covariance.
static mut BUF_RAW_IMU: [u8; 48] = [0; 48]; // Note: No covariance.
static mut BUF_PRESSURE: [u8; 8] = [0; 8];
static mut BUF_TEMPERATURE: [u8; 8] = [0; 8];
static mut BUF_GLOBAL_NAVIGATION_SOLUTION: [u8; 64] = [0; 64]; // todo: Size

// Per DC spec.
pub const NODE_ID_MIN_VALUE: u8 = 0;
pub const NODE_ID_MAX_VALUE: u8 = 127;

use defmt::println;

// todo t
use crate::{
    gnss::GlobalNavSolution,
    types::{HardwareVersion, NodeHealth, NodeMode, NodeStatus, SoftwareVersion},
};
use fdcan::{
    frame::{FrameFormat, RxFrameInfo, TxFrameHeader},
    id::{ExtendedId, Id},
    FdCan, Mailbox, NormalOperationMode, ReceiveOverrun,
};

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

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_NODE_STATUS as u8) + 1];
    let mut buf = unsafe { &mut BUF_NODE_STATUS };

    buf[..PAYLOAD_SIZE_NODE_STATUS].clone_from_slice(&status.to_bytes());

    let transfer_id = TRANSFER_ID_NODE_STATUS.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_NODE_STATUS,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_NODE_STATUS as u16,
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
) -> Result<(), CanError> {
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

    let len = (PAYLOAD_SIZE_NODE_INFO_WITHOUT_NAME + node_name.len()) as u16;

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_GET_NODE_INFO,
        node_id,
        transfer_id as u8,
        buf,
        len,
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
    let mut buf = unsafe { &mut BUF_TRANSPORT_STATS };

    buf[..6].clone_from_slice(&num_transmitted.to_le_bytes()[..6]);
    buf[6..12].clone_from_slice(&num_received.to_le_bytes()[..6]);
    buf[12..18].clone_from_slice(&num_errors.to_le_bytes()[..6]);

    // Not used: Can interface stats.

    let transfer_id = TRANSFER_ID_TRANSPORT_STATS.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_TRANSPORT_STATS,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_TRANSPORT_STATS as u16,
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

    let transfer_id = TRANSFER_ID_PANIC.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_PANIC,
        node_id,
        transfer_id as u8,
        reason_text,
        reason_text.len() as u16,
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
    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_TIME_SYNC as u8) + 1];
    let mut buf = unsafe { &mut BUF_TIME_SYNC };

    buf[..7].clone_from_slice(
        &(previous_transmission_timestamp_usec & 0xff_ffff_ffff_ffff).to_le_bytes(),
    );

    let transfer_id = TRANSFER_ID_GLOBAL_TIME_SYNC.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Low,
        DATA_TYPE_ID_GLOBAL_TIME_SYNC,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_TIME_SYNC as u16,
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
    let mut buf = unsafe { &mut BUF_PRESSURE };

    buf[..4].copy_from_slice(&pressure.to_le_bytes());

    let pressure_variance = f16::from_f32(pressure_variance);
    buf[4..6].copy_from_slice(&pressure_variance.to_le_bytes());

    let transfer_id = TRANSFER_ID_STATIC_PRESSURE.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_STATIC_PRESSURE,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_STATIC_PRESSURE as u16,
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
    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_STATIC_TEMPERATURE as u8) + 1];
    let mut buf = unsafe { &mut BUF_TEMPERATURE };

    let temperature = f16::from_f32(temperature);
    buf[..2].clone_from_slice(&temperature.to_le_bytes());

    let temperature_variance = f16::from_f32(temperature_variance);
    buf[2..4].clone_from_slice(&temperature_variance.to_le_bytes());

    let transfer_id = TRANSFER_ID_STATIC_TEMPERATURE.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Nominal,
        DATA_TYPE_ID_STATIC_TEMPERATURE,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_STATIC_TEMPERATURE as u16,
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
        MsgPriority::Nominal,
        DATA_TYPE_ID_MAGNETIC_FIELD_STRENGTH2,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u16,
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
        MsgPriority::High,
        DATA_TYPE_ID_RAW_IMU,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_RAW_IMU as u16,
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
    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let mut buf = unsafe { &mut BUF_GLOBAL_NAVIGATION_SOLUTION };

    buf[..PAYLOAD_SIZE_GLOBAL_NAVIGATION_SOLUTION].clone_from_slice(&data.pack().unwrap());

    let transfer_id = TRANSFER_ID_MAGNETIC_FIELD_STRENGTH2.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Nominal,
        DATA_TYPE_ID_GLOBAL_NAVIGATION_SOLUTION,
        node_id,
        transfer_id as u8,
        buf,
        PAYLOAD_SIZE_GLOBAL_NAVIGATION_SOLUTION as u16,
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
) -> Result<(), CanError> {
    let mut num_bytes = [0; 8];
    num_bytes[..5].clone_from_slice(payload);
    let magic_number = u64::from_le_bytes(num_bytes);

    let transfer_id = TRANSFER_ID_RESTART.fetch_add(1, Ordering::Relaxed);

    if magic_number != 0xAC_CE55_1B1E {
        broadcast(
            can,
            MsgPriority::Low,
            DATA_TYPE_ID_RESTART,
            node_id,
            transfer_id as u8,
            &mut [0], // ie false; error
            1,
            fd_mode,
        )?;

        return Err(CanError::PayloadData);
    }

    broadcast(
        can,
        MsgPriority::Low,
        DATA_TYPE_ID_RESTART,
        node_id,
        transfer_id as u8,
        &mut [1], // ie true; success
        1,
        fd_mode,
    )?;

    let cp = unsafe { cortex_m::Peripherals::steal() };
    // todo: Not working.
    // cp.SCB.sys_reset();

    Ok(())
}

/// Acknowledge that node ID was successfully changed.
pub fn ack_can_id_change(can: &mut crate::Can_, fd_mode: bool, node_id: u8,) {
    let mut name = [0; 30]; // todo: Is this ok?

    let mut buf = unsafe { &mut BUF_GLOBAL_NAVIGATION_SOLUTION };

    name[0..PARAM_NAME_NODE_ID.len()].copy_from_slice(crate::types::PARAM_NAME_NODE_ID);
    let resp = GetSetResponse {
        value: Value::Integer(node_id_requested),
        default_value: None,
        max_value: Some(NumericValue::Integer(NODE_ID_MAX_VALUE as i64)),
        min_value: Some(NumericValue::Integer(NODE_ID_MIN_VALUE as i64)),
        name,
    };

    let transfer_id = TRANSFER_ID_GET_SET.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Low,
        DATA_TYPE_ID_GET_SET,
        node_id,
        transfer_id as u8,
        &mut resp.to_bytes(),
        1, // todo!
        fd_mode,
    )?;


}