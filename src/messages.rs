//! This module contains standard messages all nodes must
//! or can implement. It's described in the [DSDL repo, protcols page]
//! (https://github.com/dronecan/DSDL/tree/master/uavcan/protocol)

use crate::{Can, CanError};

use cortex_m;

pub const DATA_TYPE_ID_GET_NODE_INFO: u16 = 1;
pub const DATA_TYPE_ID_NODE_STATUS: u16 = 341;
pub const DATA_TYPE_ID_GLOBAL_TIME_SYNC: u16 = 4; // todo?
pub const DATA_TYPE_ID_TRANSPORT_STATS: u16 = 4; // todo?
pub const DATA_TYPE_ID_PANIC: u16 = 5; // todo?
pub const DATA_TYPE_ID_RESTART: u16 = 5; // todo?

pub static TRANSFER_ID_NODE_STATUS: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GLOBAL_TIME_SYNC: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_TRANSPORT_STATS: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_PANIC: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_RESTART: AtomicUsize = AtomicUsize::new(0);

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

impl NOdeStatus {
    pub fn to_bytes(&self) -> [u8; 7] {
        let mut result = [0; 7];

        payload[0..4].clone_from_slice(&self.uptime_sec.to_le_bytes());

        // Health and mode. Submode is reserved by the spec for future use,
        // but is currently not used.
        payload[4] = ((self.health as u8) << 6) | ((self.mode as u8) << 3);

        payload[5..7].clone_from_slice(&self.vendor_specific_status_code.to_le_bytes());

        result
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
    /// Certificate of authenticity (COA) of the hardware, 255 bytes max.
    // pub certificate_of_authority: &'a [u8],
    pub certificate_of_authority: u8, // todo: Hardcoded as 1 byte.
}

impl HardwareVersion {
    // pub fn to_bytes(&self, buf: &mut [u8]) {
    pub fn to_bytes(&self) -> [u8; 19] {
        let mut buf = [0; 19];

        buf[0] = self.major;
        buf[1] = self.minor;
        buf[2..18].clone_from_slice(&self.unique_id);
        // buf[18..self.certificate_of_authority.len() + 18]
        //     .clone_from_slice(self.certificate_of_authority);
        buf[19] = self.certificate_of_authority;
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
    pub fn to_bytes(self) -> [u8; 15] {
        let mut result = [0; 15];

        result[0] = self.major;
        result[1] = self.minor;
        result[2] = self.optional_field_flags;
        result[3..7].clone_from_slice(self.vcs_commit.to_le_bytes());
        result[7..15].clone_from_slice(self.image_crc.to_le_bytes());

        result
    }
}

impl CanInterfaceStats {
    pub fn to_bytes(self) -> [u8; 15] {
        let mut result = [0; 15];

        result[0] = self.major;
        result[1] = self.minor;
        result[2] = self.optional_field_flags;
        result[3..7].clone_from_slice(self.vcs_commit.to_le_bytes());
        result[7..15].clone_from_slice(self.image_crc.to_le_bytes());

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

/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_node_status(
    can: &mut setup::Can_,
    health: NodeHealth,
    mode: NodeMode,
    vendor_specific_status_code: u16,
    tick_timer_elapsed_s: f32,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let uptime_sec = (crate::tick_count_fm_overflows_s() + tick_timer_elapsed_s) as u32;

    let status = NodeStatus {
        uptime_sec: u32,
        health,
        mode,
        vendor_specific_status_code,
    };

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_NODE_STATUS,
        node_id,
        transfer_id as u8,
        &status.to_bytes(),
        payload.len() as u16,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/1.GetNodeInfo.uavcan
/// A composite type sent in response to a request.
pub fn publish_node_info(
    can: &mut setup::Can_,
    health: NodeHealth,
    mode: NodeMode,
    vendor_specific_status_code: u16,
    tick_timer_elapsed_s: f32,
    software_version: &SoftwareVersion,
    hardware_version: &HardwareVersion,
    node_name: &[u8],
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    // todo: We have temporarily hardcoded this buffer fo a name len of 8.
    let mut payload = [0; 49];
    let uptime_sec = (crate::tick_count_fm_overflows_s() + tick_timer_elapsed_s) as u32;

    let status = NodeStatus {
        uptime_sec: u32,
        health,
        mode,
        vendor_specific_status_code,
    };

    payload[0..7].clone_from_slice(&status.to_bytes());
    payload[7..22].clone_from_slice(software_version.to_bytes());
    payload[22..41].clone_from_slice(hardware_version.to_bytes());
    payload[41..node_name.len().clone_from_slice(node_name)];

    let payload_len = 41 + node_name.len() as u16;

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_NODE_STATUS,
        node_id,
        transfer_id as u8,
        &payload,
        payload_len,
        fd_mode,
    )
}

/// Standard data type: uavcan.protocol.GetTransportStats
/// This is published in response to a requested.
/// todo: What is the data type ID? 4 is in conflict.
pub fn publish_transport_stats(
    can: &mut setup::Can_,
    num_transmitted: u64,
    num_received: u64,
    num_errors: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let mut payload = [0_u8; 18];

    payload[0..6].clone_from_slice(&num_transmitted.to_le_bytes()[0..6]);
    payload[6..12].clone_from_slice(&num_received.to_le_bytes()[0..6]);
    payload[12..18].clone_from_slice(&num_errors.to_le_bytes()[0..6]);

    let transfer_id = TRANSFER_ID_TRANSPORT_STATS.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        MsgPriority::Slow,
        DATA_TYPE_ID_TRANSPORT_STATS,
        node_id,
        transfer_id as u8,
        &payload,
        payload.len() as u16,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.Panic.uavcan
/// todo: What is the data type ID? 5 is in conflict.
/// "Nodes that are expected to react to this message should wait for at least MIN_MESSAGES subsequent messages
/// with any reason text from any sender published with the interval no higher than MAX_INTERVAL_MS before
/// undertaking any emergency actions." (Min messages: 3. Max interval: 500ms)
pub fn publish_panic(
    can: &mut setup::Can_,
    reason_text: &[u8],
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
        payload.len() as u16,
        fd_mode,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.RestartNode.uavcan
pub fn handle_restart_request(
    can: &mut setup::Can_,
    payload: &[u8],
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let mut buf = [0; 8];
    buf[0..5].clone_from_slice(payload);
    let magic_number = u64::from_le_bytes(buf);

    if magic_number != 0xAC_CE55_1B1E {
        broadcast(
            can,
            MsgPriority::Low,
            DATA_TYPE_ID_RESTART,
            node_id,
            TRANSFER_ID_RESTART,
            &[0],
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
        TRANSFER_ID_RESTART,
        &[1], // ie true; success
        1,
        fd_mode,
    )?;

    let cp = unsafe { cortex_m::Peripherals::steal() };
    cp.SCB.sys_reset();

    Ok(())
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn publish_time_sync(
    can: &mut setup::Can_,
    previous_transmission_timestamp_usec: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let mut payload = [0; 7];
    payload[0..7].clone_from_slice(
        &(previous_transmission_timestamp_usec & ff_ffff_ffff_ffff).to_le_bytes(),
    );

    broadcast(
        can,
        MsgPriority::Low,
        DATA_TYPE_ID_GLOBAL_TIME_SYNC,
        node_id,
        TRANSFER_ID_GLOBAL_TIME_SYNC,
        &payload,
        payload.len() as u16,
        fd_mode,
    )?;

    Ok(())
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn handle_time_sync(
    can: &mut setup::Can_,
    payload: &[u8],
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let mut buf = [0; 8];
    buf[0..7].clone_from_slice(payload);
    let previous_transmission_timestamp_usec = u64::from_le_bytes(buf);

    // todo: Handle.

    Ok(())
}
