//! This module contains code related to broadcasting messages over CAN.
//! It is hard-coded to work with our HAL.

use core::sync::atomic::{AtomicUsize, Ordering};

use fdcan::{
    frame::{FrameFormat, RxFrameInfo, TxFrameHeader},
    id::{ExtendedId, Id},
    FdCan, Mailbox, NormalOperationMode, ReceiveOverrun,
};

use stm32_hal2::can::Can;

use bitvec::prelude::*;

use defmt::println;

use crate::{
    crc::TransferCrc,
    dsdl::{
        GetSetResponse, HardwareVersion, IdAllocationData, LinkStats, NodeHealth, NodeMode,
        NodeStatus, SoftwareVersion,
    },
    f16, find_tail_byte_index,
    gnss::{FixDronecan, GlobalNavSolution, GnssAuxiliary},
    make_tail_byte,
    messages::MsgType,
    protocol::{CanId, FrameType, RequestResponse, ServiceData, TransferComponent},
    CanError,
};

use packed_struct::PackedStruct;

use core::convert::Infallible;

type Can_ = FdCan<Can, NormalOperationMode>;

// Note: These are only capable of handling one message at a time. This is especially notable
// for reception.
static mut MULTI_FRAME_BUFS_TX: [[u8; 64]; 20] = [[0; 64]; 20];
// static mut MULTI_FRAME_BUFS_TX_FD: [[u8; 64]; 2] = [[0; 64]; 2];
// static mut MULTI_FRAME_BUFS_TX_LEGACY: [[u8; 8]; 20] = [[0; 8]; 20];

pub(crate) const DATA_FRAME_MAX_LEN_FD: u8 = 64;
pub(crate) const DATA_FRAME_MAX_LEN_LEGACY: u8 = 8;

pub static TRANSFER_ID_ID_ALLOCATION: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_NODE_INFO: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_NODE_STATUS: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GLOBAL_TIME_SYNC: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_TRANSPORT_STATS: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_PANIC: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_RESTART: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GET_SET: AtomicUsize = AtomicUsize::new(0);

pub static TRANSFER_ID_AHRS_SOLUTION: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_MAGNETIC_FIELD_STRENGTH2: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_RAW_IMU: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_AIR_DATA: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_STATIC_PRESSURE: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_STATIC_TEMPERATURE: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GNSS_AUX: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_FIX2: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_GLOBAL_NAVIGATION_SOLUTION: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_RC_INPUT: AtomicUsize = AtomicUsize::new(0);

pub static TRANSFER_ID_CH_DATA: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_LINK_STATS: AtomicUsize = AtomicUsize::new(0);
pub static TRANSFER_ID_ARDUPILOT_GNSS_STATUS: AtomicUsize = AtomicUsize::new(0);

// todo: Impl these Arudpilot-specific types:
// https://github.com/dronecan/DSDL/tree/master/ardupilot/gnss

pub static TRANSFER_ID_ACK: AtomicUsize = AtomicUsize::new(0);

// Static buffers, to ensure they live long enough through transmission. Note; This all include room for a tail byte,
// based on payload len. We also assume no `certificate_of_authority` for hardware size.
// static mut BUF_NODE_INFO: [u8; 64] = [0; MsgType::GetNodeInfo.buf_size()]; // todo: This approach would be better, but not working.

// These ID allocation buffers accomodate the length including full 16-bit unique id, and tail byte.
static mut BUF_ID_ALLOCATION: [u8; 20] = [0; 20];
// static mut BUF_ID_RESP: [u8; 17] = [0; 17];
// This node info buffer is padded to accomodate a 20-character name.
static mut BUF_NODE_INFO: [u8; 64] = [0; 64];
static mut BUF_NODE_STATUS: [u8; 8] = [0; 8];
static mut BUF_TIME_SYNC: [u8; 8] = [0; 8];
static mut BUF_TRANSPORT_STATS: [u8; 20] = [0; 20];
// Rough size that includes enough room for i64 on most values, and a 40-len name field.
// Also long enough to support padding to the tail byte of 32-len for a 2-frame FD transfer.
static mut BUF_GET_SET: [u8; 90] = [0; 90];

static mut BUF_AHRS_SOLUTION: [u8; 48] = [0; 48]; // Note: No covariance.
                                                  // static mut BUF_MAGNETIC_FIELD_STRENGTH2: [u8; 8] = [0; 8]; // Note: No covariance.
                                                  // Potentially need size 12 for mag strength in FD mode, even with no cov.
static mut BUF_MAGNETIC_FIELD_STRENGTH2: [u8; 12] = [0; 12]; // Note: No covariance.
static mut BUF_RAW_IMU: [u8; 64] = [0; 64]; // Note: No covariance.
static mut BUF_PRESSURE: [u8; 8] = [0; 8];
static mut BUF_TEMPERATURE: [u8; 8] = [0; 8];
static mut BUF_GNSS_AUX: [u8; 20] = [0; 20]; // 16 bytes, but needs a tail byte, so 20.
static mut BUF_FIX2: [u8; 64] = [0; 64]; // 48-byte payload; pad to 64.
static mut BUF_GLOBAL_NAVIGATION_SOLUTION: [u8; 90] = [0; 90];

// This buffer accomodates up to 16 12-bit channels. (224 bits or 28 bytes)
static mut BUF_RC_INPUT: [u8; 32] = [0; 32];
static mut BUF_LINK_STATS: [u8; 12] = [0; 12];
static mut BUF_ARDUPILOT_GNSS_STATUS: [u8; 8] = [0; 8];

// Per DC spec.
pub const NODE_ID_MIN_VALUE: u8 = 1;
pub const NODE_ID_MAX_VALUE: u8 = 127;

use stm32_hal2::pac;

// todo t
/// Write out packet to the CAN peripheral.
fn can_send(
    can: &mut Can_,
    can_id: u32,
    frame_data: &[u8],
    frame_data_len: u8,
    fd_mode: bool,
) -> Result<(), CanError> {
    let max_frame_len = if fd_mode {
        DATA_FRAME_MAX_LEN_FD
    } else {
        DATA_FRAME_MAX_LEN_LEGACY
    };

    if frame_data_len > max_frame_len {
        return Err(CanError::FrameSize);
    }

    let frame_format = if fd_mode {
        FrameFormat::Fdcan
    } else {
        FrameFormat::Standard
    };

    let id = Id::Extended(ExtendedId::new(can_id).unwrap());

    let frame_header = TxFrameHeader {
        len: frame_data_len,
        frame_format,
        id,
        // bit_rate_switching: false,
        bit_rate_switching: true,
        marker: None,
    };

    // Some example codes:
    // 1800: Good code, where no ESP is in place. Constant.

    // No LEC.Idle. Last of 111 (no change). Thi is means all is well.

    // 1896: Error + warning. No LEC. Doesn't hang.
    // 1864: Warning only. No LEC. Doesn't hang.

    // These cause a hang, and both have LEC: Bit0Error.
    // "Bit0Error: During the transmission of a message (or acknowledge bit, or active error
    // flag, or overload flag), the device wanted to send a dominant level (data or identifier bit logical
    // value 0), but the monitored bus value was recessive."
    // 2021 means idle. 1901 means synchronizing.
    // Both have Error passive and Error warning status.
    // 2021 means Bus off (!)
    // Both have Data Last Error code of 111. (no change)

    // 1901: // bad code, prior to hanging.
    // 2021: // bad code, at hanging.

    // These eventually hang.
    // 616 has no Last Error code. 617 has Stuff Error. 618 has Form Error
    // Rest same. Idle, EW + EP, DLEC = 001.

    // 616:
    // 617:
    // 618:

    // This wait appears to be required, pending handling using a queue in the callback.

    let mut count: u16 = 0;
    while !can.is_transmitter_idle() {
        count += 1;
        const TIMEOUT_COUNT: u16 = 50_000; // todo: What should this be?

        if count >= TIMEOUT_COUNT {
            println!("\nCAN loop timeout:");
            unsafe {
                let regs = &(*pac::FDCAN1::ptr());
                println!("SR: {}", regs.psr.read().bits());
            }

            return Err(CanError::Hardware);
        }
    }

    // Not sure if this helps or is required etc.
    // atomic::compiler_fence(Ordering::SeqCst);

    match can.transmit_preserve(frame_header, frame_data, &mut message_pending_handler) {
        Ok(_) => Ok(()),
        Err(_e) => Err(CanError::Hardware),
    }
}

/// Handles splitting a payload into multiple frames, including DroneCAN and Cyphal requirements,
/// eg CRC and data type signature.
fn send_multiple_frames(
    can: &mut Can_,
    payload: &[u8],
    payload_len: u16,
    can_id: u32,
    transfer_id: u8,
    fd_mode: bool,
    base_crc: u16,
) -> Result<(), CanError> {
    let frame_payload_len = if fd_mode {
        DATA_FRAME_MAX_LEN_FD as usize
    } else {
        DATA_FRAME_MAX_LEN_LEGACY as usize
    };

    let mut crc = TransferCrc::new(base_crc);
    crc.add_payload(payload, payload_len as usize);

    // We use slices of the FD buf, even for legacy frames, to keep code simple.
    let bufs = unsafe { &mut MULTI_FRAME_BUFS_TX };

    // See https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/,
    // "Multi-frame transfer" re requirements such as CRC and Toggle bit.
    let mut component = TransferComponent::MultiStart;
    let mut active_frame = 0;
    // This tracks the index of the payload we sent in the previous frame.

    // Populate the first frame. This is different from the others due to the CRC.
    let mut tail_byte = make_tail_byte(TransferComponent::MultiStart, transfer_id);
    let mut payload_len_this_frame = frame_payload_len - 3; // -3 for CRC and tail byte.

    // Add 2 for the CRC.
    let tail_byte_i = find_tail_byte_index(payload_len_this_frame as u8 + 2);

    bufs[active_frame][..2].clone_from_slice(&crc.value.to_le_bytes());
    bufs[active_frame][2..frame_payload_len - 1]
        .clone_from_slice(&payload[..payload_len_this_frame]);
    bufs[active_frame][tail_byte_i] = tail_byte.value();

    can_send(
        can,
        can_id,
        &bufs[active_frame][..frame_payload_len],
        frame_payload_len as u8,
        fd_mode,
    )?;

    let mut latest_i_sent = payload_len_this_frame - 1;

    active_frame += 1;

    // Populate subsequent frames.
    while latest_i_sent < payload_len as usize - 1 {
        let payload_i = latest_i_sent + 1;
        if payload_i + frame_payload_len <= payload_len as usize {
            // Not the last frame.
            payload_len_this_frame = frame_payload_len - 1;
            component = TransferComponent::MultiMid(tail_byte.toggle);
        } else {
            // Last frame.
            payload_len_this_frame = payload_len as usize - payload_i;
            component = TransferComponent::MultiEnd(tail_byte.toggle);
        }

        tail_byte = make_tail_byte(component, transfer_id);

        bufs[active_frame][0..payload_len_this_frame]
            .clone_from_slice(&payload[payload_i..payload_i + payload_len_this_frame]);

        let tail_byte_i = find_tail_byte_index(payload_len_this_frame as u8);
        bufs[active_frame][tail_byte_i] = tail_byte.value();

        // todo: You may need to cut the active fram eoff early for the last frame.
        can_send(
            can,
            can_id,
            // &bufs[active_frame][..tail_byte_i + 1],
            &bufs[active_frame][..tail_byte_i + 1],
            tail_byte_i as u8 + 1,
            fd_mode,
        )?;

        latest_i_sent += payload_len_this_frame; // 8 byte frame size, - 1 for each frame's tail byte.
        active_frame += 1;
    }

    Ok(())
}

/// Send a DroneCAN "broadcast" message. See [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
/// Should be broadcast at interval between 2 and 1000ms.
/// Note: The payload must be static, and include space for the tail byte.
pub fn broadcast(
    can: &mut Can_,
    frame_type: FrameType,
    msg_type: MsgType,
    source_node_id: u8,
    transfer_id: u8,
    payload: &mut [u8],
    fd_mode: bool,
    payload_size: Option<usize>, // Overrides that of message_type if present.
) -> Result<(), CanError> {
    // This saves some if logic in node firmware re decision to broadcast.

    if source_node_id == 0 && frame_type != FrameType::MessageAnon {
        return Err(CanError::PayloadData);
    }

    let can_id = CanId {
        priority: msg_type.priority(),
        type_id: msg_type.id(),
        source_node_id,
        frame_type,
    };

    let payload_len = match payload_size {
        Some(l) => l as u16,
        None => msg_type.payload_size() as u16,
    };

    let frame_payload_len = if fd_mode {
        DATA_FRAME_MAX_LEN_FD
    } else {
        DATA_FRAME_MAX_LEN_LEGACY
    };

    // The transfer payload is up to 7 bytes for non-FD DRONECAN.
    // If data is longer than a single frame, set up a multi-frame transfer.
    // We subtract one to accomodate the tail byte.
    if payload_len > (frame_payload_len - 1) as u16 {
        return send_multiple_frames(
            can,
            payload,
            payload_len,
            can_id.value(),
            transfer_id,
            fd_mode,
            msg_type.base_crc(),
        );
    }

    let tail_byte = make_tail_byte(TransferComponent::SingleFrame, transfer_id);
    let tail_byte_i = find_tail_byte_index(payload_len as u8);

    if tail_byte_i >= payload.len() {
        return Err(CanError::PayloadSize);
    }

    payload[tail_byte_i] = tail_byte.value();

    payload[tail_byte_i] = tail_byte.value();

    can_send(
        can,
        can_id.value(),
        &payload[..tail_byte_i + 1], // todo: Ideal to not pass whole thing, but TS demons
        // &payload,
        // payload_len as u8,
        // The frame data length here includes the tail byte
        tail_byte_i as u8 + 1,
        fd_mode,
    )
}

// todo: You need a fn to get a payload from multiple frames

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/341.NodeStatus.uavcan
/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_node_status(
    can: &mut Can_,
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
    let buf = unsafe { &mut BUF_NODE_STATUS };

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
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/1.GetNodeInfo.uavcan
/// A composite type sent in response to a request.
pub fn publish_node_info(
    can: &mut Can_,
    status: &NodeStatus,
    software_version: &SoftwareVersion,
    hardware_version: &HardwareVersion,
    node_name: &[u8],
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::GetNodeInfo;
    let buf = unsafe { &mut BUF_NODE_INFO };

    if node_name.len() > buf.len() - m_type.payload_size() as usize {
        return Err(CanError::PayloadData);
    }

    // println!("Node name: {:?}", node_name);

    buf[..7].clone_from_slice(&status.to_bytes());
    buf[7..22].clone_from_slice(&software_version.to_bytes());
    buf[22..41].clone_from_slice(&hardware_version.to_bytes());

    // Important: In legacy mode, there is no node len field for name, due to tail array
    // optimization; We jump right into the byte representation.
    // In FD mode, a 7-bit node len field is required.

    // From experiments, doesn't seem to need 1 added for FD with currently tested
    // inputs?
    let mut payload_size = m_type.payload_size() as usize + node_name.len();

    if fd_mode {
        let bits = buf.view_bits_mut::<Msb0>();

        let mut i_bit = 41 * 8;

        bits[i_bit..i_bit + 7].store_le(node_name.len() as u8);
        i_bit += 7;

        for char in node_name {
            // Why big endian? Not sure, but by trial+error, this is it.
            // todo: Why BE here? Confirm in non-FD it's the same
            bits[i_bit..i_bit + 8].store_be(*char);
            i_bit += 8;
        }

        payload_size += 1;
    } else {
        buf[41..41 + node_name.len()].clone_from_slice(node_name);
    }

    // println!("Buf: {:?}", buf);

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
        Some(payload_size),
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GetTransportStats.uavcan
/// Standard data type: uavcan.protocol.GetTransportStats
/// This is published in response to a requested.
/// todo: What is the data type ID? 4 is in conflict.
pub fn publish_transport_stats(
    can: &mut Can_,
    num_transmitted: u64,
    num_received: u64,
    num_errors: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::TransportStats;

    let buf = unsafe { &mut BUF_TRANSPORT_STATS };

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
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/5.Panic.uavcan
/// "Nodes that are expected to react to this message should wait for at least MIN_MESSAGES subsequent messages
/// with any reason text from any sender published with the interval no higher than MAX_INTERVAL_MS before
/// undertaking any emergency actions." (Min messages: 3. Max interval: 500ms)
pub fn publish_panic(
    can: &mut Can_,
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
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn publish_time_sync(
    can: &mut Can_,
    previous_transmission_timestamp_usec: u64,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::GlobalTimeSync;

    let buf = unsafe { &mut BUF_TIME_SYNC };

    buf[..7].clone_from_slice(&previous_transmission_timestamp_usec.to_le_bytes());

    let transfer_id = TRANSFER_ID_GLOBAL_TIME_SYNC.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        None,
    )?;

    Ok(())
}

// todo: Publish air data

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1028.StaticPressure.uavcan
pub fn publish_static_pressure(
    can: &mut Can_,
    pressure: f32,          // Pascal
    pressure_variance: f32, // Pascal^2. 16-bit
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::StaticPressure;

    let buf = unsafe { &mut BUF_PRESSURE };

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
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/air_data/1029.StaticTemperature.uavcan
pub fn publish_temperature(
    can: &mut Can_,
    temperature: f32,          // Kelvin. 16-bit.
    temperature_variance: f32, // Kelvin^2. 16-bit
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::StaticTemperature;

    let buf = unsafe { &mut BUF_TEMPERATURE };

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
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1000.Solution.uavcan
pub fn publish_ahrs_solution(
    can: &mut Can_,
    timestamp: u64,              // 7-bytes, us.
    orientation: &[f32; 4],      // f16. X Y Z W
    angular_velocity: &[f32; 3], // f16. X, Y, Z.
    linear_accel: &[f32; 3],     // f16. X, Y, Z.
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::AhrsSolution;

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let buf = unsafe { &mut BUF_AHRS_SOLUTION };

    let or_x = f16::from_f32(orientation[0]);
    let or_y = f16::from_f32(orientation[1]);
    let or_z = f16::from_f32(orientation[2]);
    let or_w = f16::from_f32(orientation[3]);

    let ang_v_x = f16::from_f32(angular_velocity[0]);
    let ang_v_y = f16::from_f32(angular_velocity[1]);
    let ang_v_z = f16::from_f32(angular_velocity[2]);

    let lin_acc_x = f16::from_f32(linear_accel[0]);
    let lin_acc_y = f16::from_f32(linear_accel[1]);
    let lin_acc_z = f16::from_f32(linear_accel[2]);

    buf[..7].clone_from_slice(&timestamp.to_le_bytes()[0..7]);

    let bits = buf.view_bits_mut::<Msb0>();

    let mut i = 56; // bits

    for v in &[or_x, or_y, or_z, or_w] {
        let v = u16::from_le_bytes(v.to_le_bytes());
        bits[i..i + 16].store_le(v);
        i += 16;
    }

    // 4-bit pad and 0-len covar
    bits[i..i + 8].store_le(0);
    i += 8;

    for v in &[ang_v_x, ang_v_y, ang_v_z] {
        let v = u16::from_le_bytes(v.to_le_bytes());
        bits[i..i + 16].store_le(v);
        i += 16;
    }

    // 4-bit pad and 0-len covar
    bits[i..i + 8].store_le(0);
    i += 8;

    for v in &[lin_acc_x, lin_acc_y, lin_acc_z] {
        let v = u16::from_le_bytes(v.to_le_bytes());
        bits[i..i + 16].store_le(v);
        i += 16;
    }

    // For FD mode, ensure final len field for lin acc cov is 0.
    bits[i..i + 4].store_le(0);

    // todo: Covariance as-required.

    let transfer_id = TRANSFER_ID_AHRS_SOLUTION.fetch_add(1, Ordering::Relaxed);

    let payload_size = if fd_mode {
        m_type.payload_size() + 1 // Due to no TCO on final cov array.
    } else {
        m_type.payload_size()
    };

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        Some(payload_size as usize),
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1002.MagneticFieldStrength2.uavcan
pub fn publish_mag_field_strength(
    can: &mut Can_,
    sensor_id: u8,
    magnetic_field: &[f32; 3],          // f16. Gauss; X, Y, Z.
    _magnetic_field_covariance: &[f32], // f16
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::MagneticFieldStrength2;

    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let buf = unsafe { &mut BUF_MAGNETIC_FIELD_STRENGTH2 };

    let field_x = f16::from_f32(magnetic_field[0]);
    let field_y = f16::from_f32(magnetic_field[1]);
    let field_z = f16::from_f32(magnetic_field[2]);

    buf[0] = sensor_id;
    buf[1..3].clone_from_slice(&field_x.to_le_bytes());
    buf[3..5].clone_from_slice(&field_y.to_le_bytes());
    buf[5..7].clone_from_slice(&field_z.to_le_bytes());

    // Must specify covariance length if on FD mode.
    let payload_size = if fd_mode {
        buf[7] = 0;
        Some(8)
    } else {
        None
    };
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
        payload_size,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/ahrs/1003.RawIMU.uavcan
pub fn publish_raw_imu(
    can: &mut Can_,
    timestamp: u64,  // 7-bytes, us.
    gyro: [f32; 3],  // f16. x, y, z. rad/s (Roll, pitch, yaw)
    accel: [f32; 3], // f16. x, y, z. m/s^2
    // todo: integration?
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::RawImu;

    let buf = unsafe { &mut BUF_RAW_IMU };

    buf[..7].clone_from_slice(&timestamp.to_le_bytes()[0..7]);
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

    let payload_size = if fd_mode {
        m_type.payload_size() + 1 // Due to no TCO on final cov array.
    } else {
        m_type.payload_size()
    };

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        Some(payload_size as usize),
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/navigation/2000.GlobalNavigationSolution.uavcan
pub fn publish_global_navigation_solution(
    can: &mut Can_,
    data: &GlobalNavSolution,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::GlobalNavigationSolution;

    let buf = unsafe { &mut BUF_GLOBAL_NAVIGATION_SOLUTION };

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    let transfer_id = TRANSFER_ID_GLOBAL_NAVIGATION_SOLUTION.fetch_add(1, Ordering::Relaxed);

    let payload_size = if fd_mode {
        m_type.payload_size() + 1 // Due to no TCO on final cov array.
    } else {
        m_type.payload_size()
    };

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        Some(payload_size as usize),
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1061.Auxiliary.uavcan
pub fn publish_gnss_aux(
    can: &mut Can_,
    data: &GnssAuxiliary,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    // let mut buf = [0; crate::find_tail_byte_index(PAYLOAD_SIZE_MAGNETIC_FIELD_STRENGTH2 as u8) + 1];
    let buf = unsafe { &mut BUF_GNSS_AUX };

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
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/equipment/gnss/1063.Fix2.uavcan
pub fn publish_fix2(
    can: &mut Can_,
    data: &FixDronecan,
    // todo: Covariances?
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let buf = unsafe { &mut BUF_FIX2 };

    let m_type = MsgType::Fix2;

    buf[..m_type.payload_size() as usize].clone_from_slice(&data.pack().unwrap());

    let transfer_id = TRANSFER_ID_FIX2.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/ardupilot/gnss/20003.Status.uavcan
/// Standard data type: uavcan.protocol.NodeStatus
/// Must be broadcast at intervals between 2 and 1000ms. FC firmware should
/// consider the node to be faulty if this is not received for 3s.
pub fn publish_ardupilot_gnss_status(
    can: &mut Can_,
    error_codes: u32,
    healthy: bool,
    status: u32,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::ArdupilotGnssStatus;

    let buf = unsafe { &mut BUF_ARDUPILOT_GNSS_STATUS };

    buf[0..4].copy_from_slice(&error_codes.to_le_bytes());

    // let status = status & 0b111_1111_1111_1111_1111_1111;

    // todo: Sort this out.
    // buf[4..8].copy_from_slice(&((healthy as u32) << 23 | status).to_le_bytes());

    buf[4] = 0x81; // armable and status 0. Having trouble with bit masks.

    let transfer_id = TRANSFER_ID_ARDUPILOT_GNSS_STATUS.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/dronecan/sensors/rc/1140.RCInput.uavcan
pub fn publish_rc_input(
    can: &mut Can_,
    status: u16,   // 1: valid. 2: failsafe.
    quality: u8,   // `quality` is scaled between 0 (no signal) and 255 (full signal)
    id: u8,        // u4
    rc_in: &[u16], // Includes control and aux channels. Each is 12-bits
    num_channels: u8,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let buf = unsafe { &mut BUF_RC_INPUT };

    const CHAN_SIZE_BITS: usize = 12;

    let quality = 255; // todo temp

    let m_type = MsgType::RcInput;

    buf[0..2].copy_from_slice(&status.to_le_bytes());
    buf[2] = quality;
    buf[3] = (id & 0b1111) << 4;

    let bits = buf.view_bits_mut::<Msb0>();

    let mut i_bits = 28; // (u16 + u8 + u4 status, quality, id)

    // 12 bits per channel.
    let mut rcin_len = crate::bit_size_to_byte_size(num_channels as usize * CHAN_SIZE_BITS);

    // For FD, add the length field of 6 bits.
    if fd_mode {
        bits[i_bits..6].store_le(rcin_len as u8);
        i_bits += 6;
        rcin_len += 1; // Perhaps not, depending?
    }

    for ch in rc_in {
        bits[i_bits..i_bits + CHAN_SIZE_BITS].store_be(*ch);

        // Bit level alignment mess sorted out by examining DC messages
        let nibble_0 = ch & 0xf;
        let nibble_1 = (ch >> 4) & 0xf;
        let nibble_2 = (ch >> 8) & 0xf;

        let re_arranged = (nibble_1 << 8) | (nibble_0 << 4) | &nibble_2;
        bits[i_bits..i_bits + CHAN_SIZE_BITS].store_be(re_arranged);

        i_bits += CHAN_SIZE_BITS;
    }

    let transfer_id = TRANSFER_ID_RC_INPUT.fetch_add(1, Ordering::Relaxed);

    let payload_len = m_type.payload_size() as usize + rcin_len;

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        Some(payload_len),
    )
}

pub fn publish_link_stats(
    can: &mut Can_,
    data: &LinkStats,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let buf = unsafe { &mut BUF_LINK_STATS };

    let m_type = MsgType::LinkStats;

    buf[0..m_type.payload_size() as usize].copy_from_slice(&data.to_bytes());

    let transfer_id = TRANSFER_ID_LINK_STATS.fetch_add(1, Ordering::Relaxed);

    broadcast(
        can,
        FrameType::Message,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        None,
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/dynamic_node_id/1.Allocation.uavcan
/// Send while the node is anonymous; requests an ID.
pub fn request_id_allocation_req(
    can: &mut Can_,
    data: &IdAllocationData,
    fd_mode: bool,
    node_id: u8,
) -> Result<(), CanError> {
    let buf = unsafe { &mut BUF_ID_ALLOCATION };

    let m_type = MsgType::IdAllocation;

    buf[0..m_type.payload_size() as usize].clone_from_slice(&data.to_bytes(fd_mode));

    let transfer_id = TRANSFER_ID_ID_ALLOCATION.fetch_add(1, Ordering::Relaxed);

    // 6 bytes of unique_id unless in the final stage; then 4.
    let len = if fd_mode {
        m_type.payload_size() as usize
    } else if data.stage == 2 {
        5
    } else {
        7
    };

    broadcast(
        can,
        FrameType::MessageAnon,
        m_type,
        node_id,
        transfer_id as u8,
        buf,
        fd_mode,
        Some(len),
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/param/11.GetSet.uavcan
/// We send this after receiving an (empty) GetSet request. We respond to the parameter
/// associated with the index requested. If the requested index doesn't match with a parameter
/// we have, we reply with an empty response. This indicates that we have passed all parameters.
/// The requester increments the index starting at 0 to poll parameters available.
pub fn publish_getset_resp(
    can: &mut Can_,
    data: &GetSetResponse,
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError> {
    let buf = unsafe { &mut BUF_GET_SET };

    // //Empty the buffer in case this message is shorter than the previous one; variable length.
    // *buf = [0; unsafe { BUF_GET_SET.len() }];

    let m_type = MsgType::GetSet;

    let len = data.to_bytes(buf, fd_mode);

    let transfer_id = TRANSFER_ID_GET_SET.fetch_add(1, Ordering::Relaxed);

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
        Some(len),
    )
}

/// https://github.com/dronecan/DSDL/blob/master/uavcan/protocol/4.GlobalTimeSync.uavcan
pub fn _handle_time_sync(
    can: &mut Can_,
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
    can: &mut Can_,
    payload: &[u8],
    fd_mode: bool,
    node_id: u8,
    requester_node_id: u8,
) -> Result<(), CanError> {
    let m_type = MsgType::Restart;

    let mut num_bytes = [0; 8];
    num_bytes[..5].clone_from_slice(payload);
    let magic_number = u64::from_le_bytes(num_bytes);

    let transfer_id = TRANSFER_ID_RESTART.fetch_add(1, Ordering::Relaxed);

    if magic_number != 0xAC_CE55_1B1E {
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
            &mut [0], // ie false; error
            fd_mode,
            None,
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
        None,
    )?;

    // let cp = unsafe { cortex_m::Peripherals::steal() };
    // todo: Not working.
    // cp.SCB.sys_reset();

    Ok(())
}

fn message_pending_handler(mailbox: Mailbox, header: TxFrameHeader, buf: &[u32]) {
    println!("Mailbox overflow!");
}

/// Function to help parse the nested result from CAN rx results
pub fn get_frame_info(
    rx_result: Result<ReceiveOverrun<RxFrameInfo>, nb::Error<Infallible>>,
) -> Result<RxFrameInfo, CanError> {
    // todo: This masks overruns currently.

    match rx_result {
        Ok(r) => match r {
            ReceiveOverrun::NoOverrun(frame_info) => Ok(frame_info),
            ReceiveOverrun::Overrun(frame_info) => Ok(frame_info),
        },
        Err(_) => Err(CanError::Hardware),
    }
}
