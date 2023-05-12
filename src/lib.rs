//! This module contains code that prepares a payload and ID in accordance with the DroneCAN
//! and Cyphal specifications.

//! [Relevant section of DroneCAN specification](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
//! [Cyphal specification](https://opencyphal.org/specification/Cyphal_Specification.pdf)

//! Todo: Protocol version detection using toggle bit polarity.

#![no_std]

use core::{
    convert::Infallible,
    sync::atomic::{self, AtomicBool, Ordering},
};

use fdcan::{
    frame::{FrameFormat, RxFrameInfo, TxFrameHeader},
    id::{ExtendedId, Id},
    FdCan, Mailbox, NormalOperationMode, ReceiveOverrun,
};

use stm32_hal2::{can::Can, dma::DmaInterrupt::TransferComplete};

use defmt::println;

mod crc;
pub mod gnss;
pub mod messages;
pub mod types;

pub use messages::*;
pub use types::*;

type Can_ = FdCan<Can, NormalOperationMode>;

const DATA_FRAME_MAX_LEN_FD: u8 = 64;
const DATA_FRAME_MAX_LEN_LEGACY: u8 = 8;

// Enough for global navigation solution, our current largest payload, + crc.
// Static buffers for splitting up mult-frame payloads.
// static mut MULTI_FRAME_BUFS_LEGACY: [[u8; 8]; 10] = [
//     [0; 8],
//     [0; 8],
//     [0; 8],
//     [0; 8],
//     [0; 8],
//     [0; 8],
//     [0; 8],
//     [0; 8],
//     [0; 8],
//     [0; 8],
// ];

static mut MULTI_FRAME_BUFS_FD: [[u8; 64]; 10] = [
    [0; 64], [0; 64], [0; 64], [0; 64], [0; 64], [0; 64], [0; 64], [0; 64], [0; 64], [0; 64],
];

// static FD_MODE: AtomicBool = AtomicBool::new(true);
// todo: Protocol enum instead?
static USING_CYPHAL: AtomicBool = AtomicBool::new(false);

use crate::crc::{TransferCrc, Signature};

#[derive(Clone, Copy)]
pub enum CanError {
    CanHardware,
    PayloadSize,
    FrameSize,
    PayloadData,
}

/// This includes configuration data that we use on all nodes, and is not part of the official
/// DroneCAN spec.
pub struct ConfigCommon {
    /// Used to distinguish between multiple instances of this device. Stored in
    /// flash. Must be configured without any other instances of this device connected to the bus.
    pub node_id: u8,
    /// Ie, capable of 64-byte frame lens, vice 8.
    pub fd_mode: bool,
    /// Kbps. If less than 1_000, arbitration and data bit rate are the same.
    /// If greater than 1_000, arbitration bit rate stays at 1_000 due to protocol limits
    /// , while data bit rate is this value.
    pub can_bitrate: u16,
}

impl Default for ConfigCommon {
    fn default() -> Self {
        Self {
            // Between 1 and 127.
            node_id: 1,
            fd_mode: false,
            can_bitrate: 1_000,
        }
    }
}

impl ConfigCommon {
    pub fn from_bytes(buf: &[u8]) -> Self {
        Self {
            node_id: buf[0],
            fd_mode: buf[1] != 0,
            can_bitrate: u16::from_le_bytes(buf[2..4].try_into().unwrap()),
        }
    }

    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_CONFIG_COMMON] {
        let mut result = [0; PAYLOAD_SIZE_CONFIG_COMMON];

        result[0] = self.node_id;
        result[1] = self.fd_mode as u8;
        result[2..4].clone_from_slice(&self.can_bitrate.to_le_bytes());

        result
    }
}

/// Accounts for the slight difference in CAN ID layout between DroneCAN and Cyphal.
#[derive(Clone, Copy)]
pub enum Protocol {
    DroneCan,
    Cyphal,
}

#[derive(Clone, Copy)]
/// Distinguish single and multi-part transfers. The inner value is the toggle value of the
/// previous frame.
pub enum TransferComponent {
    SingleFrame,
    MultiStart,
    MultiMid(bool),
    MultiEnd(bool),
}

/// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
/// Valid values for priority range from 0 to 31, inclusively, where 0 corresponds to highest priority
/// (and 31 corresponds to lowest priority).
/// In multi-frame transfers, the value of the priority field must be identical for all frames of the transfer.
///
/// Cyphal: (Transfer Priority spec section)[https://opencyphal.org/specification/Cyphal_Specification.pdf]:
/// Valid values for transfer priority range from 0 to 7, inclusively, where 0 corresponds to the highest priority, and
/// 7 corresponds to the lowest priority (according to the CAN bus arbitration policy).
/// In multi-frame transfers, the value of the priority field shall be identical for all frames of the transfer.
///
/// We use the Cyphal specification, due to its specificity.
#[derive(Clone, Copy)]
pub enum MsgPriority {
    Exceptional,
    Immediate,
    Fast,
    High,
    Nominal,
    Low,
    Slow,
    Optional,
    Other(u8),
}

impl MsgPriority {
    pub fn val(&self) -> u8 {
        match self {
            Self::Exceptional => 0,
            Self::Immediate => 1,
            Self::Fast => 2,
            Self::High => 3,
            Self::Nominal => 4,
            Self::Low => 5,
            Self::Slow => 6,
            Self::Optional => 7,
            Self::Other(val) => *val,
        }
    }

    pub fn from_val(val: u8) -> Self {
        match val {
            0 => Self::Exceptional,
            1 => Self::Immediate,
            2 => Self::Fast,
            3 => Self::High,
            4 => Self::Nominal,
            5 => Self::Low,
            6 => Self::Slow,
            _ => Self::Other(val),
        }
    }
}

/// Construct a CAN ID. See DroneCAN Spec, CAN bus transport layer doc, "ID field" section.
/// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
/// "DroneCAN uses only CAN 2.0B frame format (29-bit identifiers).
/// DroneCAN can share the same bus with other protocols based on CAN 2.0A (11-bit identifiers)."
///  This means we always use extended Id.
///
/// "In the case of a message broadcast transfer, the CAN ID field of every frame of the transfer will contain the following fields:
/// - Priority (5 bits)
/// - Message type ID: Data type ID of the encoded message (16 bits)
/// - Service not message: Always 0. 1 bit.
/// - Source nod ID.Can be 1-27. 7 bits.
pub struct CanId {
    // Valid values for priority range from 0 to 31, inclusively, where 0 corresponds to highest
    // priority (and 31 corresponds to lowest priority).
    // In multi-frame transfers, the value of the priority field must be identical for all frames of the transfer.
    // (We use the enum which constrains to Cyphal's values).
    pub priority: MsgPriority,
    // Valid values of message type ID range from 0 to 65535, inclusively.
    // Valid values of message type ID range for anonymous message transfers range from 0 to 3, inclusively.
    // This limitation is due to the fact that only 2 lower bits of the message type ID are available in this case.
    pub message_type_id: u16, // Valid values of Node ID range from 1 to 127, inclusively.
    // Note that Node ID is represented by a 7-bit unsigned integer value and that zero is reserved,
    // to represent either an unknown node or all nodes, depending on the context.
    pub source_node_id: u8,
    pub service: bool,
}

impl CanId {
    pub fn value(&self) -> u32 {
        let on_cyphal = USING_CYPHAL.load(Ordering::Acquire);

        let mut message_type_id = self.message_type_id;
        // Cyphal restricts message id to 13 bits.
        let (priority_bits, priority_shift) = if on_cyphal {
            message_type_id = message_type_id & 0b1_1111_1111_1111;

            (self.priority.val() as u32 & 0b111, 26)
        } else {
            (self.priority.val() as u32 & 0b1_1111, 24)
        };

        // todo: for cyphal, bit 25 is 1 if a service transfer.

        // The `&` operations are to enforce the smaller-bit-count allowed than the `u8` datatype allows.

        let mut result = (priority_bits << priority_shift)
            | ((self.message_type_id as u32) << 8)
            | ((self.service as u32) << 7)
            | ((self.source_node_id & 0b111_1111) as u32);

        // On cyphal, Bits 21 and 22 are 1 when transmitting.
        if on_cyphal {
            result |= 0b11 << 21;
        }

        result
    }

    /// Pull priority, type id, and source node id from the CAN id.
    /// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
    /// todo: Currently hard-coded for DroneCAN.
    pub fn from_value(val: u32) -> Self {
        let source_node_id = val as u8 & 0b111_1111;

        let message_type_id = (val >> 8) as u16;

        let priority = MsgPriority::from_val((val >> 24) as u8 & 0b1_1111);

        Self {
            priority,
            message_type_id,
            source_node_id,
            service: false, // todo - hard-coded
        }
    }
}

pub struct TailByte {
    pub start_of_transfer: bool,
    pub end_of_transfer: bool,
    pub toggle: bool,
    pub transfer_id: u8,
}

impl TailByte {
    pub fn value(&self) -> u8 {
        // (DroneCAN):
        // For single-frame transfers, the value of this field is always 1.
        // For multi-frame transfers, the value of this field is 1 if the current frame is the first
        // frame of the transfer, and 0 otherwise.
        ((self.start_of_transfer as u8) << 7)
            // For single-frame transfers, the value of this field is always 1.
            // For multi-frame transfers, the value of this field is 1 if the current frame is the last
            // frame of the transfer, and 0 otherwise.
            | ((self.end_of_transfer as u8) << 6)
            // For single-frame transfers, the value of this field is always 0.
            // For multi-frame transfers, this field contains the value of the toggle bit. As specified
            // above this will alternate value between frames, starting at 0 for the first frame.
            // Cyphal note: Behavior of this bit is reversed from DroneCAN. Ie it's 1 for single-frame,
            // and first of multi-frame.
            | ((self.toggle as u8) << 5)
            | (self.transfer_id & 0b1_1111)
    }

    /// Pull start_of_transfer, end_of_transfer, and toggle flags from the tail byte. We don't
    /// convert to TransferComponent due to ambiguities in Single vs multi-mid.
    /// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
    /// todo: Currently hard-coded for DroneCAN.
    pub fn from_value(val: u8) -> Self {
        let transfer_id = val & 0b1_1111;
        let toggle = (val >> 5) & 1;
        let end = (val >> 6) & 1;
        let start = (val >> 7) & 1;

        Self {
            transfer_id,
            toggle: toggle != 0,
            end_of_transfer: end != 0,
            start_of_transfer: start != 0,
        }
    }
}

/// Determine the index for placing the tail byte of the payload. This procedure doesn't appear to
/// be defined in the docs, but is defined in official software implementations.
///
/// Const fn for use in creating statically-sized buffers.
/// It appears this is only valid for FD; the answer is always 7 on classic CAN.
const fn find_tail_byte_index(payload_len: u8) -> usize {
    // We take this comparatively verbose approach vice the loop below to be compatible
    // with const fns.

    // Less than, not equals, since if it's len 8, for example, we need to leave room for the
    // tail byte at index 7.
    if payload_len < 8 {
        return 7;
    }
    if payload_len < 12 {
        return 11;
    }
    if payload_len < 16 {
        return 15;
    }
    if payload_len < 20 {
        return 19;
    }
    if payload_len < 24 {
        return 23;
    }
    if payload_len < 32 {
        return 31;
    }
    if payload_len < 48 {
        return 47;
    }

    63
}

fn message_pending_handler(mailbox: Mailbox, header: TxFrameHeader, buf: &[u32]) {
    println!("Mailbox overflow!");
}

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
        bit_rate_switching: true,
        marker: None,
    };

    // This wait appears to be required, pending handling using a queue in the callback.
    while !can.is_transmitter_idle() {}

    // Not sure if this helps or is required etc.
    // atomic::compiler_fence(Ordering::SeqCst);

    match can.transmit_preserve(frame_header, frame_data, &mut message_pending_handler) {
        Ok(_) => Ok(()),
        Err(e) => Err(CanError::CanHardware),
    }
}

/// Construct a tail byte. See DroneCAN Spec, CAN bus transport layer.
/// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
/// "The Data field of the CAN frame is shared between the following fields:
/// - Transfer payload
/// - 0 tail byte, which contains the following fields. Start of transfer (1 bit), End of transfer (1 bit)
/// toggle bit (1 bit), Transfer id (5 bits)."
/// Cyphal works in a similar way, but with ar reversed toggle bit.
fn make_tail_byte(transfer_comonent: TransferComponent, transfer_id: u8) -> TailByte {
    // Defaults for a single-frame transfer using the DroneCAN spec..
    let mut start_of_transfer = true;
    let mut end_of_transfer = true;
    let mut toggle = false;

    match transfer_comonent {
        TransferComponent::MultiStart => {
            end_of_transfer = false;
        }
        TransferComponent::MultiMid(toggle_prev) => {
            start_of_transfer = false;
            end_of_transfer = false;
            toggle = !toggle_prev;
        }
        TransferComponent::MultiEnd(toggle_prev) => {
            start_of_transfer = false;
            toggle = !toggle_prev;
        }
        _ => (),
    }

    if USING_CYPHAL.load(Ordering::Acquire) {
        toggle = !toggle;
    }

    TailByte {
        start_of_transfer,
        end_of_transfer,
        toggle,
        transfer_id,
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
    // data_type_signature: u64,
    base_crc: u16,
) -> Result<(), CanError> {
    let frame_payload_len = if fd_mode {
        DATA_FRAME_MAX_LEN_FD as usize
    } else {
        DATA_FRAME_MAX_LEN_LEGACY as usize
    };

    let mut crc = TransferCrc::new(base_crc);
    crc.add_payload(payload, payload_len as usize, frame_payload_len);

    // We use slices of the FD buf, even for legacy frames, to keep code simple.
    let bufs = unsafe { &mut MULTI_FRAME_BUFS_FD };

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

    // println!("\nWHOle payload: {:?}", payload);

    // println!("\n\nFirst frame");
    // println!("PL this frame: {}", payload_len_this_frame);
    // println!("Latest sent i: {}", -1);

    let mut latest_i_sent = payload_len_this_frame - 1;
    // println!("Payload: {:?}", bufs[active_frame][..frame_payload_len]);

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

        // println!("Payload: {:?}", bufs[active_frame][..frame_payload_len]);

        // todo: You may need to cut the active fram eoff early for the last frame.
        can_send(
            can,
            can_id,
            // &bufs[active_frame][..tail_byte_i + 1],
            &bufs[active_frame][..tail_byte_i as usize + 1],
            tail_byte_i as u8 + 1,
            fd_mode,
        )?;

        latest_i_sent += payload_len_this_frame; // 8 byte frame size, - 1 for each frame's tail byte.
        active_frame += 1;
    }

    return Ok(());
}

/// Send a DroneCAN "broadcast" message. See [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
/// Should be broadcast at interval between 2 and 1000ms.
/// Note: The payload must be static, and include space for the tail byte.
pub fn broadcast(
    can: &mut Can_,
    msg_type: MsgType,
    // priority: MsgPriority,
    // message_type_id: u16,
    source_node_id: u8,
    transfer_id: u8,
    // mutable for adding tail byte.
    // payload: &'static mut [u8],
    payload: &mut [u8],
    // payload_len: u16,
    fd_mode: bool,
) -> Result<(), CanError> {
    // todo: Accept a message type instead that includes priority, message type id, and payload len?

    let can_id = CanId {
        priority: msg_type.priority(),
        message_type_id: msg_type.id(),
        source_node_id,
        service: false, // todo: Customizable.
    };

    let payload_len = msg_type.payload_size();

    // We subtract 1 to accomodate the tail byte.
    // let frame_payload_len = if FD_MODE.load(Ordering::Acquire) {
    let frame_payload_len = if fd_mode {
        DATA_FRAME_MAX_LEN_FD
    } else {
        DATA_FRAME_MAX_LEN_LEGACY
    };

    // The transfer payload is up to 7 bytes for non-FD DRONECAN.
    // If data is longer than a single frame, set up a multi-frame transfer.
    // We subtract one to accomodate the tail byte.
    if payload_len as u16 > (frame_payload_len - 1) as u16 {

    // For info on the type signature, see https://dronecan.github.io/Specification/3._Data_structure_description_language/,
    // `Signature` section.
    // Explicit algorithm, as the docs here are wanting:
    // https://github.com/dronecan/pydronecan/blob/master/dronecan/dsdl/parser.py#L309


    // let signature = Signature::new(None);
    // let signature = msg_type.signature();
    // signature.add(sig_bytes); // todo: What is the signature computed with??
        // todo: You probably need a dedicated module for crc and signature.

        return send_multiple_frames(
            can,
            payload,
            payload_len as u16,
            can_id.value(),
            transfer_id,
            fd_mode,
            // signature,
            msg_type.base_crc(),
        );
    }

    let tail_byte = make_tail_byte(TransferComponent::SingleFrame, transfer_id);
    let tail_byte_i = find_tail_byte_index(payload_len as u8);

    payload[tail_byte_i] = tail_byte.value();

    // if message_type_id == messages::DATA_TYPE_ID_NODE_STATUS {
    //     println!("Node status. tail i: {}, Tail: s{} e{} val{}",
    //              tail_byte_i, tail_byte.start_of_transfer, tail_byte.end_of_transfer, tail_byte.value());
    //
    //     println!("PL: {:?}", payload);
    // }

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
        Err(_) => Err(CanError::CanHardware),
    }
}

pub fn get_tail_byte(payload: &[u8], frame_len: u8) -> Result<TailByte, CanError> {
    let i = find_tail_byte_index(frame_len);

    if i > payload.len() {
        return Err(CanError::PayloadSize);
    }

    Ok(TailByte::from_value(payload[i]))
}

// todo: You need a fn to get a payload from multiple frames
