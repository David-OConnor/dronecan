//! This module contains code that prepares a payload and ID in accordance with the DroneCAN
//! and Cyphal specifications.

//! [Relevant section of DroneCAN specification](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
//! [Cyphal specification](https://opencyphal.org/specification/Cyphal_Specification.pdf)

#![no_std]

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use fdcan::{
    frame::{FrameFormat, TxFrameHeader},
    id::{ExtendedId, Id},
    FdCan, NormalOperationMode,
};

use stm32_hal2::can::Can;

type Can_ = FdCan<Can, NormalOperationMode>;

// Node ID must be between 1 and 127 (?)
const SOURCE_NODE_ID: u8 = 69;

const DATA_FRAME_MAX_LEN_FD: u8 = 64;
const DATA_FRAME_MAX_LEN_LEGACY: u8 = 8;

const DATA_TYPE_ID_NODE_STATUS: u16 = 341;
const DATA_TYPE_ID_CH_DATA: u16 = 1020;
const DATA_TYPE_ID_LINK_STATS: u16 = 1021;

static TRANSFER_ID_NODE_STATUS: AtomicUsize = AtomicUsize::new(0);
static TRANSFER_ID_CH_DATA: AtomicUsize = AtomicUsize::new(0);
static TRANSFER_ID_LINK_STATS: AtomicUsize = AtomicUsize::new(0);

static FD_MODE: AtomicBool = AtomicBool::new(true);
// todo: Protocol enum instead?
static USING_CYPHAL: AtomicBool = AtomicBool::new(false);

// Used to create a buffer with tail byte.
// todo: How does DRONECAN handle this for FD? Open question. ie where the tail byte goes.
const MAX_PAYLOAD_SIZE: usize = 23; // todo: T. RC channels of CRSF + 1 for trail byte.

pub struct CanError {}

/// Accounts for the slight difference in CAN ID layout between DroneCAN and Cyphal.
/// #[derive(Clone, Copy)]
pub enum Protocol {
    DroneCan,
    Cyphal,
}

/// Defined for the standard data type uavcan.protocol.NodeStatus
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum NodeHealth {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Critical = 3,
}

/// Defined for the standard data type uavcan.protocol.NodeStatus
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum NodeMode {
    Operational = 0,
    Initialization = 1,
    Maintenance = 2,
    SoftwareUpdate = 3,
    Offline = 7,
}

#[derive(Clone, Copy)]
/// Distinguish single and multi-part transfers. The inner value is the toggle value of the
/// previous frame.
pub enum TransferComponent {
    SingleFrame,
    MultiStart(bool),
    MultiMid(bool),
    MultiEnd(bool),
}

#[derive(Clone, Copy)]
#[repr(u8)]
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
pub enum MsgPriority {
    Exceptional = 0,
    Immediate = 1,
    Fast = 2,
    High = 3,
    Nominal = 4,
    Low = 5,
    Slow = 6,
    Optional = 7,
}

/// Code for computing CRC for multi-frame transfers:
/// Adapted from https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
struct TransferCrc {
    pub value: u16,
}

impl TransferCrc {
    pub fn new() -> Self {
        Self { value: 0xffff }
    }

    pub fn add_byte(&mut self, byte: u8) {
        self.value ^= (byte as u16) << 8;

        for _ in 0..8 {
            if (self.value & 0x8000) != 0 {
                self.value = (self.value << 1) ^ 0x1021;
            } else {
                self.value = self.value << 1;
            }
        }
    }

    fn add_multiple_bytes(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.add_byte(*byte)
        }
    }
}

/// Write out packet to the CAN peripheral.
fn can_send(
    can: &mut Can_,
    can_id: u32,
    frame_data: &[u8],
    frame_data_len: u8,
) -> Result<(), CanError> {
    let max_len = if FD_MODE.load(Ordering::Acquire) {
        DATA_FRAME_MAX_LEN_FD
    } else {
        DATA_FRAME_MAX_LEN_LEGACY
    };

    if frame_data_len > max_len {
        return Err(CanError {});
    }

    const CAN_EFF_FLAG: u32 = 0; // todo: What is this? from DroneCAN simple example.

    let frame_format = if FD_MODE.load(Ordering::Acquire) {
        FrameFormat::Fdcan
    } else {
        FrameFormat::Standard
    };

    let id = Id::Extended(ExtendedId::new(can_id | CAN_EFF_FLAG).unwrap());

    let frame_header = TxFrameHeader {
        len: frame_data_len,
        frame_format,
        id,
        bit_rate_switching: true, // todo?
        marker: None,
    };

    let _tx_result = can.transmit(frame_header, frame_data).unwrap();

    Ok(())
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
fn make_can_id(
    // Valid values for priority range from 0 to 31, inclusively, where 0 corresponds to highest
    // priority (and 31 corresponds to lowest priority).
    // In multi-frame transfers, the value of the priority field must be identical for all frames of the transfer.
    // (We use the enum which constrains to Cyphal's values).
    priority: MsgPriority,
    // Valid values of message type ID range from 0 to 65535, inclusively.
    // Valid values of message type ID range for anonymous message transfers range from 0 to 3, inclusively.
    // This limitation is due to the fact that only 2 lower bits of the message type ID are available in this case.
    mut message_type_id: u16,
    // Valid values of Node ID range from 1 to 127, inclusively.
    // Note that Node ID is represented by a 7-bit unsigned integer value and that zero is reserved,
    // to represent either an unknown node or all nodes, depending on the context.
    source_node_id: u8,
) -> u32 {
    let on_cyphal = USING_CYPHAL.load(Ordering::Acquire);
    // Cyphal restricts message id to 13 bits.
    let priority_shift = if on_cyphal {
        message_type_id = message_type_id & 0b1_1111_1111_1111;
        26
    } else {
        24
    };

    // todo: for cyphal, bit 25 is 1 if a service transfer.

    // The `&` operations are to enforce the smaller-bit-count allowed than the `u8` datatype allows.

    let mut result = ((priority as u32) << priority_shift)
        | ((message_type_id as u32) << 8)
        | ((source_node_id & 0b111_1111) as u32);

    // On cyphal, Bits 21 and 22 are 1 when transmitting.
    if on_cyphal {
        result |= 0b11 << 21;
    }

    result
}

/// Construct a tail byte. See DroneCAN Spec, CAN bus transport layer.
/// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
/// "The Data field of the CAN frame is shared between the following fields:
/// - Transfer payload
/// - 0 tail byte, which contains the following fields. Start of transfer (1 bit), End of transfer (1 bit)
/// toggle bit (1 bit), Transfer id (5 bits)."
/// Cyphal works in a similar way, but with ar reversed toggle bit.
fn make_tail_byte(transfer_comonent: TransferComponent, transfer_id: u8) -> u8 {
    // Defaults for a single-frame transfer using the DroneCAN spec..
    let mut start_of_transfer = 1;
    let mut end_of_transfer = 1;
    let mut toggle = 0;

    match transfer_comonent {
        TransferComponent::MultiStart(toggle_prev) => {
            end_of_transfer = 0;
            toggle = !(toggle_prev as u8);
        }
        TransferComponent::MultiMid(toggle_prev) => {
            start_of_transfer = 0;
            end_of_transfer = 0;
            toggle = !(toggle_prev as u8);
        }
        TransferComponent::MultiEnd(toggle_prev) => {
            start_of_transfer = 0;
            toggle = !(toggle_prev as u8);
        }
        _ => (),
    }

    if USING_CYPHAL.load(Ordering::Acquire) {
        toggle = !toggle;
    }

    // (DroneCAN):
    // For single-frame transfers, the value of this field is always 1.
    // For multi-frame transfers, the value of this field is 1 if the current frame is the first
    // frame of the transfer, and 0 otherwise.
    (start_of_transfer << 7)
        // For single-frame transfers, the value of this field is always 1.
        // For multi-frame transfers, the value of this field is 1 if the current frame is the last
        // frame of the transfer, and 0 otherwise.
        | (end_of_transfer << 6)
        // For single-frame transfers, the value of this field is always 0.
        // For multi-frame transfers, this field contains the value of the toggle bit. As specified
        // above this will alternate value between frames, starting at 0 for the first frame.
        // Cyphal note: Behavior of this bit is reversed from DroneCAN. Ie it's 1 for single-frame,
        // and first of multi-frame.
        | (toggle << 5)
        | (transfer_id & 0b1_1111)
}

/// Send a DroneCAN "broadcast" message. See [The DroneCAN spec, transport layer page](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
pub fn broadcast(
    can: &mut Can_,
    priority: MsgPriority,
    message_type_id: u16,
    transfer_id: u8,
    payload: &[u8],
    payload_len: u16,
) -> Result<(), CanError> {
    // The transfer payload is up to 7 bytes for non-FD DRONECAN.
    if payload_len > 63 {
        // todo: FD can DRONECAN limit??
        // We don't support multi-frame transfers at this point.
        // todo: Need multi-frame support for non-FD.

        // See https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/,
        // "Multi-frame transfer" re requirements such as CRC and Toggle bit.
        return Err(CanError {});
    }

    let can_id = make_can_id(priority, message_type_id, SOURCE_NODE_ID);
    let tail_byte = make_tail_byte(TransferComponent::SingleFrame, transfer_id);

    let mut payload_with_tail_byte = [0; MAX_PAYLOAD_SIZE];
    payload_with_tail_byte[0..payload_len as usize].clone_from_slice(payload);

    // todo: For now, we append the tail byte after the payload data on FD. Is this right?
    payload_with_tail_byte[payload_len as usize] = tail_byte;

    can_send(can, can_id, &payload_with_tail_byte, MAX_PAYLOAD_SIZE as u8)
}
