//! This module contains code that prepares a payload and ID in accordance with the DroneCAN
//! and Cyphal specifications.

//! [Relevant section of DroneCAN specification](https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/)
//! [Cyphal specification](https://opencyphal.org/specification/Cyphal_Specification.pdf)

//! Todo: Protocol version detection using toggle bit polarity.

#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};

use fdcan::{
    frame::{FrameFormat, TxFrameHeader},
    id::{ExtendedId, Id},
    FdCan, NormalOperationMode,
};

use stm32_hal2::can::Can;
use stm32_hal2::dma::DmaInterrupt::TransferComplete;

type Can_ = FdCan<Can, NormalOperationMode>;

// Node ID must be between 1 and 127 (?)
const SOURCE_NODE_ID: u8 = 69;

const DATA_FRAME_MAX_LEN_FD: u8 = 64;
const DATA_FRAME_MAX_LEN_LEGACY: u8 = 8;

static FD_MODE: AtomicBool = AtomicBool::new(true);
// todo: Protocol enum instead?
static USING_CYPHAL: AtomicBool = AtomicBool::new(false);

const CRC_POLY: u16 = 0x1021;
const SIGNATURE_POLY: u64 = 0x42F0_E1EB_A9EA_3693;
const SIGNATURE_MASK64: u64 = 0xFFFF_FFFF_FFFF_FFFF;

pub struct CanError {}

/// Accounts for the slight difference in CAN ID layout between DroneCAN and Cyphal.
#[derive(Clone, Copy)]
pub enum Protocol {
    DroneCan,
    Cyphal,
}

/// todo: QC spec.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum NodeHealth {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Critical = 3,
}

/// todo: QC spec.
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
    MultiStart,
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

    fn add_byte(&mut self, byte: u8) {
        self.value ^= (byte as u16) << 8;

        for _ in 0..8 {
            if (self.value & 0x8000) != 0 {
                self.value = (self.value << 1) ^ CRC_POLY;
            } else {
                self.value = self.value << 1;
            }
        }
    }

    fn add_bytes(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.add_byte(*byte)
        }
    }
}

/// Code for computing the data type signature for multi-frame transfers:
/// Adapted from https://dronecan.github.io/Specification/3._Data_structure_description_language/
struct Signature {
    pub crc: u64,
}

impl Signature {
    pub fn new(extend_from: Option<u64>) -> Self {
        let crc = match extend_from {
            Some(e) => (e & SIGNATURE_MASK64) ^ SIGNATURE_MASK64,
            None => SIGNATURE_MASK64,
        };

        Self { crc }
    }

    fn add(&mut self, data_bytes: &[u8]) {
        for byte in data_bytes {
            self.crc ^= ((*byte as u64) << 56) & SIGNATURE_MASK64;

            for _ in 0..8 {
                if self.crc & (1 << 64) != 0 {
                    self.crc = ((self.crc << 1) & SIGNATURE_MASK64) ^ SIGNATURE_POLY;
                } else {
                    self.crc <<= 1;
                }
            }
        }
    }

    pub fn value(&self) -> u64 {
        (self.crc & SIGNATURE_MASK64) ^ SIGNATURE_MASK64
    }
}

/// Determine the index for placing the tail byte of the payload. This procedure doesn't appear to
/// be defined in the docs, but is defined in official software implementations.
fn find_tail_byte_index(packet_len: u8) -> usize {
    const DATA_LENGTHS: [u8; 8] = [8, 12, 16, 20, 24, 32, 48, 64];

    for data_len in &DATA_LENGTHS {
        if packet_len <= *data_len {
            return *data_len as usize - 1;
        }
    }
    63
}

/// Write out packet to the CAN peripheral.
fn can_send(
    can: &mut Can_,
    can_id: u32,
    frame_data: &[u8],
    frame_data_len: u8,
) -> Result<(), CanError> {
    let max_frame_len = if FD_MODE.load(Ordering::Acquire) {
        DATA_FRAME_MAX_LEN_FD
    } else {
        DATA_FRAME_MAX_LEN_LEGACY
    };

    if frame_data_len > max_frame_len {
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
    let (priority_bits, priority_shift) = if on_cyphal {
        message_type_id = message_type_id & 0b1_1111_1111_1111;

        (priority as u32 & 0b111, 26)
    } else {
        (priority as u32 & 0b1_1111, 24)
    };

    // todo: for cyphal, bit 25 is 1 if a service transfer.

    // The `&` operations are to enforce the smaller-bit-count allowed than the `u8` datatype allows.

    let mut result = (priority_bits << priority_shift)
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
        TransferComponent::MultiStart => {
            end_of_transfer = 0;
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
        | ((transfer_id as u8) & 0b1_1111)
}

/// Handles splitting a payload into multiple frames, including DroneCAN and Cyphal requirements,
/// eg CRC and data type signature.
fn send_multiple_frames(
    can: &mut Can_,
    payload: &[u8],
    payload_len: u16,
    can_id: u32,
    transfer_id: u8,
) -> Result<(), CanError> {
    // See https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/,
    // "Multi-frame transfer" re requirements such as CRC and Toggle bit.
    let mut component = TransferComponent::MultiStart;

    let mut crc = TransferCrc::new();
    // "The Transfer CRC is computed from the transfer payload, prepended with a data type
    // signature, in little-endian byte order. The diagram below illustrates the input of the
    // transfer CRC function"

    // For info on the type signature, see https://dronecan.github.io/Specification/3._Data_structure_description_language/,
    // `Signature` section.
    let signature = Signature::new(None);
    // signature.add(sig_bytes); // todo: What is the signature computed with??
    crc.add_bytes(&signature.value().to_le_bytes());
    crc.add_bytes(payload);

    // todo: How do we apply the (u16) crc to the start of the message?
    // todo: Modify this for multi-frame FDcan xfers.
    let mut current_frame_buf = [0; 8];

    current_frame_buf[0..2].clone_from_slice(&crc.value.to_le_bytes());
    current_frame_buf[6..8].clone_from_slice(&payload[0..6]);

    let mut latest_i_sent = 6;

    // while latest_i_sent < payload_len {
    //     let mut payload_to_send = if FD_MODE.load(Ordering::Acquire) {
    //         &[0; DATA_FRAME_MAX_LEN_FD as usize][..]
    //     } else {
    //         &[0; DATA_FRAME_MAX_LEN_LEGACY as usize][..]
    //     };
    //
    //     let payload_to_send = if let TransferComponent::MultiEnd(_) = component {
    //         let tail_byte = make_tail_byte(component, transfer_id);
    //         payload_with_tail_byte[0..payload_len as usize].clone_from_slice(payload);
    //         payload_with_tail_byte[payload_len as usize] = tail_byte;
    //
    //         payload_with_tail_byte
    //     } else {
    //         payload
    //     };
    //
    //     can_send(can, can_id, payload_to_send, MAX_PAYLOAD_SIZE as u8)?;
    //
    //     match component {
    //         TransferComponent::MultiStart => {
    //             component = TransferComponent::MultiMid(false);
    //         }
    //         TransferComponent::MultiMid(toggle_prev) => {
    //             component = TransferComponent::MultiMid(toggle_prev);
    //             // todo: End if at end.
    //         }
    //         TransferComponent::MultiEnd(_) => break,
    //         TransferComponent::SingleFrame => panic!("Single frame transfer; code logic error"),
    //     }
    //
    //     latest_i_sent += 7; // 8 byte frame size, - 1 for each frame's tail byte.
    // }
    return Ok(());
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
    let can_id = make_can_id(priority, message_type_id, SOURCE_NODE_ID);

    // We subtract 1 to accomodate the tail byte.
    let frame_payload_len = if FD_MODE.load(Ordering::Acquire) {
        DATA_FRAME_MAX_LEN_FD
    } else {
        DATA_FRAME_MAX_LEN_LEGACY
    };
    // The transfer payload is up to 7 bytes for non-FD DRONECAN.
    // If data is longer than a single frame, set up a multi-frame transfer.
    // We subtract one to accomodate the tail byte.
    if payload_len > (frame_payload_len - 1) as u16 {
        return send_multiple_frames(can, payload, payload_len, can_id, transfer_id);
    }

    let tail_byte = make_tail_byte(TransferComponent::SingleFrame, transfer_id);
    let tail_byte_i = find_tail_byte_index(payload_len as u8);

    let mut payload_with_tail_byte = [0; DATA_FRAME_MAX_LEN_LEGACY as usize];

    // todo: Not sure how to make this work
    // let payload_with_tail_byte = if FD_MODE.load(Ordering::Acquire) {
    //     &mut [0; DATA_FRAME_MAX_LEN_FD as usize][..]
    // } else {
    //     &mut [0; DATA_FRAME_MAX_LEN_LEGACY as usize][..]
    // };

    payload_with_tail_byte[0..payload_len as usize].clone_from_slice(payload);
    payload_with_tail_byte[tail_byte_i] = tail_byte;

    can_send(can, can_id, &payload_with_tail_byte[0..tail_byte_i + 1], tail_byte_i as u8 + 1)
}
