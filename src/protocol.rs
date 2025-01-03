use core::{
    convert::TryInto,
    sync::atomic::{AtomicUsize, Ordering},
};

#[cfg(feature = "hal")]
use hal::rng;

use crate::{CanBitrate, CanError, PAYLOAD_SIZE_CONFIG_COMMON};

const MAX_RX_FRAMES_LEGACY: usize = 24;
const MAX_RX_FRAMES_FD: usize = 3;

// This one may be accessed by applications directly.
pub static mut MULTI_FRAME_BUFS_RX_LEGACY: [[u8; 8]; MAX_RX_FRAMES_LEGACY] =
    [[0; 8]; MAX_RX_FRAMES_LEGACY];
pub static mut MULTI_FRAME_BUFS_RX_FD: [[u8; 64]; MAX_RX_FRAMES_FD] = [[0; 64]; MAX_RX_FRAMES_FD];

// todo: DRY from braodcast
pub(crate) const DATA_FRAME_MAX_LEN_FD: u8 = 64;
pub(crate) const DATA_FRAME_MAX_LEN_LEGACY: u8 = 8;

// Used to identify the next frame to load.
pub static RX_FRAME_I: AtomicUsize = AtomicUsize::new(0);

/// This includes configuration data that we use on all nodes, and is not part of the official
/// DroneCAN spec.
pub struct ConfigCommon {
    /// Used to distinguish between multiple instances of this device. Stored in
    /// flash. If dyanmic id allocation is enabled, this is overwritten, but passed as
    /// the desired ID. If not, it is the node ID.
    /// Defaults to 69.
    pub node_id: u8,
    /// If true, the `node_id` field is a desired ID; get ID from an allocator.
    /// if false, hard-set the node id field. Defaults to `true`.
    pub dynamic_id_allocation: bool,
    /// Ie, capable of 64-byte frame lens, vice 8.
    pub fd_mode: bool,
    /// Kbps. If less than 1_000, arbitration and data bit rate are the same.
    /// If greater than 1_000, arbitration bit rate stays at 1_000 due to protocol limits
    /// while data bit rate is this value.
    pub can_bitrate: CanBitrate,
}

impl Default for ConfigCommon {
    fn default() -> Self {
        Self {
            // Between 1 and 127. Initialize to 0; this is expected by AP and
            // Px4, where id is assigned through a node ID server.
            node_id: 69,
            dynamic_id_allocation: true,
            fd_mode: false,
            can_bitrate: CanBitrate::default(),
        }
    }
}

impl ConfigCommon {
    pub fn from_bytes(buf: &[u8]) -> Self {
        Self {
            node_id: buf[0],
            dynamic_id_allocation: buf[1] != 0,
            fd_mode: buf[2] != 0,
            can_bitrate: buf[3].try_into().unwrap_or_default(),
        }
    }

    pub fn to_bytes(&self) -> [u8; PAYLOAD_SIZE_CONFIG_COMMON] {
        let mut result = [0; PAYLOAD_SIZE_CONFIG_COMMON];

        result[0] = self.node_id;
        result[1] = self.dynamic_id_allocation as u8;
        result[2] = self.fd_mode as u8;
        result[3] = self.can_bitrate as u8;

        result
    }
}

#[derive(Clone, Copy)]
pub enum Protocol {
    DroneCan,
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

/// Differentiates between messages and services
#[derive(PartialEq)]
pub enum FrameType {
    Message,
    MessageAnon,
    Service(ServiceData),
}

/// Data present in services, but not messages.
#[derive(PartialEq, Copy, Clone)]
pub struct ServiceData {
    pub dest_node_id: u8, // 7 bits
    pub req_or_resp: RequestResponse,
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
pub const fn find_tail_byte_index(payload_len: u8) -> usize {
    // We take this comparatively verbose approach vice the loop below to be compatible
    // with const fns.

    if payload_len < 8 {
        return payload_len as usize;
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

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum RequestResponse {
    Request = 1,
    Response = 0,
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
    pub priority: MsgPriority,
    /// Valid values of message type ID range from 0 to 65535, inclusively.
    /// Valid values of message type ID range for anonymous message transfers range from 0 to 3, inclusively.
    /// This limitation is due to the fact that only 2 lower bits of the message type ID are available in this case.\
    /// Note: This is `service_type_id` for services.
    pub type_id: u16, // Valid values of Node ID range from 1 to 127, inclusively.
    // Note that Node ID is represented by a 7-bit unsigned integer value and that zero is reserved,
    // to represent either an unknown node or all nodes, depending on the context.
    pub source_node_id: u8,
    pub frame_type: FrameType,
}

impl CanId {
    pub fn value(&self) -> u32 {
        let mut message_type_id = self.type_id;
        let (priority_bits, priority_shift) = (self.priority.val() as u32 & 0b1_1111, 24);

        // The `&` operations are to enforce the smaller-bit-count allowed than the datatype allows.

        let mut frame_type_val = 0;

        if let FrameType::Service(_) = self.frame_type {
            frame_type_val = 1;
        }

        let mut result = (priority_bits << priority_shift)
            | (frame_type_val << 7)
            | ((self.source_node_id & 0b111_1111) as u32);

        // The middle 16 bits vary depending on frame type.
        match &self.frame_type {
            FrameType::Message => {
                result |= (self.type_id as u32) << 8;
            }
            // FrameType::MessageAnon(discriminator) => {
            FrameType::MessageAnon => {
                // 14-bit discriminator. Discriminator should be random. We use the RNG peripheral.
                // Once set, we keep it.
                #[cfg(feature = "hal")]
                let discriminator = (rng::read() & 0b11_1111_1111_1111) as u32;
                #[cfg(not(feature = "hal"))]
                let discriminator = 335;

                result |= (discriminator << 10) | ((self.type_id & 0b11) as u32) << 8;
            }
            FrameType::Service(service_data) => {
                result |= ((self.type_id as u32) << 16)
                    | ((service_data.req_or_resp as u8 as u32) << 15)
                    | (((service_data.dest_node_id & 0b111_1111) as u32) << 8)
            }
        }

        result
    }

    /// Pull priority, type id, and source node id from the CAN id.
    /// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
    /// todo: Currently hard-coded for DroneCAN.
    pub fn from_value(val: u32) -> Self {
        let source_node_id = val as u8 & 0b111_1111;

        let priority = MsgPriority::from_val((val >> 24) as u8 & 0b1_1111);

        let mut frame_type = FrameType::Message;
        let mut type_id = 0;

        match (val >> 7) & 1 {
            0 => match source_node_id {
                0 => {
                    frame_type = FrameType::MessageAnon;
                    type_id = ((val >> 8) & 0b11) as u16;
                }
                _ => {
                    type_id = (val >> 8) as u16;
                }
            },
            1 => {
                frame_type = FrameType::Service(ServiceData {
                    dest_node_id: ((val >> 8) & 0b111_1111) as u8,
                    req_or_resp: if (val >> 15) == 1 {
                        RequestResponse::Request
                    } else {
                        RequestResponse::Response
                    },
                });
                type_id = ((val >> 16) & 0xff) as u16;
            }
            _ => unreachable!(),
        }

        Self {
            priority,
            type_id,
            source_node_id,
            frame_type,
        }
    }
}

pub fn get_tail_byte(payload: &[u8], frame_len: u8) -> Result<TailByte, CanError> {
    let i = find_tail_byte_index(frame_len);

    if i > payload.len() {
        return Err(CanError::PayloadSize);
    }

    Ok(TailByte::from_value(payload[i]))
}

/// Construct a tail byte. See DroneCAN Spec, CAN bus transport layer.
/// https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
/// "The Data field of the CAN frame is shared between the following fields:
/// - Transfer payload
/// - 0 tail byte, which contains the following fields. Start of transfer (1 bit), End of transfer (1 bit)
/// toggle bit (1 bit), Transfer id (5 bits)."
pub fn make_tail_byte(transfer_comonent: TransferComponent, transfer_id: u8) -> TailByte {
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

    TailByte {
        start_of_transfer,
        end_of_transfer,
        toggle,
        transfer_id,
    }
}

/// Handle a new received frame. This returns true, and fills `payload` with the assembled payload
/// if this is the last frame in a multi-frame transfer, or if it's a single-frame transfer.
/// Returns false and doesn't modify the buffer if part of a multi-frame payload, but adjusts
/// some atomic state variables as-required.
pub fn handle_frame_rx(rx_buf: &[u8], payload: &mut [u8], fd_mode: bool) -> bool {
    let frame_i = RX_FRAME_I.fetch_add(1, Ordering::Relaxed);

    let mut single_frame_payload = false;
    let mut rx_frame_complete = false;

    let frame_len = if fd_mode {
        DATA_FRAME_MAX_LEN_FD
    } else {
        DATA_FRAME_MAX_LEN_LEGACY
    } as usize;

    let mut tail_byte_i = frame_len - 1;

    // If the value at the tail byte index is 0, this is the final frame in the sequence (or a
    // single-frame payload).
    if rx_buf[tail_byte_i] == 0 {
        // The last frame. Locate and parse the tail byte, so we know if this was a single-frame
        // transfer.
        for i in 0..frame_len {
            // Read from the end, towards the front.
            let i_word = (frame_len - 1) - i;
            if rx_buf[i_word] != 0 {
                // We update the tail byte index so we know not to include it in the compiled payload
                // if this is the last frame.
                tail_byte_i = i_word;
                let tail_byte = TailByte::from_value(rx_buf[tail_byte_i]);
                if tail_byte.end_of_transfer {
                    rx_frame_complete = true;
                    if tail_byte.start_of_transfer {
                        single_frame_payload = true;
                    }
                    break;
                } else {
                    break;
                }
            }
        }
    } else {
        // Check the end of the frame for the tail byte.
        // todo: DRY with above tail byte check.
        let tail_byte = TailByte::from_value(rx_buf[tail_byte_i]);
        if tail_byte.end_of_transfer {
            rx_frame_complete = true;
            if tail_byte.start_of_transfer {
                single_frame_payload = true;
            }
        }
    }

    // If we aren't at the end of the frame, load the data into our static buffer, and return false.
    if !rx_frame_complete {
        unsafe {
            if fd_mode {
                if frame_i < MULTI_FRAME_BUFS_RX_FD.len() {
                    MULTI_FRAME_BUFS_RX_FD[frame_i][..frame_len]
                        .clone_from_slice(&rx_buf[..frame_len]);
                }
            } else if frame_i < MULTI_FRAME_BUFS_RX_LEGACY.len() {
                MULTI_FRAME_BUFS_RX_LEGACY[frame_i][..frame_len]
                    .clone_from_slice(&rx_buf[..frame_len]);
            }
        }
        return false;
    }

    // If it's a single-frame payload, don't bother loading data into the static buffer.
    if single_frame_payload {
        payload[..tail_byte_i].clone_from_slice(&rx_buf[..tail_byte_i]);
        RX_FRAME_I.store(0, Ordering::Release);
        return true;
    }

    // If we are at the end of a multi-frame payload, compile the frame from parts, and return true.

    let mut payload_i = 0;

    // Reset `tail_byte_i`; may have been altered during our previous use of it.
    let tail_byte_i = frame_len - 1;
    // todo: This currently does not verify CRC.

    // todo: Nasty DRY here for switching buf; figure it out.
    if fd_mode {
        for (frame_i, frame_buf) in unsafe { MULTI_FRAME_BUFS_RX_FD }.iter().enumerate() {
            let mut frame_empty = true;

            // We use a `frame_buf` slice here since for non-FD, we still are usign 64-frame size.
            for (word_i, word) in frame_buf[..frame_len].iter().enumerate() {
                if payload_i >= payload.len() - 1 {
                    break;
                }

                if *word != 0 {
                    frame_empty = false;
                }

                if (frame_i == 0 && (word_i == 0 || word_i == 1)) || word_i == tail_byte_i {
                    // the first two bytes are the CRC; not part of the payload.
                    // The last in each frame (except for the last frame) is the tail byte.
                    continue;
                }

                payload[payload_i] = *word;
                payload_i += 1;
            }

            if frame_empty {
                break;
            }
        }
    } else {
        for (frame_i, frame_buf) in unsafe { MULTI_FRAME_BUFS_RX_LEGACY }.iter().enumerate() {
            let mut frame_empty = true;
            for (word_i, word) in frame_buf[0..frame_len].iter().enumerate() {
                if payload_i >= payload.len() - 1 {
                    break;
                }

                if *word != 0 {
                    frame_empty = false;
                }

                if (frame_i == 0 && (word_i == 0 || word_i == 1)) || word_i == tail_byte_i {
                    continue;
                }

                payload[payload_i] = *word;
                payload_i += 1;
            }

            if frame_empty {
                break;
            }
        }
    }

    // These must match that defined initially.
    unsafe {
        if fd_mode {
            MULTI_FRAME_BUFS_RX_FD = [[0; 64]; MAX_RX_FRAMES_FD];
        } else {
            MULTI_FRAME_BUFS_RX_LEGACY = [[0; 8]; MAX_RX_FRAMES_LEGACY];
        }
    }
    // Zero-out for next use.
    // *frame_bufs = [[0; 64]; 20];
    // *frame_bufs = [[0; 8]; 20];

    RX_FRAME_I.store(0, Ordering::Release);

    true
}
