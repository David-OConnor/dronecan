//! Code for the CRC and data type signature, used in multi-frame transfers.

const CRC_POLY: u16 = 0x1021;

/// Code for computing CRC for multi-frame transfers:
/// Adapted from https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
pub struct TransferCrc {
    pub value: u16,
}

impl TransferCrc {
    /// Use `pydronecan` to find each message type's base CRC:
    /// `dronecan.DATATYPES[(message_type_id, 1 or 0)].base_crc`
    pub fn new(base: u16) -> Self {
        Self { value: base }
    }

    /// See https://github.com/dronecan/pydronecan/blob/master/dronecan/dsdl/common.py#L50
    /// Verified against the Python lib with a few examples.
    fn add_byte(&mut self, byte: u8) {
        self.value ^= (byte as u16) << 8;

        for _bit in 0..8 {
            if (self.value & 0x8000) != 0 {
                self.value = (self.value << 1) ^ CRC_POLY;
            } else {
                self.value <<= 1;
            }
        }
    }

    pub fn add_payload(&mut self, payload: &[u8], payload_len: usize) {
        for i in 0..payload_len {
            self.add_byte(payload[i]);
        }
    }
}
