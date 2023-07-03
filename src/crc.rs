//! Code for the CRC and data type signature, used in multi-frame transfers.

const CRC_POLY: u16 = 0x1021;
const SIGNATURE_POLY: u64 = 0x42F0_E1EB_A9EA_3693;
const SIGNATURE_MASK64: u64 = 0xFFFF_FFFF_FFFF_FFFF;

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

    pub fn add_payload(&mut self, payload: &[u8], payload_len: usize, frame_payload_len: usize) {
        for i in 0..payload_len {
            self.add_byte(payload[i]);
        }
    }
}

/// Code for computing the data type signature for multi-frame transfers:
/// Adapted from https://dronecan.github.io/Specification/3._Data_structure_description_language/
///
/// "Data type signature is a 64-bit integer value that is guaranteed to be equal for compatible data types.
///
/// Hence, it is said that data types are compatible if their names and signatures are equal.
///
/// The following is the data type signature computation algorithm for a given data type, where hash is the signature hash function described earlier:
///
///     Initialize the hash value with the DSDL signature of the given type.
///     Starting from the top of the DSDL definition, do the following for each nested data structure:
///         Extend the current hash value with the data type signature of the nested data structure.
///     The resulting hash value will be the data type signature."
///
/// Verification example:
///     let mut sig = Signature::new(None);
///         sig.add(&[49, 50, 51]);
///         sig.add(&[52, 53, 54, 55, 56 , 57]);
///
///     println!("sig value: {}. (Should be 0x62EC59E3F1A4F00A)", sig.value());
/// (Passes CAO 11 May 2023)
pub struct _Signature {
    crc: u64,
}

impl _Signature {
    pub fn _new(extend_from: Option<u64>) -> Self {
        let crc = match extend_from {
            Some(e) => (e & SIGNATURE_MASK64) ^ SIGNATURE_MASK64,
            None => SIGNATURE_MASK64,
        };

        Self { crc }
    }

    pub fn _add(&mut self, data_bytes: &[u8]) {
        for byte in data_bytes {
            self.crc ^= ((*byte as u64) << 56) & SIGNATURE_MASK64;

            for _ in 0..8 {
                if self.crc & (1 << 63) != 0 {
                    self.crc = ((self.crc << 1) & SIGNATURE_MASK64) ^ SIGNATURE_POLY;
                } else {
                    self.crc <<= 1;
                }
            }
        }
    }

    pub fn _value(&self) -> u64 {
        (self.crc & SIGNATURE_MASK64) ^ SIGNATURE_MASK64
    }
}
