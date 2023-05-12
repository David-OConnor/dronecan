//! Code for the CRC and data type signature, used in multi-frame transfers.

const CRC_POLY: u16 = 0x1021;
const SIGNATURE_POLY: u64 = 0x42F0_E1EB_A9EA_3693;
const SIGNATURE_MASK64: u64 = 0xFFFF_FFFF_FFFF_FFFF;

/// Code for computing CRC for multi-frame transfers:
/// Adapted from https://dronecan.github.io/Specification/4._CAN_bus_transport_layer/
pub struct TransferCrc {
    pub value: u16,
}

use defmt::println;

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
        for byte in payload {
            self.add_byte(*byte)
        }

        // It appears from experimenting, that we need to 0-pad to the length up to the tail byte.
        let padded_len = crate::find_tail_byte_index(payload_len as u8);

        println!("pl len: {}, padded len: {}", payload_len, padded_len);

        // for pl len 16, total msg len 20, padded len = 19, add 3 bytes works.
        // (index=18 is the last byte pre final tail. This is 3 bytes past the
        // last data byte.)

        // I think we're padding wrong for 8-byte frames and/or multi-frame xfer.
        // Specifically, we don't use a tail-byte index of 64 here.
        // We pad to *the last frame*'s 8, for example.

        // Let's examine a 48-frame payload being split into frames of len=8.
        // Frame 1: bytes 0..5
        // Frame 2: bytes 5..12
        // Frame 3: bytes 12..19
        // Frame 4: bytes 19..26
        // Frame 5: bytes 26..33
        // Frame 6: Bytes 33..40
        // Frame 7: Bytes 40..47
        // Frame 8: byte 47
        // (An 8th frame is required; it will contain the first byte and  the CRC)

        // for pl len 48, total msg len 64, padded len = _, add _ bytes works.
        // Padded len is
        //  index =53 is the last byte pre final tail)

        // for _ in pl_len..padded_len {
        for _ in 0..6 {
            self.add_byte(0);
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
pub struct Signature {
    crc: u64,
}

impl Signature {
    pub fn new(extend_from: Option<u64>) -> Self {
        let crc = match extend_from {
            Some(e) => (e & SIGNATURE_MASK64) ^ SIGNATURE_MASK64,
            None => SIGNATURE_MASK64,
        };

        Self { crc }
    }

    pub fn add(&mut self, data_bytes: &[u8]) {
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

    pub fn value(&self) -> u64 {
        (self.crc & SIGNATURE_MASK64) ^ SIGNATURE_MASK64
    }
}