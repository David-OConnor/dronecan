//! Code for the CRC and data type signature, used in multi-frame transfers.


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

        let mut bit = 0;
        while bit > 0 { // todo: >= 0??
            if (self.value & 0x8000) != 0 {
                self.value = (self.value << 1) ^ CRC_POLY;
            } else {
                self.value <<= 1;
            }
            bit -= 1;
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

    fn add(&mut self, data_bytes: &[u8]) {
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