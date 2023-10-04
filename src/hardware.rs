//! This module contains hardware setup code, shared between firmwares. Requires the `hal` feature.

use stm32_hal2::{can::Can, pac::FDCAN1};

use fdcan::{
    config as can_config,
    filter::{Action, ExtendedFilter, ExtendedFilterSlot, FilterType},
    interrupt::{Interrupt, InterruptLine},
    ConfigMode, FdCan, NormalOperationMode,
};

use crate::{CanBitrate, FrameType, MsgType, RequestResponse, ServiceData};

pub type Can_ = FdCan<Can, NormalOperationMode>;

use core::num::{NonZeroU16, NonZeroU8};

#[derive(Clone, Copy)]
pub enum CanClock {
    Mhz160,
    Mhz170,
    Mhz120,
    Mhz100,
}

use defmt::println;

pub fn set_dronecan_filter(
    can: &mut FdCan<Can, ConfigMode>,
    slot: ExtendedFilterSlot,
    frame_type: FrameType,
    id: u16,
) {
    let filter_type = match frame_type {
        FrameType::Message => {
            FilterType::BitMask {
                filter: (id as u32) << 8,
                // "A 0 bit at the filter mask masks out the corresponding bit position of the configured ID filter,
                // e.g. the value of the received Message ID at that bit position is not relevant for acceptance
                // filtering. Only those bits of the received Message ID where the corresponding mask bits are
                // one are relevant for acceptance filtering.
                // In case all mask bits are one, a match occurs only when the received Message ID and the
                // Message ID filter are identical. If all mask bits are 0, all Message IDs match."
                // So, we filter the message ID field only. (bits 8 - 23)
                mask: 0xffff << 8,
            }
        }
        FrameType::Service(_) => FilterType::BitMask {
            filter: ((id as u32) << 16) | (1 << 7),
            mask: (0xff << 16) | (1 << 7),
        },
        FrameType::MessageAnon => unimplemented!(),
    };

    let filter = ExtendedFilter {
        filter: filter_type,
        action: Action::StoreInFifo0,
    };

    can.set_extended_filter(slot, filter);
}

pub fn setup_can(can_pac: FDCAN1, can_clock: CanClock, bitrate: CanBitrate) -> Can_ {
    let mut can = FdCan::new(Can::new(can_pac)).into_config_mode();
    // Nominal (arbitration) bit rate is almost 1Mhz or less, for compatibility
    // with non-FD devices on the bus.

    // todo: Hard-coded for 160, for now at least.
    let (prescaler_nom, seg1_nom, seg2_nom) = match can_clock {
        CanClock::Mhz170 => match bitrate {
            CanBitrate::B250k => CanBitrate::B250k.timings_170_mhz(),
            CanBitrate::B500k => CanBitrate::B500k.timings_170_mhz(),
            _ => CanBitrate::B1m.timings_170_mhz(),
        },
        CanClock::Mhz160 => match bitrate {
            CanBitrate::B250k => CanBitrate::B250k.timings_160_mhz(),
            CanBitrate::B500k => CanBitrate::B500k.timings_160_mhz(),
            _ => CanBitrate::B1m.timings_160_mhz(),
        },
        CanClock::Mhz120 => match bitrate {
            CanBitrate::B250k => CanBitrate::B250k.timings_120_mhz(),
            CanBitrate::B500k => CanBitrate::B500k.timings_120_mhz(),
            _ => CanBitrate::B1m.timings_120_mhz(),
        },
        CanClock::Mhz100 => match bitrate {
            CanBitrate::B250k => CanBitrate::B250k.timings_100_mhz(),
            CanBitrate::B500k => CanBitrate::B500k.timings_100_mhz(),
            _ => CanBitrate::B1m.timings_100_mhz(),
        },
    };

    let (prescaler_data, seg1_data, seg2_data) = match can_clock {
        CanClock::Mhz170 => bitrate.timings_170_mhz(),
        CanClock::Mhz160 => bitrate.timings_160_mhz(),
        CanClock::Mhz120 => bitrate.timings_120_mhz(),
        CanClock::Mhz100 => bitrate.timings_100_mhz(),
    };

    let nominal_bit_timing = can_config::NominalBitTiming {
        prescaler: NonZeroU16::new(prescaler_nom).unwrap(),
        seg1: NonZeroU8::new(seg1_nom).unwrap(),
        seg2: NonZeroU8::new(seg2_nom).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    };

    let data_bit_timing = can_config::DataBitTiming {
        prescaler: NonZeroU8::new(prescaler_data as u8).unwrap(),
        seg1: NonZeroU8::new(seg1_data).unwrap(),
        seg2: NonZeroU8::new(seg2_data).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
        transceiver_delay_compensation: true,
    };

    // When Protocol exception
    // handling is enabled (CCCR.PXHD = 0), this causes the operation state to change from
    // Receiver (PSR.ACT = 10) to Integrating (PSR.ACT = 00) at the next sample point. In case
    // Protocol exception Handling is disabled (CCCR.PXHD = 1), the FDCAN treats a recessive
    // res bit as a form error and responds with an error frame.
    // (Default enabled)
    can.set_protocol_exception_handling(true);
    can.set_nominal_bit_timing(nominal_bit_timing);
    can.set_data_bit_timing(data_bit_timing);

    // Node: H7 has up to 64 filters available. This is set up for G4's limitations.
    // This `GetNodeInfo` also matches dynamic ID allocation.

    can.set_frame_transmit(can_config::FrameTransmissionConfig::AllowFdCanAndBRS);

    can.enable_interrupt(Interrupt::RxFifo0NewMsg);

    // This appears to be backwards in the hardware; Need line 1 on G4 for FIFO 0.
    can.enable_interrupt_line(InterruptLine::_1, true);

    can.into_normal()
}

pub fn setup_protocol_filters(can: Can_) -> Can_ {
    let mut can = can.into_config_mode();

    let s = ServiceData {
        dest_node_id: 0, // 7 bits
        req_or_resp: RequestResponse::Request,
    };

    set_dronecan_filter(
        &mut can,
        ExtendedFilterSlot::_0,
        FrameType::Service(s),
        MsgType::GetNodeInfo.id(),
    );
    set_dronecan_filter(
        &mut can,
        ExtendedFilterSlot::_1,
        FrameType::Message,
        MsgType::IdAllocation.id(),
    );
    set_dronecan_filter(
        &mut can,
        ExtendedFilterSlot::_2,
        FrameType::Service(s),
        MsgType::GetSet.id(),
    );
    // set_dronecan_id_filter(&mut can, ExtendedFilterSlot::_3, FrameType::Service(s), MsgType::Restart.id());
    set_dronecan_filter(
        &mut can,
        ExtendedFilterSlot::_4,
        FrameType::Service(s),
        MsgType::ConfigGnssGet.id(),
    );
    set_dronecan_filter(
        &mut can,
        ExtendedFilterSlot::_5,
        FrameType::Service(s),
        MsgType::ConfigRxGet.id(),
    );
    // set_dronecan_filter(
    //     &mut can,
    //     ExtendedFilterSlot::_6,
    //     FrameType::Service(s),
    //     MsgType::SetConfig.id(),
    // );

    // Place this reject filter in the filal slot, rejecting all messages not explicitly accepted
    // by our dronecan ID filters.
    let reject_filter = ExtendedFilter::reject_all();
    can.set_extended_filter(ExtendedFilterSlot::_7, reject_filter);

    can.into_normal()
}
