//! Phone-to-firmware interoperability, including the mobile client's non-greedy
//! final-packet rule. These exercise the production receiver, not a second server.
use controller_protocols::ota::{self, Command, Receive, Sectors};

// Existing Android OtaChunking / iOS OTAUpdateService wire contract. Keep one
// byte for the final packet rather than greedily exhausting a sector early.
fn phone_packets(sector: u16, bytes: &[u8], packet_size: usize) -> Vec<Vec<u8>> {
    let mut packets = Vec::new();
    let mut offset = 0;
    let mut sequence = 0u8;
    while offset < bytes.len() {
        let remaining = bytes.len() - offset;
        let last = remaining <= packet_size - 5;
        let count = if last {
            remaining
        } else {
            (remaining - (packet_size - 5)).clamp(1, packet_size - 3)
        };
        let mut packet = sector.to_le_bytes().to_vec();
        packet.push(if last { 255 } else { sequence });
        packet.extend_from_slice(&bytes[offset..offset + count]);
        if last {
            packet.extend_from_slice(&ota::crc16(bytes).to_le_bytes());
        } else {
            sequence += 1;
        }
        packets.push(packet);
        offset += count;
    }
    packets
}

fn feed(receiver: &mut Sectors, packets: &[Vec<u8>], expected: &[u8], now: u32) {
    for (index, packet) in packets.iter().enumerate() {
        let result = receiver.receive(packet, now.wrapping_add(index as u32));
        if index + 1 == packets.len() {
            assert_eq!(result, Receive::Complete(expected.len()));
            assert_eq!(receiver.data(expected.len()), expected);
        } else {
            assert_eq!(result, Receive::Buffered);
        }
    }
    // Only a successful flash write advances the receiver's sector index.
    receiver.written(expected.len());
}

#[test]
fn phone_mtu_fallback_and_large_mtu_transfer_full_and_partial_sectors() {
    for packet_size in [20, 497, 500] {
        let image: Vec<u8> = (0..5084).map(|n| (n * 37 + n / 256) as u8).collect();
        let mut receiver = Sectors::default();
        receiver.start(image.len() as u32, 0);
        for (sector, bytes) in image.chunks(4096).enumerate() {
            let packets = phone_packets(sector as u16, bytes, packet_size);
            assert!(packets.iter().all(|p| p.len() <= packet_size));
            feed(&mut receiver, &packets, bytes, sector as u32 * 1000);
        }
        assert_eq!(receiver.received, image.len() as u32);
        assert_eq!(receiver.sector, 2);
    }
}

#[test]
fn mobile_non_greedy_middle_packet_is_accepted() {
    let bytes = vec![0xa5; 988];
    let packets = phone_packets(0, &bytes, 497);
    assert_eq!(
        packets.iter().map(Vec::len).collect::<Vec<_>>(),
        [497, 5, 497]
    );
    let mut receiver = Sectors::default();
    receiver.start(988, 0);
    feed(&mut receiver, &packets, &bytes, 1);
}

#[test]
fn crc_failure_after_sector_zero_retries_without_advancing_or_corrupting_data() {
    let bytes: Vec<u8> = (0..4096).map(|n| n as u8).collect();
    let mut receiver = Sectors::default();
    receiver.start(8192, 0);
    feed(&mut receiver, &phone_packets(0, &bytes, 497), &bytes, 1);
    let mut packets = phone_packets(1, &bytes, 497);
    let last = packets.last_mut().unwrap();
    *last.last_mut().unwrap() ^= 1;
    for packet in &packets[..packets.len() - 1] {
        assert_eq!(receiver.receive(packet, 100), Receive::Buffered);
    }
    assert_eq!(
        receiver.receive(packets.last().unwrap(), 101),
        Receive::Rejected {
            sector: 1,
            status: 1,
            expected: 0
        }
    );
    assert_eq!(receiver.received, 4096);
    assert_eq!(receiver.sector, 1);
    feed(&mut receiver, &phone_packets(1, &bytes, 497), &bytes, 102);
    assert_eq!(receiver.received, 8192);
}

#[test]
fn wrong_index_and_overflow_reset_accumulator_for_clean_retry() {
    let mut receiver = Sectors::default();
    receiver.start(3, 0);
    assert_eq!(receiver.receive(&[0, 0, 0, 9], 1), Receive::Buffered);
    assert_eq!(
        receiver.receive(&[2, 0, 0, 1], 2),
        Receive::Rejected {
            sector: 2,
            status: 2,
            expected: 0
        }
    );
    let mut oversized = vec![0; 3 + 4097];
    oversized[2] = 0;
    assert_eq!(
        receiver.receive(&oversized, 3),
        Receive::Rejected {
            sector: 0,
            status: 3,
            expected: 0
        }
    );
    feed(
        &mut receiver,
        &phone_packets(0, &[1, 2, 3], 20),
        &[1, 2, 3],
        4,
    );
    assert_eq!(receiver.received, 3);
}

#[test]
fn timeout_survives_millisecond_wrap_and_malformed_short_packet() {
    let mut receiver = Sectors::default();
    let start = u32::MAX - 500;
    receiver.start(1, start);
    assert_eq!(receiver.receive(&[0, 0], 0), Receive::Ignored);
    assert!(!receiver.timed_out(start.wrapping_add(30_000)));
    assert!(receiver.timed_out(start.wrapping_add(30_001)));
}

#[test]
fn mobile_start_end_and_ack_layouts() {
    let mut start = [0u8; 20];
    start[0] = 1;
    start[2..6].copy_from_slice(&0x12345678u32.to_le_bytes());
    let crc = ota::crc16(&start[..18]);
    start[18..].copy_from_slice(&crc.to_le_bytes());
    assert_eq!(ota::parse_command(&start), Some(Command::Start(0x12345678)));
    start[0] = 2;
    start[2..18].fill(0);
    let crc = ota::crc16(&start[..18]);
    start[18..].copy_from_slice(&crc.to_le_bytes());
    assert_eq!(ota::parse_command(&start), Some(Command::End));
    assert_eq!(&ota::command_ack(1, 0)[..6], &[3, 0, 1, 0, 0, 0]);
    assert_eq!(&ota::command_ack(1, 1)[..6], &[3, 0, 1, 0, 1, 0]);
    assert_eq!(
        ota::sector_ack(0x1234, 2, 0x4567),
        [0x34, 0x12, 2, 0, 0x67, 0x45]
    );
}
