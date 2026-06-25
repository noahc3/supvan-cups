use crate::cmd::{DATA_TYPE, MAGIC1, MAGIC2, PROTO_ID};

/// Data packet magic bytes.
pub const DATA_MAGIC1: u8 = 0xAA;
pub const DATA_MAGIC2: u8 = 0xBB;

/// Max payload per data packet.
pub const DATA_PAYLOAD_SIZE: usize = 500;

/// Transfer-frame payload length declared in bytes 2-3 (= 506-byte packet + 2).
const DATA_FRAME_PAYLOAD_LEN: u16 = 508;

/// Build a 506-byte data packet (0xAA 0xBB format).
///
/// Layout:
///   [0]     0xAA
///   [1]     0xBB
///   [2..3]  checksum LE (sum of bytes 4..505)
///   [4]     packet index (0-based)
///   [5]     total packet count
///   [6..505] payload (500 bytes, zero-padded)
pub fn make_data_packet(data_chunk: &[u8], pkt_idx: u8, pkt_total: u8) -> [u8; 506] {
    let mut pkt = [0u8; 506];
    pkt[0] = DATA_MAGIC1;
    pkt[1] = DATA_MAGIC2;
    pkt[4] = pkt_idx;
    pkt[5] = pkt_total;

    let copy_len = data_chunk.len().min(DATA_PAYLOAD_SIZE);
    pkt[6..6 + copy_len].copy_from_slice(&data_chunk[..copy_len]);

    let chk: u16 = pkt[4..506].iter().map(|&b| b as u16).sum();
    pkt[2..4].copy_from_slice(&chk.to_le_bytes());
    pkt
}

/// Wrap a 506-byte data packet in a 512-byte transfer frame.
///
/// Layout:
///   [0]     0x7E
///   [1]     0x5A
///   [2..3]  0x01FC (payload length = 508)
///   [4]     0x10 (protocol ID)
///   [5]     0x02 (data transfer type)
///   [6..511] 506 bytes of payload (the AA BB packet)
pub fn wrap_data_frame(payload: &[u8; 506]) -> [u8; 512] {
    let mut frame = [0u8; 512];
    frame[0] = MAGIC1;
    frame[1] = MAGIC2;
    frame[2..4].copy_from_slice(&DATA_FRAME_PAYLOAD_LEN.to_le_bytes());
    frame[4] = PROTO_ID;
    frame[5] = DATA_TYPE;
    frame[6..512].copy_from_slice(payload);
    frame
}

/// Split compressed data into 506-byte data packets wrapped in 512-byte frames.
///
/// Returns a Vec of 512-byte frames ready to send.
pub fn build_data_frames(compressed: &[u8]) -> Vec<[u8; 512]> {
    let num_packets = compressed.len().div_ceil(DATA_PAYLOAD_SIZE);
    let pkt_total = num_packets as u8;
    let mut frames = Vec::with_capacity(num_packets);

    for i in 0..num_packets {
        let offset = i * DATA_PAYLOAD_SIZE;
        let end = (offset + DATA_PAYLOAD_SIZE).min(compressed.len());
        let chunk = &compressed[offset..end];
        let pkt = make_data_packet(chunk, i as u8, pkt_total);
        frames.push(wrap_data_frame(&pkt));
    }

    frames
}

/// Build a single 512-byte E-series bulk frame (`0xD1` / `0xBB`).
///
/// Byte-verified layout (`docs/E_SERIES_PROTOCOL.md`):
///   [0..2]  0x7E 0x5A
///   [2..4]  0x01FC  (payload length = 508)
///   [4..7]  0x10 0x02 0xAA  (command marker, same as a normal command frame)
///   [7]     cmd  (0xD1 or 0xBB)
///   [8..10] chunk checksum LE = sum of bytes [10..512] (= idx+tot+payload)
///   [10]    chunk index (0-based)
///   [11]    chunk total
///   [12..512] up to 500 LZMA bytes, zero-padded
///
/// Unlike [`make_data_packet`]/[`wrap_data_frame`] (the T50 `0xAA 0xBB`
/// data-packet format), the E-series reuses the command marker plus a command
/// byte. The checksum spans `[idx][tot] + the 500 payload bytes`.
pub fn make_eseries_bulk_frame(cmd: u8, payload: &[u8], idx: u8, total: u8) -> [u8; 512] {
    let mut frame = [0u8; 512];
    frame[0] = MAGIC1;
    frame[1] = MAGIC2;
    frame[2..4].copy_from_slice(&DATA_FRAME_PAYLOAD_LEN.to_le_bytes());
    frame[4] = PROTO_ID;
    frame[5] = DATA_TYPE;
    frame[6] = 0xAA; // MARKER_AA
    frame[7] = cmd;
    frame[10] = idx;
    frame[11] = total;

    let copy_len = payload.len().min(DATA_PAYLOAD_SIZE);
    frame[12..12 + copy_len].copy_from_slice(&payload[..copy_len]);

    let chk: u16 = frame[10..].iter().map(|&b| b as u16).sum();
    frame[8..10].copy_from_slice(&chk.to_le_bytes());
    frame
}

/// Split an LZMA stream into E-series bulk frames for one page command
/// (`0xD1` or `0xBB`). Each frame carries 500 LZMA bytes (last zero-padded).
pub fn build_eseries_bulk_frames(cmd: u8, lzma: &[u8]) -> Vec<[u8; 512]> {
    let num = lzma.len().div_ceil(DATA_PAYLOAD_SIZE).max(1);
    let total = num as u8;
    let mut frames = Vec::with_capacity(num);
    for i in 0..num {
        let off = i * DATA_PAYLOAD_SIZE;
        let end = (off + DATA_PAYLOAD_SIZE).min(lzma.len());
        frames.push(make_eseries_bulk_frame(cmd, &lzma[off..end], i as u8, total));
    }
    frames
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_make_data_packet() {
        let data = [0x42u8; 500];
        let pkt = make_data_packet(&data, 0, 3);
        assert_eq!(pkt[0], DATA_MAGIC1);
        assert_eq!(pkt[1], DATA_MAGIC2);
        assert_eq!(pkt[4], 0);
        assert_eq!(pkt[5], 3);
        assert_eq!(&pkt[6..506], &data[..]);
        let chk: u16 = pkt[4..506].iter().map(|&b| b as u16).sum();
        assert_eq!(pkt[2], (chk & 0xFF) as u8);
        assert_eq!(pkt[3], (chk >> 8) as u8);
    }

    #[test]
    fn test_make_data_packet_short() {
        let data = [0xFFu8; 100];
        let pkt = make_data_packet(&data, 2, 5);
        assert_eq!(pkt[4], 2);
        assert_eq!(pkt[5], 5);
        // First 100 bytes should be 0xFF, rest 0x00
        assert_eq!(&pkt[6..106], &[0xFF; 100]);
        assert_eq!(&pkt[106..506], &[0x00; 400]);
    }

    #[test]
    fn test_wrap_data_frame() {
        let pkt = [0xAA; 506];
        let frame = wrap_data_frame(&pkt);
        assert_eq!(frame[0], MAGIC1);
        assert_eq!(frame[1], MAGIC2);
        assert_eq!(frame[2], 0xFC);
        assert_eq!(frame[3], 0x01);
        assert_eq!(frame[4], PROTO_ID);
        assert_eq!(frame[5], DATA_TYPE);
        assert_eq!(&frame[6..512], &pkt[..]);
    }

    #[test]
    fn test_build_data_frames() {
        // 1100 bytes -> 3 packets (500+500+100)
        let data = vec![0x42u8; 1100];
        let frames = build_data_frames(&data);
        assert_eq!(frames.len(), 3);
        // Check packet indices
        assert_eq!(frames[0][6 + 4], 0); // pkt_idx
        assert_eq!(frames[0][6 + 5], 3); // pkt_total
        assert_eq!(frames[1][6 + 4], 1);
        assert_eq!(frames[2][6 + 4], 2);
    }

    #[test]
    fn test_eseries_bulk_frame_layout() {
        // Verify against the captured D1 chunk shape (docs/E_SERIES_PROTOCOL.md):
        // 7E 5A FC 01 10 02 AA <cmd> [chk LE][idx][tot] <500 LZMA>.
        let lzma = vec![0x5du8; 600]; // 2 chunks (500 + 100 padded)
        let frames = build_eseries_bulk_frames(0xD1, &lzma);
        assert_eq!(frames.len(), 2);
        let f = &frames[0];
        assert_eq!(&f[0..7], &[0x7e, 0x5a, 0xfc, 0x01, 0x10, 0x02, 0xaa]);
        assert_eq!(f[7], 0xD1, "command byte");
        assert_eq!(f[10], 0, "idx");
        assert_eq!(f[11], 2, "total");
        // checksum = LE u16 sum over [idx][tot] + 500 payload bytes (frame[10..])
        let chk: u16 = f[10..].iter().map(|&b| b as u16).sum();
        assert_eq!(u16::from_le_bytes([f[8], f[9]]), chk);
        assert_eq!(f.len(), 512);
        // last chunk: 100 real bytes, rest zero-padded to 500
        let g = &frames[1];
        assert_eq!(g[7], 0xD1);
        assert_eq!(g[10], 1);
        assert_eq!(g[11], 2);
        assert!(g[12..112].iter().all(|&b| b == 0x5d));
        assert!(g[112..512].iter().all(|&b| b == 0));
    }
}
