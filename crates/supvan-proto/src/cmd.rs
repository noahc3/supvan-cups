/// Protocol magic bytes.
pub const MAGIC1: u8 = 0x7E;
pub const MAGIC2: u8 = 0x5A;
pub const PROTO_ID: u8 = 0x10;
pub const PROTO_VER: u8 = 0x01;
pub const MARKER_AA: u8 = 0xAA;
pub const DATA_TYPE: u8 = 0x02;

/// Command bytes.
pub const CMD_BUF_FULL: u8 = 0x10;
pub const CMD_INQUIRY_STA: u8 = 0x11;
pub const CMD_CHECK_DEVICE: u8 = 0x12;
pub const CMD_START_PRINT: u8 = 0x13;
pub const CMD_STOP_PRINT: u8 = 0x14;
pub const CMD_RD_DEV_NAME: u8 = 0x16;
pub const CMD_READ_REV: u8 = 0x17;
/// RD_LAB_DPI (34 / 0x22) — query the loaded material's resolution.
/// The vendor app sends this with param 0 and decodes a dots/mm × 100 value
/// from the response (offset depends on PaperType). Not part of the T50 print
/// flow the rest of this crate implements; used by the `dpi` diagnostic to
/// determine non-T50 (e.g. E-series tape) printer geometry.
pub const CMD_RD_LAB_DPI: u8 = 0x22;
pub const CMD_RETURN_MAT: u8 = 0x30;
pub const CMD_NEXT_ZIPPEDBULK: u8 = 0x5C;
pub const CMD_READ_FWVER: u8 = 0xC5;

/// Build a standard 16-byte command frame (0x7E 0x5A format).
///
/// Layout:
///   [0]  0x7E   [1]  0x5A
///   [2]  0x0C   [3]  0x00   (payload length = 12)
///   [4]  0x10   [5]  0x01   (protocol ID, version)
///   [6]  0xAA   [7]  CMD
///   [8..9]  checksum LE (sum of bytes 10..15)
///   [10] 0x00  [11] 0x01
///   [12..13] param LE
///   [14..15] 0x0000
pub fn make_cmd(cmd: u8, param: u16) -> [u8; 16] {
    // A plain command is a start-transfer frame with no block_count; the param
    // occupies the block_size field at bytes 12-13.
    make_cmd_start_trans(cmd, param, 0)
}

/// Declared payload length in byte 2 of every command frame (12 bytes).
const CMD_PAYLOAD_LEN: u8 = 0x0C;

/// Build a 16-byte start-transfer command.
///
/// Same as `make_cmd` but bytes 12-15 carry block_size and block_count.
/// Used for CMD_NEXT_ZIPPEDBULK (block_size=512, block_count=num_packets)
/// and CMD_BUF_FULL (block_size=compressed_len, block_count=speed).
pub fn make_cmd_start_trans(cmd: u8, block_size: u16, block_count: u16) -> [u8; 16] {
    let mut pkt = [0u8; 16];
    pkt[0] = MAGIC1;
    pkt[1] = MAGIC2;
    pkt[2] = CMD_PAYLOAD_LEN;
    pkt[4] = PROTO_ID;
    pkt[5] = PROTO_VER;
    pkt[6] = MARKER_AA;
    pkt[7] = cmd;
    pkt[11] = 0x01;
    pkt[12..14].copy_from_slice(&block_size.to_le_bytes());
    pkt[14..16].copy_from_slice(&block_count.to_le_bytes());

    let chk: u16 = pkt[10..16].iter().map(|&b| b as u16).sum();
    pkt[8..10].copy_from_slice(&chk.to_le_bytes());
    pkt
}

/// Build a variable-length command frame for commands whose param block is
/// longer than the standard 4 bytes (e.g. the E-series `0xD0` setup, `0xB0`
/// date). `params` is the body that follows the fixed `00 01` prefix.
///
/// Layout: `7E 5A [plen LE] 10 01 AA <cmd> [chk LE] 00 01 <params...>` where
/// `plen = 4 (header after marker incl. cmd+chk) ... ` is computed as
/// `2 (chk) + 2 (00 01) + params.len()`'s container — concretely the declared
/// length matches the captured frames (`0x15` for D0's 9 extra param bytes).
/// The checksum is the LE u16 sum over `00 01 <params>`.
pub fn make_cmd_ext(cmd: u8, params: &[u8]) -> Vec<u8> {
    // Body after the checksum field is `00 01` then params.
    let body_len = 2 + params.len();
    // Declared payload length (byte 2): captured frames use 0x0C for the 4-byte
    // body and grow by the extra param bytes. 0x0C corresponds to body_len=6
    // (00 01 + 4 param bytes), i.e. plen = body_len + 6.
    let plen = (body_len + 6) as u16;
    let mut pkt = Vec::with_capacity(10 + body_len);
    pkt.extend_from_slice(&[MAGIC1, MAGIC2]);
    pkt.extend_from_slice(&plen.to_le_bytes());
    pkt.extend_from_slice(&[PROTO_ID, PROTO_VER, MARKER_AA, cmd]);
    pkt.extend_from_slice(&[0, 0]); // checksum placeholder
    pkt.push(0x00);
    pkt.push(0x01);
    pkt.extend_from_slice(params);
    let chk: u16 = pkt[10..].iter().map(|&b| b as u16).sum();
    pkt[8..10].copy_from_slice(&chk.to_le_bytes());
    pkt
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_make_cmd_check_device() {
        let pkt = make_cmd(CMD_CHECK_DEVICE, 0);
        assert_eq!(pkt[0], MAGIC1);
        assert_eq!(pkt[1], MAGIC2);
        assert_eq!(pkt[2], 0x0C);
        assert_eq!(pkt[3], 0x00);
        assert_eq!(pkt[7], CMD_CHECK_DEVICE);
        assert_eq!(pkt[10], 0x00);
        assert_eq!(pkt[11], 0x01);
        assert_eq!(pkt[12], 0x00);
        assert_eq!(pkt[13], 0x00);
        assert_eq!(pkt[14], 0x00);
        assert_eq!(pkt[15], 0x00);
        // checksum: sum of bytes 10..16 = 0+1+0+0+0+0 = 1
        assert_eq!(pkt[8], 0x01);
        assert_eq!(pkt[9], 0x00);
    }

    #[test]
    fn test_make_cmd_with_param() {
        let pkt = make_cmd(CMD_INQUIRY_STA, 0x1234);
        assert_eq!(pkt[12], 0x34);
        assert_eq!(pkt[13], 0x12);
        let chk: u16 = pkt[10..16].iter().map(|&b| b as u16).sum();
        assert_eq!(pkt[8], (chk & 0xFF) as u8);
        assert_eq!(pkt[9], (chk >> 8) as u8);
    }

    #[test]
    fn test_make_cmd_start_trans() {
        let pkt = make_cmd_start_trans(CMD_NEXT_ZIPPEDBULK, 512, 3);
        assert_eq!(pkt[7], CMD_NEXT_ZIPPEDBULK);
        assert_eq!(pkt[12], 0x00); // 512 & 0xFF
        assert_eq!(pkt[13], 0x02); // 512 >> 8
        assert_eq!(pkt[14], 0x03);
        assert_eq!(pkt[15], 0x00);
        let chk: u16 = pkt[10..16].iter().map(|&b| b as u16).sum();
        assert_eq!(pkt[8], (chk & 0xFF) as u8);
        assert_eq!(pkt[9], (chk >> 8) as u8);
    }

    #[test]
    fn test_make_cmd_ext_matches_captured_d0() {
        // Captured E10pro D0 setup frame (docs/E_SERIES_PROTOCOL.md):
        // 7e5a15001001aad0 0101 0001 02 0000 02 0000 f801 03 0000 0000
        let params = [
            0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0xf8, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00,
        ];
        let frame = make_cmd_ext(0xD0, &params);
        let expected =
            hex_to_vec("7e5a15001001aad00101000102000002 0000f8010300000000");
        assert_eq!(frame, expected);
    }

    #[test]
    fn test_make_cmd_ext_matches_captured_b0() {
        // Captured B0 frame carrying the ASCII date "20260625".
        let mut params = b"20260625".to_vec();
        params.extend_from_slice(&[0, 0, 0, 0, 0, 0]); // trailing zeros from capture
        let frame = make_cmd_ext(0xB0, &params);
        let expected = hex_to_vec("7e5a16001001aab0980100013230323630363235000000000000");
        assert_eq!(frame, expected);
    }

    fn hex_to_vec(s: &str) -> Vec<u8> {
        let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }
}
