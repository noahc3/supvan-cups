use crate::error::{Error, Result};

/// Compress data using LZMA1 (alone format) with printer-compatible parameters.
///
/// Parameters: dict_size=8192, lc=3, lp=0, pb=2 (from Android LzmaUtils.java).
/// The printer firmware has limited RAM - larger dictionary sizes will fail.
///
/// Patches the LZMA header to include the exact uncompressed size (Python's
/// lzma module writes -1 by default; we write the real size).
pub fn compress_lzma(data: &[u8]) -> Result<Vec<u8>> {
    compress_lzma_dict(data, 8192)
}

/// As [`compress_lzma`] but with an explicit dictionary size.
///
/// The T50 path uses 8192. The E-series Katasymbol app uses 0x00200000 (2 MiB);
/// see `docs/E_SERIES_PROTOCOL.md`. The properties byte (lc/lp/pb) is unchanged.
pub fn compress_lzma_dict(data: &[u8], dict_size: u32) -> Result<Vec<u8>> {
    use liblzma::stream::{LzmaOptions, Stream};

    let mut opts =
        LzmaOptions::new_preset(6).map_err(|e| Error::Compression(format!("preset: {e}")))?;
    opts.dict_size(dict_size)
        .literal_context_bits(3)
        .literal_position_bits(0)
        .position_bits(2)
        .nice_len(128);

    let stream =
        Stream::new_lzma_encoder(&opts).map_err(|e| Error::Compression(format!("encoder: {e}")))?;

    let mut compressed = Vec::with_capacity(data.len());
    let mut encoder = liblzma::write::XzEncoder::new_stream(&mut compressed, stream);
    std::io::Write::write_all(&mut encoder, data)
        .map_err(|e| Error::Compression(format!("write: {e}")))?;
    encoder
        .finish()
        .map_err(|e| Error::Compression(format!("finish: {e}")))?;

    // The XzEncoder with LZMA encoder stream produces raw LZMA1 alone format:
    //   [0]     properties byte (lc + lp*9 + pb*45 = 3 + 0 + 90 = 93 = 0x5D)
    //   [1..4]  dict_size LE (8192 = 0x00002000)
    //   [5..12] uncompressed size LE (or 0xFFFFFFFFFFFFFFFF for unknown)
    //   [13..]  compressed data

    // Patch header to ensure correct uncompressed size
    if compressed.len() >= 13 {
        let size_bytes = (data.len() as u64).to_le_bytes();
        compressed[5..13].copy_from_slice(&size_bytes);
    }

    Ok(compressed)
}

/// Decompress an LZMA1-alone stream produced by [`compress_lzma`].
///
/// `compress_lzma` patches the alone header with the definite uncompressed size
/// (what the printer firmware reads); combined with the encoder's trailing
/// end-of-stream marker, strict liblzma builds (e.g. the bundled liblzma CI
/// links, reproducible with `LZMA_API_STATIC=1`) reject that as
/// `LZMA_DATA_ERROR`. This restores the "unknown size" sentinel so liblzma
/// decodes the encoder's native marker-terminated stream.
pub fn decompress_lzma(data: &[u8]) -> Result<Vec<u8>> {
    use liblzma::stream::Stream;
    use std::io::Read;

    if data.len() < 13 {
        return Err(Error::Compression(format!(
            "lzma stream too short: {} bytes",
            data.len()
        )));
    }
    let mut stream_bytes = data.to_vec();
    stream_bytes[5..13].copy_from_slice(&u64::MAX.to_le_bytes());
    let stream = Stream::new_lzma_decoder(u64::MAX)
        .map_err(|e| Error::Compression(format!("decoder: {e}")))?;
    let mut decoder = liblzma::read::XzDecoder::new_stream(stream_bytes.as_slice(), stream);
    let mut out = Vec::new();
    decoder
        .read_to_end(&mut out)
        .map_err(|e| Error::Compression(format!("decompress: {e}")))?;
    Ok(out)
}

/// Compress concatenated print buffers for transfer.
///
/// Takes a slice of 4096-byte print buffers, concatenates them, and compresses
/// as a single LZMA stream. The printer's decoder reads the 14-byte header at
/// each 4096-byte boundary internally, so a single LZMA stream covering N
/// buffers is the right thing to send.
///
/// Returns (compressed_data, average_compressed_per_buffer).
pub fn compress_buffers(
    buffers: &[[u8; crate::buffer::PRINT_BUF_SIZE]],
) -> Result<(Vec<u8>, usize)> {
    if buffers.is_empty() {
        return Err(Error::InvalidParam("no buffers to compress".into()));
    }

    let mut concat = Vec::with_capacity(buffers.len() * crate::buffer::PRINT_BUF_SIZE);
    for buf in buffers {
        concat.extend_from_slice(buf);
    }

    let compressed = compress_lzma(&concat)?;
    let avg = compressed.len() / buffers.len();

    Ok((compressed, avg))
}

/// E-series dictionary size, matching the Katasymbol app (`docs/E_SERIES_PROTOCOL.md`).
pub const ESERIES_DICT_SIZE: u32 = 0x0020_0000;

/// Compress one E-series page (its concatenated 4096-byte buffers) into a
/// single LZMA1-alone stream with the 2 MiB dictionary the app uses.
///
/// A page is sent as one bulk command (`0xD1` for the first page, `0xBB` for
/// subsequent pages); the firmware reads the 14-byte header at each 4096-byte
/// boundary internally, so one LZMA stream covers the whole page.
pub fn compress_page_e(
    buffers: &[[u8; crate::buffer::PRINT_BUF_SIZE]],
) -> Result<Vec<u8>> {
    if buffers.is_empty() {
        return Err(Error::InvalidParam("no buffers to compress".into()));
    }
    let mut concat = Vec::with_capacity(buffers.len() * crate::buffer::PRINT_BUF_SIZE);
    for buf in buffers {
        concat.extend_from_slice(buf);
    }
    compress_lzma_dict(&concat, ESERIES_DICT_SIZE)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compress_lzma_header() {
        let data = vec![0u8; 4096];
        let compressed = compress_lzma(&data).unwrap();

        // Check header
        assert!(
            compressed.len() >= 13,
            "compressed too short: {}",
            compressed.len()
        );
        // Properties byte: lc=3, lp=0, pb=2 -> 0x5D
        assert_eq!(compressed[0], 0x5D, "wrong properties byte");
        // Dict size: 8192 LE
        assert_eq!(&compressed[1..5], &8192u32.to_le_bytes(), "wrong dict size");
        // Uncompressed size: 4096 LE
        assert_eq!(
            &compressed[5..13],
            &4096u64.to_le_bytes(),
            "wrong uncompressed size"
        );
    }

    #[test]
    fn test_compress_roundtrip() {
        let data = vec![0x42u8; 1024];
        let compressed = compress_lzma(&data).unwrap();

        // Verifies the LZMA payload round-trips via the shared decoder. The
        // patched header bytes are checked separately by `test_compress_lzma_header`.
        let decompressed = decompress_lzma(&compressed).unwrap();
        assert_eq!(decompressed, data);
    }

    #[test]
    fn test_compress_buffers() {
        let buf = [0u8; 4096];
        let buffers = vec![buf; 3];
        let (compressed, avg) = compress_buffers(&buffers).unwrap();
        assert!(compressed.len() > 13); // at least header
        assert!(avg > 0);
    }
}
