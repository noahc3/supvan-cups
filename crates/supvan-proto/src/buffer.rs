/// Max image data bytes per print buffer (from Android R2.drawable.sf5334_).
pub const MAX_BUF_DATA: usize = 4074;

/// Print buffer size.
pub const PRINT_BUF_SIZE: usize = 4096;

/// Header size in print buffer.
pub const PRINT_BUF_HEADER: usize = 14;

/// Margin clamp range (dots) for the print-buffer header.
const MARGIN_MAX_DOTS: u16 = 900;

/// Maximum density / red-deepness value encoded in the buffer header.
const MAX_DENSITY: u8 = 15;

/// The firmware re-reads the running checksum at every Nth byte; the builder
/// folds in the byte just before each boundary.
const CHECKSUM_STRIDE: usize = 256;

/// Parameters for PAGE_REG_BITS construction.
#[derive(Debug, Clone, Default)]
pub struct PageRegBits {
    pub page_st: bool,
    pub page_end: bool,
    pub prt_end: bool,
    pub cut: u8,
    pub savepaper: bool,
    pub first_cut: u8,
    pub nodu: u8,
    pub mat: u8,
}

/// Build PAGE_REG_BITS (2 bytes) for a print buffer header.
///
/// Byte 0:
///   bit 1: PageSt (first buffer of page)
///   bit 2: PageEnd (last buffer of page)
///   bit 3: PrtEnd (end of print job)
///   bits 4-6: Cut mode (3 bits)
///   bit 7: Savepaper
///
/// Byte 1:
///   bits 0-1: FirstCut
///   bits 2-5: Nodu (density, 0-15)
///   bits 6-7: Mat (material type)
pub fn build_page_reg_bits(p: &PageRegBits) -> [u8; 2] {
    let mut b0: u8 = 0;
    if p.page_st {
        b0 |= 0x02;
    }
    if p.page_end {
        b0 |= 0x04;
    }
    if p.prt_end {
        b0 |= 0x08;
    }
    b0 &= 0x0F;
    b0 |= (p.cut & 0x07) << 4;
    if p.savepaper {
        b0 |= 0x80;
    }

    let mut b1: u8 = 0;
    b1 |= p.first_cut & 0x03;
    b1 |= (p.nodu & 0x0F) << 2;
    b1 |= (p.mat & 0x03) << 6;

    [b0, b1]
}

/// Parameters for building a print buffer.
pub struct PrintBufferParams<'a> {
    pub image_data: &'a [u8],
    pub per_line_byte: u8,
    pub cols_in_buf: u16,
    pub page_st: bool,
    pub page_end: bool,
    pub prt_end: bool,
    pub margin_top: u16,
    pub margin_bottom: u16,
    /// `nodu` field in PAGE_REG_BITS b1 (bits 2-5). On the T50 this carries the
    /// density (0-15). On the E-series it is a fixed `4` and the real burn
    /// energy lives in `energy` (byte 12) instead.
    pub nodu: u8,
    /// Burn-energy / red-deepness byte at offset 12.
    ///
    /// `Some(v)` writes `v` **unclamped** (the E-series path: real captured
    /// values reach 23). `None` preserves the legacy T50 behaviour: byte 12 is
    /// set to `nodu` clamped to `MAX_DENSITY`. The firmware zeroes this byte on
    /// the final buffer of a page (see `split_into_buffers_e`).
    pub energy: Option<u8>,
    /// Cut mode (3 bits) in PAGE_REG_BITS b0 (bits 4-6). T50 uses 0; E-series
    /// multi-buffer pages use 1 on the image buffers.
    pub cut: u8,
    /// Material type (0-3) encoded into PAGE_REG_BITS. Must match the loaded
    /// label's reported `PaperType`/`label_type`; a mismatch can trip a
    /// firmware "label mode error". T50-class die-cut labels use 1; E-series
    /// continuous tape reports 0.
    pub mat: u8,
}

/// Build a 4096-byte print buffer.
///
/// Layout:
///   [0..1]   Checksum (LE)
///   [2..3]   PAGE_REG_BITS
///   [4..5]   Column count (LE)
///   [6]      Bytes per line
///   [7]      Reserved (0)
///   [8..9]   Margin top (LE, 1-900 dots)
///   [10..11] Margin bottom (LE, 1-900 dots)
///   [12]     Density / red deepness (0-15)
///   [13]     0
///   [14..]   Image data
pub fn build_print_buffer(p: &PrintBufferParams) -> [u8; PRINT_BUF_SIZE] {
    let mut buf = [0u8; PRINT_BUF_SIZE];

    // PAGE_REG_BITS
    let page_bits = build_page_reg_bits(&PageRegBits {
        page_st: p.page_st,
        page_end: p.page_end,
        prt_end: p.prt_end,
        cut: p.cut,
        nodu: p.nodu,
        mat: p.mat,
        ..Default::default()
    });
    buf[2] = page_bits[0];
    buf[3] = page_bits[1];

    // Column count
    buf[4..6].copy_from_slice(&p.cols_in_buf.to_le_bytes());

    // Bytes per line
    buf[6] = p.per_line_byte;

    // Margins (clamped 1..=MARGIN_MAX_DOTS)
    let mt = p.margin_top.clamp(1, MARGIN_MAX_DOTS);
    let mb = p.margin_bottom.clamp(1, MARGIN_MAX_DOTS);
    buf[8..10].copy_from_slice(&mt.to_le_bytes());
    buf[10..12].copy_from_slice(&mb.to_le_bytes());

    // Burn energy (byte 12). E-series passes an explicit unclamped value; the
    // legacy T50 path mirrors `nodu` clamped to MAX_DENSITY.
    buf[12] = match p.energy {
        Some(e) => e,
        None => p.nodu.min(MAX_DENSITY),
    };

    // Image data at offset 14
    let data_len = p.image_data.len().min(PRINT_BUF_SIZE - PRINT_BUF_HEADER);
    buf[PRINT_BUF_HEADER..PRINT_BUF_HEADER + data_len].copy_from_slice(&p.image_data[..data_len]);

    // Checksum: sum(buf[2..14]) + sum of bytes at each 256-byte boundary
    let data_end = (p.cols_in_buf as usize) * (p.per_line_byte as usize) + PRINT_BUF_HEADER;
    let mut chk: u32 = buf[2..14].iter().map(|&b| b as u32).sum();
    let n_strides = data_end / CHECKSUM_STRIDE;
    for i in 1..=n_strides {
        let idx = i * CHECKSUM_STRIDE - 1;
        if idx < buf.len() {
            chk += buf[idx] as u32;
        }
    }
    buf[0..2].copy_from_slice(&(chk as u16).to_le_bytes());

    buf
}

/// Split column-major image data into multiple print buffers (T50 path).
///
/// `density` is written into both `nodu` (PAGE_REG) and the energy byte
/// (clamped to `MAX_DENSITY`), matching the original T50 behaviour. For the
/// E-series, use [`split_into_buffers_e`].
///
/// Returns a Vec of 4096-byte print buffers ready for LZMA compression.
pub fn split_into_buffers(
    image_data: &[u8],
    per_line_byte: u8,
    total_cols: u16,
    margin_top: u16,
    margin_bottom: u16,
    density: u8,
    mat: u8,
) -> Vec<[u8; PRINT_BUF_SIZE]> {
    let max_cols = (MAX_BUF_DATA / per_line_byte as usize) as u16;
    let image_cols = total_cols - margin_top - margin_bottom;
    let mut buffers = Vec::new();
    let mut cols_remaining = image_cols;
    let mut current_col: u16 = 0;

    while cols_remaining > 0 {
        let cols_in_buf = cols_remaining.min(max_cols);
        let is_first = current_col == 0;
        let is_last = cols_remaining <= max_cols;

        let img_start = (margin_top + current_col) as usize * per_line_byte as usize;
        let img_end = img_start + cols_in_buf as usize * per_line_byte as usize;
        let img_chunk = &image_data[img_start..img_end.min(image_data.len())];

        let buf = build_print_buffer(&PrintBufferParams {
            image_data: img_chunk,
            per_line_byte,
            cols_in_buf,
            page_st: is_first,
            page_end: is_last,
            prt_end: is_last,
            margin_top,
            margin_bottom,
            nodu: density,
            energy: None,
            cut: 0,
            mat,
        });
        buffers.push(buf);
        current_col += cols_in_buf;
        cols_remaining -= cols_in_buf;
    }

    buffers
}

/// Knobs for the E-series (E10pro) print-buffer header, byte-verified from a
/// real btsnoop capture (see `docs/E_SERIES_PROTOCOL.md`).
#[derive(Debug, Clone, Copy)]
pub struct ESeriesBufOpts {
    /// PAGE_REG `nodu` (fixed 4 in the capture).
    pub nodu: u8,
    /// Burn energy (byte 12), unclamped. Captured value: 23. Written on every
    /// buffer EXCEPT the page-end buffer, where the firmware sends 0.
    pub energy: u8,
    /// Cut mode on the image buffers (1 on the captured multi-buffer page).
    pub cut: u8,
    /// Material type (0 = continuous tape).
    pub mat: u8,
    /// Header `margin_top` (feed dots before the image). Captured: 1.
    pub margin_top: u16,
    /// Header `margin_bottom` (feed dots after the image). Captured: 1. Raising
    /// this is the way to feed the tape clear of the trailing cut (trailing
    /// blank raster columns get trimmed by the firmware).
    pub margin_bottom: u16,
}

impl Default for ESeriesBufOpts {
    fn default() -> Self {
        // Defaults straight from the captured E10pro buffers.
        Self {
            nodu: 4,
            energy: 23,
            cut: 1,
            mat: 0,
            margin_top: 1,
            margin_bottom: 1,
        }
    }
}

/// Split column-major image data into E-series print buffers (one page).
///
/// Split column-major image data into E-series print buffers (one page).
///
/// `total_cols` is the number of raster columns in `image_data` (all are
/// shipped as image data — unlike the T50 [`split_into_buffers`], margins are
/// NOT carved out of the raster here). The header `margin_top`/`margin_bottom`
/// (feed-before/after-image dots) come from [`ESeriesBufOpts`].
///
/// Differs from [`split_into_buffers`] in these byte-verified ways:
///   * `nodu` (PAGE_REG) and `energy` (byte 12) are independent; `energy` is
///     written unclamped.
///   * `energy` is set to **0** on the page-end buffer (the firmware does this).
///   * `cut`/`mat`/`margin_*` come from [`ESeriesBufOpts`].
pub fn split_into_buffers_e(
    image_data: &[u8],
    per_line_byte: u8,
    total_cols: u16,
    opts: ESeriesBufOpts,
) -> Vec<[u8; PRINT_BUF_SIZE]> {
    let max_cols = (MAX_BUF_DATA / per_line_byte as usize) as u16;
    let mut buffers = Vec::new();
    let mut cols_remaining = total_cols;
    let mut current_col: u16 = 0;

    while cols_remaining > 0 {
        let cols_in_buf = cols_remaining.min(max_cols);
        let is_first = current_col == 0;
        let is_last = cols_remaining <= max_cols;

        let img_start = current_col as usize * per_line_byte as usize;
        let img_end = img_start + cols_in_buf as usize * per_line_byte as usize;
        let img_chunk = &image_data[img_start..img_end.min(image_data.len())];

        // The firmware zeroes the energy byte on the final buffer of a page.
        let energy = if is_last { 0 } else { opts.energy };

        let buf = build_print_buffer(&PrintBufferParams {
            image_data: img_chunk,
            per_line_byte,
            cols_in_buf,
            page_st: is_first,
            page_end: is_last,
            prt_end: is_last,
            margin_top: opts.margin_top,
            margin_bottom: opts.margin_bottom,
            nodu: opts.nodu,
            energy: Some(energy),
            cut: opts.cut,
            mat: opts.mat,
        });
        buffers.push(buf);
        current_col += cols_in_buf;
        cols_remaining -= cols_in_buf;
    }

    buffers
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_page_reg_bits_defaults() {
        let bits = build_page_reg_bits(&PageRegBits {
            nodu: 4,
            mat: 1,
            ..Default::default()
        });
        // b0: no flags, cut=0, savepaper=0 -> 0x00
        assert_eq!(bits[0], 0x00);
        // b1: first_cut=0, nodu=4 (<<2 = 0x10), mat=1 (<<6 = 0x40) -> 0x50
        assert_eq!(bits[1], 0x50);
    }

    #[test]
    fn test_build_page_reg_bits_first_last() {
        let bits = build_page_reg_bits(&PageRegBits {
            page_st: true,
            page_end: true,
            prt_end: true,
            nodu: 4,
            mat: 1,
            ..Default::default()
        });
        // b0: PageSt=0x02, PageEnd=0x04, PrtEnd=0x08 = 0x0E
        assert_eq!(bits[0], 0x0E);
        assert_eq!(bits[1], 0x50);
    }

    #[test]
    fn test_build_print_buffer_checksum() {
        let data = vec![0u8; 84 * 48]; // 84 cols * 48 bytes/line
        let buf = build_print_buffer(&PrintBufferParams {
            image_data: &data,
            per_line_byte: 48,
            cols_in_buf: 84,
            page_st: true,
            page_end: true,
            prt_end: true,
            margin_top: 8,
            margin_bottom: 8,
            nodu: 4,
            energy: None,
            cut: 0,
            mat: 1,
        });
        // Verify buffer structure
        assert_eq!(buf[6], 48); // bytes per line
        assert_eq!(buf[4], 84); // cols low
        assert_eq!(buf[5], 0); // cols high
        assert_eq!(buf[8], 8); // margin top
        assert_eq!(buf[12], 4); // density
        // Checksum should be non-zero (at least header bytes contribute)
        let chk = buf[0] as u16 | ((buf[1] as u16) << 8);
        assert!(chk > 0);
    }

    #[test]
    fn test_split_into_buffers() {
        // 48 bytes/line, total 240 cols, margins 8+8 = 224 image cols
        // max_cols = 4074/48 = 84
        // 224 / 84 = 2 full + 56 remainder = 3 buffers
        let per_line_byte = 48u8;
        let total_cols = 240u16;
        let image_data = vec![0u8; total_cols as usize * per_line_byte as usize];
        let bufs = split_into_buffers(&image_data, per_line_byte, total_cols, 8, 8, 4, 1);
        assert_eq!(bufs.len(), 3);
    }

    #[test]
    fn test_eseries_buffer_header_matches_capture() {
        // Reproduce the byte-verified E10pro D1 page header (docs/E_SERIES_PROTOCOL.md):
        // nodu=4, energy=23, cut=1, mat=0, margins 1/1; energy zeroed on page-end.
        let per_line_byte = 12u8;
        // 377 raster cols -> max_cols = 4074/12 = 339, so 339 + 38 = 2 buffers.
        let total_cols = 377u16;
        let image_data = vec![0u8; total_cols as usize * per_line_byte as usize];
        let bufs = split_into_buffers_e(
            &image_data,
            per_line_byte,
            total_cols,
            ESeriesBufOpts::default(),
        );
        assert_eq!(bufs.len(), 2);

        // buf1 (first, not last): PageSt + cut=1 -> b0=0x12, b1=0x10
        assert_eq!(bufs[0][2], 0x12, "buf1 PAGE_REG b0");
        assert_eq!(bufs[0][3], 0x10, "buf1 PAGE_REG b1 (nodu=4, mat=0)");
        assert_eq!(bufs[0][6], 12, "buf1 bytes/line");
        assert_eq!(bufs[0][8], 1, "buf1 margin_top");
        assert_eq!(bufs[0][10], 1, "buf1 margin_bottom");
        assert_eq!(bufs[0][12], 23, "buf1 energy (unclamped)");

        // buf2 (last/page-end): PageEnd + PrtEnd + cut=1 -> b0=0x1c, energy zeroed
        assert_eq!(bufs[1][2], 0x1c, "buf2 PAGE_REG b0");
        assert_eq!(bufs[1][3], 0x10, "buf2 PAGE_REG b1");
        assert_eq!(bufs[1][12], 0, "buf2 energy zeroed on page-end");
    }

    #[test]
    fn test_eseries_single_buffer_page() {
        // Single-buffer page (like the captured BB page): PageSt+PageEnd+PrtEnd, cut=0.
        let per_line_byte = 12u8;
        let total_cols = 137u16;
        let image_data = vec![0u8; total_cols as usize * per_line_byte as usize];
        let bufs = split_into_buffers_e(
            &image_data,
            per_line_byte,
            total_cols,
            ESeriesBufOpts {
                cut: 0,
                ..ESeriesBufOpts::default()
            },
        );
        assert_eq!(bufs.len(), 1);
        // PageSt|PageEnd|PrtEnd = 0x02|0x04|0x08 = 0x0e, cut=0
        assert_eq!(bufs[0][2], 0x0e, "single-buffer PAGE_REG b0");
        assert_eq!(bufs[0][4], 137, "single-buffer cols low");
    }
}
