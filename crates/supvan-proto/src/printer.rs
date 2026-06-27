//! High-level printer operations.
//!
//! Implements the print flow from T50PlusPrint.doPrint():
//! CHECK_DEVICE -> poll ready -> START_PRINT -> poll printing ->
//! transfer buffers -> poll complete.

use crate::cmd::*;
use crate::data::DATA_PAYLOAD_SIZE;
use crate::error::{Error, Result};
use crate::speed::calc_speed;
use crate::status::{MaterialInfo, PrinterStatus};
use crate::transport::Transport;
use std::time::Duration;

/// Status-poll attempt budgets for the print state machine; each is multiplied
/// by the poll interval inside its wait loop.
const READY_ATTEMPTS: usize = 60;
const PRINTING_ATTEMPTS: usize = 60;
const BUFFER_READY_ATTEMPTS: usize = 200;

/// Wait-for-completion budget: COMPLETION_POLLS × COMPLETION_POLL_INTERVAL = 30s.
const COMPLETION_POLL_INTERVAL: Duration = Duration::from_millis(100);
const COMPLETION_POLLS: usize = 300;

/// Fixed mechanical printhead→cutter gap (E10pro), in dots at 8 dots/mm.
///
/// When a print starts, the tape feeds this distance before the first column
/// reaches the cutter, so it always appears as extra leading margin. Measured
/// at ~3 mm (a requested 2 mm leading margin printed as ~5 mm). Used by
/// `calibrate_print_e` to balance the leading/trailing calibration margins; the
/// real print path adds no feed at all.
const HEAD_CUTTER_GAP_DOTS: u32 = 24;

/// High-level printer interface over a pluggable transport.
pub struct Printer {
    transport: Box<dyn Transport>,
}

impl Printer {
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Self { transport }
    }

    /// Open a USB HID printer at the given `/dev/hidrawN` path.
    pub fn open_usb(path: &str) -> Result<Self> {
        let dev = crate::hidraw::HidrawDevice::open(path)?;
        Ok(Self::new(Box::new(
            crate::usb_transport::UsbHidTransport::new(dev),
        )))
    }

    /// Open a Bluetooth printer at the given RFCOMM address (`AA:BB:CC:DD:EE:FF`).
    pub fn open_bt(addr: &str) -> Result<Self> {
        let sock = crate::rfcomm::RfcommSocket::connect_default(addr)?;
        Ok(Self::new(Box::new(crate::spp_pipe::SppCodec::new(sock))))
    }

    /// Open a BLE GATT printer by address (E11/E12-class hardware). Async
    /// because the `bluer` GATT client is natively async. Requires the `ble`
    /// feature.
    #[cfg(feature = "ble")]
    pub async fn open_ble(addr: &str) -> Result<Self> {
        let pipe = crate::ble::BlePipe::connect(addr).await?;
        Ok(Self::new(Box::new(crate::spp_pipe::SppCodec::new(pipe))))
    }

    /// Open a printer from a target string: a `/dev/hidrawN` path selects USB
    /// HID, anything else is treated as a Bluetooth address.
    pub fn open_target(target: &str) -> Result<Self> {
        if target.starts_with("/dev/hidraw") {
            Self::open_usb(target)
        } else {
            Self::open_bt(target)
        }
    }

    /// CHECK_DEVICE (0x12) - verify printer is present.
    pub async fn check_device(&self) -> Result<bool> {
        log::info!("CHECK_DEVICE");
        let resp = self.transport.send_cmd(CMD_CHECK_DEVICE, 0).await?;
        Ok(resp.is_some_and(|r| self.transport.validate_response(&r, CMD_CHECK_DEVICE)))
    }

    /// INQUIRY_STA (0x11) - query printer status.
    pub async fn query_status(&self) -> Result<Option<PrinterStatus>> {
        let resp = self.transport.send_cmd(CMD_INQUIRY_STA, 0).await?;
        Ok(resp.and_then(|r| self.transport.parse_status_response(&r)))
    }

    /// RETURN_MAT (0x30) - query material/label info.
    pub async fn query_material(&self) -> Result<Option<MaterialInfo>> {
        log::info!("RETURN_MAT");
        let resp = self.transport.send_cmd(CMD_RETURN_MAT, 0).await?;
        Ok(resp.and_then(|r| self.transport.parse_material_response(&r)))
    }

    /// RD_DEV_NAME (0x16) - read device name.
    pub async fn read_device_name(&self) -> Result<Option<String>> {
        log::info!("RD_DEV_NAME");
        let resp = self.transport.send_cmd(CMD_RD_DEV_NAME, 0).await?;
        Ok(resp.and_then(|r| self.transport.parse_device_name_response(&r)))
    }

    /// READ_FWVER (0xC5) - read firmware version.
    pub async fn read_firmware_version(&self) -> Result<Option<u8>> {
        log::info!("READ_FWVER");
        let resp = self.transport.send_cmd(CMD_READ_FWVER, 0).await?;
        Ok(resp.and_then(|r| self.transport.parse_firmware_version_response(&r)))
    }

    /// READ_REV (0x17) - read protocol version.
    pub async fn read_version(&self) -> Result<Option<String>> {
        log::info!("READ_REV");
        let resp = self.transport.send_cmd(CMD_READ_REV, 0).await?;
        Ok(resp.and_then(|r| self.transport.parse_version_response(&r)))
    }

    /// RD_LAB_DPI (0x22) - query loaded-material resolution; returns the raw,
    /// validated response frame.
    ///
    /// This is a diagnostic for non-T50 geometry (e.g. E-series tape printers,
    /// which the vendor app reports at ~11.8 dots/mm rather than the T50's 8).
    /// We deliberately return the *raw* response rather than a decoded value:
    /// the vendor app's field offset varies by PaperType and is indexed off a
    /// framing convention that differs from this crate's BT response buffer, so
    /// the caller dumps and interprets the bytes instead of trusting a guess.
    /// Returns `None` if there is no response or it doesn't echo the command.
    pub async fn read_label_dpi_raw(&self) -> Result<Option<Vec<u8>>> {
        log::info!("RD_LAB_DPI");
        let resp = self.transport.send_cmd(CMD_RD_LAB_DPI, 0).await?;
        Ok(resp.filter(|r| self.transport.validate_response(r, CMD_RD_LAB_DPI)))
    }

    /// Send an arbitrary command byte with one parameter and return whatever
    /// the printer sends back, **unfiltered** (no validation, no echo check).
    ///
    /// Diagnostic escape hatch for reverse-engineering unknown command codes
    /// (e.g. probing which DPI-query variant a given model answers). `None`
    /// means the read genuinely timed out with zero bytes received.
    pub async fn send_raw_cmd(&self, cmd: u8, param: u16) -> Result<Option<Vec<u8>>> {
        log::info!("RAW CMD 0x{cmd:02X} param={param}");
        self.transport.send_cmd(cmd, param).await
    }

    /// START_PRINT (0x13).
    ///
    /// `param` carries the material-type code on E-series printers
    /// (1=continuous tape, 2=die-cut, 3=plate); the T50 flow uses 0. A wrong
    /// value here engages the feed motor but not the burn mode (blank output).
    pub async fn start_print(&self, param: u16) -> Result<Option<Vec<u8>>> {
        log::info!("START_PRINT param={param}");
        self.transport.send_cmd(CMD_START_PRINT, param).await
    }

    /// STOP_PRINT (0x14).
    pub async fn stop_print(&self) -> Result<Option<Vec<u8>>> {
        log::info!("STOP_PRINT");
        self.transport.send_cmd(CMD_STOP_PRINT, 0).await
    }

    /// PAPER_SKIP (0x2E) — feed/advance one blank label. Returns `Ok(())` once
    /// the device acks; errors if there is no response.
    pub async fn paper_skip(&self) -> Result<()> {
        log::info!("PAPER_SKIP");
        let resp = self.transport.send_cmd(CMD_PAPER_SKIP, 0).await?;
        if resp.is_some_and(|r| self.transport.validate_response(&r, CMD_PAPER_SKIP)) {
            Ok(())
        } else {
            Err(Error::InvalidResponse("PAPER_SKIP: no ack".into()))
        }
    }

    /// Wait for device to be idle (not busy, not printing).
    pub async fn wait_ready(&self, max_attempts: usize) -> Result<Option<PrinterStatus>> {
        for _ in 0..max_attempts {
            let st = self.query_status().await?;
            if let Some(ref s) = st
                && !s.device_busy
                && !s.printing
            {
                return Ok(st);
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        Ok(None)
    }

    /// Wait for printing station to become active.
    ///
    /// Aborts early via `Error::InvalidResponse` if the printer raises an
    /// error flag (label end, cover open, mode mismatch, etc.) — those
    /// states cause the firmware to drop the BT link and beep, and there's
    /// no point continuing the print.
    pub async fn wait_printing(&self, max_attempts: usize) -> Result<Option<PrinterStatus>> {
        for _ in 0..max_attempts {
            let st = self.query_status().await?;
            if let Some(ref s) = st {
                if s.has_error() {
                    return Err(Error::InvalidResponse(format!(
                        "printer error after START_PRINT: {}",
                        s.error_description().unwrap_or_default()
                    )));
                }
                if s.printing {
                    return Ok(st);
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        Ok(None)
    }

    /// Wait for buffer space available (buf_full == false).
    pub async fn wait_buffer_ready(&self, max_attempts: usize) -> Result<Option<PrinterStatus>> {
        for i in 0..max_attempts {
            tokio::time::sleep(Duration::from_millis(20)).await;
            let st = self.query_status().await?;
            if let Some(ref s) = st {
                if s.has_error() {
                    return Err(Error::InvalidResponse(format!(
                        "printer error while waiting for buffer: {}",
                        s.error_description().unwrap_or_default()
                    )));
                }
                if !s.buf_full {
                    return Ok(st);
                }
            }
            if i % 10 == 0 && i > 0 {
                log::debug!("waiting for buffer space... ({i})");
            }
        }
        Ok(None)
    }

    /// Transfer the compressed print buffers as a single LZMA stream:
    /// NEXT_ZIPPEDBULK -> data packets -> BUF_FULL.
    ///
    /// The printer's decoder splits the decompressed stream on 4096-byte
    /// boundaries internally, so one transfer covers all the page's buffers.
    pub async fn transfer_compressed(&self, compressed: &[u8], speed: u16) -> Result<()> {
        let compressed_len = compressed.len() as u16;

        // CMD_NEXT_ZIPPEDBULK (0x5C): each transport encodes the header in its
        // own convention (SPP: block_size=512 + packet count; USB: total length).
        let num_packets = compressed.len().div_ceil(DATA_PAYLOAD_SIZE);
        log::info!(
            "transfer: {} bytes, {} packets, speed={}",
            compressed.len(),
            num_packets,
            speed
        );
        let resp = self
            .transport
            .send_bulk_header(compressed_len, num_packets)
            .await?;
        if resp.is_none() {
            return Err(Error::InvalidResponse(
                "no response to NEXT_ZIPPEDBULK".into(),
            ));
        }

        // Send data packets via the transport. We do NOT read a response
        // after the last frame: the protocol acks the bulk only via the
        // BUF_FULL reply that follows. Polling for a non-existent response
        // here blocks for the read timeout (2s on BT), during which the
        // printer queues the bytes, times out waiting for BUF_FULL, errors
        // (3-beep) and drops the RFCOMM link before BUF_FULL arrives.
        self.transport.send_bulk_data(compressed, false).await?;

        // 20ms delay after last data packet
        tokio::time::sleep(Duration::from_millis(20)).await;

        // CMD_BUF_FULL: param=compressed_length, param2=speed
        log::info!("BUF_FULL: len={}, speed={}", compressed_len, speed);
        self.transport
            .send_cmd_two(CMD_BUF_FULL, compressed_len, speed)
            .await?;

        Ok(())
    }

    /// Execute a full print job with pre-compressed data.
    ///
    /// This is the main print flow from T50PlusPrint.doPrint():
    /// 1. CHECK_DEVICE
    /// 2. Wait ready
    /// 3. START_PRINT
    /// 4. Wait printing station
    /// 5. Wait buffer ready + transfer
    /// 6. Wait completion
    pub async fn print_compressed(
        &self,
        compressed: &[u8],
        speed: u16,
        start_param: u16,
    ) -> Result<()> {
        // Step 1: Check device
        if !self.check_device().await? {
            return Err(Error::InvalidResponse("CHECK_DEVICE failed".into()));
        }

        // Step 2: Wait ready
        let status = self
            .wait_ready(READY_ATTEMPTS)
            .await?
            .ok_or_else(|| Error::InvalidResponse("timeout waiting for device ready".into()))?;
        if status.has_error() {
            return Err(Error::InvalidResponse(format!(
                "printer error: {}",
                status.error_description().unwrap_or_default()
            )));
        }

        // Step 3: Start print
        self.start_print(start_param).await?;

        // Step 4: Wait printing station
        self.wait_printing(PRINTING_ATTEMPTS)
            .await?
            .ok_or_else(|| Error::InvalidResponse("timeout waiting for printing station".into()))?;

        // Step 5: Wait buffer + transfer
        let buf_status = self
            .wait_buffer_ready(BUFFER_READY_ATTEMPTS)
            .await?
            .ok_or_else(|| Error::InvalidResponse("timeout waiting for buffer space".into()))?;
        if buf_status.has_error() {
            self.stop_print().await?;
            return Err(Error::InvalidResponse(format!(
                "printer error: {}",
                buf_status.error_description().unwrap_or_default()
            )));
        }
        self.transfer_compressed(compressed, speed).await?;

        // Step 6: Wait completion
        for _ in 0..COMPLETION_POLLS {
            tokio::time::sleep(COMPLETION_POLL_INTERVAL).await;
            if let Some(s) = self.query_status().await?
                && !s.printing
                && !s.device_busy
            {
                log::info!("print complete");
                return Ok(());
            }
        }

        log::warn!("timeout waiting for print completion");
        Err(Error::Timeout("print completion"))
    }

    /// Full test print workflow: generate test pattern, build buffers, compress, print.
    pub async fn test_print(&self, mat: &MaterialInfo, density: u8) -> Result<()> {
        use crate::bitmap::create_test_pattern;
        use crate::buffer::split_into_buffers;
        use crate::compress::compress_buffers;

        let label_width_mm = (mat.width_mm as u32).min(crate::bitmap::PRINTHEAD_WIDTH_MM);
        let height_mm = if mat.height_mm == 0 {
            crate::status::DEFAULT_LABEL_HEIGHT_MM as u32
        } else {
            mat.height_mm as u32
        };

        log::info!(
            "test print: {}mm x {}mm, density={}",
            label_width_mm,
            height_mm,
            density
        );

        let (image_data, _w, h, bpl) = create_test_pattern(label_width_mm, height_mm);
        let buffers = split_into_buffers(&image_data, bpl as u8, h as u16, 8, 8, density, 1);
        log::info!("{} print buffers", buffers.len());

        let (compressed, avg) = compress_buffers(&buffers)?;
        let speed = calc_speed(avg);
        log::info!(
            "compressed: {} bytes, avg={}/buf, speed={}",
            compressed.len(),
            avg,
            speed
        );

        self.print_compressed(&compressed, speed, 0).await
    }

    /// Print a geometry-calibration pattern defined purely in **dots** (no
    /// dots/mm assumption), so the operator can measure the printed result and
    /// back out the true resolution / printhead width of an unknown model.
    ///
    /// Unlike [`test_print`], this does NOT center the image in a fixed T50
    /// printhead canvas: it packs exactly `width_dots` across (the natural
    /// model for continuous-tape printers, where print width == declared
    /// `per_line_byte`). `mat` is the material/PaperType byte for the buffer
    /// header (0 for E-series continuous tape).
    ///
    /// The pattern is a 2-dot hollow border of exactly `width_dots`×`length_dots`
    /// plus 6-dot tick stubs every 10 dots along the top and left edges and a
    /// top-left→bottom-right diagonal (to reveal rotation/mirroring). Measure
    /// the outer border with a ruler: dots_per_mm = width_dots / measured_mm.
    pub async fn calibrate_print(
        &self,
        width_dots: u32,
        length_dots: u32,
        density: u8,
        mat: u8,
    ) -> Result<()> {
        self.calibrate_print_opts(width_dots, length_dots, density, mat, false, 1)
            .await
    }

    /// As [`calibrate_print`], but `solid` fills the entire area (every dot on)
    /// instead of drawing the border/tick pattern — the clearest test of
    /// whether the head burns at all and at what darkness.
    pub async fn calibrate_print_opts(
        &self,
        width_dots: u32,
        length_dots: u32,
        density: u8,
        mat: u8,
        solid: bool,
        start_param: u16,
    ) -> Result<()> {
        use crate::buffer::split_into_buffers;
        use crate::compress::compress_buffers;

        let bytes_per_line = width_dots.div_ceil(8);
        // Column-major LSB-first: outer index = column along feed (length),
        // inner = bytes across the head (width).
        let mut buf = vec![0u8; bytes_per_line as usize * length_dots as usize];
        let mut set = |x: u32, y: u32| {
            // x = dot across head (0..width_dots), y = column along feed.
            if x >= width_dots || y >= length_dots {
                return;
            }
            let idx = y as usize * bytes_per_line as usize + (x / 8) as usize;
            buf[idx] |= 1 << (x % 8);
        };

        for y in 0..length_dots {
            for x in 0..width_dots {
                let mut on = solid;
                // 2-dot outer border.
                if x < 2 || x >= width_dots - 2 || y < 2 || y >= length_dots - 2 {
                    on = true;
                }
                // Diagonal (orientation marker).
                if (x * (length_dots.max(1) - 1) / width_dots.max(1)).abs_diff(y) < 1 {
                    on = true;
                }
                // Tick stubs every 10 dots: 6 dots inward from top and left edges.
                if y % 10 == 0 && x < 6 {
                    on = true;
                }
                if x % 10 == 0 && y < 6 {
                    on = true;
                }
                if on {
                    set(x, y);
                }
            }
        }

        log::info!(
            "calibrate print: {width_dots}x{length_dots} dots, {bytes_per_line} bytes/line, density={density}, mat={mat}"
        );

        // No margins in the column (feed) direction — the pattern already
        // includes its own border; pass 0/0 so all length_dots columns ship.
        let buffers = split_into_buffers(&buf, bytes_per_line as u8, length_dots as u16, 0, 0, density, mat);
        log::info!("{} print buffers", buffers.len());

        let (compressed, avg) = compress_buffers(&buffers)?;
        let speed = calc_speed(avg);
        self.print_compressed(&compressed, speed, start_param).await
    }

    /// Execute an E-series (E10pro) print, byte-faithful to the captured
    /// Katasymbol sequence (`docs/E_SERIES_PROTOCOL.md`).
    ///
    /// `pages` is a list of pages; each page is a set of E-series print buffers
    /// built by [`crate::buffer::split_into_buffers_e`]. In the capture there
    /// were two pages: the first shipped via `0xD1`, the second (after a `0x5C`
    /// handshake) via `0xBB`. `total_feed_cols` is the whole job's column count
    /// for the `0xD0` setup field.
    ///
    /// Sequence (verified, two-page capture):
    ///   CHECK_DEVICE -> wait ready -> 0xD0 setup -> 0xD1 page[0] bulk ->
    ///   0xB0 (date) -> 0xC9 -> START_PRINT(0) -> wait printing -> 0xBA ->
    ///   for each subsequent page: 0x5C(512,n) -> wait buffer -> 0xBB bulk ->
    ///   0x10 BUF_FULL(0,0) -> wait completion.
    ///
    /// SINGLE-PAGE CAVEAT: we have no single-page capture, so when `pages` has
    /// one entry we still emit the `0x5C` + `0xBB` handshake carrying that same
    /// page, mirroring the two-stage handshake the firmware expects. If a
    /// single page double-prints, the single-page flow needs its own capture.
    /// The `0xC9`/`0xBA` params are reproduced as captured constants.
    pub async fn print_eseries(
        &self,
        pages: &[Vec<[u8; crate::buffer::PRINT_BUF_SIZE]>],
        total_feed_cols: u16,
    ) -> Result<()> {
        use crate::cmd::{make_cmd_ext, CMD_BUF_FULL, CMD_NEXT_ZIPPEDBULK};
        use crate::compress::compress_page_e;
        use crate::data::DATA_PAYLOAD_SIZE;

        if pages.is_empty() || pages[0].is_empty() {
            return Err(Error::InvalidParam("no pages/buffers".into()));
        }

        // Step 1: CHECK_DEVICE + wait ready.
        if !self.check_device().await? {
            return Err(Error::InvalidResponse("CHECK_DEVICE failed".into()));
        }
        let status = self
            .wait_ready(READY_ATTEMPTS)
            .await?
            .ok_or_else(|| Error::InvalidResponse("timeout waiting for device ready".into()))?;
        if status.has_error() {
            return Err(Error::InvalidResponse(format!(
                "printer error: {}",
                status.error_description().unwrap_or_default()
            )));
        }

        // Compress each page (2 MiB dict, as the app does).
        let page_lzma: Vec<Vec<u8>> = pages
            .iter()
            .map(|p| compress_page_e(p))
            .collect::<Result<_>>()?;
        log::info!(
            "E-series print: {} pages, {} feed cols",
            pages.len(),
            total_feed_cols
        );

        // Step 2: 0xD0 setup. Captured body (after the 00 01 prefix):
        //   02 00 00 02 00 00 <feed_cols LE> 03 00 00 00 00
        // The 0xf801 field (504) matched the captured feed cols; we write the
        // real total. Trailing bytes reproduced as captured constants.
        let fc = total_feed_cols.to_le_bytes();
        let d0_params = [
            0x02, 0x00, 0x00, 0x02, 0x00, 0x00, fc[0], fc[1], 0x03, 0x00, 0x00, 0x00, 0x00,
        ];
        self.transport
            .send_raw_cmd_frame(&make_cmd_ext(0xD0, &d0_params))
            .await?;

        // Step 3: 0xD1 carries the first page.
        self.transport.send_eseries_bulk(0xD1, &page_lzma[0]).await?;

        // Step 4: 0xB0 carries the current date as ASCII "YYYYMMDD" then zeros.
        let mut b0_params = current_date_yyyymmdd().into_bytes();
        b0_params.extend_from_slice(&[0, 0, 0, 0, 0, 0]);
        self.transport
            .send_raw_cmd_frame(&make_cmd_ext(0xB0, &b0_params))
            .await?;

        // Step 5: 0xC9 (captured param 0x006e). Empirical constant.
        self.transport.send_cmd(0xC9, 0x006e).await?;

        // Step 6: START_PRINT with param 0 (NOT a material-type code).
        self.start_print(0).await?;
        self.wait_printing(PRINTING_ATTEMPTS)
            .await?
            .ok_or_else(|| Error::InvalidResponse("timeout waiting for printing station".into()))?;

        // Step 7: 0xBA (captured param 0x0019). Empirical constant.
        self.transport.send_cmd(0xBA, 0x0019).await?;

        // Step 8: subsequent pages via 0x5C handshake + 0xBB bulk. For a single
        // page, the captured second page is absent; we still emit one 0xBB pass
        // carrying the first page to satisfy the two-stage handshake (see CAVEAT).
        let bb_pages: Vec<&Vec<u8>> = if pages.len() == 1 {
            vec![&page_lzma[0]]
        } else {
            page_lzma[1..].iter().collect()
        };
        for lzma in bb_pages {
            let num_chunks = lzma.len().div_ceil(DATA_PAYLOAD_SIZE).max(1);
            self.transport
                .send_cmd_two(CMD_NEXT_ZIPPEDBULK, 512, num_chunks as u16)
                .await?;
            let buf_status = self
                .wait_buffer_ready(BUFFER_READY_ATTEMPTS)
                .await?
                .ok_or_else(|| Error::InvalidResponse("timeout waiting for buffer space".into()))?;
            if buf_status.has_error() {
                self.stop_print().await?;
                return Err(Error::InvalidResponse(format!(
                    "printer error: {}",
                    buf_status.error_description().unwrap_or_default()
                )));
            }
            self.transport.send_eseries_bulk(0xBB, lzma).await?;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;

        // Step 9: BUF_FULL with BOTH params zero (captured), not (len, speed).
        self.transport.send_cmd_two(CMD_BUF_FULL, 0, 0).await?;

        // Step 10: wait completion.
        for _ in 0..COMPLETION_POLLS {
            tokio::time::sleep(COMPLETION_POLL_INTERVAL).await;
            if let Some(s) = self.query_status().await?
                && !s.printing
                && !s.device_busy
            {
                log::info!("E-series print complete");
                return Ok(());
            }
        }
        log::warn!("timeout waiting for E-series print completion");
        Err(Error::Timeout("print completion"))
    }

    /// E-series calibration print: build a dots-defined pattern, split into
    /// E-series buffers, and run [`print_eseries`].
    ///
    /// Feed handling is a **calibration convenience only** (the real print path
    /// adds no feed — callers supply their own whitespace). `lead_feed` /
    /// `trail_feed` are the desired blank margins in dots before / after the
    /// pattern. There is a fixed ~3 mm printhead→cutter gap ([`HEAD_CUTTER_GAP_DOTS`])
    /// that always lands on the leading side, so the leading raster pad is
    /// reduced by that gap to make equal `lead_feed`/`trail_feed` values yield
    /// visually equal margins on both ends.
    #[allow(clippy::too_many_arguments)]
    pub async fn calibrate_print_e(
        &self,
        width_dots: u32,
        length_dots: u32,
        energy: u8,
        solid: bool,
        lead_feed: u32,
        trail_feed: u32,
        opts: crate::buffer::ESeriesBufOpts,
    ) -> Result<()> {
        use crate::buffer::split_into_buffers_e;

        // The fixed mechanical printhead→cutter gap adds to the leading margin,
        // so subtract it from the leading raster pad to balance the ends.
        let lead_pad = lead_feed.saturating_sub(HEAD_CUTTER_GAP_DOTS);
        let trail_pad = trail_feed;

        let bytes_per_line = width_dots.div_ceil(8);
        // Raster feed extent = leading blank + pattern + trailing blank.
        let total_len = lead_pad + length_dots + trail_pad;
        let mut buf = vec![0u8; bytes_per_line as usize * total_len as usize];
        // `y` is the column within the pattern (0..length_dots), shifted by
        // `lead_pad` into the full buffer.
        let mut set = |x: u32, y: u32| {
            if x >= width_dots || y >= length_dots {
                return;
            }
            let feed_col = y + lead_pad;
            let idx = feed_col as usize * bytes_per_line as usize + (x / 8) as usize;
            buf[idx] |= 1 << (x % 8);
        };
        for y in 0..length_dots {
            for x in 0..width_dots {
                let mut on = solid;
                if x < 2 || x >= width_dots - 2 || y < 2 || y >= length_dots - 2 {
                    on = true;
                }
                if (x * (length_dots.max(1) - 1) / width_dots.max(1)).abs_diff(y) < 1 {
                    on = true;
                }
                if y % 10 == 0 && x < 6 {
                    on = true;
                }
                if x % 10 == 0 && y < 6 {
                    on = true;
                }
                if on {
                    set(x, y);
                }
            }
        }

        let opts = crate::buffer::ESeriesBufOpts { energy, ..opts };
        log::info!(
            "E-series calibrate: {width_dots}x{length_dots} dots pattern, lead_feed={lead_feed} (pad {lead_pad} + {HEAD_CUTTER_GAP_DOTS} gap), trail_feed={trail_feed}, total {total_len} feed cols, {bytes_per_line} bytes/line, energy={energy}, nodu={}, cut={}, mat={}",
            opts.nodu, opts.cut, opts.mat
        );

        // All `total_len` raster columns ship as image data (lead/trail blank
        // included), matching the app; header margins stay at opts (1/1).
        let buffers = split_into_buffers_e(&buf, bytes_per_line as u8, total_len as u16, opts);
        self.print_eseries(&[buffers], total_len as u16).await
    }
}

/// Current local date as "YYYYMMDD" for the E-series 0xB0 setup command.
fn current_date_yyyymmdd() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    // Minimal civil-date conversion (UTC) to avoid pulling in chrono.
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let days = secs / 86_400;
    // Howard Hinnant's days_from_civil inverse (civil_from_days).
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{:04}{:02}{:02}", y, m, d)
}
