//! `supvan-cli` — a diagnostic tool for talking to a Supvan printer directly,
//! bypassing the IPP/CUPS stack. Connect over Bluetooth (an address) or USB HID
//! (a `/dev/hidrawN` path) and run a subcommand: `probe` (device/status/material/
//! version), `material` (loaded label + RFID + remaining count), `test-print`
//! (a built-in pattern), or `discover` (scan for Supvan Bluetooth devices).

use std::error::Error;
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use supvan_proto::bitmap::PRINTHEAD_WIDTH_MM;
use supvan_proto::printer::Printer;
use supvan_proto::status::{DEFAULT_LABEL_GAP_MM, DEFAULT_LABEL_HEIGHT_MM, MaterialInfo};

type CliResult = Result<(), Box<dyn Error>>;

#[derive(Parser)]
#[command(name = "supvan-cli", about = "Supvan T50 Pro printer tool")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Probe printer: check device, status, material, version info
    Probe {
        /// Bluetooth address or /dev/hidrawN path
        target: String,
    },
    /// Query and print label material info
    Material {
        /// Bluetooth address or /dev/hidrawN path
        target: String,
    },
    /// Send a test print pattern
    TestPrint {
        /// Bluetooth address or /dev/hidrawN path
        target: String,
        /// Print density (0-15)
        #[arg(short, long, default_value_t = 4)]
        density: u8,
    },
    /// Query the loaded material's resolution (RD_LAB_DPI, 0x22). Read-only;
    /// dumps the raw response and interprets it. Used to determine geometry
    /// for non-T50 printers (e.g. E-series tape models).
    Dpi {
        /// Bluetooth address or /dev/hidrawN path
        target: String,
    },
    /// Print a geometry-calibration pattern defined in dots (not mm). Measure
    /// the printed border with a ruler to derive the true dots/mm and head
    /// width for an unknown model. Geometry is fully overridable.
    Calibrate {
        /// Bluetooth address or /dev/hidrawN path
        target: String,
        /// Pattern width across the head, in dots (keep ≤ the tape's printable width)
        #[arg(long, default_value_t = 120)]
        width_dots: u32,
        /// Pattern length along the feed, in dots
        #[arg(long, default_value_t = 240)]
        length_dots: u32,
        /// Print density (0-15)
        #[arg(short, long, default_value_t = 4)]
        density: u8,
        /// Material/PaperType byte for the buffer header (0 = continuous tape / E-series)
        #[arg(long, default_value_t = 0)]
        mat: u8,
        /// Fill the whole area solid black (clearest burn test) instead of the border pattern
        #[arg(long, default_value_t = false)]
        solid: bool,
        /// START_PRINT parameter = material-type code (1=continuous tape, 2=die-cut, 3=plate; 0=T50 mode)
        #[arg(long, default_value_t = 1)]
        start_param: u16,
        /// Use the E-series (E10pro) print path: 0xD1/0xBB two-stage transfer,
        /// per-buffer energy byte, 2 MiB-dict LZMA (docs/E_SERIES_PROTOCOL.md).
        #[arg(long, default_value_t = false)]
        e_series: bool,
        /// E-series burn energy (byte 12), unclamped. Captured value: 23.
        #[arg(long, default_value_t = 23)]
        energy: u8,
        /// E-series PAGE_REG nodu field (captured: 4).
        #[arg(long, default_value_t = 4)]
        nodu: u8,
        /// E-series cut mode on image buffers (captured: 1 on multi-buffer page).
        #[arg(long, default_value_t = 1)]
        cut: u8,
        /// E-series only: blank dot-columns of raster BEFORE the pattern
        /// (feeds the start clear of the cutter).
        #[arg(long, default_value_t = 0)]
        lead_feed: u32,
        /// E-series only: blank dot-columns of raster AFTER the pattern
        /// (feeds the end clear of the cutter). Same mechanism/scale as lead_feed.
        #[arg(long, default_value_t = 0)]
        trail_feed: u32,
    },
    /// Scan for Supvan Bluetooth devices (via BlueZ D-Bus)
    Discover,
}

fn connect(target: &str) -> Result<Printer, Box<dyn Error>> {
    if target.starts_with("/dev/hidraw") {
        eprintln!("Opening USB HID {target}...");
    } else {
        eprintln!("Connecting to {target} (Bluetooth)...");
    }
    let printer = Printer::open_target(target)?;
    eprintln!("Connected.");
    Ok(printer)
}

fn cmd_probe(target: &str) -> CliResult {
    let printer = connect(target)?;

    if printer.check_device()? {
        eprintln!("Device: OK");
    } else {
        return Err("device check: no response".into());
    }

    if let Some(status) = printer.query_status()? {
        eprintln!("Status:");
        eprintln!("  printing:     {}", status.printing);
        eprintln!("  device_busy:  {}", status.device_busy);
        eprintln!("  buf_full:     {}", status.buf_full);
        eprintln!("  low_battery:  {}", status.low_battery);
        eprintln!("  cover_open:   {}", status.cover_open);
        eprintln!("  print_count:  {}", status.print_count);
        if let Some(errs) = status.error_description() {
            eprintln!("  ERRORS:       {errs}");
        }
    }

    if let Some(name) = printer.read_device_name()? {
        eprintln!("Device name: {name}");
    }
    if let Some(fw) = printer.read_firmware_version()? {
        eprintln!("Firmware:    {fw}");
    }
    if let Some(ver) = printer.read_version()? {
        eprintln!("Protocol:    {ver}");
    }

    if let Some(mat) = printer.query_material()? {
        eprintln!("Material:");
        eprintln!("  Label:     {}mm x {}mm", mat.width_mm, mat.height_mm);
        eprintln!("  Type:      {}", mat.label_type);
        eprintln!("  Gap:       {}mm", mat.gap_mm);
        eprintln!("  SN:        {}", mat.sn);
        eprintln!("  UUID:      {}", mat.uuid);
        eprintln!("  Code:      {}", mat.code);
        if let Some(remaining) = mat.remaining {
            eprintln!("  Remaining: {remaining} labels");
        }
        if let Some(ref dev_sn) = mat.device_sn {
            eprintln!("  Device SN: {dev_sn}");
        }
    }
    Ok(())
}

fn cmd_material(target: &str) -> CliResult {
    let printer = connect(target)?;

    if !printer.check_device()? {
        return Err("device not responding".into());
    }

    let mat = printer
        .query_material()?
        .ok_or("no material info (label not installed?)")?;

    println!(
        "Label:     {}mm x {}mm  (type={}, gap={}mm)",
        mat.width_mm, mat.height_mm, mat.label_type, mat.gap_mm
    );
    println!("Label SN:  {}", mat.sn);
    println!("RFID UID:  {}", mat.uuid);
    println!("RFID code: {}", mat.code);
    match mat.remaining {
        Some(r) => println!("Remaining: {r} labels"),
        None => println!("Remaining: (not reported)"),
    }
    match mat.device_sn {
        Some(s) => println!("Device SN: {s}"),
        None => println!("Device SN: (not in this response)"),
    }
    Ok(())
}

fn cmd_test_print(target: &str, density: u8) -> CliResult {
    let printer = connect(target)?;

    // Query material to get label dimensions, falling back to printhead-width
    // defaults if no label is installed.
    let mat = match printer.query_material()? {
        Some(m) => m,
        None => {
            eprintln!(
                "No material info, using defaults ({PRINTHEAD_WIDTH_MM}mm x {DEFAULT_LABEL_HEIGHT_MM}mm)"
            );
            MaterialInfo {
                width_mm: PRINTHEAD_WIDTH_MM as u8,
                height_mm: DEFAULT_LABEL_HEIGHT_MM,
                gap_mm: DEFAULT_LABEL_GAP_MM,
                ..Default::default()
            }
        }
    };

    eprintln!(
        "Printing test pattern on {}mm x {}mm label...",
        mat.width_mm, mat.height_mm
    );
    printer.test_print(&mat, density)?;
    eprintln!("Done.");
    Ok(())
}

/// Query the DPI/resolution command(s) and interpret the raw response.
///
/// The vendor app exposes more than one resolution-query command depending on
/// the printer subclass: `RD_LAB_DPI` (0x22) on some, `RD_LAB_DPI24` (0x24) /
/// `RD_LAB_DPI25` (0x25) on others. We don't know which the E10pro answers, so
/// we send each in turn and dump whatever comes back — unfiltered — then scan
/// every little-endian u16 for a value in the plausible 11.0–12.8 dots/mm range
/// (1100–1280 when scaled ×100), flagging candidates.
fn cmd_dpi(target: &str) -> CliResult {
    let printer = connect(target)?;

    if !printer.check_device()? {
        return Err("device not responding".into());
    }

    // PaperType drives the app's field-offset choice; surface it for context.
    match printer.query_material()?.map(|m| m.label_type) {
        Some(t) => eprintln!("PaperType (label_type): {t}"),
        None => eprintln!("PaperType: (no material reported)"),
    }

    // (command byte, app name)
    const DPI_CMDS: [(u8, &str); 3] =
        [(0x22, "RD_LAB_DPI"), (0x24, "RD_LAB_DPI24"), (0x25, "RD_LAB_DPI25")];

    let mut any_response = false;
    for (cmd, name) in DPI_CMDS {
        println!("\n=== {name} (0x{cmd:02X}) ===");
        match printer.send_raw_cmd(cmd, 0)? {
            None => println!("  (no response — printer does not recognize this command)"),
            Some(resp) => {
                any_response = true;
                println!("  raw ({} bytes): {}", resp.len(), hex_dump(&resp));
                if resp.len() > 7 {
                    let echo = resp[7];
                    println!(
                        "  echo byte[7]: 0x{echo:02X} ({})",
                        if echo == cmd { "matches command" } else { "differs" }
                    );
                }
                report_dpi_candidates(&resp);
            }
        }
    }

    if !any_response {
        println!(
            "\nNo DPI command answered. Re-run with RUST_LOG=debug to see raw TX/RX, \
             and the value may need to be read mid-print-flow instead."
        );
    }
    println!("\nFor comparison, the T50 family the driver hardcodes is 8.0 dots/mm (203 DPI).");
    Ok(())
}

/// Scan a response for any LE u16 that maps to 11.0–12.8 dots/mm and print it.
fn report_dpi_candidates(resp: &[u8]) {
    let mut found = false;
    for i in 0..resp.len().saturating_sub(1) {
        let v = u16::from_le_bytes([resp[i], resp[i + 1]]);
        let dpm = v as f64 / 100.0;
        if (11.0..=12.8).contains(&dpm) {
            println!(
                "  candidate @offset {i:>2}: 0x{v:04X} ({v}) -> {dpm:.3} dots/mm ({:.0} DPI)",
                dpm * 25.4
            );
            found = true;
        }
    }
    if !found {
        println!("  (no dots/mm candidate in the 11.0–12.8 range)");
    }
}

/// Space-separated uppercase hex of a byte slice.
fn hex_dump(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[allow(clippy::too_many_arguments)]
fn cmd_calibrate(
    target: &str,
    width_dots: u32,
    length_dots: u32,
    density: u8,
    mat: u8,
    solid: bool,
    start_param: u16,
    e_series: bool,
    energy: u8,
    nodu: u8,
    cut: u8,
    lead_feed: u32,
    trail_feed: u32,
) -> CliResult {
    if width_dots < 8 || length_dots < 8 {
        return Err("width_dots and length_dots must be at least 8".into());
    }
    let printer = connect(target)?;
    if e_series {
        eprintln!(
            "E-series calibration print: {width_dots} x {length_dots} dots (energy={energy}, nodu={nodu}, cut={cut}, mat={mat}, lead_feed={lead_feed}, trail_feed={trail_feed}, solid={solid})."
        );
        if !solid {
            eprintln!("After it prints, measure the OUTER border with a ruler:");
            eprintln!("  dots/mm (across) = {width_dots} / measured_width_mm");
            eprintln!("  dots/mm (feed)   = {length_dots} / measured_length_mm");
        }
        let opts = supvan_proto::buffer::ESeriesBufOpts {
            nodu,
            energy,
            cut,
            mat,
            ..Default::default()
        };
        printer.calibrate_print_e(
            width_dots,
            length_dots,
            energy,
            solid,
            lead_feed,
            trail_feed,
            opts,
        )?;
        eprintln!("Done.");
        return Ok(());
    }
    eprintln!(
        "Calibration print: {width_dots} x {length_dots} dots (density={density}, mat={mat}, solid={solid}, start_param={start_param})."
    );
    if !solid {
        eprintln!("After it prints, measure the OUTER border with a ruler:");
        eprintln!("  dots/mm (across) = {width_dots} / measured_width_mm");
        eprintln!("  dots/mm (feed)   = {length_dots} / measured_length_mm");
    }
    printer.calibrate_print_opts(width_dots, length_dots, density, mat, solid, start_param)?;
    eprintln!("Done.");
    Ok(())
}

fn cmd_discover() {
    eprintln!("Scanning for Supvan devices...");
    eprintln!("(For full D-Bus discovery, use the CUPS backend with 0 args)");
    eprintln!();
    eprintln!("Manual discovery:");
    eprintln!("  bluetoothctl devices | grep -i 'T0117\\|T50\\|Supvan\\|Katasymbol'");
}

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let cli = Cli::parse();
    let result = match cli.command {
        Command::Probe { target } => cmd_probe(&target),
        Command::Material { target } => cmd_material(&target),
        Command::TestPrint { target, density } => cmd_test_print(&target, density),
        Command::Dpi { target } => cmd_dpi(&target),
        Command::Calibrate {
            target,
            width_dots,
            length_dots,
            density,
            mat,
            solid,
            start_param,
            e_series,
            energy,
            nodu,
            cut,
            lead_feed,
            trail_feed,
        } => cmd_calibrate(
            &target,
            width_dots,
            length_dots,
            density,
            mat,
            solid,
            start_param,
            e_series,
            energy,
            nodu,
            cut,
            lead_feed,
            trail_feed,
        ),
        Command::Discover => {
            cmd_discover();
            Ok(())
        }
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Cli, Command};
    use clap::Parser;

    #[test]
    fn parse_probe_with_target() {
        let cli = Cli::try_parse_from(["supvan-cli", "probe", "/dev/hidraw3"]).unwrap();
        match cli.command {
            Command::Probe { target } => assert_eq!(target, "/dev/hidraw3"),
            _ => panic!("expected Probe"),
        }
    }

    #[test]
    fn probe_requires_target() {
        // `target` is a required positional now (no hardcoded default).
        assert!(Cli::try_parse_from(["supvan-cli", "probe"]).is_err());
    }

    #[test]
    fn parse_test_print_density() {
        let cli = Cli::try_parse_from([
            "supvan-cli",
            "test-print",
            "AA:BB:CC:DD:EE:FF",
            "--density",
            "7",
        ])
        .unwrap();
        match cli.command {
            Command::TestPrint { target, density } => {
                assert_eq!(target, "AA:BB:CC:DD:EE:FF");
                assert_eq!(density, 7);
            }
            _ => panic!("expected TestPrint"),
        }
    }

    #[test]
    fn parse_discover() {
        let cli = Cli::try_parse_from(["supvan-cli", "discover"]).unwrap();
        assert!(matches!(cli.command, Command::Discover));
    }
}
