# E-series (E10pro) print protocol — byte-verified from btsnoop

Source of truth: a real Katasymbol → E10pro RFCOMM/SPP capture, decoded in
`capture/s20/` (Samsung S20 FE AOSP `btsnoop_hci.log`). All field values below
were re-derived directly from the captured frames and the decompressed print
buffers (`capture/s20/dec/cmd{d1,bb}_dec.bin`), **not** from the vendor app's
JS or the T50 driver. Where this document contradicts the earlier handoff
prose, this document is correct (the handoff misread several little-endian
param fields).

Printer: E10pro, MAC `A4:93:40:42:79:49`, 15 mm continuous tape.
Transport: classic Bluetooth RFCOMM/SPP channel 1 (same as T50).

## Decoded print buffers (the header is the same 14-byte layout as T50)

The E-series uses the **same 14-byte print-buffer header** as the T50
(`buffer.rs:build_print_buffer`), with different field values:

```
[0..2]  checksum LE
[2..4]  PAGE_REG_BITS (b0, b1)
[4..6]  column count LE (cols in this buffer)
[6]     bytes per line  (= 12 for the E10pro 96-dot head)
[7]     0
[8..10] margin_top LE   (= 1)
[10..12] margin_bottom LE (= 1)
[12]    energy / burn density   (independent of nodu; see below)
[13]    0
[14..]  column-major LSB-first raster
```

Three real buffers from a small solid-ish print:

| buffer | b0 | b1 | cols | bpl | mt | mb | energy | PAGE_REG decode |
|---|---|---|---|---|---|---|---|---|
| D1 buf1 | 0x12 | 0x10 | 340 | 12 | 1 | 1 | **23** | PageSt, cut=1, nodu=4, mat=0 |
| D1 buf2 | 0x1c | 0x10 | 37  | 12 | 1 | 1 | **0**  | PageEnd, PrtEnd, cut=1, nodu=4, mat=0 |
| BB buf  | 0x0e | 0x10 | 137 | 12 | 1 | 1 | **23** | PageSt, PageEnd, PrtEnd, cut=0, nodu=4, mat=0 |

PAGE_REG_BITS layout (matches `buffer.rs:build_page_reg_bits`):
- b0: `PageSt(0x02) | PageEnd(0x04) | PrtEnd(0x08) | cut<<4 | savepaper<<7`
- b1: `first_cut(0..1) | nodu<<2 | mat<<6`

### Key header findings (corrections vs the handoff)

1. **`energy` (byte 12) and `nodu` (PAGE_REG b1) are independent.** `nodu`
   is a fixed **4** on every E-series buffer. `energy` is the real burn knob
   and is **23** here — the T50 driver clamps byte 12 to `MAX_DENSITY=15` and
   writes the same value into both fields, so it never sends enough energy →
   blank output. This is the primary blank-output cause.
2. **`energy` is per-buffer and is `0` on the last buffer of a page.** D1 buf2
   (PageEnd+PrtEnd) carries energy=0 despite holding real raster. Only the
   first/middle buffers of a page carry the real energy value.
3. `mat = 0` (continuous tape), not 1. `margin_top = margin_bottom = 1`, not 8.
4. `cut = 1` on the D1 (multi-buffer) page's image buffers; `cut = 0` on the
   single-buffer BB page. (Effect on a continuous-tape unit TBD.)

### Geometry
`bytes/line = 12` → **96-dot** printhead (12×8). At 15 mm tape ≈ 6.4 dots/mm
across the head — NOT the 11.8 figure from `key-functions.js` (that is the
feed-direction DPI clamp). Confirm with a ruler once non-blank prints work.

#### Measured geometry (live calibration prints)
- **Resolution ≈ 8 dots/mm both axes (≈203 DPI)** — the 11.8 dots/mm figure from
  `key-functions.js` was a red herring (a feed-rate clamp, not print res).
- **Feed direction:** a 240-dot pattern measured ~30 mm (8.0 dots/mm); a
  100-dot length *difference* (60→160 dots) produced ~13.25 mm of extra tape
  (7.5 dots/mm). So feed res is ~7.5–8.0 dots/mm.
- **Cross-head:** a 96-dot pattern measured ~11 mm of black on the 15 mm tape
  (≈ 8.7 dots/mm), full rectangle visible (nothing clipped). **A 120-dot-wide
  pattern printed NOTHING** — so the printable head is between 96 and 120 dots
  wide; 96 dots (≈12 mm) is within range, 120 (15 bytes/line) is rejected.
- Working assumption pending a precise width read: **DOTS_PER_MM = 8,
  printhead = 96 dots (12 mm printable)** on 15 mm tape.

#### Feed behaviour (manual cutter)
- The vendor app adds **no software feed margin**: every captured buffer has
  header `margin_top = margin_bottom = 1`, and after `BUF_FULL` the app sends
  only status polls (no feed/advance command). The leading margin is
  effectively zero.
- Feed/whitespace is carried as **blank raster columns** in the image, not
  header margins. The captured BB page was ~88% trailing blank columns; the D1
  page had 1 leading + 23 trailing blank columns. (My earlier guess that the
  firmware trims trailing blank raster was wrong — it feeds them fine.)
- There is a **fixed ~3 mm (≈24-dot) mechanical printhead→cutter gap** that
  always lands on the leading side: a requested 2 mm leading margin printed as
  ~5 mm. The trailing side gets exactly the blank columns supplied.
- **Driver/IPP print path: adds zero feed** — prints the raster exactly as
  given (callers supply their own whitespace). Only the CLI `calibrate
  --e-series` command pads, via `--lead-feed`/`--trail-feed` (desired dot
  margins; the leading pad is reduced by the head→cutter gap so equal values
  give visually equal margins). See `printer.rs:HEAD_CUTTER_GAP_DOTS`.

## Transfer framing — two-stage `0xD1` / `0xBB`, NOT the driver's `0x5C` data-packet path

Each **page** is sent as its own bulk command. A page is split internally on
the `MAX_BUF_DATA` (4074-byte) boundary, same as `split_into_buffers`.

- **`0xD1`** carries the first page (here 2 buffers → 8192 decompressed bytes).
- **`0xBB`** carries the next page (here 1 buffer → 4000 decompressed bytes,
  PageSt+PageEnd+PrtEnd — a complete separate page; the D1 page already ended).

### Bulk frame layout (differs from `data.rs:wrap_data_frame`)

Driver T50 bulk frame:  `7E 5A FC01 10 02 | AA BB [chk LE][idx][tot] <500B>`
E-series bulk frame:    `7E 5A FC01 10 02 AA | <cmd> [chk LE][idx][tot] <500B LZMA>`

i.e. the E-series reuses the **command marker `10 02 AA` + a command byte**
(`0xD1` or `0xBB`), then a 4-byte chunk header `[checksum LE u16][idx u8][total
u8]`, then exactly 500 LZMA bytes. Each SPP write is exactly **512 bytes**:
`7E 5A FC 01 10 02 AA <cmd>` (8 bytes) + `[chk LE u16][idx][tot]` (4 bytes) +
500 LZMA bytes = 512. **Verified** against all three captured chunks.

- **Chunk checksum** = LE u16 sum over `[idx] + [tot] + the 500 LZMA bytes`
  (i.e. sum of frame bytes `[10:]`). Confirmed exact on all three chunks.
- **The final chunk is zero-padded to 500 bytes.** The BB page's real LZMA
  stream is only ~92 bytes but the chunk ships 500 (408 trailing zeros). The
  firmware reads the definite uncompressed size from the LZMA header and stops,
  ignoring the pad. So a page ships `total = ceil(lzma_len / 500)` chunks, each
  500 bytes, last one zero-padded.

LZMA is **LZMA1-alone**, prop=0x5D (lc=3,lp=0,pb=2), **dict=0x00200000 (2 MiB)**
in the captured header — the driver currently uses dict_size=8192. The firmware
accepted 8192-dict streams in prior experiments (fed correct length), so dict
size is probably not load-bearing, but the E-series path will use 2 MiB to match
the app exactly and eliminate it as a variable. Decoding the alone stream
requires streaming with `max_length=declared_size` (the encoder wrote both a
definite size AND an end marker); `decompress_lzma` already handles this by
patching the size field to `u64::MAX`.

## Full command sequence (TX = phone → printer), print window

```
0x30 RETURN_MAT          (param 0)
0x18 CHECK_DEVICE        (param 0; E-series uses 0x18, driver uses 0x12 — both answered)
0xD0  setup    plen=21  body: 02 00 00 02 00 00 f8 01 03 00 00 00 00
                         (f8 01 = 0x01F8 = 504 ~ total feed cols; 03 = count? content-dependent)
0xD1  bulk     idx=0 tot=2   (512B SPP write)
0xD1  bulk     idx=1 tot=2   → 1000 LZMA → 8192 = 2 buffers (page 1)
0xB0  setup    plen=22  body: <param 0x0198> "20260625"  (ASCII date string)
0x30 / 0x18              (status)
0xC9  setup    param=0x006E (110)        content-dependent?
0x13 START_PRINT param=0x0000            ← NOT a material-type code
0xBA  setup    param=0x0019 (25)         content-dependent?
0x5C NEXTFRM_BULK  send_cmd_two(0x5C, 512, 1)   ← block_size=512, block_count=num_packets (matches driver)
0xBB  bulk     idx=0 tot=1   → 500 LZMA → 4000 = 1 buffer (page 2)
0x10 BUF_FULL  send_cmd_two(0x10, 0, 0)         ← both params ZERO (driver sends len,speed)
... 0x11 status polls until printing clears ...
```

Standard command frames (`7E 5A 0C00 10 01 AA <cmd> <chk> 00 01 <param LE> 0000`)
are byte-identical to `cmd.rs:make_cmd`, so `make_cmd` is correct for E-series.
The setup commands `0xD0/0xB0/0xC9/0xBA` carry content-dependent params that are
only partially reverse-engineered from this single capture — treat C9/BA as
empirical constants and D0/B0 as structured (col count / date) until a second
capture confirms.

## Implementation status / open questions
- D0/C9/BA param derivation is unconfirmed (single capture). A second capture
  of a differently-sized label would pin down which fields scale with content.
- Whether `cut`, the 2 MiB dict size, or the energy-zeroing on the last buffer
  are load-bearing for non-blank output is to be determined empirically against
  the real printer.
