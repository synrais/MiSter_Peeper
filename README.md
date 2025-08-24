# mister_peeper

Minimal per-frame sampler for the MiSTer scaler output.  
It reads the scaler header directly from `/dev/mem`, lightly polls for new frames, hashes each sampled frame to detect changes, and reports the dominant on-screen color using a fast RGB565 histogram over a sparse grid.

## Console Output

    time=HH:MM:SS  unchanged=secs  dom_rgb=#RRGGBB (Name)

## Features

- Zero-copy access to the MiSTer scaler (ASCAL) header at `0x20000000` via `/dev/mem`.
- Change detection using a per-frame hash with no retries or tolerance.
- Dominant color detection through a fast 5-6-5 histogram on a sparse grid.
- RGB565 autodetect that determines RGB/BGR and LE/BE ordering and re-detects on core/geometry change.
- Low CPU usage thanks to gentle ~100 Hz polling.
- Clear logs: one-line info to `stderr` on 565 variant choice and scaler changes.

## How It Works

- Maps ~24 MiB starting at the MiSTer scaler base (`0x20000000`) and parses the packed `FbHeader`.
- Waits for the frame counters in the scaler headers to tick (supports triple buffering and large triple layouts).
- Samples pixels on a sparse grid (default every 16 px) to:
  - Update a rolling FNV-like frame hash for change tracking.
  - Increment a 565 histogram using an epoch trick (no full clears).
- Picks the mode bin as the dominant color, expands to 8-bit, and prints `#RRGGBB` with a nearest name from a tiny palette.
- For 16-bit formats, chooses the correct RGB565 loader (RGB/BGR × LE/BE) with a single-frame, multi-offset, variance-based scorer and re-runs this detection whenever the scaler geometry or format changes.

## Requirements

- MiSTer FPGA environment exposing the scaler at `ASCAL` (0x20000000).
- Linux userland with access to `/dev/mem` (root or proper capabilities).
- `g++` (C++17 or newer recommended).

> Note: Reading `/dev/mem` typically requires root. Consider limiting scope with capabilities or a wrapper if hardening is needed.

## Build

    g++ -O3 -march=native -fno-exceptions -fno-rtti -Wall -Wextra -o mister_peeper mister_peeper.cpp

## Run

    sudo ./mister_peeper

Press Ctrl+C to exit (SIGINT/SIGTERM handled).

### Sample Output

    time=00:00:05  unchanged=0.000  dom_rgb=#101010 (Black)
    time=00:00:05  unchanged=0.016  dom_rgb=#101010 (Black)
    time=00:00:06  unchanged=0.998  dom_rgb=#C8C800 (Yellow)
    time=00:00:06  unchanged=0.014  dom_rgb=#C8C800 (Yellow)

`stderr` occasionally includes informational one-liners, e.g.:

    info=rgb16_loader variant=RGB565-LE samples=896
    info=scaler_changed w=1280 h=720 line=2560 fmt=0 triple=1

## Tunables (edit & rebuild)

At the top of the source file:

    static constexpr int    kStep            = 16;     // sampling grid step (pixels)
    static constexpr size_t FB_BASE_ADDRESS  = 0x20000000u; // MiSTer scaler base (ASCAL)
    static constexpr size_t MAP_LEN          = 2048u * 1024u * 12u; // ~24 MiB mapping
    static constexpr int    kPollMs          = 10;     // ~100 Hz idle polling

- `kStep`: Increase for lower CPU, decrease for finer color sampling.
- `kPollMs`: Increase to idle more, decrease for snappier frame pickup.

## Notes & Behavior

- Supports scaler pixel formats: RGB16 / RGB24 / RGBA32 (as exposed by the scaler).
- 24/32-bit paths assume RGB byte order.
- 16-bit path auto-detects RGB/BGR × LE/BE and re-detects on any scaler/core/geometry change.
- Prints only one line per frame to `stdout`. Info messages go to `stderr`.
- Dominant color’s name is the nearest of a small, human-friendly palette.

## Performance Tips

- Keep `kStep` at 16 or higher for minimal CPU load.
- Redirect `stdout` to a file if you want a log:

        sudo ./mister_peeper | tee peeper.log

- If running continuously, consider a lightweight service wrapper.

## Troubleshooting

- `open(/dev/mem)` fails: run as root (`sudo`) or grant capabilities:

        sudo setcap cap_sys_rawio,cap_dac_read_search+ep ./mister_peeper

  (Capability availability varies; using `sudo` is the simplest.)

- No header / wrong base: ensure `FB_BASE_ADDRESS` matches your MiSTer’s scaler mapping.
- Weird colors on 16-bit: the loader auto-detect re-triggers on geometry/format change; check `stderr` for `info=rgb16_loader` lines.
- High CPU: increase `kStep` and/or `kPollMs`.

---

Minimalism for the MiSTer scaler: grab frames, hash them, and know what color dominates—all with minimal fuss.
"""
