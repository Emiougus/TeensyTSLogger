# Currently only tested on EpicEFi firmware from 2025 Sep/should work with other RusEFI and other versions of EpicEFI but output channel block size might be different and cause errors

# TeensyTSLogger

A standalone data logger for ECUs using Tunerstudio protocols built on a **Teensy 4.1**. It connects to the ECU over USB host, reads output channels via the TunerStudio binary protocol, and writes log files directly to an SD card in MegaLogViewer `.msl` format — no laptop required.

## Features

- Plug-and-play USB host connection to RusEFI ECU
- Reads ECU output channels using TunerStudio CRC binary protocol at 20 Hz
- Logs all channels defined in your tune's INI file (or a `[Datalog]` subset)
- Writes standard `.msl` files readable by [MegaLogViewer](https://www.efianalytics.com/MegaLogViewer/)
- Human-readable filenames: `1201pm Feb 25 2026.msl` inside a `Feb 25 2026/` folder
- Falls back to `LOG001.msl` sequential naming if RTC is not set
- Internal RTC keeps time between power cycles (requires coin cell on VBAT pin)
- SD card accessible via MTP when logging is stopped — copy logs with Image Capture on macOS
- LED status indicator for all states

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Teensy 4.1 |
| SD card | Built-in SDIO slot (formatted exFAT or FAT32) |
| RTC backup | Coin cell connected to Teensy VBAT pin |
| ECU connection | USB A host cable to RusEFI USB port |

No external RTC module is required — the Teensy 4.1 has a built-in RTC backed by VBAT.

> **Note on back-power:** The ECU's USB port can back-feed power to the Teensy through the host cable. If the Teensy fails to boot when the ECU is connected but unpowered, turn car off and make sure Teensy boots before ECU is powered.

## SD Card Setup

1. Format the SD card as exFAT or FAT32
2. Copy your RusEFI tune INI file to the root of the SD card

The logger looks for the INI file by a hash of the ECU's firmware signature:

```
/<XXXXXXXX>.INI    ← preferred (named by djb2 hash of ECU signature)
/DEFAULT.INI       ← fallback for single-tune setups
```

The simplest setup is to rename your INI to `DEFAULT.INI`. To find the correct hash filename, check the serial output after connecting the ECU — it prints the expected filename.

## Log File Layout

```
/Feb 25 2026/
    1201pm Feb 25.msl
    0530pm Feb 25.msl
    ...
/LOG001.msl         ← fallback if RTC not set
```

## LED Status

| Pattern | Meaning |
|---------|---------|
| Slow blink (1 Hz) | Waiting for ECU |
| Fast blink (5 Hz) | Connecting / handshake |
| Short flash (1 Hz) | Logging |
| Medium blink (2.5 Hz) | Stopped — SD accessible via MTP |
| Solid on | Error (SD not found or INI missing) |

## Building & Flashing

This project uses [PlatformIO](https://platformio.org/).

```bash
pio run --target upload
```

Or use the PlatformIO extension in VS Code.

**Dependencies** (fetched automatically by PlatformIO):
- [MTP_Teensy](https://github.com/KurtE/MTP_Teensy) — MTP device support
- `USBHost_t36`, `SD`, `TimeLib` — bundled with Teensyduino

### Disabling MTP for Serial Debugging

If you need to use the serial monitor without MTP interference, comment out the MTP flag and library in [platformio.ini](platformio.ini):

```ini
build_flags =
    ; -D USB_MTPDISK_SERIAL
    -D USB_SERIAL
    -D DISABLE_MTP

lib_deps =
    ; https://github.com/KurtE/MTP_Teensy.git
```


## Accessing Log Files

### macOS
The Teensy appears as an MTP device. Use **Image Capture** (built-in macOS app) to browse and copy log files off the SD card. Set "Connecting this device opens: No application" to prevent auto-import.

> macOS Finder does not support MTP natively. Image Capture works without any extra software.

### Windows
The Teensy MTP device should appear in File Explorer automatically.

## RTC / Time

The Teensy's internal RTC is automatically set to the compile timestamp every time you flash new firmware (if the stored time is older than the compile time). This keeps the clock reasonably accurate across firmware updates.

A coin cell connected to the **VBAT** pin on the Teensy 4.1 keeps the RTC running when the board is unpowered.
