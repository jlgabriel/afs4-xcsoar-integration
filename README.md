# AFS4 XCSoar Integration

Bridge between **Aerofly FS4** flight simulator and **XCSoar** glide computer. Connects via DLL shared memory (preferred) or UDP, converts flight data to standard NMEA sentences, and serves them via TCP to XCSoar.

XCSoar gets full glide computer functionality: GPS position, barometric altitude, direct vario, IAS/TAS, wind direction and speed, and heading.

![AFS4 XCSoar Integration](https://img.shields.io/badge/AFS4-XCSoar%20Bridge-blue)
![Python 3.6+](https://img.shields.io/badge/Python-3.6%2B-green)
![License MIT](https://img.shields.io/badge/License-MIT-yellow)
![No Dependencies](https://img.shields.io/badge/Dependencies-None-brightgreen)

## Features

- **DLL shared memory support** — Direct access to AFS4 data via AeroflyReader DLL at 50-60Hz
- **Direct vario** — No calculation lag, instant vertical speed from the simulator
- **IAS / TAS** — Indicated airspeed from DLL, true airspeed via ISA atmosphere model
- **Wind** — Real wind direction and speed from the simulator (ECEF to local ENU conversion)
- **$LXWP0 sentence** — Condor3-compatible format for XCSoar with TAS, vario, and wind
- Real-time NMEA data conversion (GPRMC, GPGGA, HCHDT, PGRMZ, PTAS1, LXWP0)
- Multi-client TCP server (connect multiple XCSoar instances)
- GUI with real-time flight data display, DLL status, and connection indicators
- Terminal mode for headless operation
- Automatic fallback to UDP if DLL is not available
- Zero external dependencies (Python stdlib + tkinter only)
- Configurable update rate, ports, and magnetic variation

## Data Sources

| Feature | UDP Only | With DLL |
|---------|----------|----------|
| GPS position | XGPS protocol | Shared memory |
| Heading | XATT protocol | Shared memory (math→nav conversion) |
| Vario | Calculated from altitude deltas (with lag) | **Direct from simulator (50Hz, no lag)** |
| IAS / TAS | Not available | **IAS from DLL + ISA atmosphere model** |
| Wind | Not available | **Direct from simulator (ECEF→ENU projected)** |
| Baro altitude | MSL from GPS | **Actual simulator altitude** |

## Requirements

- Python 3.6 or later (or use the standalone .exe release)
- Aerofly FS 4
- XCSoar (PC or mobile)
- **For DLL mode**: [AeroflyReader.dll](https://github.com/jlgabriel/Aerofly-FS4-Bridge) installed in AFS4

## Quick Start

### 1. Install the DLL (recommended)

Copy `AeroflyReader.dll` to AFS4's external DLL folder:

```
%USERPROFILE%\Documents\Aerofly FS 4\external_dll\AeroflyReader.dll
```

Download the DLL from [Aerofly-FS4-Bridge releases](https://github.com/jlgabriel/Aerofly-FS4-Bridge/releases) or use the pre-built binary in `simplified/bin/`.

### 2. Configure Aerofly FS4 (for UDP fallback)

In Aerofly FS4, go to **Settings > Miscellaneous**:

- **Broadcast flight info to IP address**: `on`
- **Broadcast IP address**: `192.168.000.255` (or your network broadcast)
- **Broadcast IP port**: `49002`

> Note: UDP broadcast is optional when using the DLL, but recommended as a fallback.

### 3. Run the bridge

```bash
python afs4-xcsoar-integration.py
```

Or use the standalone executable: `AFS4-XCSoar-Bridge.exe`

This opens the GUI. Click **Start** to begin. The bridge will automatically detect the DLL if available.

For terminal mode (no GUI):

```bash
python afs4-xcsoar-integration.py --no-gui
```

### 4. Configure XCSoar

> **IMPORTANT**: XCSoar must be in **FLY mode**, not SIM mode. SIM mode disables all device ports.

In XCSoar, go to **Config > System > Devices** and configure a device:

| Setting | Value |
|---------|-------|
| **Port** | TCP Client |
| **IP Address** | `127.0.0.1` (same computer) or the bridge computer's IP |
| **TCP Port** | `4353` |
| **Driver** | **Condor3** (recommended with DLL) or Generic |

**Driver selection:**
- **Condor3** — Parses $LXWP0 for TAS, direct vario, and wind. Recommended when DLL is active.
- **Generic** — Parses GPRMC/GPGGA/PGRMZ/PTAS1. Works with and without DLL.

## Command Line Options

```
usage: afs4-xcsoar-integration.py [-h] [--udp-port UDP_PORT]
                                   [--tcp-port TCP_PORT]
                                   [--update-rate UPDATE_RATE]
                                   [--mag-var MAG_VAR]
                                   [--debug {0,1,2}]
                                   [--no-gui]
                                   [--no-dll]

Options:
  --udp-port     UDP port to receive Aerofly data (default: 49002)
  --tcp-port     TCP port to serve NMEA data (default: 4353)
  --update-rate  Update rate in Hz (default: 5)
  --mag-var      Magnetic variation in degrees, East positive (default: 0.0)
  --debug        Debug level: 0=minimal, 1=normal, 2=verbose (default: 1)
  --no-gui       Run in terminal mode without GUI
  --no-dll       Disable DLL shared memory reader (UDP only)
```

## NMEA Sentences

| Sentence | Source | Purpose |
|----------|--------|---------|
| `$GPRMC` | UDP + DLL | Position, speed, time, date, magnetic variation |
| `$GPGGA` | UDP + DLL | Position, altitude, fix quality |
| `$HCHDT` | UDP + DLL | True heading |
| `$PGRMZ` | UDP + DLL | Barometric altitude in feet |
| `$PTAS1` | UDP + DLL | Vertical speed, average vario, altitude, TAS |
| `$LXWP0` | DLL only | TAS, baro altitude, vario, wind (Condor3 format) |

## Architecture

```
Aerofly FS4
    |
    +--[DLL Shared Memory]---> DLLReader (50Hz, preferred)
    |   "AeroflyReaderData"        |
    |                              +-> ECEF to ENU wind conversion
    +--[UDP port 49002]------> AeroflyReceiver (fallback)
    |   XGPS/XATT protocol        |
    |                              v
    |                    NMEAConverter
    |                    (GPRMC, GPGGA, HCHDT, PGRMZ, PTAS1, LXWP0)
    |                              |
    |                    TCPServer (port 4353)
    |                              |
    +-----<--TCP NMEA sentences----+----> XCSoar
```

## Technical Notes

### DLL Data Conversions

The AFS4 DLL exposes raw simulator data that requires several conversions:

- **Heading**: AFS4 uses math convention (CCW from East). Converted to navigation (CW from North): `heading_nav = (90 - heading_math) % 360`
- **Longitude**: DLL returns 0-2π range. Normalized to -180°/+180°.
- **Wind vector**: AFS4 Vector3D values are in ECEF coordinates. Projected to local East-North-Up (ENU) frame using aircraft lat/lon.
- **IAS to TAS**: Full ISA standard atmosphere model (valid up to 11,000m troposphere ceiling).

### Network Setup

**Same Computer**: Use `127.0.0.1` as the IP address in XCSoar.

**Different Computers**: Run the bridge on the same computer as AFS4. Use the IP address shown by the bridge at startup. Ensure your firewall allows TCP connections on port 4353.

## Troubleshooting

### XCSoar shows "N/A" for the device
- **Most common cause**: XCSoar is in SIM mode. Switch to FLY mode.
- Check that the bridge is running and shows "Started TCP server on port 4353"

### GUI shows "DLL: Not found"
- Ensure AeroflyReader.dll is in `%USERPROFILE%\Documents\Aerofly FS 4\external_dll\`
- Start Aerofly FS4 and load a flight before starting the bridge
- The bridge will fall back to UDP mode automatically

### No data from Aerofly FS4
- Verify "Broadcast flight info to IP address" is ON in AFS4 settings
- Check that the broadcast port is 49002

### XCSoar connects but shows no vario
- With DLL: Vario should appear immediately (direct from simulator)
- Without DLL: The vario needs a few seconds of altitude data to start calculating

## License

MIT License - see [LICENSE](LICENSE) for details.

## Credits

- [XCSoar](https://xcsoar.org/) - Open source glide computer
- [Aerofly FS 4](https://www.aerofly.com/) - Flight simulator by IPACS
- [Aerofly-FS4-Bridge](https://github.com/jlgabriel/Aerofly-FS4-Bridge) - DLL bridge for AFS4 external apps
