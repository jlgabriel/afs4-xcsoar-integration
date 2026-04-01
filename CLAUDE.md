# afs4-xcsoar-integration

Bridge that converts Aerofly FS4 flight simulator data to NMEA format for XCSoar glide computer.

## Architecture

Single-file Python application (`afs4-xcsoar-integration.py`) with these components:

| Class | Responsibility |
|-------|---------------|
| `AeroflyReceiver` | UDP listener on port 49002, parses ForeFlight protocol (XGPS/XATT) |
| `DLLReader` | Reads AeroflyReader DLL shared memory ("AeroflyReaderData") for IAS, vario, wind |
| `VarioCalculator` | Calculates vertical speed from altitude deltas (fallback when no DLL) |
| `NMEAConverter` | Generates NMEA sentences: GPRMC, GPGGA, HCHDT, PGRMZ, PTAS1, LXWP0 |
| `TCPServer` | Serves NMEA to XCSoar clients on port 4353 |
| `AeroflyToXCSoar` | Orchestrator: DLL (preferred) or UDP -> vario -> converter -> TCP |
| `BridgeGUI` | Tkinter GUI with real-time flight data, DLL status, and connection status |
| `ias_to_tas()` | IAS to TAS conversion using full ISA standard atmosphere model |

## Data flow

```
AFS4 DLL -> Shared Memory -> DLLReader (50Hz, preferred)
AFS4 UDP 49002 -> AeroflyReceiver (fallback)
                          |
                    NMEAConverter -> TCPServer -> TCP 4353 -> XCSoar
```

## NMEA sentences generated

- **GPRMC** -- position, speed, time, magnetic variation
- **GPGGA** -- position, altitude, fix quality
- **HCHDT** -- true heading (from XATT or DLL)
- **PGRMZ** -- barometric/pressure altitude in feet (parsed by XCSoar Generic driver)
- **PTAS1** -- vario, average vario, altitude, TAS (parsed by XCSoar Generic driver)
- **LXWP0** -- TAS, baro alt, vario, wind (DLL only; parsed by XCSoar Condor3 driver)

## DLL integration

The DLLReader connects to AeroflyReader DLL shared memory ("AeroflyReaderData") and provides:
- **IAS** (indicated airspeed) -> converted to **TAS** using ISA atmosphere model (`ias_to_tas()`)
- **Vertical speed** (direct, 50-60Hz) -- bypasses VarioCalculator smoothing
- **Wind** (3D vector in ECEF) -> projected to local ENU, converted to direction/speed
- Auto-reconnection if DLL disconnects, transparent fallback to UDP

GPSData extended fields for DLL: `indicated_airspeed`, `wind_x`, `wind_y`, `wind_z` (all Optional[float]).
After ECEF-to-ENU conversion: `wind_x` = North component, `wind_z` = East component.

## AFS4 DLL data conventions (CRITICAL)

These conventions were discovered empirically and are essential for correct data interpretation:

1. **Vector3D values are in ECEF coordinates** (Earth-Centered Earth-Fixed), NOT local North/East/Up.
   - Position vector magnitude = ~6,350 km (Earth radius) confirms ECEF
   - Wind/velocity vectors must be projected to local ENU frame using aircraft lat/lon:
     ```
     wind_east  = -sin(lon) * ecef_x + cos(lon) * ecef_y
     wind_north = -sin(lat)*cos(lon) * ecef_x - sin(lat)*sin(lon) * ecef_y + cos(lat) * ecef_z
     ```

2. **Heading uses math convention** (counter-clockwise from East axis), NOT navigation convention.
   - Convert to navigation: `heading_nav = (90 - math.degrees(heading_rad)) % 360`

3. **Longitude** comes in 0-2pi range from the DLL, must normalize to -180/+180 degrees.

4. **Wind vector points in the FROM direction** (where wind comes from), not air velocity direction.
   - Use `atan2(wind_east, wind_north)` WITHOUT negation to get meteorological FROM direction.

5. **Altitude** from the DLL is in meters MSL (despite struct comment saying feet).

## Shared memory layout (AeroflyReaderData)

Key offsets used by DLLReader (based on C++ struct in `aerofly_reader_dll.cpp`):
- Header: timestamp(0), data_valid(8), update_counter(12)
- Latitude/Longitude: offsets 16/24 (radians)
- Altitude: offset 32 (meters MSL)
- Heading: offset 64 (radians, math convention)
- IAS: offset 80 (m/s)
- Vertical speed: offset 96 (m/s)
- Wind vector (ECEF): offsets 192/200/208 (m/s, x/y/z components)

Note: The SDK Python package in Aerofly-FS4-Bridge/simplified/sdk has DIFFERENT offsets
(state before vectors). The actual DLL binary follows the C++ struct order (vectors before state).

## Key constraints

- **XCSoar must be in FLY mode** (not SIM) -- SIM mode disables all device ports
- XCSoar device config: TCP Client, 127.0.0.1:4353
  - Driver: **Condor3** (recommended with DLL -- gets TAS, vario, wind via LXWP0)
  - Driver: **Generic** (basic mode, works with and without DLL)
- No external dependencies -- pure Python stdlib + tkinter
- DLL shared memory is Windows-only (mmap)
- `--no-dll` flag disables DLL reader (UDP only mode)

## Building the exe

```bash
pyinstaller AFS4-XCSoar-Bridge.spec --clean
```

Output: `dist/AFS4-XCSoar-Bridge.exe` (~10 MB standalone, no Python required)

## Running

```bash
python afs4-xcsoar-integration.py              # GUI mode with DLL support (default)
python afs4-xcsoar-integration.py --no-dll     # GUI mode, UDP only
python afs4-xcsoar-integration.py --no-gui     # Terminal mode
python afs4-xcsoar-integration.py --help       # All options
```
