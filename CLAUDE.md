# afs4-xcsoar-integration

Bridge that converts Aerofly FS4 flight simulator data to NMEA format for XCSoar glide computer.

## Architecture

Single-file Python application (`afs4-xcsoar-integration.py`) with these components:

| Class | Responsibility |
|-------|---------------|
| `AeroflyReceiver` | UDP listener on port 49002, parses ForeFlight protocol (XGPS/XATT) |
| `VarioCalculator` | Calculates vertical speed from altitude deltas (5-sample smooth, 30s average) |
| `NMEAConverter` | Generates NMEA sentences: GPRMC, GPGGA, HCHDT, PGRMZ, PTAS1 |
| `TCPServer` | Serves NMEA to XCSoar clients on port 4353 |
| `AeroflyToXCSoar` | Orchestrator: connects receiver → vario → converter → TCP |
| `BridgeGUI` | Tkinter GUI with real-time flight data and connection status |

## Data flow

```
AFS4 ──UDP 49002──► AeroflyReceiver ──► VarioCalculator
                                    ──► NMEAConverter ──► TCPServer ──TCP 4353──► XCSoar
```

## NMEA sentences generated

- **GPRMC** — position, speed, time, magnetic variation
- **GPGGA** — position, altitude, fix quality
- **HCHDT** — true heading (from XATT)
- **PGRMZ** — barometric/pressure altitude in feet (parsed by XCSoar Generic driver)
- **PTAS1** — vario, average vario, altitude, TAS (parsed by XCSoar Generic driver)

## Key constraints

- **XCSoar must be in FLY mode** (not SIM) — SIM mode disables all device ports (confirmed in XCSoar source: `Descriptor.cpp` line 466)
- XCSoar device config: TCP Client, 127.0.0.1:4353, Driver: Generic
- No external dependencies — pure Python stdlib + tkinter
- ForeFlight protocol only provides: lat, lon, alt MSL, track, groundspeed, heading, pitch, roll
- Vario is calculated (not direct), so there's a small lag inherent to the smoothing

## Future: DLL integration

GPSData has `vertical_speed` and `barometric_altitude` fields (Optional) prepared for direct data from Aerofly-FS4-Bridge DLL. When DLL is connected:
- `vertical_speed` will come direct instead of calculated
- `barometric_altitude` will be actual baro alt instead of MSL
- UDP will remain as fallback if DLL is not available
- GUI will be enhanced for the DLL version

## Running

```bash
python afs4-xcsoar-integration.py            # GUI mode (default)
python afs4-xcsoar-integration.py --no-gui   # Terminal mode
python afs4-xcsoar-integration.py --help     # All options
```
