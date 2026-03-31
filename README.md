# AFS4 XCSoar Integration

Bridge between **Aerofly FS4** flight simulator and **XCSoar** glide computer. Receives flight data from AFS4 via UDP, converts it to standard NMEA sentences, and serves them via TCP to XCSoar.

XCSoar gets full glide computer functionality: GPS position, barometric altitude, vario (vertical speed), average vario, true airspeed, and heading.

![AFS4 XCSoar Integration](https://img.shields.io/badge/AFS4-XCSoar%20Bridge-blue)
![Python 3.6+](https://img.shields.io/badge/Python-3.6%2B-green)
![License MIT](https://img.shields.io/badge/License-MIT-yellow)
![No Dependencies](https://img.shields.io/badge/Dependencies-None-brightgreen)

## Features

- Real-time NMEA data conversion (GPRMC, GPGGA, HCHDT, PGRMZ, PTAS1)
- Calculated vario with 5-sample smoothing and 30-second average
- Barometric altitude for XCSoar's glide computer
- Multi-client TCP server (connect multiple XCSoar instances)
- GUI with real-time flight data display and connection status
- Terminal mode for headless operation
- Zero external dependencies (Python stdlib + tkinter only)
- Configurable update rate, ports, and magnetic variation

## Requirements

- Python 3.6 or later
- Aerofly FS 4 with UDP broadcast enabled
- XCSoar (PC or mobile)

## Quick Start

### 1. Configure Aerofly FS4

In Aerofly FS4, go to **Settings > Miscellaneous**:

- **Broadcast flight info to IP address**: `on`
- **Broadcast IP address**: `192.168.000.255` (or your network broadcast)
- **Broadcast IP port**: `49002`

### 2. Run the bridge

```bash
python afs4-xcsoar-integration.py
```

This opens the GUI. Click **Start** to begin receiving data and serving NMEA.

For terminal mode (no GUI):

```bash
python afs4-xcsoar-integration.py --no-gui
```

### 3. Configure XCSoar

> **IMPORTANT**: XCSoar must be in **FLY mode**, not SIM mode. SIM mode disables all device ports.

In XCSoar, go to **Config > System > Devices** and configure a device:

| Setting | Value |
|---------|-------|
| **Port** | TCP Client |
| **IP Address** | `127.0.0.1` (same computer) or the bridge computer's IP |
| **TCP Port** | `4353` |
| **Driver** | Generic |

After saving, the device should show: `Posicion GPS; Baro; Velocidad aerodinamica; Vario`

## Command Line Options

```
usage: afs4-xcsoar-integration.py [-h] [--udp-port UDP_PORT]
                                   [--tcp-port TCP_PORT]
                                   [--update-rate UPDATE_RATE]
                                   [--mag-var MAG_VAR]
                                   [--debug {0,1,2}]
                                   [--no-gui]

Options:
  --udp-port     UDP port to receive Aerofly data (default: 49002)
  --tcp-port     TCP port to serve NMEA data (default: 4353)
  --update-rate  Update rate in Hz (default: 5)
  --mag-var      Magnetic variation in degrees, East positive (default: 0.0)
  --debug        Debug level: 0=minimal, 1=normal, 2=verbose (default: 1)
  --no-gui       Run in terminal mode without GUI
```

## NMEA Sentences

The bridge generates the following NMEA sentences at each update cycle:

| Sentence | Purpose | Data |
|----------|---------|------|
| `$GPRMC` | Recommended Minimum | Position, speed, time, date, magnetic variation |
| `$GPGGA` | GPS Fix Data | Position, altitude, fix quality, satellite count |
| `$HCHDT` | True Heading | Heading from AFS4 attitude data |
| `$PGRMZ` | Pressure Altitude | Barometric altitude in feet (Garmin proprietary) |
| `$PTAS1` | Vario + Airspeed | Vertical speed, average vario, altitude, TAS (Tasman proprietary) |

## How It Works

```
Aerofly FS4                          XCSoar
    |                                   |
    |--UDP broadcast (port 49002)-->    |
    |   XGPS: lon,lat,alt,track,speed   |
    |   XATT: heading,pitch,roll        |
    |                                   |
    |          [Bridge]                 |
    |   AeroflyReceiver (UDP)           |
    |        |                          |
    |   VarioCalculator                 |
    |   (altitude delta -> vario)       |
    |        |                          |
    |   NMEAConverter                   |
    |   (GPRMC,GPGGA,HCHDT,PGRMZ,PTAS1)|
    |        |                          |
    |   TCPServer (port 4353)           |
    |                                   |
    |      <--TCP NMEA sentences-----   |
```

## Network Setup

### Same Computer
If AFS4 and XCSoar run on the same PC, use `127.0.0.1` as the IP address in XCSoar.

### Different Computers
If XCSoar runs on a tablet or another PC on the network:
1. Run the bridge on the same computer as AFS4
2. Note the IP address shown by the bridge at startup
3. In XCSoar, use that IP address for the TCP Client connection
4. Ensure your firewall allows TCP connections on port 4353

## Troubleshooting

### XCSoar shows "N/A" for the device
- **Most common cause**: XCSoar is in SIM mode. Switch to FLY mode.
- Check that the bridge is running and shows "Started TCP server on port 4353"

### No data from Aerofly FS4
- Verify "Broadcast flight info to IP address" is ON in AFS4 settings
- Check that the broadcast port is 49002
- Ensure the bridge shows "Received GPS data" in terminal/debug mode

### XCSoar connects but shows no vario
- The vario needs a few seconds of altitude data to start calculating
- Fly for 5-10 seconds and the vario will appear

### Firewall issues
- Windows may prompt to allow Python through the firewall on first run
- Allow access for both private and public networks if needed

## License

MIT License - see [LICENSE](LICENSE) for details.

## Credits

- [XCSoar](https://xcsoar.org/) - Open source glide computer
- [Aerofly FS 4](https://www.aerofly.com/) - Flight simulator by IPACS
