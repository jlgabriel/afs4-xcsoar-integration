#!/usr/bin/env python3
# Copyright (c) 2026 Juan Luis Gabriel
# Aerofly FS4 to XCSoar NMEA Converter
# This software is released under the MIT License.

"""
This program acts as a bridge between Aerofly FS4 and XCSoar by:
1. Receiving UDP data from Aerofly FS4 flight simulator
2. Converting the data to NMEA format
3. Serving the NMEA data via TCP for XCSoar to connect

The program can be used with XCSoar running on:
- The same computer (connect to 127.0.0.1)
- Another computer on the network (connect to this computer's IP address)
"""

import socket
import threading
import struct
import math
import mmap
import re
import time
import datetime
import argparse
import random
import sys
import tkinter as tk
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

VERSION = "2.1.0"

# Default configuration
DEFAULT_UDP_PORT = 49002          # Aerofly FS4 default UDP port
DEFAULT_TCP_PORT = 4353           # Port for TCP server (for XCSoar to connect)
DEFAULT_UPDATE_RATE = 5           # How many NMEA sentences per second
DEFAULT_MAGNETIC_VARIATION = 0.0  # Magnetic variation in degrees (East positive, West negative)
DEFAULT_DEBUG_LEVEL = 1           # Debug level: 0=minimal, 1=normal, 2=verbose

@dataclass
class GPSData:
    """Store GPS data received from Aerofly FS4 (UDP or DLL)."""
    longitude: float = 0.0
    latitude: float = 0.0
    altitude: float = 0.0          # MSL altitude in meters
    track: float = 0.0
    ground_speed: float = 0.0      # m/s
    timestamp: float = 0.0
    # Extended fields (calculated from UDP, or direct from DLL)
    vertical_speed: Optional[float] = None    # m/s, None = not yet available
    barometric_altitude: Optional[float] = None  # meters, None = use MSL altitude
    # DLL-only fields
    indicated_airspeed: Optional[float] = None   # m/s (IAS from DLL)
    wind_x: Optional[float] = None               # m/s (wind vector X component)
    wind_y: Optional[float] = None               # m/s (wind vector Y component)
    wind_z: Optional[float] = None               # m/s (wind vector Z component)

@dataclass
class AttitudeData:
    """Store attitude data received from Aerofly FS4."""
    true_heading: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    timestamp: float = 0.0

class VarioCalculator:
    """
    Calculates vertical speed (vario) from altitude changes over time.
    Uses a moving average for smoothing and tracks a longer-term average.
    When DLL is available, this class can be bypassed with direct vario data.
    """
    SMOOTH_WINDOW = 5       # samples for instantaneous vario smoothing
    AVG_WINDOW_SEC = 30.0   # seconds for average vario calculation

    def __init__(self):
        self._prev_altitude: Optional[float] = None
        self._prev_timestamp: float = 0.0
        self._smooth_buffer: deque = deque(maxlen=self.SMOOTH_WINDOW)
        self._avg_buffer: deque = deque()  # (timestamp, vario) pairs
        self.vario: float = 0.0            # current smoothed vario (m/s)
        self.average_vario: float = 0.0    # 30-second average vario (m/s)

    def update(self, altitude: float, timestamp: float) -> None:
        """Feed a new altitude sample. Call this on every GPS update."""
        if self._prev_altitude is not None and timestamp > self._prev_timestamp:
            dt = timestamp - self._prev_timestamp
            if 0.01 < dt < 5.0:  # ignore bogus time deltas
                raw_vario = (altitude - self._prev_altitude) / dt
                self._smooth_buffer.append(raw_vario)
                self.vario = sum(self._smooth_buffer) / len(self._smooth_buffer)

                # Feed long-term average buffer
                self._avg_buffer.append((timestamp, self.vario))
                cutoff = timestamp - self.AVG_WINDOW_SEC
                while self._avg_buffer and self._avg_buffer[0][0] < cutoff:
                    self._avg_buffer.popleft()
                if self._avg_buffer:
                    self.average_vario = sum(v for _, v in self._avg_buffer) / len(self._avg_buffer)

        self._prev_altitude = altitude
        self._prev_timestamp = timestamp

    @property
    def is_valid(self) -> bool:
        """True once we have enough samples for a meaningful reading."""
        return len(self._smooth_buffer) >= 2


class AeroflyReceiver:
    """
    Receives and parses UDP data from Aerofly FS4.
    """
    def __init__(self, port: int = DEFAULT_UDP_PORT, debug_level: int = DEFAULT_DEBUG_LEVEL):
        self.port = port
        self.socket = None
        self.gps_data = GPSData()
        self.attitude_data = AttitudeData()
        self.running = False
        self.receive_thread = None
        self.last_receive_time = 0
        self.debug_level = debug_level

    def start(self):
        """Start receiving UDP data from Aerofly."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.settimeout(0.5)
        self.socket.bind(('', self.port))
        
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        print(f"Started UDP receiver on port {self.port}")

    def _receive_loop(self):
        """Main UDP receiving loop."""
        while self.running:
            try:
                data, _ = self.socket.recvfrom(1024)
                self.last_receive_time = time.time()
                message = data.decode('utf-8')
                
                if message.startswith('XGPS'):
                    gps_data = self._parse_gps_message(message)
                    if gps_data:
                        self.gps_data = gps_data
                        if self.debug_level >= 1:
                            print(f"Received GPS data: lat={gps_data.latitude:.5f}, lon={gps_data.longitude:.5f}", end="\r")
                
                elif message.startswith('XATT'):
                    attitude_data = self._parse_attitude_message(message)
                    if attitude_data:
                        self.attitude_data = attitude_data
                        if self.debug_level >= 2:
                            print(f"Received attitude data: heading={attitude_data.true_heading:.1f}°", end="\r")
                
            except socket.timeout:
                # Expected timeout, continue loop
                pass
            except Exception as e:
                print(f"Error receiving UDP data: {e}")
    
    def _parse_gps_message(self, message: str) -> Optional[GPSData]:
        """Parse GPS data from Aerofly message."""
        pattern = r'XGPSAerofly FS 4,([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)'
        match = re.match(pattern, message)
        if match:
            lon, lat, alt, trk, spd = map(float, match.groups())
            return GPSData(
                longitude=lon,
                latitude=lat,
                altitude=alt,
                track=trk,
                ground_speed=spd,
                timestamp=time.time()
            )
        return None

    def _parse_attitude_message(self, message: str) -> Optional[AttitudeData]:
        """Parse attitude data from Aerofly message."""
        pattern = r'XATTAerofly FS 4,([-\d.]+),([-\d.]+),([-\d.]+)'
        match = re.match(pattern, message)
        if match:
            heading, pitch, roll = map(float, match.groups())
            return AttitudeData(
                true_heading=heading,
                pitch=pitch,
                roll=roll,
                timestamp=time.time()
            )
        return None
    
    def is_connected(self, timeout=5.0):
        """Check if we're receiving data from Aerofly."""
        return (time.time() - self.last_receive_time) < timeout
    
    def stop(self):
        """Stop the UDP receiver."""
        self.running = False
        # Wait for the thread to finish before closing socket
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(1.0)  # Wait up to 1 second
        if self.socket:
            self.socket.close()


class DLLReader:
    """
    Reads flight data from AeroflyReader DLL via Windows Shared Memory.
    Provides high-quality data at 50-60Hz including IAS, direct vario, and wind.
    Falls back gracefully when DLL is not available.

    Shared memory layout based on AeroflyReaderData struct in aerofly_reader_dll.cpp.
    """
    MAPPING_NAME = "AeroflyReaderData"
    POLL_INTERVAL = 0.02   # 50Hz read rate
    RECONNECT_INTERVAL = 3.0  # seconds between reconnect attempts
    STALE_TIMEOUT = 2.0    # seconds before data is considered stale

    # Offsets calculated from AeroflyReaderData C++ struct layout:
    # Header: uint64(8) + uint32(4) + uint32(4) = 16 bytes
    # Position/Orientation: 8 doubles = 64 bytes
    # Speeds: 5 doubles = 40 bytes
    # Physics vectors: 4 x tm_vector3d(3 doubles) = 96 bytes
    # Aircraft state: 5 doubles, Engine: 4 doubles, Nav: 6, AP: 5, VSpeeds: 5
    _OFF_TIMESTAMP = 0           # uint64_t
    _OFF_DATA_VALID = 8          # uint32_t
    _OFF_UPDATE_COUNTER = 12     # uint32_t
    _OFF_LATITUDE = 16           # double (radians)
    _OFF_LONGITUDE = 24          # double (radians)
    _OFF_ALTITUDE = 32           # double (meters MSL)
    _OFF_HEIGHT = 40             # double (meters AGL)
    _OFF_PITCH = 48              # double (radians)
    _OFF_BANK = 56               # double (radians)
    _OFF_TRUE_HEADING = 64       # double (radians)
    _OFF_MAGNETIC_HEADING = 72   # double (radians)
    _OFF_IAS = 80                # double (m/s)
    _OFF_GROUND_SPEED = 88       # double (m/s)
    _OFF_VERTICAL_SPEED = 96     # double (m/s)
    # Physics vectors start at offset 120 (after 5 speed doubles ending at 112+8=120)
    # position(24) + velocity(24) + acceleration(24) + wind(24) = 96 bytes
    _OFF_WIND_X = 192            # double (m/s) — wind vector starts at 120+72=192
    _OFF_WIND_Y = 200            # double (m/s)
    _OFF_WIND_Z = 208            # double (m/s)
    # Minimum shared memory size needed for the fields we read
    _MIN_SIZE = 216

    def __init__(self, debug_level: int = DEFAULT_DEBUG_LEVEL):
        self.debug_level = debug_level
        self._mmap: Optional[mmap.mmap] = None
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_counter = 0
        self._last_valid_time = 0.0
        # Public data (updated by reader thread)
        self.gps_data = GPSData()
        self.attitude_data = AttitudeData()

    def start(self):
        """Start the DLL reader thread."""
        if sys.platform != 'win32':
            print("DLL shared memory is only available on Windows.")
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print("DLL reader started, looking for AeroflyReader shared memory...")

    def _try_connect(self) -> bool:
        """Open existing shared memory (without creating a new one).
        Uses ctypes OpenFileMappingW to avoid mmap(-1,...) which creates
        an empty mapping if it doesn't exist yet, shadowing the DLL's real one.
        """
        try:
            import ctypes
            from ctypes import wintypes
            kernel32 = ctypes.windll.kernel32
            FILE_MAP_READ = 0x0004
            handle = kernel32.OpenFileMappingW(FILE_MAP_READ, False, self.MAPPING_NAME)
            if not handle:
                return False
            kernel32.CloseHandle(handle)
            self._mmap = mmap.mmap(-1, 1024, self.MAPPING_NAME, access=mmap.ACCESS_READ)
            self._connected = True
            if self.debug_level >= 1:
                print(f"Connected to DLL shared memory: {self.MAPPING_NAME}")
            return True
        except Exception:
            self._cleanup()
            return False

    def _cleanup(self):
        """Close shared memory resources."""
        if self._mmap:
            try:
                self._mmap.close()
            except Exception:
                pass
        self._mmap = None
        self._connected = False

    def _read_double(self, offset: int) -> float:
        self._mmap.seek(offset)
        return struct.unpack('d', self._mmap.read(8))[0]

    def _read_uint32(self, offset: int) -> int:
        self._mmap.seek(offset)
        return struct.unpack('I', self._mmap.read(4))[0]

    def _read_loop(self):
        """Main loop: connect, read, reconnect."""
        last_reconnect = 0.0
        was_connected = False

        while self._running:
            if not self._connected:
                now = time.time()
                if now - last_reconnect >= self.RECONNECT_INTERVAL:
                    last_reconnect = now
                    if self._try_connect() and not was_connected:
                        was_connected = True
                else:
                    time.sleep(0.1)
                    continue

            try:
                data_valid = self._read_uint32(self._OFF_DATA_VALID)
                counter = self._read_uint32(self._OFF_UPDATE_COUNTER)

                if data_valid > 0 and counter != self._last_counter:
                    self._last_counter = counter
                    self._last_valid_time = time.time()
                    self._update_data()

                elif time.time() - self._last_valid_time > self.STALE_TIMEOUT and self._last_valid_time > 0:
                    if self.debug_level >= 1:
                        print("DLL data stale, reconnecting...")
                    self._cleanup()
                    was_connected = False

            except Exception as e:
                if self.debug_level >= 1:
                    print(f"DLL read error: {e}")
                self._cleanup()
                was_connected = False

            time.sleep(self.POLL_INTERVAL)

    def _update_data(self):
        """Read all fields from shared memory and update GPS/attitude data."""
        now = time.time()

        lat_rad = self._read_double(self._OFF_LATITUDE)
        lon_rad = self._read_double(self._OFF_LONGITUDE)
        alt_m = self._read_double(self._OFF_ALTITUDE)
        height_m = self._read_double(self._OFF_HEIGHT)
        ias_ms = self._read_double(self._OFF_IAS)
        gs_ms = self._read_double(self._OFF_GROUND_SPEED)
        vs_ms = self._read_double(self._OFF_VERTICAL_SPEED)
        true_hdg_rad = self._read_double(self._OFF_TRUE_HEADING)
        mag_hdg_rad = self._read_double(self._OFF_MAGNETIC_HEADING)
        pitch_rad = self._read_double(self._OFF_PITCH)
        bank_rad = self._read_double(self._OFF_BANK)
        wind_ecef_x = self._read_double(self._OFF_WIND_X)
        wind_ecef_y = self._read_double(self._OFF_WIND_Y)
        wind_ecef_z = self._read_double(self._OFF_WIND_Z)

        # Convert radians to degrees
        lat_deg = math.degrees(lat_rad)
        lon_deg = math.degrees(lon_rad)
        # Normalize longitude: AFS4 DLL returns 0-360, NMEA needs -180/+180
        if lon_deg > 180.0:
            lon_deg -= 360.0
        # AFS4 heading uses math convention (CCW from East axis).
        # Convert to navigation convention (CW from North): nav = (90 - math) % 360
        true_hdg_deg = (90.0 - math.degrees(true_hdg_rad)) % 360
        mag_hdg_deg = (90.0 - math.degrees(mag_hdg_rad)) % 360
        pitch_deg = math.degrees(pitch_rad)
        bank_deg = math.degrees(bank_rad)

        # Convert wind from ECEF to local ENU (East-North-Up) frame.
        # AFS4 Vector3D values are in ECEF: x→(0°N,0°E), y→(0°N,90°E), z→NorthPole
        # Local East  = (-sin(lon), cos(lon), 0)
        # Local North = (-sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat))
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)

        wind_east = -sin_lon * wind_ecef_x + cos_lon * wind_ecef_y
        wind_north = (-sin_lat * cos_lon * wind_ecef_x
                      - sin_lat * sin_lon * wind_ecef_y
                      + cos_lat * wind_ecef_z)

        # Track: use true heading as approximation (DLL only gives scalar groundspeed)
        track = true_hdg_deg

        self.gps_data = GPSData(
            longitude=lon_deg,
            latitude=lat_deg,
            altitude=alt_m,
            track=track,
            ground_speed=gs_ms,
            timestamp=now,
            vertical_speed=vs_ms,
            barometric_altitude=alt_m,
            indicated_airspeed=ias_ms,
            wind_x=wind_north,
            wind_y=0.0,  # vertical wind not used for direction
            wind_z=wind_east,
        )

        self.attitude_data = AttitudeData(
            true_heading=true_hdg_deg,
            pitch=pitch_deg,
            roll=bank_deg,
            timestamp=now,
        )

    def is_connected(self, timeout=2.0) -> bool:
        """Check if DLL is actively providing data."""
        if not self._connected:
            return False
        return (time.time() - self._last_valid_time) < timeout

    def stop(self):
        """Stop the DLL reader."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(1.0)
        self._cleanup()


def ias_to_tas(ias_ms: float, altitude_m: float) -> float:
    """Convert IAS to TAS using the ISA standard atmosphere model.

    Uses the barometric formula for the troposphere (below 11,000m):
      T = T0 - L * h
      rho = rho0 * (T / T0) ^ (g/(R*L) - 1)
      TAS = IAS * sqrt(rho0 / rho)

    Args:
        ias_ms: Indicated airspeed in m/s
        altitude_m: Altitude in meters MSL

    Returns:
        True airspeed in m/s
    """
    T0 = 288.15     # ISA sea-level temperature (K)
    L = 0.0065      # Temperature lapse rate (K/m)
    g = 9.80665     # Gravitational acceleration (m/s^2)
    R = 287.058     # Specific gas constant for dry air (J/(kg*K))

    # Clamp altitude to troposphere (ISA model valid up to 11,000m)
    h = max(0.0, min(altitude_m, 11000.0))

    T = T0 - L * h
    if T <= 0:
        return ias_ms

    exponent = g / (R * L) - 1.0  # ≈ 4.2559
    density_ratio = (T / T0) ** exponent  # rho / rho0

    if density_ratio <= 0:
        return ias_ms

    return ias_ms / math.sqrt(density_ratio)


class NMEAConverter:
    """
    Converts Aerofly data to NMEA sentences for XCSoar.
    """
    def __init__(self, magnetic_variation=DEFAULT_MAGNETIC_VARIATION, debug_level=DEFAULT_DEBUG_LEVEL):
        self.magnetic_variation = magnetic_variation
        self.debug_level = debug_level

    def create_nmea_sentences(self, gps: GPSData, attitude: AttitudeData,
                              vario: Optional['VarioCalculator'] = None,
                              dll_connected: bool = False) -> List[str]:
        """Create appropriate NMEA sentences from Aerofly data."""
        sentences = []

        # Only generate sentences if we have valid GPS data
        if gps and gps.latitude != 0 and gps.longitude != 0:
            # Create a UTC timestamp from the current time
            # Use timezone-aware datetime to avoid deprecation warning
            try:
                utc_time = datetime.datetime.now(datetime.UTC)  # Python 3.11+
            except AttributeError:
                # Fallback for older Python versions
                utc_time = datetime.datetime.utcnow()

            time_str = utc_time.strftime("%H%M%S.%f")[:-4]  # Format: HHMMSS.SS
            date_str = utc_time.strftime("%d%m%y")  # Format: DDMMYY

            # Add RMC sentence (position, speed, time)
            sentences.append(self._create_rmc_sentence(gps, time_str, date_str))

            # Add GGA sentence (position, altitude, time)
            sentences.append(self._create_gga_sentence(gps, time_str))

            # If we have valid attitude data, add HDT sentence (true heading)
            if attitude and attitude.timestamp > 0:
                sentences.append(self._create_hdt_sentence(attitude))

            # Add pressure altitude (PGRMZ)
            sentences.append(self._create_pgrmz_sentence(gps))

            # Add vario + altitude + TAS (PTAS1) if vario data is available
            if vario and vario.is_valid:
                sentences.append(self._create_ptas1_sentence(gps, vario))

            # Add LXWP0 sentence when DLL provides IAS and wind data
            # (Condor-compatible format parsed by XCSoar Condor3 driver)
            if dll_connected and gps.indicated_airspeed is not None:
                sentences.append(self._create_lxwp0_sentence(gps, attitude))

        return sentences

    def _create_rmc_sentence(self, gps: GPSData, time_str: str, date_str: str) -> str:
        """Create GPRMC sentence (Recommended minimum data)."""
        # Convert decimal degrees to NMEA degree format (DDMM.MMMM)
        lat_nmea, ns = self._convert_latitude_to_nmea(gps.latitude)
        lon_nmea, ew = self._convert_longitude_to_nmea(gps.longitude)
        
        # Convert m/s to knots
        speed_knots = gps.ground_speed * 1.94384
        
        # Calculate magnetic heading from true heading
        mag_track = (gps.track - self.magnetic_variation + 360) % 360
        
        # Magnetic variation direction
        var_dir = "E" if self.magnetic_variation >= 0 else "W"
        mag_var_str = f"{abs(self.magnetic_variation):.2f},{var_dir}"
        
        # Build sentence parts
        parts = [
            "GPRMC",                # Sentence identifier
            time_str,               # UTC time
            "A",                    # Status (A=valid, V=invalid)
            lat_nmea,               # Latitude in DDMM.MMMM format
            ns,                     # N/S indicator
            lon_nmea,               # Longitude in DDDMM.MMMM format
            ew,                     # E/W indicator
            f"{speed_knots:.2f}",   # Speed over ground in knots
            f"{gps.track:.2f}",     # Track made good (true)
            date_str,               # UTC date
            mag_var_str,            # Magnetic variation with direction
            "A"                     # Mode indicator
        ]
        
        # Join parts and calculate checksum
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        
        return f"${sentence}*{checksum}"

    def _create_gga_sentence(self, gps: GPSData, time_str: str) -> str:
        """Create GPGGA sentence (Fix data)."""
        # Convert decimal degrees to NMEA degree format
        lat_nmea, ns = self._convert_latitude_to_nmea(gps.latitude)
        lon_nmea, ew = self._convert_longitude_to_nmea(gps.longitude)
        
        # Altitude in meters to feet then back to meters for NMEA format
        alt_meters = gps.altitude
        
        # Build sentence parts
        parts = [
            "GPGGA",                # Sentence identifier
            time_str,               # UTC time
            lat_nmea,               # Latitude
            ns,                     # N/S indicator
            lon_nmea,               # Longitude
            ew,                     # E/W indicator
            "1",                    # Fix quality (1=GPS, 0=Invalid)
            "8",                    # Number of satellites (fixed at 8)
            "1.0",                  # HDOP (fixed at 1.0)
            f"{alt_meters:.1f}",    # Altitude in meters
            "M",                    # Units (meters)
            "0.0",                  # Height of geoid above WGS84 ellipsoid
            "M",                    # Units (meters)
            "",                     # Time since last DGPS update (empty)
            ""                      # DGPS reference station ID (empty)
        ]
        
        # Join parts and calculate checksum
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        
        return f"${sentence}*{checksum}"

    def _create_hdt_sentence(self, attitude: AttitudeData) -> str:
        """Create HCHDT sentence (True heading)."""
        # Build sentence parts
        parts = [
            "HCHDT",                         # Sentence identifier
            f"{attitude.true_heading:.1f}",  # True heading
            "T"                              # T = True
        ]
        
        # Join parts and calculate checksum
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        
        return f"${sentence}*{checksum}"

    def _create_pgrmz_sentence(self, gps: GPSData) -> str:
        """Create PGRMZ sentence (Garmin pressure altitude).
        XCSoar Generic driver parses this for barometric/pressure altitude.
        Format: $PGRMZ,<alt_feet>,f,3*CS
        """
        alt_source = gps.barometric_altitude if gps.barometric_altitude is not None else gps.altitude
        alt_feet = alt_source * 3.28084
        parts = ["PGRMZ", f"{alt_feet:.0f}", "f", "3"]
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        return f"${sentence}*{checksum}"

    def _create_ptas1_sentence(self, gps: GPSData, vario: 'VarioCalculator') -> str:
        """Create PTAS1 sentence (Tasman Instruments).
        XCSoar Generic driver parses this for vario, avg vario, baro alt, and TAS.
        Format: $PTAS1,<vario_raw>,<avg_vario_raw>,<alt_raw>,<tas_raw>*CS
        Encoding:
          vario_raw    = vario_ms * 10 + 200
          avg_vario_raw = avg_vario_ms * 10 + 200
          alt_raw      = altitude_feet + 2000
          tas_raw      = true_airspeed_knots (we approximate from groundspeed)
        """
        # Use direct vertical_speed from DLL if available, otherwise calculated
        vario_ms = gps.vertical_speed if gps.vertical_speed is not None else vario.vario
        avg_vario_ms = vario.average_vario

        # Encode vario values: raw = value * 10 + 200
        vario_raw = int(round(vario_ms * 10 + 200))
        avg_vario_raw = int(round(avg_vario_ms * 10 + 200))

        # Encode altitude: raw = altitude_feet + 2000
        alt_source = gps.barometric_altitude if gps.barometric_altitude is not None else gps.altitude
        alt_feet = alt_source * 3.28084
        alt_raw = int(round(alt_feet + 2000))

        # TAS approximation: use groundspeed (best we can do without airspeed data)
        tas_knots = gps.ground_speed * 1.94384
        tas_raw = int(round(tas_knots))

        parts = ["PTAS1", str(vario_raw), str(avg_vario_raw), str(alt_raw), str(tas_raw)]
        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        return f"${sentence}*{checksum}"

    def _create_lxwp0_sentence(self, gps: GPSData, attitude: AttitudeData) -> str:
        """Create $LXWP0 sentence (LX Navigation format, same as Condor 3).
        XCSoar Condor3 driver parses this for TAS, baro alt, vario, and wind.
        Format: $LXWP0,<logger>,<TAS_kph>,<baro_alt_m>,<vario_ms>,,,,,,<heading>,<wind_dir>,<wind_speed_kph>

        Wind direction follows Condor 3 convention: direction wind is coming FROM.
        """
        # TAS from IAS using ISA atmosphere model
        ias_ms = gps.indicated_airspeed if gps.indicated_airspeed is not None else 0.0
        alt_m = gps.barometric_altitude if gps.barometric_altitude is not None else gps.altitude
        tas_ms = ias_to_tas(ias_ms, alt_m)
        tas_kph = tas_ms * 3.6

        # Vario (direct from DLL)
        vario_ms = gps.vertical_speed if gps.vertical_speed is not None else 0.0

        # Heading
        heading = attitude.true_heading if attitude and attitude.timestamp > 0 else 0.0

        # Wind: convert Vector3D (m/s) to direction (FROM) and speed (kph)
        # AFS4 Vector3D axes: x = North, y = Up, z = East
        # Use x (North) and z (East) for horizontal wind, ignore y (vertical)
        wn = gps.wind_x if gps.wind_x is not None else 0.0  # North component
        we = gps.wind_z if gps.wind_z is not None else 0.0  # East component
        wind_speed_ms = math.sqrt(wn * wn + we * we)
        wind_speed_kph = wind_speed_ms * 3.6

        # AFS4 wind vector already points in the FROM direction (not air velocity).
        # atan2(East, North) directly gives the meteorological "from" direction.
        if wind_speed_ms > 0.1:
            wind_dir = math.degrees(math.atan2(we, wn)) % 360.0
        else:
            wind_dir = 0.0

        parts = [
            "LXWP0",
            "Y",                        # logger_stored
            f"{tas_kph:.1f}",           # TAS in kph
            f"{alt_m:.1f}",             # baro altitude in meters
            f"{vario_ms:.2f}",          # vario in m/s
            "", "", "", "", "",         # fields 4-8 (unused)
            f"{heading:.0f}",           # heading of plane
            f"{wind_dir:.0f}",          # wind course (FROM, degrees)
            f"{wind_speed_kph:.1f}",    # wind speed in kph
        ]

        sentence = ",".join(parts)
        checksum = self._calculate_checksum(sentence)
        return f"${sentence}*{checksum}"

    @staticmethod
    def _convert_latitude_to_nmea(latitude: float) -> Tuple[str, str]:
        """Convert decimal latitude to NMEA format (DDMM.MMMM,N/S)."""
        lat_abs = abs(latitude)
        degrees = int(lat_abs)
        minutes = (lat_abs - degrees) * 60
        nmea_str = f"{degrees:02d}{minutes:07.4f}"
        direction = "S" if latitude < 0 else "N"
        return nmea_str, direction

    @staticmethod
    def _convert_longitude_to_nmea(longitude: float) -> Tuple[str, str]:
        """Convert decimal longitude to NMEA format (DDDMM.MMMM,E/W)."""
        lon_abs = abs(longitude)
        degrees = int(lon_abs)
        minutes = (lon_abs - degrees) * 60
        nmea_str = f"{degrees:03d}{minutes:07.4f}"
        direction = "W" if longitude < 0 else "E"
        return nmea_str, direction

    @staticmethod
    def _calculate_checksum(sentence: str) -> str:
        """Calculate the NMEA checksum for a sentence."""
        # The checksum is calculated on the data between $ and * (exclusive)
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f"{checksum:02X}"  # Return as 2-digit hex value
    
    def validate_nmea_sentence(self, sentence: str) -> bool:
        """Validate a NMEA sentence format and checksum."""
        if not sentence.startswith('$'):
            if self.debug_level >= 2:
                print(f"Invalid NMEA sentence (no leading $): {sentence}")
            return False
            
        if '*' not in sentence:
            if self.debug_level >= 2:
                print(f"Invalid NMEA sentence (no checksum): {sentence}")
            return False
            
        parts = sentence.split('*')
        if len(parts) != 2:
            if self.debug_level >= 2:
                print(f"Invalid NMEA sentence (multiple * characters): {sentence}")
            return False
            
        data, checksum = parts
        data = data[1:]  # Remove the leading $
        
        # Calculate the checksum on the data
        calculated_checksum = 0
        for char in data:
            calculated_checksum ^= ord(char)
            
        # Compare checksums
        if checksum.upper() != f"{calculated_checksum:02X}":
            if self.debug_level >= 2:
                print(f"Invalid NMEA checksum in {sentence}: got {checksum}, expected {calculated_checksum:02X}")
            return False
            
        return True

class TCPServer:
    """
    TCP server to serve NMEA data to XCSoar.
    """
    def __init__(self, port=DEFAULT_TCP_PORT, debug_level=DEFAULT_DEBUG_LEVEL):
        self.port = port
        self.clients = []
        self.clients_lock = threading.Lock()
        self.server_socket = None
        self.accept_thread = None
        self.running = False
        self.debug_level = debug_level
        # Add a counter for connected clients
        self.client_count = 0

    def start(self):
        """Start the TCP server."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to all interfaces so we can accept connections from the network
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)

        self.running = True
        self.accept_thread = threading.Thread(target=self._accept_clients)
        self.accept_thread.daemon = True
        self.accept_thread.start()

        print(f"Started TCP server on port {self.port}")
        print(f"XCSoar can connect to this computer's IP address or localhost (127.0.0.1)")

        # Show available IP addresses for connection
        self._show_available_ips()

    def _show_available_ips(self):
        """Show all available IP addresses on this machine for easier connection."""
        try:
            hostname = socket.gethostname()
            ip_addresses = []
            
            # Get all IP addresses for this host
            for addr_info in socket.getaddrinfo(hostname, None):
                ip = addr_info[4][0]
                # Skip IPv6 and localhost addresses
                if ':' not in ip and ip != '127.0.0.1':
                    ip_addresses.append(ip)
            
            # Remove duplicates
            ip_addresses = list(set(ip_addresses))
            
            if ip_addresses:
                print("Available IP addresses to connect from XCSoar:")
                for ip in ip_addresses:
                    print(f"  - {ip}")
            else:
                print("No external IP addresses found. Use 127.0.0.1 if XCSoar is on this machine.")
                
        except Exception as e:
            print(f"Could not determine IP addresses: {e}")
            print("Use 127.0.0.1 if XCSoar is on this machine.")

    def _accept_clients(self):
        """Accept incoming client connections."""
        self.server_socket.settimeout(1)  # Check running flag every second
        
        while self.running:
            try:
                client_sock, addr = self.server_socket.accept()
                client_sock.settimeout(5)  # Add timeout to detect disconnections
                
                self.client_count += 1
                client_id = self.client_count
                
                with self.clients_lock:
                    self.clients.append((client_sock, client_id))
                
                print(f"New client #{client_id} connected: {addr[0]}:{addr[1]}")
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:  # Only print if not shutting down
                    print(f"Error accepting client: {e}")

    def send_to_all_clients(self, data: str):
        """Send data to all connected clients."""
        disconnected_clients = []
        
        # Log NMEA data occasionally for debugging
        if self.debug_level >= 2 and random.randint(1, 50) == 1:
            print(f"Sending NMEA: {data.strip()}")
        
        with self.clients_lock:
            for client_sock, client_id in self.clients:
                try:
                    client_sock.sendall(data.encode('ascii') + b'\r\n')
                except Exception as e:
                    # Mark this client for removal
                    disconnected_clients.append((client_sock, client_id))
                    if self.debug_level >= 1:
                        print(f"Client #{client_id} disconnected: {e}")
        
        # Remove disconnected clients
        if disconnected_clients:
            with self.clients_lock:
                for client in disconnected_clients:
                    if client in self.clients:
                        self.clients.remove(client)

    def stop(self):
        """Stop the TCP server."""
        self.running = False
        
        # Close all client connections
        with self.clients_lock:
            for client_sock, _ in self.clients:
                try:
                    client_sock.close()
                except Exception:
                    pass
            self.clients = []
        
        # Close server socket
        if self.server_socket:
            self.server_socket.close()

class AeroflyToXCSoar:
    """
    Main application class that coordinates the UDP receiver,
    DLL reader (optional), NMEA converter, and TCP server.
    """
    def __init__(self, udp_port=DEFAULT_UDP_PORT, tcp_port=DEFAULT_TCP_PORT,
                 update_rate=DEFAULT_UPDATE_RATE, magnetic_variation=DEFAULT_MAGNETIC_VARIATION,
                 debug_level=DEFAULT_DEBUG_LEVEL, use_dll=True):
        self.udp_port = udp_port
        self.tcp_port = tcp_port
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate if update_rate > 0 else 0.2
        self.debug_level = debug_level
        self.magnetic_variation = magnetic_variation
        self.use_dll = use_dll

        # Initialize components
        self.receiver = AeroflyReceiver(udp_port, debug_level)
        self.dll_reader = DLLReader(debug_level) if use_dll else None
        self.converter = NMEAConverter(magnetic_variation, debug_level)
        self.tcp_server = TCPServer(tcp_port, debug_level)
        self.vario_calculator = VarioCalculator()

        self.running = False
        self.main_thread = None
        self.start_time = time.time()
        self.sentences_count = 0

    def start_services(self):
        """Start all background services (receiver, DLL reader, TCP server, processing loop)."""
        print(f"Starting Aerofly FS4 to XCSoar NMEA converter v{VERSION}...")
        print(f"UDP Port: {self.udp_port}, TCP Port: {self.tcp_port}")
        print(f"Update rate: {self.update_rate} Hz, Magnetic variation: {self.magnetic_variation}°")
        print(f"DLL support: {'enabled' if self.use_dll else 'disabled'}")
        print(f"Debug level: {self.debug_level}")

        self.receiver.start()
        if self.dll_reader:
            self.dll_reader.start()
        self.tcp_server.start()

        self.running = True
        self.main_thread = threading.Thread(target=self._process_loop)
        self.main_thread.daemon = True
        self.main_thread.start()

    def start(self):
        """Start services and block in terminal mode (CLI)."""
        self.start_services()
        self._print_xcsoar_instructions()

        try:
            while self.running:
                time.sleep(0.5)
                if not self.receiver.is_connected():
                    print("Waiting for data from Aerofly FS4...", end="\r")
                    sys.stdout.flush()
        except KeyboardInterrupt:
            print("\nStopping Aerofly FS4 to XCSoar converter...")
            self.stop()

    def get_status(self) -> dict:
        """Return current status for GUI consumption."""
        dll_connected = self.dll_reader and self.dll_reader.is_connected()

        # DLL data takes priority over UDP
        if dll_connected:
            gps = self.dll_reader.gps_data
            att = self.dll_reader.attitude_data
        else:
            gps = self.receiver.gps_data
            att = self.receiver.attitude_data

        with self.tcp_server.clients_lock:
            client_count = len(self.tcp_server.clients)
        runtime = time.time() - self.start_time if self.running else 0
        rate = self.sentences_count / runtime if runtime > 0 else 0

        # Vario: prefer DLL direct value over calculated
        if dll_connected and gps.vertical_speed is not None:
            vario_val = gps.vertical_speed
            avg_vario_val = gps.vertical_speed  # DLL has no avg, use instant
        elif self.vario_calculator.is_valid:
            vario_val = self.vario_calculator.vario
            avg_vario_val = self.vario_calculator.average_vario
        else:
            vario_val = None
            avg_vario_val = None

        # IAS/TAS when DLL available
        ias_ms = gps.indicated_airspeed
        tas_ms = ias_to_tas(ias_ms, gps.altitude) if ias_ms is not None else None

        # Wind when DLL available
        # AFS4 wind vector points in FROM direction (not air velocity)
        if gps.wind_x is not None and gps.wind_z is not None:
            wn, we = gps.wind_x, gps.wind_z  # North, East (after ECEF→ENU)
            wind_speed_ms = math.sqrt(wn**2 + we**2)
            wind_dir = math.degrees(math.atan2(we, wn)) % 360.0 if wind_speed_ms > 0.1 else 0.0
        else:
            wind_speed_ms = None
            wind_dir = None

        return {
            'running': self.running,
            'afs4_connected': self.receiver.is_connected() or dll_connected,
            'dll_connected': dll_connected,
            'data_source': 'DLL' if dll_connected else 'UDP',
            'xcsoar_clients': client_count,
            'latitude': gps.latitude,
            'longitude': gps.longitude,
            'altitude_m': gps.altitude,
            'altitude_ft': gps.altitude * 3.28084,
            'ground_speed_kmh': gps.ground_speed * 3.6,
            'ground_speed_kts': gps.ground_speed * 1.94384,
            'track': gps.track,
            'heading': att.true_heading,
            'pitch': att.pitch,
            'roll': att.roll,
            'vario': vario_val,
            'avg_vario': avg_vario_val,
            'ias_kts': ias_ms * 1.94384 if ias_ms is not None else None,
            'tas_kts': tas_ms * 1.94384 if tas_ms is not None else None,
            'wind_speed_kts': wind_speed_ms * 1.94384 if wind_speed_ms is not None else None,
            'wind_dir': wind_dir,
            'sentences_count': self.sentences_count,
            'sentences_rate': rate,
            'uptime': runtime,
        }

    def _print_xcsoar_instructions(self):
        """Print instructions for configuring XCSoar."""
        print("\n=== XCSoar Configuration Instructions ===")
        print("IMPORTANT: XCSoar must be in FLY mode (not SIM) for devices to work!")
        print("1. In XCSoar, go to Config > System > Devices")
        print("2. Add a new device with the following settings:")
        print("   - Port: TCP Client")
        print(f"   - IP Address: 127.0.0.1 (or this computer's IP for network)")
        print(f"   - TCP Port: {self.tcp_port}")
        if self.use_dll:
            print("   - Driver: Condor3 (recommended with DLL for TAS/vario/wind)")
            print("     OR Driver: Generic (basic mode, works with and without DLL)")
        else:
            print("   - Driver: Generic")
        print("3. Activate the device and ensure it says 'Connected'")
        print("===========================================\n")

    def _process_loop(self):
        """Main processing loop to convert data and send to TCP clients."""
        last_update_time = 0
        last_stats_time = 0

        while self.running:
            current_time = time.time()

            # Check if it's time to send an update
            if current_time - last_update_time >= self.update_interval:
                # DLL data takes priority over UDP
                dll_connected = self.dll_reader and self.dll_reader.is_connected()
                if dll_connected:
                    gps_data = self.dll_reader.gps_data
                    attitude_data = self.dll_reader.attitude_data
                else:
                    gps_data = self.receiver.gps_data
                    attitude_data = self.receiver.attitude_data

                # Update vario calculator (still useful as fallback and for avg vario)
                if gps_data.timestamp > 0:
                    self.vario_calculator.update(gps_data.altitude, gps_data.timestamp)

                # When DLL provides direct vertical_speed, feed it into PTAS1 via GPSData
                # (gps_data.vertical_speed is already set by DLLReader)

                # Convert to NMEA
                nmea_sentences = self.converter.create_nmea_sentences(
                    gps_data, attitude_data, self.vario_calculator,
                    dll_connected=dll_connected)

                # Validate sentences before sending
                valid_sentences = []
                for sentence in nmea_sentences:
                    if self.converter.validate_nmea_sentence(sentence):
                        valid_sentences.append(sentence)
                    else:
                        print(f"Invalid NMEA sentence detected: {sentence}")

                # Send to TCP clients
                for sentence in valid_sentences:
                    self.tcp_server.send_to_all_clients(sentence)
                    self.sentences_count += 1

                # Update last update time
                last_update_time = current_time

            # Periodically show statistics (every 10 seconds)
            if current_time - last_stats_time >= 10:
                with self.tcp_server.clients_lock:
                    client_count = len(self.tcp_server.clients)

                runtime = current_time - self.start_time
                rate = self.sentences_count / runtime if runtime > 0 else 0

                dll_connected = self.dll_reader and self.dll_reader.is_connected()
                source = "[DLL]" if dll_connected else "[UDP]"
                vario_str = ""
                if dll_connected and self.dll_reader.gps_data.vertical_speed is not None:
                    vario_str = f", Vario: {self.dll_reader.gps_data.vertical_speed:+.1f} m/s"
                elif self.vario_calculator.is_valid:
                    vario_str = f", Vario: {self.vario_calculator.vario:+.1f} m/s"
                print(f"{source} {self.sentences_count} NMEA sent ({rate:.1f}/sec) to {client_count} client(s){vario_str}")

                # If no clients are connected and debug level is > 0, remind about XCSoar configuration
                if client_count == 0 and self.debug_level >= 1:
                    print("No clients connected! Make sure XCSoar is properly configured.")

                last_stats_time = current_time

            # Sleep a bit to avoid busy waiting
            time.sleep(0.01)

    def stop(self):
        """Stop all components."""
        self.running = False
        self.receiver.stop()
        if self.dll_reader:
            self.dll_reader.stop()
        self.tcp_server.stop()
        print("Aerofly FS4 to XCSoar converter stopped.")

class BridgeGUI:
    """Tkinter GUI for the AFS4 to XCSoar bridge."""

    UPDATE_MS = 200          # GUI refresh interval
    FONT_LABEL = ("Arial", 10)
    FONT_VALUE = ("Consolas", 11, "bold")
    FONT_STATUS = ("Arial", 10, "bold")
    FONT_FOOTER = ("Consolas", 9)
    COLOR_GREEN = "#4CAF50"
    COLOR_RED = "#F44336"
    COLOR_BG = "#f5f5f5"

    def __init__(self, bridge: AeroflyToXCSoar):
        self.bridge = bridge

        self.root = tk.Tk()
        self.root.title(f"AFS4 → XCSoar Bridge v{VERSION}")
        self.root.geometry("420x480")
        self.root.resizable(False, False)
        self.root.configure(bg=self.COLOR_BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()

    # ── UI construction ──────────────────────────────────────────────

    def _build_ui(self):
        bg = self.COLOR_BG

        # Status bar (row 1: AFS4 + XCSoar)
        status_frame = tk.Frame(self.root, bg=bg)
        status_frame.pack(fill="x", padx=10, pady=(8, 2))

        self.lbl_afs4 = tk.Label(status_frame, text="● AFS4: ---", font=self.FONT_STATUS,
                                  fg=self.COLOR_RED, bg=bg, anchor="w")
        self.lbl_afs4.pack(side="left", expand=True, fill="x")

        self.lbl_xcsoar = tk.Label(status_frame, text="● XCSoar: ---", font=self.FONT_STATUS,
                                    fg=self.COLOR_RED, bg=bg, anchor="e")
        self.lbl_xcsoar.pack(side="right", expand=True, fill="x")

        # Status bar (row 2: DLL + data source)
        dll_frame = tk.Frame(self.root, bg=bg)
        dll_frame.pack(fill="x", padx=10, pady=(0, 4))

        self.lbl_dll = tk.Label(dll_frame, text="● DLL: ---", font=self.FONT_STATUS,
                                fg=self.COLOR_RED, bg=bg, anchor="w")
        self.lbl_dll.pack(side="left", expand=True, fill="x")

        self.lbl_source = tk.Label(dll_frame, text="Source: ---", font=self.FONT_STATUS,
                                    fg="#666666", bg=bg, anchor="e")
        self.lbl_source.pack(side="right", expand=True, fill="x")

        # Separator
        tk.Frame(self.root, height=1, bg="#cccccc").pack(fill="x", padx=10, pady=2)

        # Flight data frame
        data_frame = tk.LabelFrame(self.root, text=" Flight Data ", font=self.FONT_LABEL,
                                    bg=bg, padx=10, pady=6)
        data_frame.pack(fill="both", expand=True, padx=10, pady=4)

        self.data_labels = {}
        fields = [
            ("altitude", "Altitude"),
            ("speed", "Speed"),
            ("ias_tas", "IAS / TAS"),
            ("heading", "Heading"),
            ("track", "Track"),
            ("vario", "Vario"),
            ("avg_vario", "Avg Vario"),
            ("wind", "Wind"),
        ]
        for i, (key, label) in enumerate(fields):
            tk.Label(data_frame, text=f"{label}:", font=self.FONT_LABEL, bg=bg,
                     anchor="w", width=10).grid(row=i, column=0, sticky="w", pady=1)
            val = tk.Label(data_frame, text="---", font=self.FONT_VALUE, bg=bg, anchor="w")
            val.grid(row=i, column=1, sticky="w", padx=(8, 0), pady=1)
            self.data_labels[key] = val

        # Separator
        tk.Frame(self.root, height=1, bg="#cccccc").pack(fill="x", padx=10, pady=2)

        # Footer stats
        footer_frame = tk.Frame(self.root, bg=bg)
        footer_frame.pack(fill="x", padx=10, pady=2)

        self.lbl_nmea = tk.Label(footer_frame, text="NMEA: 0 sent (0.0/sec)",
                                  font=self.FONT_FOOTER, bg=bg, anchor="w")
        self.lbl_nmea.pack(side="left")

        self.lbl_uptime = tk.Label(footer_frame, text="Uptime: 00:00:00",
                                    font=self.FONT_FOOTER, bg=bg, anchor="e")
        self.lbl_uptime.pack(side="right")

        # Buttons
        btn_frame = tk.Frame(self.root, bg=bg)
        btn_frame.pack(fill="x", padx=10, pady=(4, 10))

        self.btn_start = tk.Button(btn_frame, text="Start", font=self.FONT_LABEL,
                                    bg=self.COLOR_GREEN, fg="white", width=12,
                                    command=self._on_start)
        self.btn_start.pack(side="left", expand=True, padx=4)

        self.btn_stop = tk.Button(btn_frame, text="Stop", font=self.FONT_LABEL,
                                   bg=self.COLOR_RED, fg="white", width=12,
                                   command=self._on_stop, state="disabled")
        self.btn_stop.pack(side="right", expand=True, padx=4)

    # ── Actions ──────────────────────────────────────────────────────

    def _on_start(self):
        self.bridge.start_services()
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self._schedule_update()

    def _on_stop(self):
        self.bridge.stop()
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self._reset_display()

    def _on_close(self):
        if self.bridge.running:
            self.bridge.stop()
        self.root.destroy()

    # ── Periodic update ──────────────────────────────────────────────

    def _schedule_update(self):
        if not self.bridge.running:
            return
        self._update_display()
        self.root.after(self.UPDATE_MS, self._schedule_update)

    def _update_display(self):
        s = self.bridge.get_status()

        # Connection status
        if s['afs4_connected']:
            self.lbl_afs4.config(text="● AFS4: Connected", fg=self.COLOR_GREEN)
        else:
            self.lbl_afs4.config(text="● AFS4: Waiting...", fg=self.COLOR_RED)

        clients = s['xcsoar_clients']
        if clients > 0:
            self.lbl_xcsoar.config(text=f"● XCSoar: {clients} client(s)", fg=self.COLOR_GREEN)
        else:
            self.lbl_xcsoar.config(text="● XCSoar: No clients", fg=self.COLOR_RED)

        # DLL status
        if s['dll_connected']:
            self.lbl_dll.config(text="● DLL: Connected", fg=self.COLOR_GREEN)
        elif self.bridge.use_dll:
            self.lbl_dll.config(text="● DLL: Not found", fg=self.COLOR_RED)
        else:
            self.lbl_dll.config(text="● DLL: Disabled", fg="#999999")

        self.lbl_source.config(text=f"Source: {s['data_source']}")

        # Flight data
        self.data_labels["altitude"].config(
            text=f"{s['altitude_m']:.0f} m  ({s['altitude_ft']:.0f} ft)")
        self.data_labels["speed"].config(
            text=f"{s['ground_speed_kmh']:.0f} km/h  ({s['ground_speed_kts']:.0f} kts)")

        if s.get('ias_kts') is not None and s.get('tas_kts') is not None:
            self.data_labels["ias_tas"].config(
                text=f"{s['ias_kts']:.0f} / {s['tas_kts']:.0f} kts")
        else:
            self.data_labels["ias_tas"].config(text="--- (no DLL)")

        self.data_labels["heading"].config(text=f"{s['heading']:.1f}°")
        self.data_labels["track"].config(text=f"{s['track']:.1f}°")

        if s['vario'] is not None:
            arrow = "▲" if s['vario'] > 0.1 else ("▼" if s['vario'] < -0.1 else "─")
            self.data_labels["vario"].config(text=f"{s['vario']:+.1f} m/s  {arrow}")
        else:
            self.data_labels["vario"].config(text="---")

        if s['avg_vario'] is not None:
            self.data_labels["avg_vario"].config(text=f"{s['avg_vario']:+.1f} m/s")
        else:
            self.data_labels["avg_vario"].config(text="---")

        if s.get('wind_speed_kts') is not None and s.get('wind_dir') is not None:
            self.data_labels["wind"].config(
                text=f"{s['wind_dir']:.0f}° / {s['wind_speed_kts']:.0f} kts")
        else:
            self.data_labels["wind"].config(text="--- (no DLL)")

        # Footer
        self.lbl_nmea.config(text=f"NMEA: {s['sentences_count']} sent ({s['sentences_rate']:.1f}/sec)")
        uptime = int(s['uptime'])
        h, m, sec = uptime // 3600, (uptime % 3600) // 60, uptime % 60
        self.lbl_uptime.config(text=f"Uptime: {h:02d}:{m:02d}:{sec:02d}")

    def _reset_display(self):
        self.lbl_afs4.config(text="● AFS4: ---", fg=self.COLOR_RED)
        self.lbl_xcsoar.config(text="● XCSoar: ---", fg=self.COLOR_RED)
        self.lbl_dll.config(text="● DLL: ---", fg=self.COLOR_RED)
        self.lbl_source.config(text="Source: ---")
        for lbl in self.data_labels.values():
            lbl.config(text="---")
        self.lbl_nmea.config(text="NMEA: 0 sent (0.0/sec)")
        self.lbl_uptime.config(text="Uptime: 00:00:00")

    # ── Run ───────────────────────────────────────────────────────────

    def run(self):
        self.root.mainloop()


def main():
    """Parse command line arguments and start the converter."""
    parser = argparse.ArgumentParser(description='Convert Aerofly FS4 UDP data to NMEA format for XCSoar')
    
    parser.add_argument('--udp-port', type=int, default=DEFAULT_UDP_PORT,
                        help=f'UDP port to receive Aerofly data (default: {DEFAULT_UDP_PORT})')
    
    parser.add_argument('--tcp-port', type=int, default=DEFAULT_TCP_PORT,
                        help=f'TCP port to serve NMEA data (default: {DEFAULT_TCP_PORT})')
    
    parser.add_argument('--update-rate', type=float, default=DEFAULT_UPDATE_RATE,
                        help=f'Update rate in Hz (default: {DEFAULT_UPDATE_RATE})')
    
    parser.add_argument('--mag-var', type=float, default=DEFAULT_MAGNETIC_VARIATION,
                        help=f'Magnetic variation in degrees, East positive (default: {DEFAULT_MAGNETIC_VARIATION})')
    
    parser.add_argument('--debug', type=int, default=DEFAULT_DEBUG_LEVEL, choices=[0, 1, 2],
                        help=f'Debug level: 0=minimal, 1=normal, 2=verbose (default: {DEFAULT_DEBUG_LEVEL})')
    
    parser.add_argument('--no-gui', action='store_true',
                        help='Run in terminal mode without GUI')

    parser.add_argument('--no-dll', action='store_true',
                        help='Disable DLL shared memory reader (UDP only)')

    parser.add_argument('--test', action='store_true',
                        help='Run in test mode (generate fake data if no Aerofly connection)')

    args = parser.parse_args()

    bridge = AeroflyToXCSoar(
        udp_port=args.udp_port,
        tcp_port=args.tcp_port,
        update_rate=args.update_rate,
        magnetic_variation=args.mag_var,
        debug_level=args.debug,
        use_dll=not args.no_dll,
    )

    if args.no_gui:
        bridge.start()
    else:
        gui = BridgeGUI(bridge)
        gui.run()

if __name__ == "__main__":
    main()
