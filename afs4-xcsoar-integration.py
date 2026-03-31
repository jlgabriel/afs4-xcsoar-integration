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

# Default configuration
DEFAULT_UDP_PORT = 49002          # Aerofly FS4 default UDP port
DEFAULT_TCP_PORT = 4353           # Port for TCP server (for XCSoar to connect)
DEFAULT_UPDATE_RATE = 5           # How many NMEA sentences per second
DEFAULT_MAGNETIC_VARIATION = 0.0  # Magnetic variation in degrees (East positive, West negative)
DEFAULT_DEBUG_LEVEL = 1           # Debug level: 0=minimal, 1=normal, 2=verbose

@dataclass
class GPSData:
    """Store GPS data received from Aerofly FS4 (or DLL in the future)."""
    longitude: float = 0.0
    latitude: float = 0.0
    altitude: float = 0.0          # MSL altitude in meters
    track: float = 0.0
    ground_speed: float = 0.0      # m/s
    timestamp: float = 0.0
    # Extended fields (calculated from UDP, direct from DLL in the future)
    vertical_speed: Optional[float] = None    # m/s, None = not yet available
    barometric_altitude: Optional[float] = None  # meters, None = use MSL altitude

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

class NMEAConverter:
    """
    Converts Aerofly data to NMEA sentences for XCSoar.
    """
    def __init__(self, magnetic_variation=DEFAULT_MAGNETIC_VARIATION, debug_level=DEFAULT_DEBUG_LEVEL):
        self.magnetic_variation = magnetic_variation
        self.debug_level = debug_level

    def create_nmea_sentences(self, gps: GPSData, attitude: AttitudeData,
                              vario: Optional['VarioCalculator'] = None) -> List[str]:
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
    NMEA converter, and TCP server.
    """
    def __init__(self, udp_port=DEFAULT_UDP_PORT, tcp_port=DEFAULT_TCP_PORT, 
                 update_rate=DEFAULT_UPDATE_RATE, magnetic_variation=DEFAULT_MAGNETIC_VARIATION,
                 debug_level=DEFAULT_DEBUG_LEVEL):
        self.udp_port = udp_port
        self.tcp_port = tcp_port
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate if update_rate > 0 else 0.2
        self.debug_level = debug_level
        self.magnetic_variation = magnetic_variation
        
        # Initialize components
        self.receiver = AeroflyReceiver(udp_port, debug_level)
        self.converter = NMEAConverter(magnetic_variation, debug_level)
        self.tcp_server = TCPServer(tcp_port, debug_level)
        self.vario_calculator = VarioCalculator()
        
        self.running = False
        self.main_thread = None
        self.start_time = time.time()
        self.sentences_count = 0

    def start_services(self):
        """Start all background services (receiver, TCP server, processing loop)."""
        print(f"Starting Aerofly FS4 to XCSoar NMEA converter v4.0...")
        print(f"UDP Port: {self.udp_port}, TCP Port: {self.tcp_port}")
        print(f"Update rate: {self.update_rate} Hz, Magnetic variation: {self.magnetic_variation}°")
        print(f"Debug level: {self.debug_level}")

        self.receiver.start()
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
        gps = self.receiver.gps_data
        att = self.receiver.attitude_data
        with self.tcp_server.clients_lock:
            client_count = len(self.tcp_server.clients)
        runtime = time.time() - self.start_time if self.running else 0
        rate = self.sentences_count / runtime if runtime > 0 else 0

        return {
            'running': self.running,
            'afs4_connected': self.receiver.is_connected(),
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
            'vario': self.vario_calculator.vario if self.vario_calculator.is_valid else None,
            'avg_vario': self.vario_calculator.average_vario if self.vario_calculator.is_valid else None,
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
                # Get latest data
                gps_data = self.receiver.gps_data
                attitude_data = self.receiver.attitude_data

                # Update vario calculator with latest altitude
                if gps_data.timestamp > 0:
                    self.vario_calculator.update(gps_data.altitude, gps_data.timestamp)

                # Convert to NMEA (pass vario calculator for PTAS1 sentence)
                nmea_sentences = self.converter.create_nmea_sentences(
                    gps_data, attitude_data, self.vario_calculator)
                
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
                
                vario_str = f", Vario: {self.vario_calculator.vario:+.1f} m/s" if self.vario_calculator.is_valid else ""
                print(f"Status: {self.sentences_count} NMEA sentences sent ({rate:.1f}/sec) to {client_count} client(s){vario_str}")
                
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
        self.root.title("AFS4 → XCSoar Bridge")
        self.root.geometry("420x380")
        self.root.resizable(False, False)
        self.root.configure(bg=self.COLOR_BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()

    # ── UI construction ──────────────────────────────────────────────

    def _build_ui(self):
        bg = self.COLOR_BG

        # Status bar
        status_frame = tk.Frame(self.root, bg=bg)
        status_frame.pack(fill="x", padx=10, pady=(8, 4))

        self.lbl_afs4 = tk.Label(status_frame, text="● AFS4: ---", font=self.FONT_STATUS,
                                  fg=self.COLOR_RED, bg=bg, anchor="w")
        self.lbl_afs4.pack(side="left", expand=True, fill="x")

        self.lbl_xcsoar = tk.Label(status_frame, text="● XCSoar: ---", font=self.FONT_STATUS,
                                    fg=self.COLOR_RED, bg=bg, anchor="e")
        self.lbl_xcsoar.pack(side="right", expand=True, fill="x")

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
            ("heading", "Heading"),
            ("track", "Track"),
            ("vario", "Vario"),
            ("avg_vario", "Avg Vario"),
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

        # Flight data
        self.data_labels["altitude"].config(
            text=f"{s['altitude_m']:.0f} m  ({s['altitude_ft']:.0f} ft)")
        self.data_labels["speed"].config(
            text=f"{s['ground_speed_kmh']:.0f} km/h  ({s['ground_speed_kts']:.0f} kts)")
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

        # Footer
        self.lbl_nmea.config(text=f"NMEA: {s['sentences_count']} sent ({s['sentences_rate']:.1f}/sec)")
        uptime = int(s['uptime'])
        h, m, sec = uptime // 3600, (uptime % 3600) // 60, uptime % 60
        self.lbl_uptime.config(text=f"Uptime: {h:02d}:{m:02d}:{sec:02d}")

    def _reset_display(self):
        self.lbl_afs4.config(text="● AFS4: ---", fg=self.COLOR_RED)
        self.lbl_xcsoar.config(text="● XCSoar: ---", fg=self.COLOR_RED)
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

    parser.add_argument('--test', action='store_true',
                        help='Run in test mode (generate fake data if no Aerofly connection)')

    args = parser.parse_args()

    bridge = AeroflyToXCSoar(
        udp_port=args.udp_port,
        tcp_port=args.tcp_port,
        update_rate=args.update_rate,
        magnetic_variation=args.mag_var,
        debug_level=args.debug
    )

    if args.no_gui:
        bridge.start()
    else:
        gui = BridgeGUI(bridge)
        gui.run()

if __name__ == "__main__":
    main()
