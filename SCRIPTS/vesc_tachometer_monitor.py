#!/usr/bin/env python3
"""
Real-time VESC Tachometer Monitor

This script reads CAN messages from a USB-CAN interface in real-time
and displays tachometer values for VESC controllers 28 and 46.

Requirements:
- python-can library
- USB-CAN interface (like CANtact, PCAN, etc.)

Usage:
    python3 vesc_tachometer_monitor.py --interface socketcan --channel can0
    python3 vesc_tachometer_monitor.py --interface pcan --channel PCAN_USBBUS1
"""

import can
import struct
import argparse
import time
import sys
from typing import Optional, Dict
from dataclasses import dataclass


@dataclass
class VESCTachometer:
    """Stores tachometer data for a VESC controller"""
    vesc_id: int
    tachometer: int
    voltage: float
    last_update: float


class VESCTachometerMonitor:
    """Real-time VESC tachometer monitor"""
    
    def __init__(self, interface: str, channel: str, bitrate: int = 500000):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        
        # VESC tachometer data storage
        self.vesc_data: Dict[int, VESCTachometer] = {}
        
        # CAN ID mappings for tachometer messages (STATUS_5)
        self.tachometer_ids = {
            0x1B1C: 28,  # VESC ID 28 tachometer
            0x1B2E: 46,  # VESC ID 46 tachometer
        }
        
        # Initialize CAN bus
        self._init_can_bus()
    
    def _init_can_bus(self):
        """Initialize CAN bus connection"""
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                interface=self.interface,
                bitrate=self.bitrate
            )
            print(f"✓ Connected to CAN bus: {self.interface} - {self.channel}")
        except Exception as e:
            print(f"✗ Failed to connect to CAN bus: {e}")
            sys.exit(1)
    
    def parse_tachometer_message(self, msg: can.Message) -> Optional[VESCTachometer]:
        """Parse VESC STATUS_5 message (tachometer + voltage)"""
        if msg.arbitration_id not in self.tachometer_ids:
            return None
        
        if len(msg.data) < 6:
            return None
        
        vesc_id = self.tachometer_ids[msg.arbitration_id]
        
        # Parse according to VESC protocol
        # B0-B3: Tachometer (int32, scale 6 for electrical revolutions)
        # B4-B5: Voltage In (int16, scale 10)
        tachometer_raw = struct.unpack('>i', msg.data[0:4])[0]
        voltage_raw = struct.unpack('>h', msg.data[4:6])[0]
        
        # Convert to mechanical revolutions (divide by 6 for electrical to mechanical)
        tachometer = tachometer_raw // 6
        voltage = voltage_raw / 10.0
        
        return VESCTachometer(
            vesc_id=vesc_id,
            tachometer=tachometer,
            voltage=voltage,
            last_update=time.time()
        )
    
    def display_tachometer_values(self):
        """Display current tachometer values in a clean format"""
        # Clear screen and move cursor to top
        print("\033[2J\033[H", end="")
        
        print("=" * 60)
        print("  VESC Real-time Tachometer Monitor")
        print("=" * 60)
        print(f"Interface: {self.interface} | Channel: {self.channel}")
        print("Press Ctrl+C to stop")
        print("-" * 60)
        
        current_time = time.time()
        
        # Display data for each VESC
        for vesc_id in [28, 46]:
            if vesc_id in self.vesc_data:
                data = self.vesc_data[vesc_id]
                age = current_time - data.last_update
                status = "LIVE" if age < 1.0 else f"OLD ({age:.1f}s)"
                
                print(f"VESC {vesc_id:2d}: Tachometer: {data.tachometer:8d} | "
                      f"Voltage: {data.voltage:5.1f}V | Status: {status}")
            else:
                print(f"VESC {vesc_id:2d}: No data received")
        
        print("-" * 60)
        
        # Calculate RPM differences if we have previous values
        if len(self.vesc_data) >= 2:
            print("Tachometer Analysis:")
            for vesc_id in [28, 46]:
                if vesc_id in self.vesc_data:
                    data = self.vesc_data[vesc_id]
                    if hasattr(data, 'prev_tachometer') and hasattr(data, 'prev_time'):
                        dt = data.last_update - data.prev_time
                        if dt > 0:
                            tacho_diff = data.tachometer - data.prev_tachometer
                            rpm = (tacho_diff / dt) * 60.0  # Convert to RPM
                            print(f"  VESC {vesc_id}: {rpm:6.1f} RPM")
        
        print(f"\nLast update: {time.strftime('%H:%M:%S', time.localtime())}")
    
    def run_monitor(self, update_interval: float = 0.1):
        """Run the real-time monitor"""
        print("Starting VESC Tachometer Monitor...")
        print("Waiting for CAN messages...")
        
        try:
            while True:
                # Read CAN message with timeout
                msg = self.bus.recv(timeout=update_interval)
                
                if msg is not None:
                    # Parse tachometer message
                    tach_data = self.parse_tachometer_message(msg)
                    
                    if tach_data is not None:
                        # Store previous values for RPM calculation
                        if tach_data.vesc_id in self.vesc_data:
                            prev_data = self.vesc_data[tach_data.vesc_id]
                            tach_data.prev_tachometer = prev_data.tachometer
                            tach_data.prev_time = prev_data.last_update
                        
                        # Update stored data
                        self.vesc_data[tach_data.vesc_id] = tach_data
                
                # Update display
                self.display_tachometer_values()
                
        except KeyboardInterrupt:
            print("\n\nMonitor stopped by user")
        except Exception as e:
            print(f"\nError during monitoring: {e}")
        finally:
            if self.bus:
                self.bus.shutdown()
                print("CAN bus connection closed")
    
    def run_simple_output(self):
        """Run simple output mode - just print tachometer values as they arrive"""
        print("Simple tachometer output mode")
        print("Format: TIMESTAMP VESC_ID TACHOMETER VOLTAGE")
        print("-" * 50)
        
        try:
            while True:
                msg = self.bus.recv(timeout=1.0)
                
                if msg is not None:
                    tach_data = self.parse_tachometer_message(msg)
                    
                    if tach_data is not None:
                        timestamp = time.strftime('%H:%M:%S.%f', time.localtime())[:-3]
                        print(f"{timestamp} VESC_{tach_data.vesc_id:02d} "
                              f"{tach_data.tachometer:8d} {tach_data.voltage:5.1f}V")
                        
        except KeyboardInterrupt:
            print("\nOutput stopped by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if self.bus:
                self.bus.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Real-time VESC tachometer monitor')
    parser.add_argument('--interface', '-i', default='socketcan', 
                       help='CAN interface type (socketcan, pcan, usb2can, etc.)')
    parser.add_argument('--channel', '-c', default='can0',
                       help='CAN channel (can0, can1, PCAN_USBBUS1, etc.)')
    parser.add_argument('--bitrate', '-b', type=int, default=500000,
                       help='CAN bitrate (default: 500000)')
    parser.add_argument('--simple', '-s', action='store_true',
                       help='Simple output mode (just print values)')
    parser.add_argument('--update-rate', '-u', type=float, default=0.1,
                       help='Display update interval in seconds (default: 0.1)')
    
    args = parser.parse_args()
    
    # Check if python-can is available
    try:
        import can
    except ImportError:
        print("Error: python-can library not found!")
        print("Install with: pip install python-can")
        sys.exit(1)
    
    # Create and run monitor
    monitor = VESCTachometerMonitor(args.interface, args.channel, args.bitrate)
    
    if args.simple:
        monitor.run_simple_output()
    else:
        monitor.run_monitor(args.update_rate)


if __name__ == "__main__":
    main()
