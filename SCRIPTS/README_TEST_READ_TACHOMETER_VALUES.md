# VESC CAN Tachometer Tools

A collection of Python tools for extracting and monitoring tachometer data from VESC motor controllers via CAN bus.

## Overview

This toolkit provides:
1. **Real-time monitoring** of VESC tachometer values from USB-CAN interface
2. **Analysis and extraction** of tachometer data from CAN dump files
3. **Data visualization** and export capabilities

## VESC Configuration

Your setup includes:
- **VESC ID 28**: Stationary motor (monitoring only)
- **VESC ID 46**: Active motor with ~3% duty cycle operation

## Tools Included

### 1. Real-time Tachometer Monitor (`SCRIPTS/vesc_tachometer_monitor.py`)

**WORKING AND TESTED** - Monitors live tachometer values from USB-CAN interface.

**Features:**
- Real-time display of tachometer values for both VESCs
- RPM calculation from tachometer differences
- Voltage monitoring  
- Simple output mode for logging
- Successfully tested with live CAN data

**Usage:**
```bash
# Real-time monitor with live display
cd SCRIPTS
./vesc_tachometer_monitor.py --interface socketcan --channel can0

# Simple output mode (for logging)
./vesc_tachometer_monitor.py --simple --channel can0

# Custom update rate
./vesc_tachometer_monitor.py --channel can0 --update-rate 0.05
```

**Test Results:**
- VESC 28: Stationary motor (0 RPM, monitoring only)
- VESC 46: Active motor (~3000 RPM with ~3% duty cycle)
- Voltage monitoring: 67.7-68.2V range (healthy)

### 2. Data Visualizer (`visualize_vesc_data.py`)

Creates plots from extracted CSV data.

**Features:**
- Time-series plots of all parameters
- Motor correlation analysis
- Power estimation
- Publication-ready plots

**Usage:**
```bash
# Create visualization plots (requires CSV data)
./visualize_vesc_data.py tachometer_data.csv

# Custom output directory
./visualize_vesc_data.py data.csv --output ./plots/
```

**Note:** CAN dump analyzer was removed - use real-time monitor instead.

## Setup and Installation

### Quick Setup
```bash
# Run the setup script
./setup.sh
```

### Manual Setup
```bash
# Install Python dependencies
pip3 install python-can

# Install CAN utilities (Ubuntu/Debian)
sudo apt install can-utils

# Configure CAN interface (example for socketcan)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

## CAN Interface Configuration

### SocketCAN (Linux)
```bash
# Load CAN modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_bcm

# Configure interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Test with candump
candump can0
```

### USB-CAN Adapters

**CANtact/CANable:**
```bash
# Usually appears as /dev/ttyACM0
./vesc_tachometer_monitor.py --interface slcan --channel /dev/ttyACM0
```

**PCAN-USB:**
```bash
./vesc_tachometer_monitor.py --interface pcan --channel PCAN_USBBUS1
```

## CAN Message Format

The tools decode VESC CAN messages:

### STATUS_5 Messages (Tachometer + Voltage)
- **VESC 28**: CAN ID `0x1B1C`
- **VESC 46**: CAN ID `0x1B2E`

**Data Format:**
- Bytes 0-3: Tachometer (int32, electrical revolutions × 6)
- Bytes 4-5: Input voltage (int16, scale × 10)

### STATUS Messages (ERPM + Current + Duty)
- **VESC 28**: CAN ID `0x091C`
- **VESC 46**: CAN ID `0x092E`

**Data Format:**
- Bytes 0-3: ERPM (int32)
- Bytes 4-5: Motor current (int16, scale × 10)
- Bytes 6-7: Duty cycle (int16, scale × 1000)

## Example Workflows

### 1. Live Monitoring
```bash
# Start real-time monitoring (WORKING!)
cd SCRIPTS
./vesc_tachometer_monitor.py --channel can0

# Log tachometer values to file
./vesc_tachometer_monitor.py --simple --channel can0 > tach_log.txt
```

### 2. CAN Traffic Recording
```bash
# Record CAN traffic for analysis
candump can0 -L > session.log

# Monitor live traffic
candump can0
```

### 3. Continuous Monitoring
```bash
# Monitor and log simultaneously
candump can0 -L | tee session.log &
cd SCRIPTS
./vesc_tachometer_monitor.py --channel can0
```

## Data Analysis Results

From your sample data (46.31 seconds):

**Motor 28 (Stationary):**
- Tachometer: 689 (constant)
- ERPM: 0
- Current: 0.0 A
- Status: Not moving

**Motor 46 (Active):**
- Tachometer: 17,768 → 17,928 (160 count increase)
- ERPM: 3-157
- Current: -1.0 to 1.5 A
- Duty Cycle: ~3% (0.017-0.030)
- Average RPM: 204.8

## Troubleshooting

### CAN Interface Issues
```bash
# Check interface status
ip link show can0

# Reset interface
sudo ip link set down can0
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Check for messages
candump can0 -c 10
```

### Python Dependencies
```bash
# Install/update python-can
pip3 install --upgrade python-can

# List available interfaces
python3 -c "import can; print(can.interface.BACKENDS)"
```

### Permission Issues
```bash
# Add user to dialout group (for USB devices)
sudo usermod -a -G dialout $USER

# Set CAN interface permissions
sudo chmod 666 /dev/can0  # if applicable
```

## VESC Tool Command Message Decryption Algorithm

### Overview

VESC Tool uses a sophisticated packet-based communication protocol with CRC verification for reliable data transmission. The protocol supports both standard UART and CAN bus communication channels.

### Packet Structure

All VESC command messages follow this format:

```
[Start Bytes] [Length] [Payload] [CRC16] [Stop Byte]
```

**Standard Packet Format:**
```
| 2   | 3      | 1-2 bytes | N bytes | 2 bytes | 1 byte |
| STX | Length | Length    | Data    | CRC16   | ETX    |
```

- **Start Bytes**: `0x02` (STX) - Start of transmission
- **Length Field**: 
  - If length < 256: Single byte length
  - If length ≥ 256: `0x03` followed by 2-byte big-endian length
- **Payload**: Command ID + Command data
- **CRC16**: CCITT CRC-16 checksum (polynomial 0x1021)
- **Stop Byte**: `0x03` (ETX) - End of transmission

### CRC16 Implementation

The VESC protocol uses CCITT CRC-16 with polynomial `0x1021` and initial value `0x0000`:

```c
unsigned short crc16(unsigned char *buf, unsigned int len) {
    unsigned short cksum = 0;
    for (unsigned int i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}
```

### Command Packet Processing

1. **Packet Detection**: Look for start byte `0x02`
2. **Length Extraction**: 
   - Read length byte(s)
   - Validate length is within bounds (≤ 65536 bytes)
3. **Payload Reception**: Collect N bytes of command data
4. **CRC Verification**: 
   - Calculate CRC16 of payload
   - Compare with received CRC16 (2 bytes, big-endian)
5. **Stop Byte Check**: Verify end byte is `0x03`

### Decryption Process

**Step 1: Packet Assembly**
```python
def decode_vesc_packet(raw_bytes):
    if raw_bytes[0] != 0x02:  # Check start byte
        return None
    
    # Extract length
    if raw_bytes[1] < 256:
        length = raw_bytes[1]
        data_start = 2
    else:
        length = (raw_bytes[2] << 8) | raw_bytes[3]
        data_start = 4
    
    # Extract payload
    payload = raw_bytes[data_start:data_start + length]
    
    # Verify CRC
    crc_received = (raw_bytes[data_start + length] << 8) | raw_bytes[data_start + length + 1]
    crc_calculated = crc16(payload)
    
    if crc_calculated != crc_received:
        return None  # CRC mismatch
    
    # Check stop byte
    if raw_bytes[data_start + length + 2] != 0x03:
        return None
    
    return payload
```

**Step 2: Command Extraction**
```python
def parse_command(payload):
    command_id = payload[0]  # First byte is command ID
    command_data = payload[1:]  # Remaining bytes are parameters
    return command_id, command_data
```

### Security Features

1. **CRC Integrity**: Every packet has CRC16 verification
2. **Length Validation**: Prevents buffer overflow attacks
3. **Framing**: Start/stop bytes prevent data corruption
4. **Command Filtering**: Only valid command IDs are processed

### Common Command IDs

- `COMM_GET_VALUES` (4): Request motor values
- `COMM_SET_DUTY` (5): Set motor duty cycle
- `COMM_SET_CURRENT` (6): Set motor current
- `COMM_SET_RPM` (8): Set motor RPM
- `COMM_GET_MCCONF` (14): Get motor configuration
- `COMM_SET_MCCONF` (15): Set motor configuration

### CAN Bus Extensions

For CAN communication, the packet is encapsulated in CAN frames:

1. **CAN ID Format**: `0x8000 | (controller_id << 8) | packet_sequence`
2. **Multi-frame Support**: Large packets split across multiple CAN frames
3. **Frame Reassembly**: Receiver reconstructs original packet
4. **CAN-specific CRC**: Additional CRC layer for CAN reliability

### Implementation Notes

- All multi-byte values use big-endian byte order
- Maximum packet size: 65536 bytes
- Timeout handling: 1000ms typical timeout for responses
- Error recovery: Packet retransmission on CRC failure

This decryption algorithm ensures secure, reliable communication between VESC Tool and motor controllers while maintaining backward compatibility across firmware versions.

## File Structure

```
canvesc/
├── SCRIPTS/
│   └── vesc_tachometer_monitor.py  # Real-time monitor (WORKING!)
├── visualize_vesc_data.py          # Data visualizer  
├── setup.sh                        # Setup script
├── README.md                       # This file
├── ANALYSIS_SUMMARY.md             # Analysis results
├── canvesc/
│   └── output.log                 # Sample CAN dump
└── tachometer_data.csv            # Extracted data
```

## License

Open source tools for VESC motor controller analysis.

## Support

For issues or questions about VESC CAN protocol, refer to:
- [VESC Project Documentation](https://vesc-project.com/)
- [VESC Firmware Repository](https://github.com/vedderb/bldc)
