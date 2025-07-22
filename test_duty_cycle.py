#!/usr/bin/env python3
"""
Simple VESC duty cycle test script
Tests the duty cycle commands with your exact CAN message format

Usage:
    python3 test_duty_cycle.py
"""

import can
import time
import struct

def send_duty_cycle(bus, vesc_id, duty_percent):
    """
    Send duty cycle command to VESC using the correct CAN format
    duty_percent: -10.0 to +10.0 (percentage)
    """
    # Convert duty cycle percentage to VESC format
    # Based on your working commands, VESC expects different scaling
    # Your examples: 00.00.07.D0 = +2%, FF.FF.F6.30 = -2.5%
    # This suggests: 07D0 hex = 2000 dec = 2% * 1000
    duty_vesc = int(duty_percent * 1000)
    
    # Create CAN message with your exact working format
    # Your working: cansend can0 0000001C#00.00.07.D0
    # This is CAN ID 0x1C with 4-byte data
    msg = can.Message(
        arbitration_id=vesc_id,  # Use direct VESC ID (28 = 0x1C, 46 = 0x2E)
        data=struct.pack('>i', duty_vesc),  # Big-endian int32 (4 bytes)
        is_extended_id=False
    )
    
    try:
        bus.send(msg)
        print(f"✓ Sent duty cycle {duty_percent:.1f}% to VESC {vesc_id} (0x{vesc_id:02X})")
        print(f"  CAN ID: 0x{vesc_id:08X}, Data: {msg.data.hex().upper()}")
        return True
    except Exception as e:
        print(f"✗ Failed to send duty cycle to VESC {vesc_id}: {e}")
        return False

def test_duty_cycle_range():
    """Test duty cycle commands with incremental safety"""
    
    try:
        # Initialize CAN bus
        bus = can.interface.Bus(channel='can0', interface='socketcan')
        print("✓ Connected to CAN bus")
        
        # Test with your VESC IDs
        left_vesc_id = 28   # 0x1C
        right_vesc_id = 46  # 0x2E
        
        print("\n🔧 Testing incremental duty cycle commands...")
        print("This will safely test motor control with small increments")
        
        # Test sequence: 0% -> 2% -> 0% -> -2% -> 0%
        test_sequence = [
            (0.0, "Stop"),
            (2.0, "Forward 2%"),
            (0.0, "Stop"),
            (-2.0, "Reverse 2%"),  
            (0.0, "Stop"),
        ]
        
        for duty, description in test_sequence:
            print(f"\n{description} (Duty: {duty:.1f}%)")
            
            # Send to both VESCs
            left_success = send_duty_cycle(bus, left_vesc_id, duty)
            right_success = send_duty_cycle(bus, right_vesc_id, duty)
            
            if left_success and right_success:
                print(f"✓ Both motors set to {duty:.1f}%")
            else:
                print("✗ Failed to send commands")
                break
                
            # Wait between commands
            time.sleep(2.0)
        
        # Final safety stop
        print(f"\n🛑 Final safety stop...")
        send_duty_cycle(bus, left_vesc_id, 0.0)
        send_duty_cycle(bus, right_vesc_id, 0.0)
        
        print("✓ Test completed safely")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
    finally:
        try:
            bus.shutdown()
        except:
            pass

def validate_can_messages():
    """Validate that our CAN message format matches your working examples"""
    
    print("🔍 Validating CAN message format...")
    
    # Your working examples:
    # VESC 28 (0x1C): cansend can0 0000001C#00.00.07.D0  (positive duty)
    # VESC 46 (0x2E): cansend can0 0000002E#FF.FF.F6.30  (negative duty)
    
    # Test positive duty cycle (should match 00.00.07.D0)
    duty_percent = 2.0  # 2%
    duty_vesc = int(duty_percent * 1000)  # 2000
    data = struct.pack('>i', duty_vesc)
    print(f"Duty +2%: {duty_vesc} -> {data.hex().upper()}")
    print(f"Expected: 00.00.07.D0, Got: {'.'.join([f'{b:02X}' for b in data])}")
    
    # Test negative duty cycle (should match FF.FF.F6.30)  
    duty_percent = -2.5  # -2.5%
    duty_vesc = int(duty_percent * 1000)  # -2500
    duty_vesc_unsigned = duty_vesc & 0xFFFFFFFF  # Convert to unsigned
    data = struct.pack('>I', duty_vesc_unsigned)  # Unsigned for display
    print(f"Duty -2.5%: {duty_vesc} -> {data.hex().upper()}")
    print(f"Expected: FF.FF.F6.7C, Got: {'.'.join([f'{b:02X}' for b in data])}")

if __name__ == "__main__":
    print("🚀 VESC Duty Cycle Safety Test")
    print("=" * 50)
    
    # First validate message format
    validate_can_messages()
    
    print("\n" + "=" * 50)
    input("Press Enter to start motor test (make sure robot is secure)...")
    
    # Then test actual motor control
    test_duty_cycle_range()
