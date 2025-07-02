#!/usr/bin/env python3
"""
One-shot script to enable ATTITUDE_QUATERNION messages
Run this once, then use C++ reader
"""

import time
import sys
from pymavlink import mavutil

def enable_quaternion_messages(connection_string='udp:127.0.0.1:14550', rate_hz=50):
    """
    Connect to ArduPilot and enable ATTITUDE_QUATERNION messages
    This only needs to be run once
    """
    print(f"Connecting to {connection_string}...")
    
    # Connect
    mav = mavutil.mavlink_connection(connection_string)
    mav.wait_heartbeat()
    print(f"✓ Connected to system {mav.target_system}")
    
    # Calculate interval
    interval_us = int(1e6 / rate_hz) if rate_hz > 0 else 0
    
    print(f"Enabling ATTITUDE_QUATERNION at {rate_hz}Hz...")
    
    # Send request
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
        interval_us,
        0, 0, 0, 0, 0
    )
    
    # Wait for ACK
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    
    if ack and ack.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✓ ATTITUDE_QUATERNION enabled successfully!")
            # Test that we're receiving messages
            print("Testing message reception...")
            msg_count = 0
            start_time = time.time()
            
            while (time.time() - start_time) < 3:  # Test for 3 seconds
                msg = mav.recv_match(type='ATTITUDE_QUATERNION', blocking=True, timeout=1)
                if msg:
                    msg_count += 1
                    if msg_count == 1:
                        print(f"✓ First quaternion received: q1={msg.q1:.3f}, q2={msg.q2:.3f}, q3={msg.q3:.3f}, q4={msg.q4:.3f}")
            
            if msg_count > 0:
                freq = msg_count / 3.0
                print(f"✓ Receiving {freq:.1f} Hz (tested with {msg_count} messages)")
                
                print(f"\n{'='*50}")
                print("SUCCESS! ATTITUDE_QUATERNION is now enabled")
                print(f"{'='*50}")
                print("You can now:")
                print("1. Compile C++ reader: g++ -o read_quaternion simple_quaternion_reader.cpp -pthread")
                print("2. Run C++ reader: ./read_quaternion 14550 30")
                print("   (reads directly from ArduPilot on port 14550)")
                print("")
                print("Or setup MAVProxy output:")
                print("1. In MAVProxy: output add 127.0.0.1:14552")
                print("2. Run C++ reader: ./read_quaternion 14552 30")
                return True
            else:
                print("✗ No quaternion messages received after enabling")
                return False
                
        else:
            print(f"✗ Request failed with result: {ack.result}")
            return False
    else:
        print("✗ No ACK received")
        return False

def main():
    rate_hz = 50
    
    if len(sys.argv) > 1:
        rate_hz = int(sys.argv[1])
    
    print("ArduPilot ATTITUDE_QUATERNION Enabler")
    print("====================================")
    print(f"Target rate: {rate_hz} Hz")
    print("Note: This enables quaternion messages system-wide")
    print()
    
    try:
        if enable_quaternion_messages(rate_hz=rate_hz):
            print("\n✓ Setup complete!")
        else:
            print("\n✗ Setup failed")
            print("Troubleshooting:")
            print("- Make sure sim_vehicle.py is running")
            print("- Check for MAVProxy conflicts")
            print("- Try stopping other ground station software")
            
    except Exception as e:
        print(f"Error: {e}")
        print("\nIf this fails due to connection issues, you can manually enable")
        print("quaternion messages in MAVProxy with:")
        print("  long SET_MESSAGE_INTERVAL 0 31 20000 0 0 0 0 0")
        print("  (sets ATTITUDE_QUATERNION to 50Hz)")

if __name__ == "__main__":
    print("Usage: python3 one_shot_setup.py [rate_hz]")
    print("Example: python3 one_shot_setup.py 50")
    print()
    main()