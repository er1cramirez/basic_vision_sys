#!/usr/bin/env python3
import socket
import struct
import json

# Simple UDP listener to test telemetry data
def listen_telemetry(port=14559):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('localhost', port))
    print(f"Listening for telemetry on port {port}...")
    
    try:
        while True:
            data, addr = sock.recvfrom(8192)
            print(f"Received {len(data)} bytes from {addr}")
            
            if len(data) < 20:
                print("Data too short for header")
                continue
                
            # Parse header: [type(4)][timestamp(8)][seq(4)][size(4)][payload]
            message_type = struct.unpack('!I', data[0:4])[0]
            timestamp = struct.unpack('!Q', data[4:12])[0]
            sequence_num = struct.unpack('!I', data[12:16])[0]
            payload_size = struct.unpack('!I', data[16:20])[0]
            
            print(f"Message type: {message_type}")
            print(f"Timestamp: {timestamp}")
            print(f"Sequence: {sequence_num}")
            print(f"Payload size: {payload_size}")
            
            if len(data) < 20 + payload_size:
                print(f"Data too short for payload: got {len(data)}, need {20 + payload_size}")
                continue
                
            payload = data[20:20 + payload_size]
            
            try:
                payload_str = payload.decode('utf-8')
                payload_json = json.loads(payload_str)
                print(f"JSON Payload:")
                print(json.dumps(payload_json, indent=2))
                print("-" * 50)
            except Exception as e:
                print(f"Error parsing payload: {e}")
                print(f"Raw payload: {payload}")
                print("-" * 50)
                    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        sock.close()

if __name__ == "__main__":
    listen_telemetry()
