#!/usr/bin/env python3
"""
UDP test script for SBUSEthernet device
- Sends keyboard input to device at 192.168.144.33:7
- Listens for responses from device at 192.168.144.133:7
"""

import socket
import threading
import sys

# Configuration
DEVICE_IP = "192.168.144.33"
DEVICE_PORT = 5007
LISTEN_IP = "0.0.0.0"  # Listen on all interfaces to avoid permission issues
LISTEN_PORT = 5007

def listen_for_data():
    """Listen for incoming data from the device"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"[RX] Listening on all interfaces (0.0.0.0):{LISTEN_PORT}")
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            print(f"[RX] Received from {addr[0]}:{addr[1]}: {data}")
            print(f"     Hex: {data.hex()}")
            print(f"     ASCII: {repr(data.decode('utf-8', errors='replace'))}")
    except KeyboardInterrupt:
        print("\n[RX] Stopped listening")
    finally:
        sock.close()

def send_keyboard_input():
    """Send keyboard input to the device"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"[TX] Connected to {DEVICE_IP}:{DEVICE_PORT}")
    print("[TX] Type messages to send (Ctrl+C to exit):\n")
    
    try:
        while True:
            user_input = input("[TX] >> ")
            if user_input:
                # Send as bytes
                data = user_input.encode('utf-8')
                sock.sendto(data, (DEVICE_IP, DEVICE_PORT))
                print(f"[TX] Sent {len(data)} bytes: {data.hex()}")
    except KeyboardInterrupt:
        print("\n[TX] Stopped sending")
    except BrokenPipeError:
        print("[TX] Connection closed")
    finally:
        sock.close()

def main():
    print("=" * 60)
    print("SBUSEthernet UDP Test Script")
    print("=" * 60)
    print(f"TX: Sending to {DEVICE_IP}:{DEVICE_PORT}")
    print(f"RX: Listening on 0.0.0.0:{LISTEN_PORT} (all interfaces)")
    print("=" * 60)
    print()
    
    # Start receiver thread
    rx_thread = threading.Thread(target=listen_for_data, daemon=True)
    rx_thread.start()
    
    # Start sender in main thread
    send_keyboard_input()

if __name__ == "__main__":
    main()
