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

def parse_sbus(data):
    """Parse SBUS packet and extract channel values"""
    if len(data) != 25:
        return None
    
    # Check start and end bytes
    if data[0] != 0x0F or data[24] != 0x00:
        return None
    
    # Extract 16 channels (11 bits each) from bytes 1-22
    channels = [0] * 16
    
    channels[0]  = ((data[1]    | data[2]<<8)                     & 0x07FF)
    channels[1]  = ((data[2]>>3 | data[3]<<5)                     & 0x07FF)
    channels[2]  = ((data[3]>>6 | data[4]<<2 | data[5]<<10)       & 0x07FF)
    channels[3]  = ((data[5]>>1 | data[6]<<7)                     & 0x07FF)
    channels[4]  = ((data[6]>>4 | data[7]<<4)                     & 0x07FF)
    channels[5]  = ((data[7]>>7 | data[8]<<1 | data[9]<<9)        & 0x07FF)
    channels[6]  = ((data[9]>>2 | data[10]<<6)                    & 0x07FF)
    channels[7]  = ((data[10]>>5| data[11]<<3)                    & 0x07FF)
    channels[8]  = ((data[12]   | data[13]<<8)                    & 0x07FF)
    channels[9]  = ((data[13]>>3| data[14]<<5)                    & 0x07FF)
    channels[10] = ((data[14]>>6| data[15]<<2 | data[16]<<10)     & 0x07FF)
    channels[11] = ((data[16]>>1| data[17]<<7)                    & 0x07FF)
    channels[12] = ((data[17]>>4| data[18]<<4)                    & 0x07FF)
    channels[13] = ((data[18]>>7| data[19]<<1 | data[20]<<9)      & 0x07FF)
    channels[14] = ((data[20]>>2| data[21]<<6)                    & 0x07FF)
    channels[15] = ((data[21]>>5| data[22]<<3)                    & 0x07FF)
    
    # Byte 23 contains flags
    flags = data[23]
    ch17 = bool(flags & 0x01)
    ch18 = bool(flags & 0x02)
    frame_lost = bool(flags & 0x04)
    failsafe = bool(flags & 0x08)
    
    return {
        'channels': channels,
        'ch17': ch17,
        'ch18': ch18,
        'frame_lost': frame_lost,
        'failsafe': failsafe
    }

def listen_for_data():
    """Listen for incoming data from the device"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"[RX] Listening on all interfaces (0.0.0.0):{LISTEN_PORT}")
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            
            # Try to parse as SBUS
            sbus_data = parse_sbus(data)
            if sbus_data:
                # Clear screen and move cursor to top
                print("\033[2J\033[H", end="")
                
                print("=" * 70)
                print(f"SBUS Data from {addr[0]}:{addr[1]} | {len(data)} bytes")
                print("=" * 70)
                
                # Display channels in a nice grid
                print("\n┌─────────────────────────────────────────────────────────────────┐")
                for i in range(0, 16, 4):
                    ch_str = "│ "
                    for j in range(4):
                        ch_num = i + j + 1
                        ch_val = sbus_data['channels'][i + j]
                        ch_str += f"Ch{ch_num:2d}: {ch_val:4d}  "
                    ch_str += "│"
                    print(ch_str)
                print("└─────────────────────────────────────────────────────────────────┘")
                
                # Status information
                print(f"\nDigital: Ch17={sbus_data['ch17']}  Ch18={sbus_data['ch18']}")
                print(f"Status:  Frame Lost={sbus_data['frame_lost']}  Failsafe={sbus_data['failsafe']}")
                print(f"\nRaw Hex: {data.hex()}")
                
            else:
                print(f"\n[RX] Received from {addr[0]}:{addr[1]}: {len(data)} bytes")
                print(f"     Not valid SBUS (expected 25 bytes with 0x0F start, 0x00 end)")
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
