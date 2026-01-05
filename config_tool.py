#!/usr/bin/env python3
"""
STM32 SBUS Ethernet Configuration Tool

This script allows updating the STM32 firmware configuration over UDP:
- IP settings (local IP, netmask, gateway, destination IP, ports)
- UART settings (baud rate, word length, stop bits, parity)

Protocol:
- Header: 0xFD
- Command ID: 1 byte (0x01 for IP config, 0x02 for UART config)
- Payload: Variable length based on command
- Checksum: 1 byte (XOR of all bytes except checksum)
"""

import socket
import struct
import argparse
import sys
from enum import IntEnum


class CommandID(IntEnum):
    """Command IDs for configuration updates"""
    CMD_SET_IP_CONFIG = 0x01
    CMD_SET_UART_CONFIG = 0x02
    CMD_SAVE_CONFIG = 0x03  # Optional: save to flash
    CMD_RESET = 0x04  # Optional: reset device


class UartWordLength(IntEnum):
    """UART Word Length values matching STM32 HAL definitions"""
    WORDLENGTH_8B = 0x00000000
    WORDLENGTH_9B = 0x00001000


class UartStopBits(IntEnum):
    """UART Stop Bits values matching STM32 HAL definitions"""
    STOPBITS_1 = 0x00000000
    STOPBITS_2 = 0x00002000


class UartParity(IntEnum):
    """UART Parity values matching STM32 HAL definitions"""
    PARITY_NONE = 0x00000000
    PARITY_EVEN = 0x00000400
    PARITY_ODD = 0x00000600

class UartInvert(IntEnum):
    INVERT_NONE = 0x00000000
    INVERT_RX = 0x00000100
    INVERT_TX = 0x00000200
    INVERT_RXTX = 0x00000300

class STM32ConfigTool:
    """Configuration tool for STM32 SBUS Ethernet device"""
    
    HEADER = 0xFD
    DEFAULT_PROG_PORT = 33  # Programming port on STM32
    TIMEOUT = 2.0  # Socket timeout in seconds
    
    def __init__(self, target_ip, target_port=DEFAULT_PROG_PORT):
        """
        Initialize the configuration tool
        
        Args:
            target_ip: IP address of the STM32 device
            target_port: Programming port (default: 33)
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock = None
        
    def _calculate_checksum(self, data):
        """
        Calculate checksum as XOR of all bytes
        
        Args:
            data: bytes to calculate checksum for
            
        Returns:
            checksum byte
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def _build_packet(self, command_id, payload):
        """
        Build a command packet with header, command ID, payload, and checksum
        
        Args:
            command_id: Command ID byte
            payload: Payload bytes
            
        Returns:
            Complete packet as bytes
        """
        # Build packet without checksum
        packet = bytes([self.HEADER, command_id]) + payload
        
        # Calculate and append checksum
        checksum = self._calculate_checksum(packet)
        packet += bytes([checksum])
        
        return packet
    
    def _send_command(self, command_id, payload):
        """
        Send a command packet to the STM32 device
        
        Args:
            command_id: Command ID
            payload: Command payload
            
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(command_id, payload)
        
        try:
            # Create UDP socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(self.TIMEOUT)
            
            # Send packet
            self.sock.sendto(packet, (self.target_ip, self.target_port))
            print(f"✓ Sent {len(packet)} bytes to {self.target_ip}:{self.target_port}")
            print(f"  Packet: {packet.hex(' ')}")
            
            # Optional: Wait for acknowledgment (if firmware implements it)
            try:
                response, addr = self.sock.recvfrom(1024)
                print(f"✓ Received response: {response.hex(' ')}")
            except socket.timeout:
                print("  No response received (timeout)")
            
            return True
            
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            return False
            
        finally:
            if self.sock:
                self.sock.close()
                self.sock = None
    
    def set_ip_config(self, local_ip, netmask, gateway, dest_ip, 
                     local_port, target_port):
        """
        Configure IP settings
        
        Args:
            local_ip: Local IP address (e.g., "192.168.1.100")
            netmask: Netmask (e.g., "255.255.255.0")
            gateway: Gateway IP (e.g., "192.168.1.1")
            dest_ip: Destination IP for UART data (e.g., "192.168.1.50")
            local_port: Local UDP receive port
            target_port: Target UDP transmit port
            
        Returns:
            True if successful
        """
        print(f"\n📡 Setting IP Configuration:")
        print(f"   Local IP:     {local_ip}")
        print(f"   Netmask:      {netmask}")
        print(f"   Gateway:      {gateway}")
        print(f"   Dest IP:      {dest_ip}")
        print(f"   Local Port:   {local_port}")
        print(f"   Target Port:  {target_port}")
        
        # Parse IP addresses
        local_ip_bytes = [int(x) for x in local_ip.split('.')]
        netmask_bytes = [int(x) for x in netmask.split('.')]
        gateway_bytes = [int(x) for x in gateway.split('.')]
        dest_ip_bytes = [int(x) for x in dest_ip.split('.')]
        
        # Build payload:
        # 4 bytes: Local IP
        # 4 bytes: Netmask
        # 4 bytes: Gateway
        # 4 bytes: Destination IP
        # 2 bytes: Local Port (little-endian)
        # 2 bytes: Target Port (little-endian)
        payload = bytes(local_ip_bytes + netmask_bytes + gateway_bytes + 
                       dest_ip_bytes)
        payload += struct.pack('<HH', local_port, target_port)
        
        return self._send_command(CommandID.CMD_SET_IP_CONFIG, payload)
    
    def set_uart_config(self, baud_rate, word_length=UartWordLength.WORDLENGTH_8B,
                       stop_bits=UartStopBits.STOPBITS_2, 
                       parity=UartParity.PARITY_EVEN,Inversion=UartInvert.INVERT_NONE):
        """
        Configure UART settings
        
        Args:
            baud_rate: Baud rate (e.g., 100000)
            word_length: Word length (8 or 9 bits)
            stop_bits: Stop bits (1 or 2)
            parity: Parity (NONE, EVEN, ODD)
            Inversion: Inversion (NONE, RX, TX, RXTX)
            
        Returns:
            True if successful
        """
        print(f"\n🔧 Setting UART Configuration:")
        print(f"   Baud Rate:    {baud_rate}")
        print(f"   Word Length:  {word_length.name}")
        print(f"   Stop Bits:    {stop_bits.name}")
        print(f"   Parity:       {parity.name}")
        print(f"   Inversion:    {Inversion.name}")
        
        # Build payload:
        # 4 bytes: Baud Rate (little-endian)
        # 4 bytes: Word Length (little-endian)
        # 4 bytes: Stop Bits (little-endian)
        # 4 bytes: Parity (little-endian)
        # 4 bytes: Inversion (little-endian)
        payload = struct.pack('<IIIII', baud_rate, word_length, 
                            stop_bits, parity, Inversion)
        
        return self._send_command(CommandID.CMD_SET_UART_CONFIG, payload)
    
    def save_config(self):
        """
        Send save configuration command (optional - requires firmware support)
        
        Returns:
            True if successful
        """
        print(f"\n💾 Saving configuration to flash...")
        return self._send_command(CommandID.CMD_SAVE_CONFIG, b'')
    
    def reset_device(self):
        """
        Send reset command (optional - requires firmware support)
        
        Returns:
            True if successful
        """
        print(f"\n🔄 Resetting device...")
        return self._send_command(CommandID.CMD_RESET, b'')


def main():
    """Main entry point for CLI usage"""
    parser = argparse.ArgumentParser(
        description='STM32 SBUS Ethernet Configuration Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Configure IP settings
  %(prog)s --target 192.168.144.201 set-ip \\
      --local-ip 192.168.1.100 \\
      --netmask 255.255.255.0 \\
      --gateway 192.168.1.1 \\
      --dest-ip 192.168.1.50 \\
      --local-port 7 \\
      --target-port 7

  # Configure UART settings
  %(prog)s --target 192.168.144.201 set-uart \\
      --baud-rate 100000 \\
      --word-length 8 \\
      --stop-bits 2 \\
      --parity EVEN
      --inversion RXTX
        ''')
    
    parser.add_argument('--target', required=True,
                       help='Target STM32 IP address')
    parser.add_argument('--prog-port', type=int, default=33,
                       help='Programming UDP port (default: 33)')
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # IP configuration subcommand
    ip_parser = subparsers.add_parser('set-ip', help='Configure IP settings')
    ip_parser.add_argument('--local-ip', required=True,
                          help='Local IP address')
    ip_parser.add_argument('--netmask', required=True,
                          help='Netmask')
    ip_parser.add_argument('--gateway', required=True,
                          help='Gateway IP address')
    ip_parser.add_argument('--dest-ip', required=True,
                          help='Destination IP for UART data')
    ip_parser.add_argument('--local-port', type=int, required=True,
                          help='Local UDP receive port')
    ip_parser.add_argument('--target-port', type=int, required=True,
                          help='Target UDP transmit port')
    
    # UART configuration subcommand
    uart_parser = subparsers.add_parser('set-uart', help='Configure UART settings')
    uart_parser.add_argument('--baud-rate', type=int, required=True,
                            help='Baud rate (e.g., 100000)')
    uart_parser.add_argument('--word-length', type=int, choices=[8, 9],
                            default=8, help='Word length in bits (default: 8)')
    uart_parser.add_argument('--stop-bits', type=int, choices=[1, 2],
                            default=2, help='Stop bits (default: 2)')
    uart_parser.add_argument('--parity', 
                            choices=['NONE', 'EVEN', 'ODD'],
                            default='EVEN', help='Parity (default: EVEN)')
    uart_parser.add_argument('--inversion', 
                            choices=['NONE', 'RX', 'TX', 'RXTX'],
                            default='NONE', help='Inversion (default: NONE)')
    
    # Optional commands
    subparsers.add_parser('save', help='Save configuration to flash')
    subparsers.add_parser('reset', help='Reset device')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    # Create configuration tool
    tool = STM32ConfigTool(args.target, args.prog_port)
    
    success = False
    
    # Execute command
    if args.command == 'set-ip':
        success = tool.set_ip_config(
            args.local_ip, args.netmask, args.gateway, args.dest_ip,
            args.local_port, args.target_port
        )
    
    elif args.command == 'set-uart':
        # Convert arguments to enum values
        word_length = (UartWordLength.WORDLENGTH_9B if args.word_length == 9 
                      else UartWordLength.WORDLENGTH_8B)
        stop_bits = (UartStopBits.STOPBITS_2 if args.stop_bits == 2 
                    else UartStopBits.STOPBITS_1)
        parity = UartParity[f'PARITY_{args.parity}']
        inversion = UartInvert[f'INVERT_{args.inversion}']
        
        success = tool.set_uart_config(
            args.baud_rate, word_length, stop_bits, parity, inversion
        )
    
    elif args.command == 'save':
        success = tool.save_config()
    
    elif args.command == 'reset':
        success = tool.reset_device()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
