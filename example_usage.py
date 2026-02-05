#!/usr/bin/env python3
"""
Example usage of the STM32 Configuration Tool
"""

from config_tool import STM32ConfigTool, UartWordLength, UartStopBits, UartParity

# Your STM32 device's current IP address
TARGET_IP = "192.168.144.33"
PROG_PORT = 33

# Create configuration tool instance
config = STM32ConfigTool(TARGET_IP, PROG_PORT)

# # Example 1: Configure IP settings
# print("=" * 60)
# print("Example 1: Configure IP Settings")
# print("=" * 60)
# config.set_ip_config(
#     local_ip="192.168.144.210",      # New local IP
#     netmask="255.255.255.0",          # Subnet mask
#     gateway="192.168.144.1",          # Gateway
#     dest_ip="192.168.144.100",        # Destination IP for UART data
#     local_port=7,                     # RX_PORT
#     target_port=7                     # TX_PORT
# )

# # Example 2: Configure UART settings for standard SBUS
# print("\n" + "=" * 60)
# print("Example 2: Configure UART for SBUS (100000 8E2)")
# print("=" * 60)
# config.set_uart_config(
#     baud_rate=100000,                            # SBUS baud rate
#     word_length=UartWordLength.WORDLENGTH_8B,    # 8 data bits
#     stop_bits=UartStopBits.STOPBITS_2,           # 2 stop bits
#     parity=UartParity.PARITY_EVEN                # Even parity
# )

# Example 3: Configure UART for different settings
print("\n" + "=" * 60)
print("Example 3: Configure UART for 115200 8N1")
print("=" * 60)
config.set_uart_config(
    baud_rate=115200,
    word_length=UartWordLength.WORDLENGTH_8B,
    stop_bits=UartStopBits.STOPBITS_1,
    parity=UartParity.PARITY_NONE
)

# # Example 4: Save configuration (requires firmware implementation)
# print("\n" + "=" * 60)
# print("Example 4: Save Configuration to Flash")
# print("=" * 60)
# config.save_config()

# # Example 5: Reset device (requires firmware implementation)
# print("\n" + "=" * 60)
# print("Example 5: Reset Device")
# print("=" * 60)
# config.reset_device()

print("\n✓ All examples completed!")
