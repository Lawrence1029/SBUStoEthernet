# STM32 SBUS Ethernet Configuration Tool

Python script for configuring STM32 SBUS Ethernet firmware over UDP.

## Features

- **IP Configuration**: Update local IP, netmask, gateway, destination IP, and UDP ports
- **UART Configuration**: Update baud rate, word length, stop bits, parity and inversion
- Simple command-line interface
- Checksummed protocol for data integrity

## Protocol Specification

### Packet Structure
```
┌────────┬────────────┬─────────┬──────────┐
│ Header │ Command ID │ Payload │ Checksum │
│ 0xFD   │   1 byte   │ N bytes │  1 byte  │
└────────┴────────────┴─────────┴──────────┘
```

- **Header**: Always 0xFD
- **Command ID**: 
  - `0x01`: Set IP Configuration
  - `0x02`: Set UART Configuration
  - `0x03`: Save Configuration (optional)
  - `0x04`: Reset Device (optional)
- **Payload**: Variable length, depends on command
- **Checksum**: XOR of all previous bytes (Header + Command ID + Payload)

### Command Payloads

#### IP Configuration (0x01)
```
Offset | Size | Description
-------|------|----------------------------------
0      | 4    | Local IP Address (4 bytes)
4      | 4    | Netmask (4 bytes)
8      | 4    | Gateway (4 bytes)
12     | 4    | Destination IP (4 bytes)
16     | 2    | Local Port (uint16, little-endian)
18     | 2    | Target Port (uint16, little-endian)
-------|------|----------------------------------
Total: 20 bytes
```

#### UART Configuration (0x02)
```
Offset | Size | Description
-------|------|----------------------------------
0      | 4    | Baud Rate (uint32, little-endian)
4      | 4    | Word Length (uint32, little-endian)
8      | 4    | Stop Bits (uint32, little-endian)
12     | 4    | Parity (uint32, little-endian)
16     | 4    | Inversion (uint32, little-endian)
-------|------|----------------------------------
Total: 16 bytes

Word Length values:
  - 0x00000000: 8 bits (UART_WORDLENGTH_8B)
  - 0x00001000: 9 bits (UART_WORDLENGTH_9B)

Stop Bits values:
  - 0x00000000: 1 stop bit (UART_STOPBITS_1)
  - 0x00002000: 2 stop bits (UART_STOPBITS_2)

Parity values:
  - 0x00000000: No parity (UART_PARITY_NONE)
  - 0x00000400: Even parity (UART_PARITY_EVEN)
  - 0x00000600: Odd parity (UART_PARITY_ODD)

Inversion values:
  - 0x00000000: No Inversion 
  - 0x00000100: Rx Inversion
  - 0x00000200: Tx Inversion
  - 0x00000300: RXTX Inversion
```

## Installation

No external dependencies required - uses only Python standard library.

```bash
chmod +x config_tool.py
```

## Usage

### Command Line Interface

#### Configure IP Settings
```bash
./config_tool.py --target 192.168.144.201 set-ip \
    --local-ip 192.168.1.100 \
    --netmask 255.255.255.0 \
    --gateway 192.168.1.1 \
    --dest-ip 192.168.1.50 \
    --local-port 7 \
    --target-port 7
```

#### Configure UART Settings (SBUS: 100000 8E2)
```bash
./config_tool.py --target 192.168.144.201 set-uart \
    --baud-rate 100000 \
    --word-length 8 \
    --stop-bits 2 \
    --parity EVEN\
    --inversion RXTX
```

#### Configure UART for 115200 8N1
```bash
./config_tool.py --target 192.168.144.201 set-uart \
    --baud-rate 115200 \
    --word-length 8 \
    --stop-bits 1 \
    --parity NONE
    --inversion NONE
```

#### Save Configuration (optional, requires firmware support)
```bash
./config_tool.py --target 192.168.144.201 save
```

#### Reset Device (optional, requires firmware support)
```bash
./config_tool.py --target 192.168.144.201 reset
```

### Python API

```python
from config_tool import STM32ConfigTool, UartWordLength, UartStopBits, UartParity

# Create tool instance
config = STM32ConfigTool("192.168.144.201", prog_port=33)

# Configure IP
config.set_ip_config(
    local_ip="192.168.1.100",
    netmask="255.255.255.0",
    gateway="192.168.1.1",
    dest_ip="192.168.1.50",
    local_port=7,
    target_port=7
)

# Configure UART
config.set_uart_config(
    baud_rate=100000,
    word_length=UartWordLength.WORDLENGTH_8B,
    stop_bits=UartStopBits.STOPBITS_2,
    parity=UartParity.PARITY_EVEN
)
```

## Example Output

```
📡 Setting IP Configuration:
   Local IP:     192.168.1.100
   Netmask:      255.255.255.0
   Gateway:      192.168.1.1
   Dest IP:      192.168.1.50
   Local Port:   7
   Target Port:  7
✓ Sent 23 bytes to 192.168.144.201:33
  Packet: fd 01 c0 a8 01 64 ff ff ff 00 c0 a8 01 01 c0 a8 01 32 07 00 07 00 8e

🔧 Setting UART Configuration:
   Baud Rate:    100000
   Word Length:  WORDLENGTH_8B
   Stop Bits:    STOPBITS_2
   Parity:       PARITY_EVEN
✓ Sent 19 bytes to 192.168.144.201:33
  Packet: fd 02 a0 86 01 00 00 00 00 00 00 20 00 00 00 04 00 00 87
```

## Firmware Implementation

To use this tool, you need to implement the command handling in your STM32 firmware. The commands are received on the programming UDP port (default: 33).

### Required Changes to `main.c`

1. **Add command processing in `prog_udp_receive_callback`**:

```c
void prog_udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port)
{
  if (p != NULL && p->len >= 3) {  // Minimum: Header + CMD + Checksum
    uint8_t *data = (uint8_t *)p->payload;
    
    // Verify header
    if (data[0] == 0xFD) {
      // Verify checksum
      uint8_t checksum = 0;
      for (uint16_t i = 0; i < p->len - 1; i++) {
        checksum ^= data[i];
      }
      
      if (checksum == data[p->len - 1]) {
        uint8_t cmd_id = data[1];
        uint8_t *payload = &data[2];
        uint16_t payload_len = p->len - 3;
        
        switch (cmd_id) {
          case 0x01:  // Set IP Config
            if (payload_len == 20) {
              // Update IP settings
              memcpy(IP_ADDRESS, payload, 4);
              memcpy(NETMASK_ADDRESS, payload + 4, 4);
              memcpy(GATEWAY_ADDRESS, payload + 8, 4);
              memcpy(DEST_IP_ADDRESS, payload + 12, 4);
              RX_PORT = *(uint16_t*)(payload + 16);
              TX_PORT = *(uint16_t*)(payload + 18);
              
              // TODO: Reinitialize network interface
              // TODO: Save to flash if needed
            }
            break;
            
          case 0x02:  // Set UART Config
            if (payload_len == 16) {
              UartBaudRate = *(uint32_t*)(payload);
              UartWordLength = *(uint32_t*)(payload + 4);
              UartStopBits = *(uint32_t*)(payload + 8);
              UartParity = *(uint32_t*)(payload + 12);
              
              // Reinitialize UART with new settings
              HAL_UART_DeInit(&huart1);
              MX_USART1_UART_Init();
              HAL_UART_Receive_IT(&huart1, &USART_RX, 1);
              
              // TODO: Save to flash if needed
            }
            break;
            
          case 0x03:  // Save Config
            // TODO: Save settings to flash
            break;
            
          case 0x04:  // Reset
            NVIC_SystemReset();
            break;
        }
      }
    }
    
    pbuf_free(p);
  }
}
```

2. **Make UART parameters accessible** (add to header if needed):

```c
extern uint32_t UartBaudRate;
extern uint32_t UartWordLength;
extern uint32_t UartStopBits;
extern uint32_t UartParity;
```

## Testing

1. Ensure your STM32 device is connected and has IP 192.168.144.201
2. Run the example script:
```bash
python3 example_usage.py
```

3. Or use the CLI tool for individual commands

## Troubleshooting

- **No response**: The current firmware doesn't send acknowledgments. This is normal unless you implement response packets.
- **Configuration not applied**: Make sure you've implemented the command handling in the firmware's `prog_udp_receive_callback` function.
- **Wrong port**: Verify the programming port is set correctly (default: 33)

## License

This tool is provided as-is for configuring STM32 SBUS Ethernet firmware.
