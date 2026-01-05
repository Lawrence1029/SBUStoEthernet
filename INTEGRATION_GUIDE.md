# Firmware Integration Guide

This guide shows how to integrate the configuration command handler into your STM32 firmware.

## Files Added

1. `Core/Src/config_handler.c` - Command processing implementation
2. `Core/Inc/config_handler.h` - Header file
3. `config_tool.py` - Python configuration tool
4. `example_usage.py` - Python usage examples

## Integration Steps

### Step 1: Add Files to Project

1. The `.c` and `.h` files are already in your project structure
2. Make sure they're included in your build (should be automatic in STM32CubeIDE)

### Step 2: Update main.c

Replace the empty `prog_udp_receive_callback` function in `main.c`:

**Before:**
```c
void prog_udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port)
{
  if (p != NULL) {
    
    // Free the pbuf after processing
    pbuf_free(p);
  }
}
```

**After:**
```c
// Remove the empty implementation from main.c
// The function is now in config_handler.c
```

Add include at the top of `main.c`:
```c
/* USER CODE BEGIN Includes */
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "config_handler.h"  // Add this line

#include "stdio.h"
/* USER CODE END Includes */
```

### Step 3: Export Variables in main.c

Make sure these variables are accessible from config_handler.c by declaring them with `extern` in main.c or adding them to main.h:

The variables already declared in main.c:
```c
uint32_t UartBaudRate = 100000;
uint32_t UartWordLength = UART_WORDLENGTH_8B;
uint32_t UartStopBits = UART_STOPBITS_2;
uint32_t UartParity = UART_PARITY_EVEN;
```

### Step 4: Export Network Variables in lwip.c

Make sure these variables are accessible by adding to `lwip.h`:

```c
/* USER CODE BEGIN 1 */
extern uint8_t IP_ADDRESS[4];
extern uint8_t NETMASK_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];
extern uint8_t DEST_IP_ADDRESS[4];
extern uint16_t RX_PORT;
extern uint16_t TX_PORT;
extern uint16_t PROG_PORT;
extern ip4_addr_t dest_ipaddr;
/* USER CODE END 1 */
```

### Step 5: Update MX_USART1_UART_Init() Function

Modify the UART initialization in `main.c` to use the dynamic configuration variables:

**Find this function:**
```c
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;  // Change this
  huart1.Init.WordLength = UART_WORDLENGTH_8B;  // Change this
  huart1.Init.StopBits = UART_STOPBITS_2;  // Change this
  huart1.Init.Parity = UART_PARITY_EVEN;  // Change this
  // ... rest of config
}
```

**Change to:**
```c
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = UartBaudRate;       // Use variable
  huart1.Init.WordLength = UartWordLength;   // Use variable
  huart1.Init.StopBits = UartStopBits;       // Use variable
  huart1.Init.Parity = UartParity;           // Use variable
  // ... rest of config stays the same
}
```

### Step 6: Build and Flash

1. Build the project in STM32CubeIDE
2. Flash to your STM32 device
3. The device should now respond to configuration commands

## Testing

### Test 1: Verify Current Setup
```bash
# Ping the device
ping 192.168.144.201
```

### Test 2: Change UART Settings
```bash
./config_tool.py --target 192.168.144.201 set-uart \
    --baud-rate 115200 \
    --word-length 8 \
    --stop-bits 1 \
    --parity NONE
```

### Test 3: Change IP Settings
```bash
./config_tool.py --target 192.168.144.201 set-ip \
    --local-ip 192.168.144.210 \
    --netmask 255.255.255.0 \
    --gateway 192.168.144.1 \
    --dest-ip 192.168.144.100 \
    --local-port 7 \
    --target-port 7
```

### Test 4: Reset Device (if needed)
```bash
./config_tool.py --target 192.168.144.201 reset
```

## Optional: Add Flash Persistence

To make configuration changes persistent across power cycles:

1. Use STM32 Flash API to write configuration to flash
2. Implement `handle_save_config_command()` in `config_handler.c`
3. On startup, read configuration from flash before initializing peripherals

Example flash storage location (last page of flash):
```c
#define CONFIG_FLASH_ADDR 0x0801F800  // Adjust based on your MCU

typedef struct {
  uint32_t magic;  // 0xC0FFEE00 to validate
  uint8_t ip_address[4];
  uint8_t netmask[4];
  uint8_t gateway[4];
  uint8_t dest_ip[4];
  uint16_t rx_port;
  uint16_t tx_port;
  uint32_t uart_baudrate;
  uint32_t uart_wordlength;
  uint32_t uart_stopbits;
  uint32_t uart_parity;
  uint32_t crc;
} Config_t;
```

## Debugging

### Enable Response Packets
The firmware now sends response packets back to the sender. You can modify `config_tool.py` to receive and display these responses:

Uncomment these lines in the `_send_command` method:
```python
# Optional: Wait for acknowledgment (if firmware implements it)
try:
    response, addr = self.sock.recvfrom(1024)
    print(f"✓ Received response: {response.hex(' ')}")
except socket.timeout:
    print("  No response received (timeout)")
```

### Response Format
```
[0xFD][CMD_ID | 0x80][STATUS][CHECKSUM]

Status codes:
  0x00 - OK
  0x01 - Invalid checksum
  0x02 - Invalid length
  0x03 - Unknown command
```

### Monitor with Wireshark
You can capture UDP traffic on port 33 to see the exact commands being sent:
```bash
sudo tcpdump -i any -X port 33
```

## Troubleshooting

### Problem: Configuration not applied
- Check that `config_handler.c` is compiled and linked
- Verify that `prog_udp_receive_callback` is being called
- Use a debugger to set a breakpoint in the callback

### Problem: Network settings don't take effect
- Network changes require a reset to take full effect
- Or implement dynamic reconfiguration of the network interface

### Problem: UART settings don't work
- Verify the HAL configuration allows dynamic UART reconfiguration
- Check that clock settings support the requested baud rate
- Ensure DMA is properly reinitialized after UART reinit

### Problem: Device becomes unreachable after IP change
- Make sure you update your target IP when testing
- Consider adding a reset button or timeout to revert changes
- Implement flash persistence with a "safe mode" boot option

## Next Steps

1. Test the basic configuration commands
2. Implement flash persistence for configuration
3. Add validation for configuration values
4. Consider adding a "factory reset" command
5. Implement configuration readback commands
