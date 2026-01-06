/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_INVERSION_NONE                0x00000000U
#define UART_INVERSION_RX                  0x00000100U
#define UART_INVERSION_TX                  0x00000200U
#define UART_INVERSION_RXTX                0x00000300U

/* Command IDs */
#define CMD_HEADER 0xFD
#define CMD_SET_IP_CONFIG 0x01
#define CMD_SET_UART_CONFIG 0x02
#define CMD_SAVE_CONFIG 0x03

/* Command IDs */
#define STATUS_OK 0x00
#define STATUS_INVALID_LENGTH 0x01
#define STATUS_UNKNOWN_COMMAND 0x02
#define STATUS_INVALID_CHECKSUM 0x03
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void prog_udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static void send_response(struct udp_pcb *upcb, const ip_addr_t *addr, u16_t port, uint8_t cmd_id, uint8_t status);
static void load_config_from_flash(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct udp_pcb *upcb;
struct udp_pcb *prog_upcb;

// USART to Ethernet buffer (circular buffer)
#define USART_TO_ETH_BUFF_SIZE 512
#define PROG_BUFF_SIZE 50
#define UDP_BATCH_SIZE 64  // Send UDP packet when this many bytes accumulated
#define UDP_TIMEOUT_MS 2   // Send after 2ms even if batch not full (adjust for your latency requirements)

// Configuration variables (loaded from flash on startup)
uint32_t UartBaudRate;
uint32_t UartWordLength;
uint32_t UartStopBits;
uint32_t UartParity;
uint32_t UartInversion;
uint8_t UartTXInversion;
uint8_t UartRXInversion;

uint8_t USART_to_Eth_BUFF[USART_TO_ETH_BUFF_SIZE];
uint8_t PROG_BUFF[PROG_BUFF_SIZE];

uint8_t USART_RX;
volatile uint16_t USART_to_Eth_BUFF_WRITE_index = 0;
volatile uint16_t USART_to_Eth_BUFF_READ_index = 0;
volatile uint32_t last_rx_time = 0;  // Timestamp of last received byte

volatile uint16_t PROG_BUFF_WRITE_index = 0;
volatile uint16_t PROG_BUFF_STATE = 0;

// Get number of bytes in buffer
uint16_t get_buffer_count(void) {
  uint16_t write_idx = USART_to_Eth_BUFF_WRITE_index;
  uint16_t read_idx = USART_to_Eth_BUFF_READ_index;
  
  if (write_idx >= read_idx) {
    return write_idx - read_idx;
  } else {
    return USART_TO_ETH_BUFF_SIZE - read_idx + write_idx;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    USART_to_Eth_BUFF[USART_to_Eth_BUFF_WRITE_index++] = USART_RX;
    if(USART_to_Eth_BUFF_WRITE_index >= USART_TO_ETH_BUFF_SIZE)
    {
      USART_to_Eth_BUFF_WRITE_index = 0;
    }
    
    // Update timestamp when new data arrives
    last_rx_time = HAL_GetTick();
    
    // Restart the interrupt-based reception for the next byte
    HAL_UART_Receive_IT(&huart1, &USART_RX, 1);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Load configuration from flash memory
  // load_config_from_flash();
  
  // if(UartBaudRate == 0xFFFFFFFF){
    //Uninitialized flash, set defaults
    UartBaudRate = 115200;
    UartWordLength = UART_WORDLENGTH_9B;
    UartStopBits = UART_STOPBITS_2;
    UartParity = UART_PARITY_EVEN;
    UartInversion = UART_INVERSION_RXTX;
    UartTXInversion = 1;
    UartRXInversion = 1;
    IP_ADDRESS[0] = 192;
    IP_ADDRESS[1] = 168;
    IP_ADDRESS[2] = 144;
    IP_ADDRESS[3] = 33;
    NETMASK_ADDRESS[0] = 255;
    NETMASK_ADDRESS[1] = 255;
    NETMASK_ADDRESS[2] = 255;
    NETMASK_ADDRESS[3] = 0;
    GATEWAY_ADDRESS[0] = 192;
    GATEWAY_ADDRESS[1] = 168;
    GATEWAY_ADDRESS[2] = 144;
    GATEWAY_ADDRESS[3] = 1;
    DEST_IP_ADDRESS[0] = 192;
    DEST_IP_ADDRESS[1] = 168;
    DEST_IP_ADDRESS[2] = 144;
    DEST_IP_ADDRESS[3] = 133;
    RX_PORT = 5007;
    TX_PORT = 5007;

    //Update flash with defaults
    // handle_save_config_command();
  // }


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
//  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &USART_RX, 1);
  UDP_SERVER_INIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    ethernetif_input(&gnetif);
    sys_check_timeouts();
    /* Poll PHY link state so netif link callbacks fire when cable is plugged/unplugged */
    ethernetif_set_link(&gnetif);

    // Send USART data to Ethernet in batches for efficiency
    uint16_t available_bytes = get_buffer_count();
    
    if (available_bytes > 0) {
      uint32_t current_time = HAL_GetTick();
      uint32_t time_since_rx = current_time - last_rx_time;
      
      // Send if: batch is full, buffer getting full, OR timeout expired
      if (available_bytes >= UDP_BATCH_SIZE || 
          available_bytes > (USART_TO_ETH_BUFF_SIZE / 2) ||
          time_since_rx >= UDP_TIMEOUT_MS) {
        
        // Determine how many bytes to send (up to UDP_BATCH_SIZE)
        uint16_t bytes_to_send = (available_bytes > UDP_BATCH_SIZE) ? UDP_BATCH_SIZE : available_bytes;
        
        // Allocate pbuf for the batch
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, bytes_to_send, PBUF_RAM);
        if (p != NULL) {
          uint8_t *payload = (uint8_t *)p->payload;
          
          // Copy bytes from circular buffer to pbuf
          for (uint16_t i = 0; i < bytes_to_send; i++) {
            payload[i] = USART_to_Eth_BUFF[USART_to_Eth_BUFF_READ_index++];
            if (USART_to_Eth_BUFF_READ_index >= USART_TO_ETH_BUFF_SIZE) {
              USART_to_Eth_BUFF_READ_index = 0;
            }
          }
          
          // Send UDP packet (upcb is already connected in UDP_SERVER_INIT)
          udp_sendto(upcb, p, &dest_ipaddr, TX_PORT);
          pbuf_free(p);
        }
      }
    }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = UartBaudRate;
  huart1.Init.WordLength = UartWordLength;
  huart1.Init.StopBits = UartStopBits;
  huart1.Init.Parity = UartParity;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void load_config_from_flash(void)
{
  // Read UART configuration from flash
  UartBaudRate = *(volatile uint32_t *)(UARTBAUDRATE_ADDRESS);
  UartWordLength = *(volatile uint32_t *)(UARTWORDLENGTH_ADDRESS);
  UartStopBits = *(volatile uint32_t *)(UARTSTOPBITS_ADDRESS);
  UartParity = *(volatile uint32_t *)(UARTPARITY_ADDRESS);
  UartInversion = *(volatile uint32_t *)(UARTINVERSION_ADDRESS);
  
  // Read IP addresses from flash (stored as 32-bit words)
  uint32_t ip_word = *(volatile uint32_t *)(IP_ADDRESS_ADDRESS);
  IP_ADDRESS[0] = ip_word & 0xFF;
  IP_ADDRESS[1] = (ip_word >> 8) & 0xFF;
  IP_ADDRESS[2] = (ip_word >> 16) & 0xFF;
  IP_ADDRESS[3] = (ip_word >> 24) & 0xFF;
  
  uint32_t netmask_word = *(volatile uint32_t *)(NETMASK_ADDRESS_ADDRESS);
  NETMASK_ADDRESS[0] = netmask_word & 0xFF;
  NETMASK_ADDRESS[1] = (netmask_word >> 8) & 0xFF;
  NETMASK_ADDRESS[2] = (netmask_word >> 16) & 0xFF;
  NETMASK_ADDRESS[3] = (netmask_word >> 24) & 0xFF;
  
  uint32_t gateway_word = *(volatile uint32_t *)(GATEWAY_ADDRESS_ADDRESS);
  GATEWAY_ADDRESS[0] = gateway_word & 0xFF;
  GATEWAY_ADDRESS[1] = (gateway_word >> 8) & 0xFF;
  GATEWAY_ADDRESS[2] = (gateway_word >> 16) & 0xFF;
  GATEWAY_ADDRESS[3] = (gateway_word >> 24) & 0xFF;
  
  uint32_t dest_ip_word = *(volatile uint32_t *)(DEST_IP_ADDRESS_ADDRESS);
  DEST_IP_ADDRESS[0] = dest_ip_word & 0xFF;
  DEST_IP_ADDRESS[1] = (dest_ip_word >> 8) & 0xFF;
  DEST_IP_ADDRESS[2] = (dest_ip_word >> 16) & 0xFF;
  DEST_IP_ADDRESS[3] = (dest_ip_word >> 24) & 0xFF;
  
  // Read ports from flash (both stored in one 32-bit word)
  uint32_t ports_word = *(volatile uint32_t *)(RX_PORT_ADDRESS);
  RX_PORT = ports_word & 0xFFFF;
  TX_PORT = (ports_word >> 16) & 0xFFFF;
}

void UDP_SERVER_INIT(void){
  err_t err;

  /* Create a new UDP control block  */
  upcb = udp_new();

  // Bind to IP_ADDR_ANY to accept packets on any interface
  err = udp_bind(upcb, IP_ADDR_ANY, RX_PORT); // Bind to port
  if (err == ERR_OK) {
    // Set a receive callback function
    udp_recv(upcb, udp_receive_callback, NULL);
  }
  else{
    udp_remove(upcb);
  }

  prog_upcb = udp_new();
  // Bind to IP_ADDR_ANY to accept packets on any interface
  err = udp_bind(prog_upcb, IP_ADDR_ANY, PROG_PORT); // Bind to port
  if (err == ERR_OK) {
    // Set a receive callback function
    udp_recv(prog_upcb, prog_udp_receive_callback, NULL);
  }
  else{
    udp_remove(prog_upcb);
  }
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port)
{
  if (p != NULL) {
    // Send the received data to USART1 using DMA (non-blocking)
    // This prevents blocking the LwIP stack processing
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)p->payload, p->len);

    // Free the pbuf after processing
    pbuf_free(p);
  }
}

static uint8_t calculate_checksum(uint8_t *data, uint16_t len)
{
  uint8_t checksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

static void handle_ip_config_command(uint8_t *payload, uint16_t payload_len)
{
  if (payload_len != 20) {
    return;  // Invalid payload length
  }
  
  // Extract IP configuration
  // Offset 0-3: Local IP
  memcpy(IP_ADDRESS, payload, 4);
  
  // Offset 4-7: Netmask
  memcpy(NETMASK_ADDRESS, payload + 4, 4);
  
  // Offset 8-11: Gateway
  memcpy(GATEWAY_ADDRESS, payload + 8, 4);
  
  // Offset 12-15: Destination IP
  memcpy(DEST_IP_ADDRESS, payload + 12, 4);
  
  // Offset 16-17: Local Port (RX_PORT) - little-endian
  RX_PORT = (uint16_t)(payload[16] | (payload[17] << 8));
  
  // Offset 18-19: Target Port (TX_PORT) - little-endian
  TX_PORT = (uint16_t)(payload[18] | (payload[19] << 8));
  
  //Reinitilize UDP PCB with new IP and ports
  udp_remove(upcb);
  udp_remove(prog_upcb);
  
  // Deinitialize network interface properly
  netif_set_down(&gnetif);
  netif_remove(&gnetif);
  
  MX_LWIP_Init();
  UDP_SERVER_INIT();
}

static void handle_uart_config_command(uint8_t *payload, uint16_t payload_len)
{
  if (payload_len != 20) {
    return;  // Invalid payload length
  }
  
  // Extract UART configuration (all little-endian uint32)
  // Offset 0-3: Baud Rate
  UartBaudRate = (uint32_t)(payload[0] | (payload[1] << 8) | 
                            (payload[2] << 16) | (payload[3] << 24));
  
  // Offset 4-7: Word Length
  UartWordLength = (uint32_t)(payload[4] | (payload[5] << 8) | 
                              (payload[6] << 16) | (payload[7] << 24));
  
  // Offset 8-11: Stop Bits
  UartStopBits = (uint32_t)(payload[8] | (payload[9] << 8) | 
                            (payload[10] << 16) | (payload[11] << 24));
  
  // Offset 12-15: Parity
  UartParity = (uint32_t)(payload[12] | (payload[13] << 8) | 
                          (payload[14] << 16) | (payload[15] << 24));

    // Offset 16-19: Inversion
  UartInversion = (uint32_t)(payload[16] | (payload[17] << 8) | 
                             (payload[18] << 16) | (payload[19] << 24));
  
  // Reconfigure UART with new settings
  HAL_UART_DeInit(&huart1);
  
  // Update UART handle with new parameters
  huart1.Init.BaudRate = UartBaudRate;
  huart1.Init.WordLength = UartWordLength;
  huart1.Init.StopBits = UartStopBits;
  huart1.Init.Parity = UartParity;
  //Inversion handling
  if(UartInversion == UART_INVERSION_NONE){
      UartTXInversion = 0;
      UartRXInversion = 0;
  }
  else if(UartInversion == UART_INVERSION_RX){
      UartTXInversion = 0;
      UartRXInversion = 1;
  }
  else if(UartInversion == UART_INVERSION_TX){
      UartTXInversion = 1;
      UartRXInversion = 0;
  }
  else if(UartInversion == UART_INVERSION_RXTX){
      UartTXInversion = 1;
      UartRXInversion = 1;
  }
  
  // Reinitialize UART
  if (HAL_UART_Init(&huart1) == HAL_OK) {
    // Restart UART receive interrupt
    extern uint8_t USART_RX;
    HAL_UART_Receive_IT(&huart1, &USART_RX, 1);
  }
  
  // TODO: Save to flash if persistence is needed
}

static void handle_save_config_command(void)
{
  // Save all current configurations to flash memory
  HAL_StatusTypeDef status;
  
  // Unlock the Flash to enable the flash control register access
  HAL_FLASH_Unlock();
  
  // Erase the flash page where config is stored
  // STM32F107 has 2KB pages, calculate page number
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;
  
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = SAVED_CONFIG_ADDRESS;
  EraseInitStruct.NbPages = 1;  // Erase 1 page (2KB)
  
  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  
  if (status == HAL_OK) {
    // Write UART configuration (5 uint32_t values)
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UARTBAUDRATE_ADDRESS, UartBaudRate);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UARTWORDLENGTH_ADDRESS, UartWordLength);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UARTSTOPBITS_ADDRESS, UartStopBits);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UARTPARITY_ADDRESS, UartParity);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, UARTINVERSION_ADDRESS, UartInversion);
    
    // Write IP addresses (4 bytes each, combine into 32-bit words)
    uint32_t ip_word = (IP_ADDRESS[3] << 24) | (IP_ADDRESS[2] << 16) | 
                       (IP_ADDRESS[1] << 8) | IP_ADDRESS[0];
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IP_ADDRESS_ADDRESS, ip_word);
    
    uint32_t netmask_word = (NETMASK_ADDRESS[3] << 24) | (NETMASK_ADDRESS[2] << 16) | 
                            (NETMASK_ADDRESS[1] << 8) | NETMASK_ADDRESS[0];
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, NETMASK_ADDRESS_ADDRESS, netmask_word);
    
    uint32_t gateway_word = (GATEWAY_ADDRESS[3] << 24) | (GATEWAY_ADDRESS[2] << 16) | 
                            (GATEWAY_ADDRESS[1] << 8) | GATEWAY_ADDRESS[0];
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, GATEWAY_ADDRESS_ADDRESS, gateway_word);
    
    uint32_t dest_ip_word = (DEST_IP_ADDRESS[3] << 24) | (DEST_IP_ADDRESS[2] << 16) | 
                            (DEST_IP_ADDRESS[1] << 8) | DEST_IP_ADDRESS[0];
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, DEST_IP_ADDRESS_ADDRESS, dest_ip_word);
    
    // Write ports (combine both 16-bit ports into one 32-bit word)
    uint32_t ports_word = (TX_PORT << 16) | RX_PORT;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, RX_PORT_ADDRESS, ports_word);
  }
  
  // Lock the Flash to disable the flash control register access
  HAL_FLASH_Lock();
}

static void send_response(struct udp_pcb *upcb, const ip_addr_t *addr, u16_t port,
                         uint8_t cmd_id, uint8_t status)
{
  uint8_t response[5];
  response[0] = CMD_HEADER;
  response[1] = cmd_id | 0x80;  // Set MSB to indicate response
  response[2] = status;
  response[3] = calculate_checksum(response, 3);
  
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 4, PBUF_RAM);
  if (p != NULL) {
    memcpy(p->payload, response, 4);
    udp_sendto(upcb, p, addr, port);
    pbuf_free(p);
  }
}

void prog_udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port)
{
  if (p != NULL && p->len >= 3) {  // Minimum: Header + CMD + Checksum
    uint8_t *data = (uint8_t *)p->payload;
    uint16_t len = p->len;
    
    // Verify header
    if (data[0] == CMD_HEADER) {
      // Calculate and verify checksum
      uint8_t calculated_checksum = calculate_checksum(data, len - 1);
      uint8_t received_checksum = data[len - 1];
      
      if (calculated_checksum == received_checksum) {
        uint8_t cmd_id = data[1];
        uint8_t *payload = &data[2];
        uint16_t payload_len = len - 3;  // Exclude header, cmd_id, checksum
        
        // Process command
        switch (cmd_id) {
          case CMD_SET_IP_CONFIG:
            if (payload_len == 20) {
              handle_ip_config_command(payload, payload_len);
              send_response(upcb, addr, port, cmd_id, STATUS_OK);
            } else {
              send_response(upcb, addr, port, cmd_id, STATUS_INVALID_LENGTH);
            }
            break;
            
          case CMD_SET_UART_CONFIG:
            if (payload_len == 20) {
              handle_uart_config_command(payload, payload_len);
              send_response(upcb, addr, port, cmd_id, STATUS_OK);
            } else {
              send_response(upcb, addr, port, cmd_id, STATUS_INVALID_LENGTH);
            }
            break;
            
          case CMD_SAVE_CONFIG:
            handle_save_config_command();
            send_response(upcb, addr, port, cmd_id, STATUS_OK);
            break;
            
          default:
            send_response(upcb, addr, port, cmd_id, STATUS_UNKNOWN_COMMAND);
            break;
        }
      } else {
        // Checksum mismatch
        send_response(upcb, addr, port, data[1], STATUS_INVALID_CHECKSUM);
      }
    }
    
    // Free the packet buffer
    pbuf_free(p);
  } else if (p != NULL) {
    // Packet too short
    pbuf_free(p);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
