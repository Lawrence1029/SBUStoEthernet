/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern struct udp_pcb *upcb;

extern volatile uint16_t USART_to_Eth_BUFF_WRITE_index;
extern volatile uint16_t USART_to_Eth_BUFF_READ_index;
extern volatile uint32_t last_rx_time;  // Timestamp of last received byte
extern uint8_t USART_RX;

extern uint32_t UartBaudRate;
extern uint32_t UartWordLength;
extern uint32_t UartStopBits;
extern uint32_t UartParity;
extern uint32_t UartInversion;
extern uint8_t UartTXInversion;
extern uint8_t UartRXInversion;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UDP_SERVER_INIT(void);
static uint8_t calculate_checksum(uint8_t *data, uint16_t len);
static void handle_ip_config_command(uint8_t *payload, uint16_t payload_len);
static void handle_uart_config_command(uint8_t *payload, uint16_t payload_len);
static void handle_save_config_command(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define SAVED_CONFIG_ADDRESS 0x0803F800
#define UARTBAUDRATE_ADDRESS        (SAVED_CONFIG_ADDRESS)
#define UARTWORDLENGTH_ADDRESS      (SAVED_CONFIG_ADDRESS + 4)
#define UARTSTOPBITS_ADDRESS       (SAVED_CONFIG_ADDRESS + 8)
#define UARTPARITY_ADDRESS         (SAVED_CONFIG_ADDRESS + 12)
#define UARTINVERSION_ADDRESS      (SAVED_CONFIG_ADDRESS + 16)
#define IP_ADDRESS_ADDRESS        (SAVED_CONFIG_ADDRESS + 20)
#define NETMASK_ADDRESS_ADDRESS    (SAVED_CONFIG_ADDRESS + 24)
#define GATEWAY_ADDRESS_ADDRESS    (SAVED_CONFIG_ADDRESS + 28)
#define DEST_IP_ADDRESS_ADDRESS    (SAVED_CONFIG_ADDRESS + 32)
#define RX_PORT_ADDRESS           (SAVED_CONFIG_ADDRESS + 36)
#define TX_PORT_ADDRESS           (SAVED_CONFIG_ADDRESS + 38)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
