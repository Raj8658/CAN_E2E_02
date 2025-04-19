#include "main.h"
#include <string.h>

CAN_HandleTypeDef hcan2;

// E2E Configuration
uint8_t expected_counter = 0;
uint8_t valid_messages = 0;

// Function Declarations
uint8_t calculate_checksum(uint8_t* data, uint8_t len);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN2_Init();

  HAL_CAN_Start(&hcan2);

  CAN_FilterTypeDef filter = {
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = ENABLE
  };
  HAL_CAN_ConfigFilter(&hcan2, &filter);

  while (1) {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0) {
      CAN_RxHeaderTypeDef rx_header;
      uint8_t rx_data[8];

      if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        uint8_t data_len = rx_header.DLC - 2; // Exclude counter & checksum

        // Counter Validation
        if (rx_data[0] != expected_counter) {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red LED
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
          continue;
        }

        // Checksum Validation
        uint8_t received_checksum = rx_data[data_len + 1];
        uint8_t calculated_checksum = calculate_checksum(rx_data, data_len + 1);

        if (received_checksum == calculated_checksum) {
          valid_messages++;
          expected_counter = (rx_data[0] + 1) % 0xFF;

          // LED Logic
          if (valid_messages <= 2) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Green
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
          } else {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
          }
        } else {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        }
      }
    }
  }
}

// ------------------------
// Checksum Function
// ------------------------
uint8_t calculate_checksum(uint8_t* data, uint8_t len) {
  uint32_t sum = 0;
  const uint8_t BLOCK_SIZE = 4; // 4-bit blocks
  uint8_t num_blocks = (len * 8) / BLOCK_SIZE;

  for (uint8_t i = 0; i < num_blocks; i++) {
    uint8_t byte_pos = i / 2;
    uint8_t bit_shift = (i % 2) * 4;
    uint8_t block = (data[byte_pos] >> bit_shift) & 0x0F;
    sum += block;
  }

  return (uint8_t)(sum & 0xFF);  // Return final 8-bit checksum
}

// ------------------------
// System Clock Config
// ------------------------
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

// ------------------------
// GPIO Init
// ------------------------
static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

// ------------------------
// CAN2 Init
// ------------------------
static void MX_CAN2_Init(void) {
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_CAN2_CLK_ENABLE();

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK) {
    Error_Handler();
  }
}

// Optional: Add error handler
void Error_Handler(void) {
  while (1) {
    // Blink red LED or stay stuck here
  }
}
