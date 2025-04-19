#include "main.h"
#include <string.h>

CAN_HandleTypeDef hcan2;

// E2E Configuration
#define MAX_COUNTER 0xFF
uint8_t e2e_counter = 0;
uint8_t transmission_count = 0;

// Keypad Buffer
char input_buffer[5]; // 4 chars + null terminator
uint8_t input_index = 0;

uint8_t calculate_checksum(uint8_t* data, uint8_t len);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
void CAN_TX(uint8_t* data, uint8_t len);
char Keypad_Scan(void);

// Keypad Scanning Function
char Keypad_Scan(void) {
  const uint8_t row_pins[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
  const uint8_t col_pins[4] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
  const char keys[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
  };

  for (int row = 0; row < 4; row++) {
    // Activate current row
    for (int r = 0; r < 4; r++) {
      HAL_GPIO_WritePin(GPIOB, row_pins[r], (r == row) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    // Check columns
    for (int col = 0; col < 4; col++) {
      if (HAL_GPIO_ReadPin(GPIOB, col_pins[col]) == GPIO_PIN_RESET) {
        HAL_Delay(50); // Debounce
        while (HAL_GPIO_ReadPin(GPIOB, col_pins[col]) == GPIO_PIN_RESET); // Wait release
        return keys[row][col];
      }
    }
  }
  return 0;
}

// System Clock Configuration (from CubeMX)
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure main internal regulator output voltage
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  // Initialize HSI oscillator
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Configure CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

// GPIO Initialization
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure PA5 LED
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Keypad Rows (PB0-PB3 as outputs)
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Keypad Columns (PB4-PB7 as inputs)
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// CAN Initialization
static void MX_CAN2_Init(void) {
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  HAL_CAN_Init(&hcan2);
}

void CAN_TX(uint8_t* data, uint8_t len) {
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x666,
    .RTR = CAN_RTR_DATA,
    .IDE = CAN_ID_STD,
    .DLC = len,
    .TransmitGlobalTime = DISABLE
  };

  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &mailbox);
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN2_Init();
  HAL_CAN_Start(&hcan2);

  while (1) {
    char key = Keypad_Scan();
    if (key != 0) {
      if (key == '#') {
        if (input_index > 0) {
          uint8_t payload[8] = {0};
          uint8_t data_len = input_index;

          // E2E Counter Logic
          if (transmission_count < 2) {
            e2e_counter = (e2e_counter + 1) % MAX_COUNTER;
            transmission_count++;
          }

          payload[0] = e2e_counter;
          memcpy(&payload[1], input_buffer, data_len);
          payload[data_len + 1] = calculate_checksum(payload, data_len + 1);

          CAN_TX(payload, data_len + 2);

          input_index = 0;
          memset(input_buffer, 0, sizeof(input_buffer));
        }
      } else if (input_index < 4) {
        input_buffer[input_index++] = key;
      }
    }
    HAL_Delay(100);
  }
}

// Checksum Calculation (4-bit blocks + 1's complement)
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

  // Add carry
  while (sum >> 4) {
    sum = (sum & 0x0F) + (sum >> 4);
  }

  return (~sum) & 0x0F;
}

// Rest of auto-generated code (MX_GPIO_Init, MX_CAN2_Init, etc.) remains unchanged
