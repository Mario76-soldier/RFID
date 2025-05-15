#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "usb_device.h"
#include "usbd_hid.h"
#include "rc522.h"
#include <string.h>
#include <stdio.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycodes[6];
} HID_KeyboardReport_TypeDef;

typedef struct
{
	uint8_t modifier;
	uint8_t keycode;
} HID_Key;

HID_Key key = {0,0};

void SystemClock_Config(void);
HID_Key ascii_to_hid(char c);
void send_key_char(char c);
void send_uid_as_keyboard_input(char* uid, uint8_t len);

uint8_t status;
uint8_t str[MAX_LEN];  // UID 저장
uint8_t uidSize;

int main(void)
{
    HAL_Init();
    SystemClock_Config();  // 시스템 클럭 설정 (CubeMX에서 자동 생성)
    MX_GPIO_Init();        // GPIO 초기화
    MX_SPI1_Init();        // SPI1 초기화 (RC522와 연결된 SPI)
    MX_USB_DEVICE_Init();
    MFRC522_Init();          // RC522 초기화
    uint8_t uid[5];              // 현재 UID
    char uid_buf[9];            // UID 문자열 저장용

    while (1)
    {
    	status = MFRC522_Request(PICC_REQIDL, str);
    	if (status != MI_OK) {
    		HAL_Delay(50); // 카드가 없으면 딜레이 후 계속
    		continue;
    	}
    	status = MFRC522_Anticoll(str);
    	if (status == MI_OK) {
    		memcpy(uid, str, 4); // UID 복사
    		// UID 문자열로 변환
    		snprintf(uid_buf, sizeof(uid_buf), "%02X%02X%02X%02X",  uid[0], uid[1], uid[2], uid[3]);
    		// USB CDC 전송
    		send_uid_as_keyboard_input(uid_buf, sizeof(uid_buf));
        }
        HAL_Delay(1000); // 폴링 주기
    }
}

HID_Key ascii_to_hid(char c) {
    HID_Key key = {0, 0};

    if (c >= 'A' && c <= 'Z') {
        key.modifier = 0x02; // Left Shift
        key.keycode = 0x04 + (c - 'A');
    } else if (c >= '0' && c <= '9') {
        if (c == '0') key.keycode = 0x27;
        else key.keycode = 0x1E + (c - '1');
    } else if (c == '\n') {
        key.keycode = 0x28; // Enter
    }
    // 필요 시 특수문자 추가 가능

    return key;
}

void send_key_char(char c) {
    HID_KeyboardReport_TypeDef report;
    memset(&report, 0, sizeof(report));

    HID_Key key = ascii_to_hid(c);
    if (key.keycode == 0) return; // 변환 실패

    report.modifier = key.modifier;
    report.keycodes[0] = key.keycode;

    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
    HAL_Delay(30);

    // Release
    memset(&report, 0, sizeof(report));
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
    HAL_Delay(10);
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
    HAL_Delay(10);
}

void send_uid_as_keyboard_input(char* uid, uint8_t len) {
    for (int i = 0; i< len; i++) {
        send_key_char(uid[i]);
    }
    send_key_char('\n'); // 줄 바꿈
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
