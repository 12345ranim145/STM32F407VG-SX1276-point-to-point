/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MODE MASTER CORRIGÉ - STM32F407VG - Transmission LoRa
  ******************************************************************************
  * Version corrigée avec gestion précise des LEDs et connexion vérifiée
  * Configuration SX1276 :
  * PC3 (SPI2_MOSI)  →   MOSI
  * PC2 (SPI2_MISO)  ←   MISO
  * PB10 (SPI2_SCK)  →   SCK
  * PA4 (NSS)        →   NSS
  * PB0 (DIO0)       ←   DIO0
  * PB1 (RESET)      →   RESET
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "usart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1276.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEVICE_MODE_MASTER 1
#define DEVICE_ID "STM32F407_MASTER_001"

// Définition des LEDs pour STM32F407VG Discovery
#define LED_GREEN_GPIO_Port GPIOD    // Transmission réussie SEULEMENT
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_ORANGE_GPIO_Port GPIOD   // Connexion LoRa établie SEULEMENT
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOD      // Erreur transmission
#define LED_RED_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOD     // Mode Master (toujours ON)
#define LED_BLUE_Pin GPIO_PIN_15

// Paramètres de transmission
#define TX_TIMEOUT 5000
#define TX_RETRY_COUNT 3
#define TX_INTERVAL_MS 3000
#define MESSAGE_LENGTH 5
#define CONNECTION_CHECK_INTERVAL 1000

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
SX1276_hw_t SX1276_hw;
SX1276_t SX1276;
int transmission_count = 0;
char tx_buffer[32];
uint8_t connection_established = 0;
uint32_t last_connection_check = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_LEDs_Init(void);
void LED_Init(void);
void LED_SetMode_Master(void);
void LED_SetConnection_Established(void);
void LED_SetConnection_Lost(void);
void LED_SetTransmission_Success(void);
void LED_SetTransmission_Failed(void);
void LED_Reset_Status(void);
int LoRa_Master_Init(void);
int LoRa_Master_CheckConnection(void);
int LoRa_Master_Transmit_Verified(const char* message);
void LoRa_Reset_And_Reinit(void);

/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 100);
    return len;
}

/* Gestion précise des LEDs */
void LED_Init(void) {
    // Éteindre toutes les LEDs au démarrage
    HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);
}

void LED_SetMode_Master(void) {
    // LED bleue TOUJOURS allumée pour indiquer le mode Master
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}

void LED_SetConnection_Established(void) {
    // LED orange SEULEMENT si connexion LoRa vérifiée
    HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
    LED_SetMode_Master(); // Maintenir bleue
}

void LED_SetConnection_Lost(void) {
    // Éteindre LED orange si pas de connexion
    HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
    LED_SetMode_Master(); // Maintenir bleue
}

void LED_SetTransmission_Success(void) {
    // LED verte SEULEMENT si transmission confirmée
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    LED_SetMode_Master(); // Maintenir bleue
}

void LED_SetTransmission_Failed(void) {
    // LED rouge pour échec de transmission
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    LED_SetMode_Master(); // Maintenir bleue
}

void LED_Reset_Status(void) {
    // Reset des LEDs de statut, garder seulement mode et connexion
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    LED_SetMode_Master(); // Maintenir bleue

    // Maintenir LED orange si connexion établie
    if(connection_established) {
        LED_SetConnection_Established();
    }
}

/* Initialisation LoRa Master */
int LoRa_Master_Init(void) {
    printf("=== LoRa Master Initialization ===\r\n");

    LED_SetMode_Master(); // LED bleue seulement

    // Configuration du hardware
    SX1276_hw.dio0.port = DIO0_GPIO_Port;     // GPIOB
    SX1276_hw.dio0.pin = DIO0_Pin;            // GPIO_PIN_0
    SX1276_hw.nss.port = NSS_GPIO_Port;       // GPIOA
    SX1276_hw.nss.pin = NSS_Pin;              // GPIO_PIN_4
    SX1276_hw.reset.port = RESET_GPIO_Port;   // GPIOB
    SX1276_hw.reset.pin = RESET_Pin;          // GPIO_PIN_1
    SX1276_hw.spi = &hspi2;                   // SPI2

    SX1276.hw = &SX1276_hw;

    // Reset matériel
    printf("Hardware reset...\r\n");
    SX1276_hw_Reset(&SX1276_hw);
    HAL_Delay(100);

    // Initialisation du module
    printf("Configuring LoRa module...\r\n");
    SX1276_init(&SX1276, 868000000, SX1276_POWER_17DBM, SX1276_LORA_SF_7,
                SX1276_LORA_BW_125KHZ, SX1276_LORA_CR_4_5, SX1276_LORA_CRC_EN, MESSAGE_LENGTH);

    HAL_Delay(500);

    // Test d'accès aux registres
    uint8_t version = SX1276_SPIRead(&SX1276, REG_LR_VERSION);
    printf("SX1276 Version: 0x%02X\r\n", version);

    if(version == 0x12) {
        printf("✓ LoRa module detected and configured\r\n");
        return 1;
    } else {
        printf("✗ LoRa module not detected\r\n");
        return 0;
    }
}

/* Vérification de connexion LoRa */
int LoRa_Master_CheckConnection(void) {
    // Test de communication avec le module local
    uint8_t version = SX1276_SPIRead(&SX1276, REG_LR_VERSION);

    if(version != 0x12) {
        printf("⚠ Local module communication failed\r\n");
        connection_established = 0;
        LED_SetConnection_Lost();
        return 0;
    }

    // Test de transmission courte pour vérifier la liaison
    SX1276_standby(&SX1276);
    HAL_Delay(10);

    int tx_entry = SX1276_LoRaEntryTx(&SX1276, MESSAGE_LENGTH, TX_TIMEOUT);

    if(tx_entry == 1) {
        // Envoyer un ping test
        const char* ping_msg = "PING";
        int tx_result = SX1276_LoRaTxPacket(&SX1276, (uint8_t*)ping_msg, MESSAGE_LENGTH, 2000);

        SX1276_standby(&SX1276);

        if(tx_result == 1) {
            if(!connection_established) {
                printf("✓ LoRa connection established\r\n");
                connection_established = 1;
                LED_SetConnection_Established();
            }
            return 1;
        }
    }

    if(connection_established) {
        printf("⚠ LoRa connection lost\r\n");
        connection_established = 0;
        LED_SetConnection_Lost();
    }

    return 0;
}

/* Transmission vérifiée */
int LoRa_Master_Transmit_Verified(const char* message) {
    if(!connection_established) {
        printf("✗ No connection - cannot transmit\r\n");
        LED_SetTransmission_Failed();
        return 0;
    }

    int retry_count = 0;
    int result = 0;

    printf("=== Transmission Attempt ===\r\n");
    LED_Reset_Status(); // Reset des LEDs de statut

    while(retry_count < TX_RETRY_COUNT && result == 0) {
        printf("Attempt %d/%d...\r\n", retry_count + 1, TX_RETRY_COUNT);

        // Entrer en mode TX
        SX1276_standby(&SX1276);
        HAL_Delay(10);

        int tx_entry = SX1276_LoRaEntryTx(&SX1276, MESSAGE_LENGTH, TX_TIMEOUT);

        if(tx_entry != 1) {
            printf("✗ Cannot enter TX mode: %d\r\n", tx_entry);
            retry_count++;
            HAL_Delay(100);
            continue;
        }

        // Transmission du paquet
        printf("Sending: '%s'\r\n", message);
        int tx_result = SX1276_LoRaTxPacket(&SX1276, (uint8_t*)message, MESSAGE_LENGTH, TX_TIMEOUT);

        if(tx_result == 1) {
            printf("✓ Message transmitted successfully\r\n");
            LED_SetTransmission_Success();
            result = 1;
        } else {
            printf("✗ Transmission failed: %d\r\n", tx_result);
            retry_count++;

            if(retry_count < TX_RETRY_COUNT) {
                printf("Retrying in 200ms...\r\n");
                HAL_Delay(200);
            }
        }
    }

    if(result == 0) {
        printf("✗ All transmission attempts failed\r\n");
        LED_SetTransmission_Failed();

        // Vérifier si la connexion est toujours valide
        LoRa_Master_CheckConnection();
    }

    // Retour en standby
    SX1276_standby(&SX1276);

    return result;
}

/* Reset et réinitialisation complète */
void LoRa_Reset_And_Reinit(void) {
    printf("=== LoRa Reset and Reinit ===\r\n");
    connection_established = 0;
    LED_SetConnection_Lost();

    // Reset matériel complet
    SX1276_hw_Reset(&SX1276_hw);
    HAL_Delay(200);

    // Réinitialisation
    if(LoRa_Master_Init() == 1) {
        printf("✓ LoRa reinitialized successfully\r\n");
        // La connexion sera vérifiée dans la boucle principale
    } else {
        printf("✗ LoRa reinitialization failed\r\n");
        LED_SetTransmission_Failed();
    }
}

/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPIO_LEDs_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  printf("\r\n");
  printf("====================================================\r\n");
  printf("    LoRa Master Transmitter - STM32F407VG\r\n");
  printf("====================================================\r\n");
  printf("Device ID: %s\r\n", DEVICE_ID);
  printf("Mode: Master (Transmitter)\r\n");
  printf("Message: 'hello'\r\n");
  printf("Frequency: 868 MHz\r\n");
  printf("Transmission Interval: %d ms\r\n", TX_INTERVAL_MS);
  printf("====================================================\r\n");
  printf("LED Status:\r\n");
  printf("  BLUE   : Master Mode (Always ON)\r\n");
  printf("  ORANGE : LoRa Connection Established\r\n");
  printf("  GREEN  : Transmission Success\r\n");
  printf("  RED    : Transmission Error\r\n");
  printf("====================================================\r\n");

  // Initialisation des LEDs
  LED_Init();
  LED_SetMode_Master(); // LED bleue immédiatement

  HAL_Delay(1000);

  // Initialisation LoRa
  if(LoRa_Master_Init() != 1) {
      printf("CRITICAL: LoRa initialization failed!\r\n");
      LED_SetTransmission_Failed();

      // Tentative de récupération
      LoRa_Reset_And_Reinit();
  }

  printf("Starting transmission loop...\r\n");
  printf("====================================================\r\n");

  last_connection_check = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    // Vérification périodique de la connexion
    if((current_tick - last_connection_check) >= CONNECTION_CHECK_INTERVAL) {
        LoRa_Master_CheckConnection();
        last_connection_check = current_tick;
    }

    // Transmission périodique seulement si connexion établie
    static uint32_t last_transmission = 0;

    if((current_tick - last_transmission) >= TX_INTERVAL_MS) {

        if(connection_established) {
            transmission_count++;
            printf("\r\n--- Transmission #%d ---\r\n", transmission_count);

            // Préparation du message
            strcpy(tx_buffer, "hello");

            // Transmission avec vérification
            int tx_success = LoRa_Master_Transmit_Verified(tx_buffer);

            if(tx_success) {
                printf("✓ Transmission #%d completed successfully\r\n", transmission_count);

                // Maintenir LED verte pendant 2 secondes
                HAL_Delay(2000);
                LED_Reset_Status(); // Reset des LEDs de statut

            } else {
                printf("✗ Transmission #%d failed\r\n", transmission_count);

                // Reset automatique après plusieurs échecs
                if(transmission_count % 5 == 0) {
                    printf("Performing automatic reset after failures...\r\n");
                    LoRa_Reset_And_Reinit();
                }
            }
        } else {
            printf("⚠ Waiting for LoRa connection...\r\n");
        }

        last_transmission = current_tick;
    }

    // Délai court pour éviter la surcharge CPU
    HAL_Delay(50);

    /* USER CODE END WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function for STM32F407VG LEDs
  * @param None
  * @retval None
  */
static void MX_GPIO_LEDs_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDs */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    // Clignotement d'urgence de toutes les LEDs
    HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);
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
  /* USER can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
