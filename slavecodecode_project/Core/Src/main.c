/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MODE SLAVE CORRIGÉ - STM32F407VG - Réception LoRa
  ******************************************************************************
  * Version corrigée avec gestion précise des LEDs et réception garantie
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
#define DEVICE_MODE_SLAVE 0
#define DEVICE_ID "STM32F407_SLAVE_001"

// Définition des LEDs pour STM32F407VG Discovery
#define LED_GREEN_GPIO_Port GPIOD    // Réception réussie SEULEMENT
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_ORANGE_GPIO_Port GPIOD   // Connexion LoRa établie SEULEMENT
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOD      // Erreur réception
#define LED_RED_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOD     // Mode Slave (toujours ON)
#define LED_BLUE_Pin GPIO_PIN_15

// Paramètres de réception
#define RX_TIMEOUT 10000
#define RX_BUFFER_SIZE 64
#define MESSAGE_LENGTH 5
#define RX_CHECK_INTERVAL 100
#define CONNECTION_CHECK_INTERVAL 2000
#define CONNECTION_TIMEOUT 5000

// Définition du registre OpMode si manquant
#ifndef REG_LR_OPMODE
#define REG_LR_OPMODE 0x01
#endif

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
SX1276_hw_t SX1276_hw;
SX1276_t SX1276;
int reception_count = 0;
char rx_buffer[RX_BUFFER_SIZE];
uint32_t last_rx_check = 0;
uint32_t last_connection_check = 0;
uint32_t last_successful_rx = 0;
uint8_t connection_established = 0;
uint8_t module_in_rx_mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_LEDs_Init(void);
void LED_Init(void);
void LED_SetMode_Slave(void);
void LED_SetConnection_Established(void);
void LED_SetConnection_Lost(void);
void LED_SetReception_Success(void);
void LED_SetReception_Failed(void);
void LED_Reset_Status(void);
void LED_Blink_Error(int count);
int LoRa_Slave_Init(void);
int LoRa_Slave_StartRx(void);
int LoRa_Slave_CheckReception(void);
int LoRa_Slave_CheckConnection(void);
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

void LED_SetMode_Slave(void) {
    // LED bleue TOUJOURS allumée pour indiquer le mode Slave
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}

void LED_SetConnection_Established(void) {
    // LED orange SEULEMENT si connexion LoRa détectée
    HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_SET);
    LED_SetMode_Slave(); // Maintenir bleue
}

void LED_SetConnection_Lost(void) {
    // Éteindre LED orange si pas de connexion
    HAL_GPIO_WritePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, GPIO_PIN_RESET);
    LED_SetMode_Slave(); // Maintenir bleue
}

void LED_SetReception_Success(void) {
    // LED verte SEULEMENT si réception confirmée
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    LED_SetMode_Slave(); // Maintenir bleue
}

void LED_SetReception_Failed(void) {
    // LED rouge clignotante pour échec de réception
    static uint32_t last_blink = 0;
    uint32_t current_tick = HAL_GetTick();

    if((current_tick - last_blink) >= 500) {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        last_blink = current_tick;
    }

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    LED_SetMode_Slave(); // Maintenir bleue
}

void LED_Reset_Status(void) {
    // Reset des LEDs de statut, garder seulement mode et connexion
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    LED_SetMode_Slave(); // Maintenir bleue

    // Maintenir LED orange si connexion établie
    if(connection_established) {
        LED_SetConnection_Established();
    }
}

void LED_Blink_Error(int count) {
    for(int i = 0; i < count; i++) {
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        LED_SetMode_Slave();
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        LED_SetMode_Slave();
        HAL_Delay(200);
    }
}

/* Initialisation LoRa Slave */
int LoRa_Slave_Init(void) {
    printf("=== LoRa Slave Initialization ===\r\n");

    LED_SetMode_Slave(); // LED bleue seulement

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

/* Démarrage de la réception */
int LoRa_Slave_StartRx(void) {
    printf("Starting RX mode...\r\n");

    // Mettre en standby d'abord
    SX1276_standby(&SX1276);
    HAL_Delay(10);

    // Entrer en mode RX
    int rx_entry = SX1276_LoRaEntryRx(&SX1276, MESSAGE_LENGTH, RX_TIMEOUT);

    if(rx_entry == 1) {
        printf("✓ RX mode active - Listening...\r\n");
        module_in_rx_mode = 1;
        return 1;
    } else {
        printf("✗ Cannot enter RX mode: %d\r\n", rx_entry);
        module_in_rx_mode = 0;
        return 0;
    }
}

/* Vérification de connexion LoRa */
int LoRa_Slave_CheckConnection(void) {
    // Test de communication avec le module local
    uint8_t version = SX1276_SPIRead(&SX1276, REG_LR_VERSION);

    if(version != 0x12) {
        printf("⚠ Local module communication failed\r\n");
        connection_established = 0;
        LED_SetConnection_Lost();
        return 0;
    }

    // Vérifier si on a reçu des données récemment (indique une connexion active)
    uint32_t current_tick = HAL_GetTick();

    if(last_successful_rx > 0 && (current_tick - last_successful_rx) < CONNECTION_TIMEOUT) {
        if(!connection_established) {
            printf("✓ LoRa connection established (receiving data)\r\n");
            connection_established = 1;
            LED_SetConnection_Established();
        }
        return 1;
    } else {
        // Pas de données reçues récemment
        if(connection_established && (current_tick - last_successful_rx) > CONNECTION_TIMEOUT) {
            printf("⚠ LoRa connection timeout\r\n");
            connection_established = 0;
            LED_SetConnection_Lost();
        }
        return 0;
    }
}

/* Vérification de réception garantie */
int LoRa_Slave_CheckReception(void) {
    // Vérifier d'abord si le module est toujours en mode RX
    uint8_t modem_stat = SX1276_SPIRead(&SX1276, LR_RegModemStat);
    uint8_t op_mode = SX1276_SPIRead(&SX1276, REG_LR_OPMODE);

    // Vérifier si on est en mode RX continu
    if((op_mode & 0x07) != 0x05) {  // 0x05 = RX Continuous mode
        printf("Module not in RX mode (OpMode: 0x%02X) - Restarting RX...\r\n", op_mode);
        LoRa_Slave_StartRx();
        return 0;
    }

    // Vérifier s'il y a des données reçues
    uint8_t bytes_received = SX1276_LoRaRxPacket(&SX1276);

    if(bytes_received > 0) {
        // Données reçues - traitement immédiat
        memset(rx_buffer, 0, RX_BUFFER_SIZE);

        uint8_t actual_bytes = SX1276_read(&SX1276, (uint8_t*)rx_buffer, bytes_received);
        rx_buffer[actual_bytes] = '\0'; // Sécurité

        reception_count++;
        last_successful_rx = HAL_GetTick();

        printf("✓ MESSAGE RECEIVED #%d (%d bytes): '%s'\r\n",
               reception_count, actual_bytes, rx_buffer);

        // LED verte pour succès de réception
        LED_SetReception_Success();

        // Établir la connexion si ce n'est pas fait
        if(!connection_established) {
            connection_established = 1;
            LED_SetConnection_Established();
            printf("✓ LoRa connection established\r\n");
        }

        // Redémarrer la réception immédiatement
        HAL_Delay(50);
        LoRa_Slave_StartRx();

        return actual_bytes;

    } else {
        // Pas de données reçues
        uint32_t current_tick = HAL_GetTick();

        // Si pas de réception depuis longtemps, considérer comme échec
        if(last_successful_rx > 0 && (current_tick - last_successful_rx) > CONNECTION_TIMEOUT) {
            LED_SetReception_Failed();
        }

        return 0;
    }
}

/* Reset et réinitialisation complète */
void LoRa_Reset_And_Reinit(void) {
    printf("=== LoRa Reset and Reinit ===\r\n");
    connection_established = 0;
    module_in_rx_mode = 0;
    LED_SetConnection_Lost();

    // Reset matériel complet
    SX1276_hw_Reset(&SX1276_hw);
    HAL_Delay(200);

    // Réinitialisation
    if(LoRa_Slave_Init() == 1) {
        printf("✓ LoRa reinitialized successfully\r\n");
        LoRa_Slave_StartRx();
    } else {
        printf("✗ LoRa reinitialization failed\r\n");
        LED_Blink_Error(5);
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
  printf("    LoRa Slave Receiver - STM32F407VG\r\n");
  printf("====================================================\r\n");
  printf("Device ID: %s\r\n", DEVICE_ID);
  printf("Mode: Slave (Receiver)\r\n");
  printf("Listening for: LoRa messages\r\n");
  printf("Frequency: 868 MHz\r\n");
  printf("Buffer Size: %d bytes\r\n", RX_BUFFER_SIZE);
  printf("====================================================\r\n");
  printf("LED Status:\r\n");
  printf("  BLUE   : Slave Mode (Always ON)\r\n");
  printf("  ORANGE : LoRa Connection Established\r\n");
  printf("  GREEN  : Message Reception Success\r\n");
  printf("  RED    : Reception Error (Blinking)\r\n");
  printf("====================================================\r\n");

  // Initialisation des LEDs
  LED_Init();
  LED_SetMode_Slave(); // LED bleue immédiatement

  HAL_Delay(1000);

  // Initialisation LoRa
  if(LoRa_Slave_Init() != 1) {
      printf("CRITICAL: LoRa initialization failed!\r\n");
      LED_Blink_Error(10);

      // Tentative de récupération
      LoRa_Reset_And_Reinit();
  }

  // Démarrer la réception
  if(LoRa_Slave_StartRx() != 1) {
      printf("CRITICAL: Cannot start RX mode!\r\n");
      LED_Blink_Error(5);
  }

  printf("Starting reception loop...\r\n");
  printf("====================================================\r\n");

  last_rx_check = HAL_GetTick();
  last_connection_check = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();

    // Vérification haute fréquence de réception (critique)
    if((current_tick - last_rx_check) >= RX_CHECK_INTERVAL) {

        int bytes_received = LoRa_Slave_CheckReception();

        if(bytes_received > 0) {
            // Message reçu avec succès
            printf("✓ Reception successful - Waiting for next message...\r\n");

            // Maintenir LED verte pendant 1 seconde
            HAL_Delay(1000);
            LED_Reset_Status(); // Retour à l'état normal

        }

        last_rx_check = current_tick;
    }

    // Vérification périodique de la connexion
    if((current_tick - last_connection_check) >= CONNECTION_CHECK_INTERVAL) {
        LoRa_Slave_CheckConnection();
        last_connection_check = current_tick;

        // Heartbeat si pas de connexion
        if(!connection_established) {
            printf("- Waiting for LoRa connection...\r\n");
            LED_SetReception_Failed(); // LED rouge clignotante
        }
    }

    // Vérification périodique de l'état du module (santé)
    static uint32_t last_health_check = 0;
    if((current_tick - last_health_check) >= 15000) {

        uint8_t version = SX1276_SPIRead(&SX1276, REG_LR_VERSION);

        if(version != 0x12) {
            printf("⚠ Module health check failed (version: 0x%02X) - Resetting...\r\n", version);
            LoRa_Reset_And_Reinit();
        } else {
            printf("✓ Module health check OK\r\n");

            // Si pas en mode RX, redémarrer
            if(!module_in_rx_mode) {
                printf("⚠ Module not in RX mode - Restarting RX...\r\n");
                LoRa_Slave_StartRx();
            }
        }

        last_health_check = current_tick;
    }

    // Délai court pour éviter la surcharge CPU
    HAL_Delay(10);

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
