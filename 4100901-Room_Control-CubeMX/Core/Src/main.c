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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led_driver.h"
#include "keypad_driver.h"
#include "ring_buffer.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- CONFIGURACION DEL SISTEMA ---
/** @brief Configuración de la contraseña y tiempos
 * @note  Ajustar estos valores según las necesidades del sistema
 * @param PASSWORD: Contraseña correcta de 4 dígitos
 * @param DEBOUNCE_TIME_MS: Tiempo de anti-rebote para las teclas
 * @param FEEDBACK_LED_TIME_MS: Tiempo que el LED se enciende al oprimir cualquier tecla
 * @param SUCCESS_LED_TIME_MS: Tiempo que el LED se enciende cuando se ingresa la contraseña correcta
 */
#define PASSWORD "1102" // Contraseña de 4 dígitos
#define PASSWORD_LEN 4
#define DEBOUNCE_TIME_MS 200     // Tiempo de anti-rebote para las teclas 
#define FEEDBACK_LED_TIME_MS 100  // Tiempo que el LED se enciende al oprimir cualquier tecla
#define SUCCESS_LED_TIME_MS 4000  // Tiempo que el LED se enciende cuando se ingresa la contraseña correcta

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/**
 * @brief Manejo del LED y del Keypad
 * @note  Contiene la lógica para el control del LED y la lectura del teclado matricial
 * @related led_handle_t, keypad_handle_t
 * @param led1: estructura que define el LED conectado al pin PA5
 * @param keypad: estructura que define el teclado matricial conectado a varios pines GPIO
 */
led_handle_t led1 = { .port = GPIOA, .pin = GPIO_PIN_5 }; // LED en PA5 (LD2)

keypad_handle_t keypad = {
    .row_ports = {KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port}, // Puertos de las filas
    .row_pins  = {KEYPAD_R1_Pin, KEYPAD_R2_Pin, KEYPAD_R3_Pin, KEYPAD_R4_Pin},                         // Pines de las filas
    .col_ports = {KEYPAD_C1_GPIO_Port, KEYPAD_C2_GPIO_Port, KEYPAD_C3_GPIO_Port, KEYPAD_C4_GPIO_Port}, // puertos de las columnas
    .col_pins  = {KEYPAD_C1_Pin, KEYPAD_C2_Pin, KEYPAD_C3_Pin, KEYPAD_C4_Pin}                          // Pines de las columnas
};

// --- Buffer circular para teclas ---
/** @brief Buffer circular para almacenar las teclas presionadas
 * @related ring_buffer_t
 * @note  Permite manejar las pulsaciones de teclas de forma no bloqueante
 * @param keypad_buffer: array que actúa como buffer del ring buffer
 * @param keypad_rb: estructura del ring buffer que gestiona el buffer circular
 */
#define KEYPAD_BUFFER_LEN 16              // Tamaño del buffer circular
uint8_t keypad_buffer[KEYPAD_BUFFER_LEN]; // Buffer para el ring buffer
ring_buffer_t keypad_rb;                  // Estructura del ring buffer

// --- VARIABLES PARA GESTION DE CONTRASEÑA ---

/** Contraseña ingresada por el usuario   */
/**
 * @brief Variables para gestionar la contraseña ingresada
 * @note  Almacena la contraseña ingresada y el índice actual
 * @param entered_password: array que almacena la contraseña ingresada
 * @param password_index: índice del siguiente dígito a ingresar
*/
char entered_password[PASSWORD_LEN + 1] = {0}; // +1 para el carácter nulo
uint8_t password_index = 0;

// --- VARIABLES DE ESTADO PARA LOGICA NO BLOQUEANTE ---
/**
 * @brief Variables para gestionar tiempos y estados
 * @note  Utilizadas para implementar lógica no bloqueante en el manejo del LED y las teclas
 * @param last_key_press_time: tiempo de la última tecla presionada (anti-rebote)
 * @param led_timer_start: tiempo de inicio del temporizador del LED
 * @param led_on_duration: duración que el LED debe permanecer encendido
 */
uint32_t last_key_press_time = 0; 
uint32_t led_timer_start = 0;      
uint32_t led_on_duration = 0;      

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */
/**
 * @brief Prototipos de funciones privadas
 * @note  Declaraciones de funciones utilizadas en el programa principal
 * @param SystemClock_Config: Configura el reloj del sistema
 * @param MX_GPIO_Init: Inicializa los pines GPIO
 * @param MX_USART2_UART_Init: Inicializa el periférico USART2 para comunicación UART 
 * @param manage_led_timer: Gestiona el apagado automático del LED sin bloquear el programa
 * @param process_key: Procesa una tecla recibida del buffer del keypad
 */

void SystemClock_Config(void); 
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void manage_led_timer(void);
void process_key(uint8_t key);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Callback de interrupción para el teclado matricial.
 * @note  Lee la tecla presionada y la almacena en el buffer circular.
 * @param GPIO_Pin El pin de la columna que generó la interrupción.
 * @param keypad_scan: Función que escanea el teclado y devuelve la tecla presionada.
 * @param ring_buffer_write: Función que escribe datos en el buffer circular.
 * 
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { //espera una interrupción por flanco descendente en una columna
    char key = keypad_scan(&keypad, GPIO_Pin); // Escanear la tecla presionada
    if (key != '\0') {
        ring_buffer_write(&keypad_rb, (uint8_t)key);
    }
}
/**
 * @brief Procesa una tecla recibida del buffer del keypad.
 * @note  Contiene la lógica principal de la aplicación: feedback visual,
 *        almacenamiento de la contraseña y verificación.
 * @param key La tecla presionada a procesar.
 * @related led_on, led_off
 * @param led_timer_start Iniciar el temporizador para el LED de feedback
 * @param led_on_duration Duración que el LED debe permanecer encendido
 */
void process_key(uint8_t key)
{
    
    led_on(&led1);
    led_timer_start = HAL_GetTick(); 
    led_on_duration = FEEDBACK_LED_TIME_MS;

    // 2. Almacenar el dígito ingresado en la contraseña

    if (password_index < PASSWORD_LEN) {                 // Si aún hay espacio en la contraseña
        entered_password[password_index++] = (char)key;  // Almacenar el dígito y avanzar el índice
        printf("Digito presionado: %c\r\n", key);
    }

    // 3. Si la contraseña se ha completado, verificarla
    // Si ya se ingresaron 4 dígitos, verificar contraseña

    if (password_index == PASSWORD_LEN) {
        if (strncmp(entered_password, PASSWORD, PASSWORD_LEN) == 0) { // Contraseña correcta strncmp compara dos cadenas hasta n caracteres
            printf("Contraseña correcta. ACCESO AUTORIZADO.\r\n");
            // Iniciar el temporizador largo para el LED de éxito
            led_on(&led1);
            led_timer_start = HAL_GetTick(); // Reiniciar el temporizador
            led_on_duration = SUCCESS_LED_TIME_MS;
        } else {
            printf("Contraseña incorrecta. ACCESO DENEGADO.\r\n");
           // Apagar el LED para indicar fallo
            led_off(&led1);
            
        }

        // 4. Reiniciar para el siguiente intento
        password_index = 0;
        memset(entered_password, 0, sizeof(entered_password)); // Limpiar la contraseña ingresada memset llena un bloque de memoria con un valor específico
        printf("\nSistema de acceso listo. Ingrese la contraseña de 4 digitos...\r\n");
    }
}

/**
 * @brief Gestiona el apagado automático del LED sin bloquear el programa.
 * @note  Esta función debe ser llamada repetidamente en el bucle principal.
 * @param  manage_led_timer: Función que gestiona el temporizador del LED.
 * @param led_timer_start Tiempo de inicio del temporizador del LED.
 * @param HAL_GetTick: Función que devuelve el tiempo actual en milisegundos.
 * @param led_on_duration Duración que el LED debe permanecer encendido.
 * @param led_timer_start Reiniciar el temporizador del LED cuando se apaga.
 */
void manage_led_timer(void)

{
  // Apaga el LED automáticamente cuando el tiempo indicado termina.
    if (led_timer_start != 0 && (HAL_GetTick() - led_timer_start > led_on_duration)) {
        led_off(&led1);
        led_timer_start = 0; //
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /**
   * @brief Inicialización de periféricos y estructuras
   * @note  Configura el LED, el buffer circular y el teclado matricial 
   * @param led_init: Función que inicializa el LED
   * @param ring_buffer_init: Función que inicializa el buffer circular
   * @param keypad_init: Función que inicializa el teclado matricial
   */
  led_init(&led1);
  ring_buffer_init(&keypad_rb, keypad_buffer, KEYPAD_BUFFER_LEN);
  keypad_init(&keypad);
  printf("Sistema listo. Esperando pulsaciones del teclado...\r\n");
  printf("Ingrese la contraseña de 4 digitos...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t key_from_buffer;
    
  // Leer teclas del buffer circular
    /**
     * @brief Lectura y procesamiento de teclas del buffer circular
     * @note  Implementa lógica de anti-rebote y llama a la función de procesamiento de teclas
     * @param ring_buffer_read: Función que lee datos del buffer circular
     * @param HAL_GetTick: Función que devuelve el tiempo actual en milisegundos.
     */
    if (ring_buffer_read(&keypad_rb, &key_from_buffer)) { // Leer una tecla del buffer circular
        uint32_t now = HAL_GetTick();
        // Anti-rebote: procesar solo si ha pasado el tiempo definido
        if (now - last_key_press_time > DEBOUNCE_TIME_MS) {
            last_key_press_time = now;
            process_key(key_from_buffer);
        }
    }
    
   manage_led_timer();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  * @param RCC_OscInitTypeDef Estructura para configurar los osciladores
  * @param RCC_ClkInitTypeDef Estructura para configurar los relojes del sistema
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  #ifdef GNUC
    setvbuf(stdout, NULL, _IONBF, 0);
  #endif
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|KEYPAD_R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin KEYPAD_C4_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin KEYPAD_R1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|KEYPAD_R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C1_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C4_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_C2_Pin KEYPAD_C3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C2_Pin|KEYPAD_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_R2_Pin KEYPAD_R4_Pin KEYPAD_R3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Retargeting printf to UART2
/**
 * @brief inicializamos el write para ustar prinf
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
