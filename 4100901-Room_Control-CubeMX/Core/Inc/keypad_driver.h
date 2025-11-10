#ifndef KEYPAD_DRIVER_H
#define KEYPAD_DRIVER_H

#include "main.h"
#include <stdint.h>

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

/**
 * @brief Estructura para manejar el teclado matricial
 * @note  Contiene los puertos y pines asociados a las filas y columnas del keypad
 * @param row_ports: array de punteros a los puertos GPIO de las filas
 * @param row_pins: array de pines asociados a las filas    
 * @param col_ports: array de punteros a los puertos GPIO de las columnas
 * @param col_pins: array de pines asociados a las columnas
 */
typedef struct {
    GPIO_TypeDef* row_ports[KEYPAD_ROWS];   // itera sobre los puertos de las filas
    uint16_t row_pins[KEYPAD_ROWS];         // itera sobre los pines de las filas
    GPIO_TypeDef* col_ports[KEYPAD_COLS];   // itera sobre los puertos de las columnas
    uint16_t col_pins[KEYPAD_COLS];         // itera sobre los pines de las columnas
} keypad_handle_t;
/**
 * @brief Inicializa el teclado matricial
 * @param keypad: puntero a la estructura del teclado
 * @related keypad_handle_t
 * @retval void
 * @note Configura las filas en nivel bajo para detectar flancos descendentes
 */
void keypad_init(keypad_handle_t* keypad);
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin);

#endif // KEYPAD_DRIVER_H