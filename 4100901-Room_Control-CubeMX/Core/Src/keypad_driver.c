#include "keypad_driver.h"
#include "main.h"
/**
 * @brief Mapa de teclas del keypad 4x4.
 * @param keypad_map Matriz que representa las teclas del keypad.
 * @param KEYPAD_ROWS Número de filas en el keypad.
 * @param KEYPAD_COLS Número de columnas en el keypad.
 */
static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

/**
 * @brief Inicializa las filas en nivel bajo.
 * Esto deja el teclado listo para detectar flancos descendentes por columna.
 * @param keypad Puntero a la estructura del keypad.
 * @param HAL_GPIO_WritePin Función para escribir en un pin GPIO.
 * @note  Todas las filas se configuran en BAJO.
 */
void keypad_init(keypad_handle_t* keypad) {
    // TAREA: Implementar esta función.
    // Todas las filas en estado ALTO (reposo: sin tecla presionada)
    
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET); // Filas en BAJO
    }
}
/**
 * @brief Escanea qué tecla fue presionada en función de la columna activada.
 * @param keypad Puntero a la estructura del keypad.
 * @param col_pin Pin de la columna que generó la interrupción.
 * @note  La función implementa anti-rebote, detección de fila y espera a la liberación de la tecla.
 */
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
   
    // 🟡 1. Anti-rebote
    HAL_Delay(5);

    // 🟡 2. Identificar qué columna generó la interrupción
    int col_index = -1;
    for (int c = 0; c < KEYPAD_COLS; c++) { // Iterar sobre las columnas
        if (keypad->col_pins[c] == col_pin) {
            col_index = c;
            break;
        }
    }
    /* Si no se encontró la columna, salir  */
    if (col_index == -1)  return '\0'; // No se encontró coincidencia
    char key_pressed = '\0';

    // 🟡 3. Poner todas las filas en ALTO (reposo)
    for (int r = 0; r < KEYPAD_ROWS; r++) {
        HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_SET);
    }

    // 🟡 4. Iterar cada fila para detectar cuál está presionada
    for (int row = 0; row < KEYPAD_ROWS; row++) {
        // Llevar una fila a BAJO
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_RESET);
        HAL_Delay(1); // Pequeño delay para estabilizar la lectura
        
        // Leer el pin de la columna en la que ocurrió la interrupción
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) {
            key_pressed = keypad_map[row][col_index];

            // 🟡 5. Esperar hasta que se libere la tecla
            while (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) 
            break;

        }

        // Regresar la fila a ALTO antes de probar la siguiente
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_SET);
    }

    // 🟡 6. Restaurar el estado del keypad
    keypad_init(keypad);

    return key_pressed;
}


