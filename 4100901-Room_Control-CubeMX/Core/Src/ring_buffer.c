#include "ring_buffer.h"

/**
 * @brief Inicializa el buffer circular.
 * @param rb Puntero a la estructura del buffer circular.
 * @param buffer Puntero al array que actuará como buffer.
 * @param capacity Capacidad máxima del buffer.
 * @param is_full Campo que indica si el buffer está lleno.
 */

void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity) 
{
    rb->head = 0;    // Inicializa la cabeza del buffer en posicion 0
    rb->tail = 0;    // Inicializa la cola del buffer en posicion 0 
    rb->capacity = capacity; //capacidad del tamaño de el buffer
    rb->buffer = buffer;    // Inicializa el puntero del buffer direcciono de memoria 
    rb->is_full = false;    // Inicializa el buffer como no lleno
}


/** 
 * @brief Escribe un dato en el buffer circular.
 * @param rb Puntero a la estructura del buffer circular.
 * @param data Dato a escribir en el buffer.
 * @param ring_buffer_write Puntero a la función de escritura.
 * @retval true si la escritura fue exitosa, false si el buffer está lleno. 
 */
bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    if (ring_buffer_is_full(rb))
    {
        // sobre escribe el dato más viejo
        rb->tail = (rb->tail + 1) % rb->capacity;
    }
    rb->buffer[rb->head] = data;              // Escribe el dato en la posición actual de head
    rb->head = (rb->head + 1) % rb->capacity; // Avanza head de manera circular
    return true;                              // Escritura exitosa
}

/**
 * @brief Lee un dato del buffer circular.
 * @param rb Puntero a la estructura del buffer circular.
 * @param data Puntero donde se almacenará el dato leído.
 * @retval true si la lectura fue exitosa, false si el buffer está vacío.
 */
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    if (ring_buffer_is_empty(rb))
    {
        return false; // No hay datos para leer
    }
    *data = rb->buffer[rb->tail];             // Lee el dato en la posición actual de tail
    rb->tail = (rb->tail + 1) % rb->capacity; // Avanza tail de manera circular
    return true;                              // Lectura exitosa
}

/**
 * @brief Cuenta el número de elementos en el buffer circular.
 * @param rb Puntero a la estructura del buffer circular.
 * @retval Número de elementos actualmente almacenados en el buffer.
 */
uint16_t ring_buffer_count(ring_buffer_t *rb) // contar no leidos de buffer 
{
    if (rb->is_full)
    {
        return rb->capacity;
    }
    if (rb->head >= rb->tail)
    {
        return rb->head - rb->tail; // head está adelante de tail
    }
    else
    {
        return rb->capacity + rb->head - rb->tail; // head ha envuelto alrededor de tail
    }
}
/**
 * @brief Verifica si el buffer circular está vacío.
 * @param rb Puntero a la estructura del buffer circular.
 * @retval true si el buffer está vacío, false en caso contrario.
 */
bool ring_buffer_is_empty(ring_buffer_t *rb) // verificar si el buffer esta vacio
{
    return rb->head == rb->tail;
}


/**
 * @brief Verifica si el buffer circular está lleno.
 * @param rb Puntero a la estructura del buffer circular.
 * @retval true si el buffer está lleno, false en caso contrario.
 */
bool ring_buffer_is_full(ring_buffer_t *rb) // verificar si el buffer esta lleno
{
    // Está lleno si avanzar head haría que coincida con tail
    return ((rb->head + 1) % rb->capacity) == rb->tail;
}

/**
 * @brief Limpia el buffer circular.
 * @param rb Puntero a la estructura del buffer circular.
 */
void ring_buffer_flush(ring_buffer_t *rb) // limpiar el buffer
{
    rb->head = 0;        // Reinicia el puntero de escritura
    rb->tail = 0;        // Reinicia el puntero de lectura
    rb->is_full = false; // Marca el buffer como no lleno
}