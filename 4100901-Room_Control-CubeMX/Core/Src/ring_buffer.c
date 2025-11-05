#include "ring_buffer.h"

//head -> donde se escribe
//tail -> donde se lee

void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity) 
{
    rb->head = 0;    // Inicializa la cabeza del buffer en posicion 0
    rb->tail = 0;    // Inicializa la cola del buffer en posicion 0 
    rb->capacity = capacity; //capacidad del tamaño de el buffer
    rb->buffer = buffer;    // Inicializa el puntero del buffer direcciono de memoria 
}

bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    uint16_t next_head = (rb->head + 1) % rb->capacity; // Calcula la siguiente posicion de la cabeza
    if (next_head == rb->tail) { // verificamos si el buffer esta lleno
        // Buffer is full
        return false;
    }
    rb->buffer[rb->head] = data;  // Escribe el dato en la posicion de la cabeza
    rb->head = next_head;   // Actualiza la posicion de la cabeza
    return true;
}

bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    if (rb->head == rb->tail) { // verificacmos si el buffer esta vacio
        return false;
    }
    // Leer el dato de la posicion de la cola
    *data = rb->buffer[rb->tail];
    // Actualizar la posicion de la cola
    rb->tail = (rb->tail + 1) % rb->capacity;

    // lectura exitosa
    return true;
}

uint16_t ring_buffer_count(ring_buffer_t *rb) // contar no leidos de buffer 
{
    if (rb->head >= rb->tail) { // si la cabeza es mayor o igual a la cola
        return rb->head - rb->tail;   // retornamos la diferencia que son los datos no leidos 
    } else {
        return rb->capacity - (rb->tail - rb->head); // si la cola es mayor a la cabeza
    }
}

bool ring_buffer_is_empty(ring_buffer_t *rb) // verificar si el buffer esta vacio
{
    return (rb->head == rb->tail);
}

bool ring_buffer_is_full(ring_buffer_t *rb) // verificar si el buffer esta lleno
{
    return ((rb->head + 1) % rb->capacity) == rb->tail; // si la siguiente posicion de la cabeza es igual a la cola

}

void ring_buffer_flush(ring_buffer_t *rb) // limpiar el buffer
{
    rb->head = 0;
    rb->tail = 0;
}