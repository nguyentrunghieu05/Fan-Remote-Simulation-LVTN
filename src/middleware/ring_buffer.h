/**
 * @file    ring_buffer.h
 * @brief   Lock-free single-producer / single-consumer byte FIFO.
 *
 * Typical use: a UART RX interrupt pushes bytes, a task pops them. Because
 * the head is only written by the producer and the tail only by the
 * consumer, no critical section is needed on a single-core MCU.
 * Capacity must be a power of two.
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    uint8_t          *buf;
    size_t            mask;   /* capacity - 1 */
    volatile size_t   head;   /* next write index (producer) */
    volatile size_t   tail;   /* next read index (consumer)  */
} ring_buffer_t;

/** @return false if capacity is not a power of two. */
bool   rb_init(ring_buffer_t *rb, uint8_t *storage, size_t capacity);
bool   rb_put(ring_buffer_t *rb, uint8_t byte);
bool   rb_get(ring_buffer_t *rb, uint8_t *byte);
size_t rb_write(ring_buffer_t *rb, const uint8_t *data, size_t len);
size_t rb_read(ring_buffer_t *rb, uint8_t *data, size_t len);
size_t rb_count(const ring_buffer_t *rb);
size_t rb_free(const ring_buffer_t *rb);
void   rb_clear(ring_buffer_t *rb);

#endif /* RING_BUFFER_H */
