#include "ring_buffer.h"

bool rb_init(ring_buffer_t *rb, uint8_t *storage, size_t capacity)
{
    if (rb == NULL || storage == NULL || capacity < 2U ||
        (capacity & (capacity - 1U)) != 0U) {
        return false;
    }
    rb->buf  = storage;
    rb->mask = capacity - 1U;
    rb->head = 0U;
    rb->tail = 0U;
    return true;
}

size_t rb_count(const ring_buffer_t *rb)
{
    return (rb->head - rb->tail) & rb->mask;
}

size_t rb_free(const ring_buffer_t *rb)
{
    /* One slot is kept empty to tell "full" from "empty". */
    return rb->mask - rb_count(rb);
}

bool rb_put(ring_buffer_t *rb, uint8_t byte)
{
    size_t next = (rb->head + 1U) & rb->mask;
    if (next == rb->tail) {
        return false;                       /* full: drop the byte */
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

bool rb_get(ring_buffer_t *rb, uint8_t *byte)
{
    if (rb->tail == rb->head) {
        return false;
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1U) & rb->mask;
    return true;
}

size_t rb_write(ring_buffer_t *rb, const uint8_t *data, size_t len)
{
    size_t n = 0U;
    while (n < len && rb_put(rb, data[n])) {
        n++;
    }
    return n;
}

size_t rb_read(ring_buffer_t *rb, uint8_t *data, size_t len)
{
    size_t n = 0U;
    while (n < len && rb_get(rb, &data[n])) {
        n++;
    }
    return n;
}

void rb_clear(ring_buffer_t *rb)
{
    rb->tail = rb->head;
}
