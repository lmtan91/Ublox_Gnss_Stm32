
/**
 * @file ring_buffer.h
 * @brief Simple circular buffer
 *
 * This implementation is not thread-safe.  In particular, none of
 * these functions is guaranteed re-entrant.
 */

#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_


#ifdef __cplusplus
extern "C"{
#endif

static int remove = 0;
static int insert = 0;
/**
 * Ring buffer type.
 *
 * The buffer is empty when head == tail.
 *
 * The buffer is full when the head is one byte in front of the tail,
 * modulo buffer length.
 *
 * One byte is left free to distinguish empty from full. */
typedef struct ring_buffer {
    volatile uint8_t *buf; /**< Buffer items are stored into */
    volatile uint16_t head;         /**< Index of the next item to remove */
    volatile uint16_t tail;         /**< Index where the next item will get inserted */
    volatile uint16_t size;         /**< Buffer capacity minus one */
} ring_buffer;

/**
 * Initialise a ring buffer.
 *
 *  @param rb   Instance to initialise
 *
 *  @param size Number of items in buf.  The ring buffer will always
 *              leave one element unoccupied, so the maximum number of
 *              elements it can store will be size - 1.  Thus, size
 *              must be at least 2.
 *
 *  @param buf  Buffer to store items into
 */
static inline void rb_init(ring_buffer *rb, uint16_t size, uint8_t *buf) {
    rb->head = 0;
    rb->tail = 0;
    rb->size = size - 1;
    rb->buf = buf;
}

/**
 * @brief Return the number of elements stored in the ring buffer.
 * @param rb Buffer whose elements to count.
 */
static inline uint16_t rb_full_count(ring_buffer *rb) {
    int32_t size = rb->tail - rb->head;
    return ( size<0 ) ? (uint16_t)(size + rb->size + 1) : (uint16_t)size;
}

/**
 * @brief Returns true if and only if the ring buffer is full.
 * @param rb Buffer to test.
 */
static inline int rb_is_full(ring_buffer *rb) {
    return (rb->tail + 1 == rb->head) ||
        (rb->tail == rb->size && rb->head == 0);
}

/**
 * @brief Returns true if and only if the ring buffer is empty.
 * @param rb Buffer to test.
 */
static inline int16_t rb_is_empty(ring_buffer *rb) {
    return rb->head == rb->tail;
}

/**
 * Append element onto the end of a ring buffer.
 * @param rb Buffer to append onto.
 * @param element Value to append.
 */
static inline void rb_insert(ring_buffer *rb, uint8_t element) {
    rb->buf[rb->tail++] = element;
    if (rb->tail > rb->size) rb->tail = 0;

    insert++;
}

/**
 * @brief Remove and return the first item from a ring buffer.
 * @param rb Buffer to remove from, must contain at least one element.
 */
static inline uint8_t rb_remove(ring_buffer *rb) {
    uint8_t ch = rb->buf[rb->head++];
    if (rb->head > rb->size) rb->head = 0;

    remove++;
    return ch;
}

/*
 * Roger Clark. 20141125,
 * added peek function.
 * @brief Return the first item from a ring buffer, without removing it
 * @param rb Buffer to remove from, must contain at least one element.
 */
static inline int16_t rb_peek(ring_buffer *rb)
{
    if (rb->head == rb->tail)
    {
        return -1;
    }
    else
    {
        return rb->buf[rb->head];
    }
}


/**
 * @brief Attempt to remove the first item from a ring buffer.
 *
 * If the ring buffer is nonempty, removes and returns its first item.
 * If it is empty, does nothing and returns a negative value.
 *
 * @param rb Buffer to attempt to remove from.
 */
static inline int16_t rb_safe_remove(ring_buffer *rb) {
    return rb_is_empty(rb) ? -1 : rb_remove(rb);
}

/**
 * @brief Attempt to insert an element into a ring buffer.
 *
 * @param rb Buffer to insert into.
 * @param element Value to insert into rb.
 * @sideeffect If rb is not full, appends element onto buffer.
 * @return If element was appended, then true; otherwise, false. */
static inline int16_t rb_safe_insert(ring_buffer *rb, uint8_t element) {
    if (rb_is_full(rb)) {
        return 0;
    }
    rb_insert(rb, element);
    return 1;
}

/**
 * @brief Append an item onto the end of a non-full ring buffer.
 *
 * If the buffer is full, removes its first item, then inserts the new
 * element at the end.
 *
 * @param rb Ring buffer to insert into.
 * @param element Value to insert into ring buffer.
 * @return On success, returns -1.  If an element was popped, returns
 *         the popped value.
 */
static inline int16_t rb_push_insert(ring_buffer *rb, uint8_t element) {
    int ret = -1;
    if (rb_is_full(rb)) {
        ret = rb_remove(rb);
    }
    rb_insert(rb, element);
    return ret;
}

/**
 * @brief Discard all items from a ring buffer.
 * @param rb Ring buffer to discard all items from.
 */
static inline void rb_reset(ring_buffer *rb) {
    rb->tail = rb->head;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
