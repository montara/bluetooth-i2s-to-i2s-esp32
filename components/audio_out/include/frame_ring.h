/*
 * The elastic buffer between an audio producer and the DAC.
 *
 * Stereo int16 frames, power-of-two capacity. head and tail are free-running
 * frame counters masked only on access: that makes "full" and "empty"
 * unambiguous without sacrificing a slot, and unsigned wraparound keeps
 * (head - tail) correct when the counters roll over.
 *
 * Split out from audio_out.c so the index arithmetic -- the part most likely to
 * hide an off-by-one -- can be compiled and tested on a host machine with no
 * ESP32 in sight. See test/test_frame_ring.c.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef FRAME_RING_HOST_TEST
/* Host builds are single-threaded; the lock is a no-op. */
typedef int frame_ring_lock_t;
#define FRAME_RING_LOCK_INIT    0
#define FRAME_RING_ENTER(r)     ((void)0)
#define FRAME_RING_EXIT(r)      ((void)0)
#else
#include "freertos/FreeRTOS.h"
typedef portMUX_TYPE frame_ring_lock_t;
#define FRAME_RING_LOCK_INIT    portMUX_INITIALIZER_UNLOCKED
#define FRAME_RING_ENTER(r)     portENTER_CRITICAL(&(r)->lock)
#define FRAME_RING_EXIT(r)      portEXIT_CRITICAL(&(r)->lock)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t  *buf;      /*!< cap * 2 samples */
    uint32_t  cap;      /*!< frames; must be a power of two */
    uint32_t  mask;     /*!< cap - 1 */
    uint32_t  head;
    uint32_t  tail;
    uint32_t  dropped;  /*!< frames lost to a full buffer, cumulative */
    frame_ring_lock_t lock;
} frame_ring_t;

/**
 * Point a ring at caller-provided storage.
 *
 * @param storage  cap * 2 int16_t of memory, owned by the caller.
 * @param cap      frames; must be a power of two.
 * @return false if cap is not a power of two or storage is NULL.
 */
bool frame_ring_init(frame_ring_t *r, int16_t *storage, uint32_t cap);

/** Frames currently queued. */
uint32_t frame_ring_fill(frame_ring_t *r);

/** Queued frames as a percentage of capacity, 0..100. */
int frame_ring_fill_pct(frame_ring_t *r);

/**
 * Append frames, never blocking.
 *
 * @return frames accepted; a shortfall means the buffer was full and the
 *         remainder was dropped and added to `dropped`.
 */
size_t frame_ring_write(frame_ring_t *r, const int16_t *src, size_t frames);

/**
 * Take up to `frames` frames.
 *
 * @return frames actually produced, which may be fewer than asked for.
 */
size_t frame_ring_read(frame_ring_t *r, int16_t *dst, size_t frames);

/**
 * Discard everything and re-seed with silence up to `fill_pct` of capacity.
 *
 * Restarting from a half-full buffer rather than an empty one is what gives the
 * drift servo room to correct in both directions from its first tick. A buffer
 * that starts empty can only ever be pushed one way, and spends its first
 * seconds underrunning while the servo works out which way that is.
 */
void frame_ring_recentre(frame_ring_t *r, int fill_pct);

#ifdef __cplusplus
}
#endif
