#include <string.h>

#include "frame_ring.h"

/* Unlocked helper; every caller already holds the lock. */
static inline uint32_t fill_unsafe(const frame_ring_t *r)
{
    return r->head - r->tail;
}

bool frame_ring_init(frame_ring_t *r, int16_t *storage, uint32_t cap)
{
    if (!r || !storage || cap == 0 || (cap & (cap - 1)) != 0) {
        return false;   /* capacity must be a power of two for the masking */
    }
    frame_ring_lock_t init = FRAME_RING_LOCK_INIT;
    r->buf = storage;
    r->cap = cap;
    r->mask = cap - 1;
    r->head = 0;
    r->tail = 0;
    r->dropped = 0;
    r->lock = init;
    memset(storage, 0, (size_t)cap * 2 * sizeof(int16_t));
    return true;
}

uint32_t frame_ring_fill(frame_ring_t *r)
{
    FRAME_RING_ENTER(r);
    uint32_t n = fill_unsafe(r);
    FRAME_RING_EXIT(r);
    return n;
}

int frame_ring_fill_pct(frame_ring_t *r)
{
    return (int)((uint64_t)frame_ring_fill(r) * 100u / r->cap);
}

size_t frame_ring_write(frame_ring_t *r, const int16_t *src, size_t frames)
{
    if (!src || frames == 0) {
        return 0;
    }

    size_t written = 0;
    FRAME_RING_ENTER(r);

    uint32_t space = r->cap - fill_unsafe(r);
    if (frames > space) {
        r->dropped += (uint32_t)(frames - space);
        frames = space;
    }

    while (written < frames) {
        uint32_t idx = r->head & r->mask;
        size_t run = r->cap - idx;              /* frames before the wrap */
        if (run > frames - written) {
            run = frames - written;
        }
        memcpy(&r->buf[(size_t)idx * 2], &src[written * 2], run * 2 * sizeof(int16_t));
        r->head += (uint32_t)run;
        written += run;
    }

    FRAME_RING_EXIT(r);
    return written;
}

size_t frame_ring_read(frame_ring_t *r, int16_t *dst, size_t frames)
{
    if (!dst || frames == 0) {
        return 0;
    }

    size_t got = 0;
    FRAME_RING_ENTER(r);

    uint32_t avail = fill_unsafe(r);
    if (frames > avail) {
        frames = avail;
    }

    while (got < frames) {
        uint32_t idx = r->tail & r->mask;
        size_t run = r->cap - idx;
        if (run > frames - got) {
            run = frames - got;
        }
        memcpy(&dst[got * 2], &r->buf[(size_t)idx * 2], run * 2 * sizeof(int16_t));
        r->tail += (uint32_t)run;
        got += run;
    }

    FRAME_RING_EXIT(r);
    return got;
}

void frame_ring_recentre(frame_ring_t *r, int fill_pct)
{
    if (fill_pct < 0) fill_pct = 0;
    if (fill_pct > 100) fill_pct = 100;

    FRAME_RING_ENTER(r);
    memset(r->buf, 0, (size_t)r->cap * 2 * sizeof(int16_t));
    r->tail = 0;
    r->head = (uint32_t)(((uint64_t)r->cap * (uint32_t)fill_pct) / 100u);
    FRAME_RING_EXIT(r);
}
