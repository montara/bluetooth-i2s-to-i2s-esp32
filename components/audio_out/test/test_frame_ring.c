/*
 * Host-side tests for the elastic buffer's index arithmetic.
 *
 * Runs on a PC, no ESP32 and no ESP-IDF required:
 *
 *   cd components/audio_out/test && ./run_tests.sh
 *
 * The point is the wrap and fullness edges. Those are where a ring buffer
 * actually goes wrong, they are invisible in normal operation, and on hardware
 * they would present as a rare click that is nearly impossible to bisect.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "frame_ring.h"

#define CAP 8

static int g_failures;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (!(cond)) {                                          \
            printf("  FAIL %s:%d: ", __func__, __LINE__);       \
            printf(__VA_ARGS__);                                \
            printf("\n");                                       \
            g_failures++;                                       \
        }                                                       \
    } while (0)

/* Frame n is the sample pair {n, -n}, so a reader can tell exactly which frames
 * it got and in what order. */
static void fill_pattern(int16_t *dst, int start, int frames)
{
    for (int i = 0; i < frames; i++) {
        dst[i * 2]     = (int16_t)(start + i);
        dst[i * 2 + 1] = (int16_t)(-(start + i));
    }
}

static int check_pattern(const int16_t *src, int start, int frames)
{
    for (int i = 0; i < frames; i++) {
        if (src[i * 2] != (int16_t)(start + i) || src[i * 2 + 1] != (int16_t)(-(start + i))) {
            return i;   /* index of first mismatch */
        }
    }
    return -1;
}

/* ------------------------------------------------------------------ */

static void test_init_rejects_bad_capacity(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2];

    CHECK(!frame_ring_init(&r, storage, 6), "capacity 6 is not a power of two");
    CHECK(!frame_ring_init(&r, storage, 0), "capacity 0 must be rejected");
    CHECK(!frame_ring_init(&r, NULL, CAP), "NULL storage must be rejected");
    CHECK(frame_ring_init(&r, storage, CAP), "capacity %d should be accepted", CAP);
}

static void test_empty_reads_nothing(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], out[CAP * 2];
    frame_ring_init(&r, storage, CAP);

    CHECK(frame_ring_fill(&r) == 0, "fresh ring should be empty");
    CHECK(frame_ring_read(&r, out, 4) == 0, "read from empty must return 0");
}

static void test_write_read_roundtrip(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], in[CAP * 2], out[CAP * 2];
    frame_ring_init(&r, storage, CAP);

    fill_pattern(in, 100, 5);
    CHECK(frame_ring_write(&r, in, 5) == 5, "should accept 5 frames");
    CHECK(frame_ring_fill(&r) == 5, "fill should be 5, got %u", frame_ring_fill(&r));

    CHECK(frame_ring_read(&r, out, 5) == 5, "should read back 5 frames");
    CHECK(check_pattern(out, 100, 5) < 0, "data mismatch at frame %d",
          check_pattern(out, 100, 5));
    CHECK(frame_ring_fill(&r) == 0, "should be empty again");
}

static void test_fills_to_exact_capacity(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], in[CAP * 2];
    frame_ring_init(&r, storage, CAP);

    fill_pattern(in, 0, CAP);
    /* The whole point of free-running counters: all CAP slots are usable, not
     * CAP-1 as in the classic "leave one empty to disambiguate" ring. */
    CHECK(frame_ring_write(&r, in, CAP) == (size_t)CAP,
          "should accept all %d frames", CAP);
    CHECK(frame_ring_fill(&r) == CAP, "fill should be %d", CAP);
    CHECK(frame_ring_fill_pct(&r) == 100, "should report 100%%");
}

static void test_overflow_is_partial_and_counted(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], in[(CAP + 4) * 2];
    frame_ring_init(&r, storage, CAP);

    fill_pattern(in, 0, CAP + 4);
    size_t n = frame_ring_write(&r, in, CAP + 4);
    CHECK(n == (size_t)CAP, "should accept only %d of %d frames, took %zu", CAP, CAP + 4, n);
    CHECK(r.dropped == 4, "should have counted 4 dropped, got %u", r.dropped);

    /* A full ring must reject further writes rather than silently overwriting
     * unread data -- dropping the newest is recoverable, corrupting the oldest
     * is a loud click. */
    size_t again = frame_ring_write(&r, in, 2);
    CHECK(again == 0, "write to full ring should take nothing, took %zu", again);
    CHECK(r.dropped == 6, "dropped should total 6, got %u", r.dropped);
}

static void test_wraps_correctly(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], in[CAP * 2], out[CAP * 2];
    frame_ring_init(&r, storage, CAP);

    /* Advance head and tail most of the way round, so the next write straddles
     * the end of the backing array. */
    fill_pattern(in, 0, 6);
    frame_ring_write(&r, in, 6);
    frame_ring_read(&r, out, 6);
    CHECK(frame_ring_fill(&r) == 0, "should be drained");

    /* head and tail are both at 6; writing 5 wraps at index 8. */
    fill_pattern(in, 200, 5);
    CHECK(frame_ring_write(&r, in, 5) == 5, "wrapped write should accept 5");
    CHECK(frame_ring_read(&r, out, 5) == 5, "wrapped read should return 5");
    int bad = check_pattern(out, 200, 5);
    CHECK(bad < 0, "wrapped data mismatch at frame %d", bad);
}

static void test_partial_read(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], in[CAP * 2], out[CAP * 2];
    frame_ring_init(&r, storage, CAP);

    fill_pattern(in, 50, 3);
    frame_ring_write(&r, in, 3);

    /* Asking for more than is queued yields what there is -- this is the
     * underrun path, where the pump pads the rest with silence. */
    size_t got = frame_ring_read(&r, out, CAP);
    CHECK(got == 3, "should return the 3 available, got %zu", got);
    CHECK(check_pattern(out, 50, 3) < 0, "partial read data mismatch");
}

static void test_counter_wraparound(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], in[CAP * 2], out[CAP * 2];
    frame_ring_init(&r, storage, CAP);

    /* Park the counters just below the 32-bit rollover. At 48 kHz the head
     * counter rolls over after about 25 hours of continuous playback, so this
     * is a real scenario for a device left on, not a theoretical one. */
    r.head = 0xFFFFFFFCu;
    r.tail = 0xFFFFFFFCu;

    fill_pattern(in, 7, 6);
    CHECK(frame_ring_write(&r, in, 6) == 6, "write across counter rollover");
    CHECK(frame_ring_fill(&r) == 6, "fill across rollover should be 6, got %u",
          frame_ring_fill(&r));
    CHECK(r.head == 2u, "head should have wrapped to 2, got %u", r.head);

    CHECK(frame_ring_read(&r, out, 6) == 6, "read across counter rollover");
    int bad = check_pattern(out, 7, 6);
    CHECK(bad < 0, "rollover data mismatch at frame %d", bad);
    CHECK(frame_ring_fill(&r) == 0, "should be empty after rollover drain");
}

static void test_recentre(void)
{
    frame_ring_t r;
    int16_t storage[CAP * 2], in[CAP * 2], out[CAP * 2];
    frame_ring_init(&r, storage, CAP);

    fill_pattern(in, 1, CAP);
    frame_ring_write(&r, in, CAP);

    frame_ring_recentre(&r, 50);
    CHECK(frame_ring_fill(&r) == CAP / 2, "should hold %d frames after recentre, got %u",
          CAP / 2, frame_ring_fill(&r));
    CHECK(frame_ring_fill_pct(&r) == 50, "should report 50%%, got %d",
          frame_ring_fill_pct(&r));

    /* What it holds must be silence, not stale audio -- re-centring after a
     * source switch exists precisely to stop the old source leaking through. */
    frame_ring_read(&r, out, CAP / 2);
    for (int i = 0; i < CAP; i++) {
        CHECK(out[i] == 0, "recentred buffer should be silent at sample %d", i);
    }

    frame_ring_recentre(&r, 150);
    CHECK(frame_ring_fill_pct(&r) == 100, "out-of-range pct should clamp to 100");
    frame_ring_recentre(&r, -10);
    CHECK(frame_ring_fill_pct(&r) == 0, "negative pct should clamp to 0");
}

static void test_streaming_churn(void)
{
    /* Long unbalanced run: writes and reads of mismatched sizes, exercising
     * every wrap alignment rather than the tidy ones the other tests hit. */
    frame_ring_t r;
    int16_t storage[64 * 2], in[64 * 2], out[64 * 2];
    frame_ring_init(&r, storage, 64);

    int next_write = 0;
    int next_read = 0;

    for (int iter = 0; iter < 2000; iter++) {
        int w = 1 + (iter * 7) % 13;
        fill_pattern(in, next_write, w);
        size_t took = frame_ring_write(&r, in, w);
        next_write += (int)took;

        int rd = 1 + (iter * 5) % 11;
        size_t got = frame_ring_read(&r, out, rd);
        if (check_pattern(out, next_read, (int)got) >= 0) {
            CHECK(0, "churn: out-of-order data at iteration %d", iter);
            return;
        }
        next_read += (int)got;

        if (frame_ring_fill(&r) > 64) {
            CHECK(0, "churn: fill exceeded capacity at iteration %d", iter);
            return;
        }
    }
    printf("  (churn: wrote %d, read %d frames)\n", next_write, next_read);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    struct { const char *name; void (*fn)(void); } tests[] = {
        { "init rejects bad capacity",     test_init_rejects_bad_capacity   },
        { "empty ring reads nothing",      test_empty_reads_nothing         },
        { "write/read roundtrip",          test_write_read_roundtrip        },
        { "fills to exact capacity",       test_fills_to_exact_capacity     },
        { "overflow is partial + counted", test_overflow_is_partial_and_counted },
        { "wraps correctly",               test_wraps_correctly             },
        { "partial read",                  test_partial_read                },
        { "counter wraparound",            test_counter_wraparound          },
        { "recentre",                      test_recentre                    },
        { "streaming churn",               test_streaming_churn             },
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = g_failures;
        printf("%-34s", tests[i].name);
        fflush(stdout);
        printf("\n");
        tests[i].fn();
        if (g_failures == before) {
            printf("  ok\n");
        }
    }

    if (g_failures) {
        printf("\n%d check(s) FAILED\n", g_failures);
        return 1;
    }
    printf("\nall tests passed\n");
    return 0;
}
