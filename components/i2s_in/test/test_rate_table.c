/*
 * Host tests for sample-rate snapping.
 *
 *   cd components/i2s_in/test && ./run_tests.sh
 *
 * The interesting property is not "48000 maps to 48000" -- it is that the
 * tolerance bands around adjacent standard rates do not overlap. If they ever
 * did, a measurement in the overlap would snap to whichever entry came first in
 * the table, and the DAC would be clocked at the wrong rate with no error
 * anywhere to show for it.
 */
#include <stdio.h>
#include <stdint.h>

#include "rate_table.h"

static int g_failures;

#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (!(cond)) {                                      \
            printf("  FAIL %s:%d: ", __func__, __LINE__);   \
            printf(__VA_ARGS__);                            \
            printf("\n");                                   \
            g_failures++;                                   \
        }                                                   \
    } while (0)

static void test_exact_rates_snap_to_themselves(void)
{
    for (size_t i = 0; i < rate_table_standard_count; i++) {
        uint32_t hz = rate_table_standard[i];
        CHECK(rate_table_snap(hz) == hz, "%u should snap to itself, got %u",
              hz, rate_table_snap(hz));
    }
}

static void test_realistic_measurement_noise(void)
{
    /* A 100 ms window at 44.1 kHz counts 4410 edges; being one or two out is
     * routine, and so is a fraction of a percent of timing jitter. */
    CHECK(rate_table_snap(44090) == 44100, "44090 should snap to 44100");
    CHECK(rate_table_snap(44110) == 44100, "44110 should snap to 44100");
    CHECK(rate_table_snap(47950) == 48000, "47950 should snap to 48000");
    CHECK(rate_table_snap(48050) == 48000, "48050 should snap to 48000");
    CHECK(rate_table_snap(95500) == 96000, "95500 should snap to 96000");
}

static void test_band_edges(void)
{
    /* 3 % of 48000 is 1440. */
    CHECK(rate_table_snap(48000 + 1440) == 48000, "upper edge should still match");
    CHECK(rate_table_snap(48000 - 1440) == 48000, "lower edge should still match");
    CHECK(rate_table_snap(48000 + 1441) != 48000, "just past the upper edge should not match 48000");
}

static void test_bands_do_not_overlap(void)
{
    /* The real invariant. Walk every adjacent pair and confirm the upper edge
     * of the lower band sits below the lower edge of the upper band. */
    for (size_t i = 0; i + 1 < rate_table_standard_count; i++) {
        uint32_t lo = rate_table_standard[i];
        uint32_t hi = rate_table_standard[i + 1];
        uint32_t lo_top = lo + (lo * RATE_TABLE_TOLERANCE_PCT) / 100;
        uint32_t hi_bot = hi - (hi * RATE_TABLE_TOLERANCE_PCT) / 100;
        CHECK(lo_top < hi_bot,
              "bands for %u and %u overlap (%u >= %u) -- a measurement between "
              "them would snap ambiguously", lo, hi, lo_top, hi_bot);
    }
}

static void test_nonsense_is_rejected(void)
{
    CHECK(rate_table_snap(0) == 0, "zero must not match anything");
    CHECK(rate_table_snap(1) == 0, "1 Hz must not match anything");
    CHECK(rate_table_snap(20000) == 0, "20 kHz is not a standard rate");
    CHECK(rate_table_snap(40000) == 0, "40 kHz sits between bands and must be rejected");
    CHECK(rate_table_snap(64000) == 0, "64 kHz is not in the table");
    CHECK(rate_table_snap(400000) == 0, "400 kHz is beyond the table");
    CHECK(rate_table_snap(0xFFFFFFFFu) == 0, "UINT32_MAX must not wrap into a match");
}

static void test_no_gaps_swallow_valid_input(void)
{
    /* Sweep the whole plausible range and confirm every value either snaps to
     * exactly one rate or to none -- never silently to the wrong one. */
    for (uint32_t hz = 1000; hz <= 200000; hz += 7) {
        uint32_t snapped = rate_table_snap(hz);
        if (snapped == 0) {
            continue;
        }
        uint32_t tol = (snapped * RATE_TABLE_TOLERANCE_PCT) / 100;
        if (hz + tol < snapped || hz > snapped + tol) {
            CHECK(0, "%u snapped to %u but is outside its tolerance band", hz, snapped);
            return;
        }
    }
}

int main(void)
{
    struct { const char *name; void (*fn)(void); } tests[] = {
        { "exact rates snap to themselves", test_exact_rates_snap_to_themselves },
        { "realistic measurement noise",    test_realistic_measurement_noise    },
        { "band edges",                     test_band_edges                     },
        { "bands do not overlap",           test_bands_do_not_overlap           },
        { "nonsense is rejected",           test_nonsense_is_rejected           },
        { "sweep finds no mis-snaps",       test_no_gaps_swallow_valid_input    },
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = g_failures;
        printf("%s\n", tests[i].name);
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
