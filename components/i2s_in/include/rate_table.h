/*
 * Snapping a measured WS frequency to a real sample rate.
 *
 * The measurement is a raw edge count over a timed window, so it arrives with
 * a percent or so of noise on it. Rather than pass that straight to the I2S
 * driver, we snap it to the nearest standard rate -- and reject anything that
 * is not near one at all, which is how a half-connected cable or a source
 * mid-startup gets caught instead of being believed.
 *
 * No ESP-IDF dependencies, so it can be tested on a host. See
 * test/test_rate_table.c.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The rates we are willing to recognise, ascending. */
extern const uint32_t rate_table_standard[];
extern const size_t rate_table_standard_count;

/** Tolerance band around each standard rate, as a percentage. */
#define RATE_TABLE_TOLERANCE_PCT 3

/**
 * @return the standard rate `measured_hz` falls within tolerance of, or 0 if
 *         it does not match any of them.
 */
uint32_t rate_table_snap(uint32_t measured_hz);

#ifdef __cplusplus
}
#endif
