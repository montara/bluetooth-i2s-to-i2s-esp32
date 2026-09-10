#include "rate_table.h"

const uint32_t rate_table_standard[] = {
    32000, 44100, 48000, 88200, 96000, 176400, 192000,
};
const size_t rate_table_standard_count =
    sizeof(rate_table_standard) / sizeof(rate_table_standard[0]);

uint32_t rate_table_snap(uint32_t measured_hz)
{
    for (size_t i = 0; i < rate_table_standard_count; i++) {
        uint32_t std = rate_table_standard[i];
        uint32_t tol = (std * RATE_TABLE_TOLERANCE_PCT) / 100;
        /* Written as `measured + tol >= std` rather than `measured >= std - tol`
         * so a small measurement near zero cannot underflow the unsigned
         * subtraction and wrap into a match. */
        if (measured_hz + tol >= std && measured_hz <= std + tol) {
            return std;
        }
    }
    return 0;
}
