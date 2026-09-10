/*
 * Serial console for bring-up.
 *
 * Small, but the difference between debugging this on a bench with a scope and
 * debugging it by reflashing. `stats` in particular is how you tell whether the
 * drift servo is working: a fill percentage parked near 50 with a steady MCLK
 * offset is a healthy loop; one climbing towards 0 or 100 is not.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register commands and start the UART REPL. */
esp_err_t console_cmds_init(void);

#ifdef __cplusplus
}
#endif
