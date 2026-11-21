/*
 * MCXScope firmware for the FRDM-MCXA156.
 *
 * Implements PROTOCOL.md (SCPI over USB CDC-ACM on the MCU's own USB port).
 * The debug UART (MCU-Link VCOM) is left free for diagnostics; nothing is ever sent
 * unsolicited on the CDC port.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"

#include "scope.h"
#include "scpi.h"
#include "timebase.h"
#include "usb_cdc.h"

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    timebase_init();
    scpi_init();
    scope_init();
    cdc_init();

    uint8_t buf[64];
    for (;;)
    {
        size_t n = cdc_read(buf, sizeof(buf));
        if (n > 0U)
        {
            scpi_input(buf, n);
        }
        scope_poll();
    }
}
