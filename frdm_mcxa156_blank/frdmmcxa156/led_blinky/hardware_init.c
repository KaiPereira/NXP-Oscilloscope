/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*${header:start}*/
#include "pin_mux.h"
#include "peripherals.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_port.h"
#include "board.h"
#include "app.h"
#include <stdbool.h>
/*${header:end}*/

/*${function:start}*/
static void InitOutputPin(PORT_Type *port, GPIO_Type *gpio, uint32_t pin, uint8_t initialLevel)
{
    const port_pin_config_t cfg = {kPORT_PullDisable,
                                   kPORT_LowPullResistor,
                                   kPORT_FastSlewRate,
                                   kPORT_PassiveFilterDisable,
                                   kPORT_OpenDrainDisable,
                                   kPORT_HighDriveStrength,
                                   kPORT_NormalDriveStrength,
                                   kPORT_MuxAlt0, /* GPIO */
                                   kPORT_InputBufferEnable,
                                   kPORT_InputNormal,
                                   kPORT_UnlockRegister};
    PORT_SetPinConfig(port, pin, &cfg);

    gpio_pin_config_t out = {kGPIO_DigitalOutput, initialLevel};
    GPIO_PinInit(gpio, pin, &out);
}

void BOARD_InitHardware(void)
{
    BOARD_InitPins();
    /* Run at 96 MHz: needed for NeoPixel bit timing and fast software PWM. */
    BOARD_BootClockFRO96M();
    BOARD_InitBootPeripherals();

    /* NeoPixel data pin lives on PORT2/GPIO2. */
    CLOCK_EnableClock(kCLOCK_GateGPIO2);
    CLOCK_EnableClock(kCLOCK_GatePORT2);
    RESET_ReleasePeripheralReset(kGPIO2_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT2_RST_SHIFT_RSTn);

    /* Onboard RGB LED is active-low: start all channels off (high). */
    InitOutputPin(PORT3, BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, LOGIC_LED_OFF);
    InitOutputPin(PORT3, BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, LOGIC_LED_OFF);
    InitOutputPin(PORT3, BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, LOGIC_LED_OFF);

    /* NeoPixel data line idles low. */
    InitOutputPin(NEOPIXEL_PORT, NEOPIXEL_GPIO, NEOPIXEL_PIN, 0U);
}
/*${function:end}*/
