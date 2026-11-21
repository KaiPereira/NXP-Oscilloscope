/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Rainbow mode: the onboard RGB LED cycles through the rainbow using software
 * PWM, and a WS2812B/NeoPixel strip on NEOPIXEL_PIN shows a fast rainbow chase.
 */

#include "board.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CORE_CLOCK_HZ 96000000U

/* Software PWM for the onboard LED: 256 steps at ~100 Hz => 25.6 kHz tick. */
#define PWM_TICK_HZ 25600U
#define TICKS_PER_MS (PWM_TICK_HZ / 1000U)

/* WS2812B bit timing in CPU cycles at 96 MHz (1 cycle ~= 10.4 ns). */
#define WS_T_PERIOD 120U /* 1.25 us per bit */
#define WS_T0H      38U  /* ~0.40 us high for a 0 */
#define WS_T1H      77U  /* ~0.80 us high for a 1 */

/* How fast the rainbow spins. Bigger = crazier. Hue wheel is 0..1535. */
#define HUE_STEP_PER_FRAME 24U
#define FRAME_MS           8U

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile uint8_t s_pwmRed, s_pwmGreen, s_pwmBlue;
static volatile uint32_t s_ticks;
static uint8_t s_pixels[NEOPIXEL_COUNT * 3U]; /* GRB order */
static bool s_haveCycleCounter;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    static uint8_t phase;

    s_ticks++;
    phase++;

    /* Active-low LED: drive low (on) while phase < duty. */
    GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, (phase < s_pwmRed) ? LOGIC_LED_ON : LOGIC_LED_OFF);
    GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN,
                  (phase < s_pwmGreen) ? LOGIC_LED_ON : LOGIC_LED_OFF);
    GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, (phase < s_pwmBlue) ? LOGIC_LED_ON : LOGIC_LED_OFF);
}

static void DelayMs(uint32_t ms)
{
    uint32_t start = s_ticks;
    while ((s_ticks - start) < ms * TICKS_PER_MS)
    {
    }
}

/* Fully saturated rainbow colour for hue 0..1535. */
static void HueToRgb(uint32_t hue, uint8_t *r, uint8_t *g, uint8_t *b)
{
    hue %= 1536U;
    uint8_t x = (uint8_t)(hue & 0xFFU);
    switch (hue >> 8)
    {
        case 0: *r = 255U;      *g = x;         *b = 0U;        break;
        case 1: *r = 255U - x;  *g = 255U;      *b = 0U;        break;
        case 2: *r = 0U;        *g = 255U;      *b = x;         break;
        case 3: *r = 0U;        *g = 255U - x;  *b = 255U;      break;
        case 4: *r = x;         *g = 0U;        *b = 255U;      break;
        default: *r = 255U;     *g = 0U;        *b = 255U - x;  break;
    }
}

static bool CycleCounterInit(void)
{
    DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return false;
    }
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return true;
}

/*
 * Bit-bang the WS2812B protocol. Timing is measured with the DWT cycle counter,
 * and this function is always compiled with optimisation so it holds timing even
 * in the -O0 debug build. Interrupts are off while the frame goes out.
 */
__attribute__((optimize("O2"))) static void NeoPixelShow(const uint8_t *data, uint32_t len)
{
    GPIO_Type *const gpio = NEOPIXEL_GPIO;
    const uint32_t mask   = 1UL << NEOPIXEL_PIN;

    uint32_t primask = DisableGlobalIRQ();
    uint32_t t       = DWT->CYCCNT;

    for (uint32_t i = 0U; i < len; i++)
    {
        uint8_t byte = data[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            uint32_t high = (byte & 0x80U) ? WS_T1H : WS_T0H;
            byte <<= 1;

            while ((DWT->CYCCNT - t) < WS_T_PERIOD)
            {
            }
            t          = DWT->CYCCNT;
            gpio->PSOR = mask;
            while ((DWT->CYCCNT - t) < high)
            {
            }
            gpio->PCOR = mask;
        }
    }

    EnableGlobalIRQ(primask);
    /* Latch/reset: line must stay low > 280 us. The frame delay covers it. */
}

/*!
 * @brief Main function
 */
int main(void)
{
    BOARD_InitHardware();

    /* Replace the 1 s blink tick with the fast PWM tick. */
    SysTick_Config(CORE_CLOCK_HZ / PWM_TICK_HZ);

    s_haveCycleCounter = CycleCounterInit();

    uint32_t hue = 0U;
    while (1)
    {
        uint8_t r, g, b;

        /* Onboard LED. */
        HueToRgb(hue, &r, &g, &b);
        s_pwmRed   = r;
        s_pwmGreen = g;
        s_pwmBlue  = b;

        /* Strip: a full rainbow spread along its length, spinning fast. */
        if (s_haveCycleCounter)
        {
            for (uint32_t i = 0U; i < NEOPIXEL_COUNT; i++)
            {
                HueToRgb(hue + (i * 1536U) / NEOPIXEL_COUNT, &r, &g, &b);
                s_pixels[i * 3U + 0U] = (uint8_t)((g * NEOPIXEL_BRIGHTNESS) / 255U);
                s_pixels[i * 3U + 1U] = (uint8_t)((r * NEOPIXEL_BRIGHTNESS) / 255U);
                s_pixels[i * 3U + 2U] = (uint8_t)((b * NEOPIXEL_BRIGHTNESS) / 255U);
            }
            NeoPixelShow(s_pixels, sizeof(s_pixels));
        }

        hue = (hue + HUE_STEP_PER_FRAME) % 1536U;
        DelayMs(FRAME_MS);
    }
}
