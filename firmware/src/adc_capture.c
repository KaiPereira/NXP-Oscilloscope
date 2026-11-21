/*
 * MCXScope firmware: ADC sampling hardware. See adc_capture.h for the signal chain.
 *
 * ADC and DMA set-up follows the MCUXpresso SDK lpadc_edma example (Copyright NXP).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_port.h"
#include "fsl_lpadc.h"
#include "fsl_edma.h"
#include "fsl_ctimer.h"
#include "fsl_inputmux.h"

#include "adc_capture.h"

#define TIMER_CLOCK_HZ 96000000U /* FRO_HF */
#define ADC_CLOCK_HZ   48000000U /* FRO_HF / 2 */
#define ADC_CMD_ID     1U
#define RING_MAX       32000U    /* eDMA major loop count is 15 bits */

typedef struct
{
    ADC_Type *adc;
    uint32_t adc_channel;
    uint32_t dma_channel;
    dma_request_source_t dma_request;
    PORT_Type *port;
    uint32_t pin;
} chan_hw_t;

static const chan_hw_t s_hw[NUM_CHANNELS] = {
    /* CH1: J4-2, P1_10 = ADC1_A8 */
    {ADC1, 8U, 0U, kDma0RequestMuxAdc1FifoRequest, PORT1, 10U},
    /* CH2: J2-7, P2_1 = ADC0_A1 */
    {ADC0, 1U, 1U, kDma0RequestMuxAdc0FifoRequest, PORT2, 1U},
};

/* All sample memory. Each captured channel gets an equal slice as its ring. */
static int16_t s_buf[SAMPLE_BUF_TOTAL];

static const int16_t *s_ring[NUM_CHANNELS];
static uint32_t s_ringLen;
static uint16_t s_mask;
static unsigned s_countCh;

/* position tracking, maintained by adc_capture_tick() and adc_capture_count() */
static volatile uint64_t s_count;
static uint32_t s_wraps;
static uint32_t s_lastPos;

static volatile bool s_running;
static volatile uint64_t s_stopAt;

/* ------------------------------------------------------------------------------------ */

static void adc_setup(ADC_Type *adc, uint32_t channel)
{
    lpadc_config_t cfg;
    LPADC_GetDefaultConfig(&cfg);
    cfg.enableAnalogPreliminary = true;
    cfg.powerUpDelay = 0x10U;
    cfg.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3; /* VDDA = 3.3 V */
    cfg.powerLevelMode = kLPADC_PowerLevelAlt4;               /* highest power, fastest */
    cfg.FIFOWatermark = 0U;                                   /* DMA request per sample */
    LPADC_Init(adc, &cfg);

#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFS) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFS
    LPADC_DoOffsetCalibration(adc);
#endif
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CAL_REQ) && FSL_FEATURE_LPADC_HAS_CTRL_CAL_REQ
    SDK_DelayAtLeastUs(1U, SystemCoreClock);
    LPADC_DoAutoCalibration(adc);
#endif

    lpadc_conv_command_config_t cmd;
    LPADC_GetDefaultConvCommandConfig(&cmd);
    cmd.channelNumber = channel;
    cmd.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    cmd.conversionResolutionMode = kLPADC_ConversionResolutionStandard; /* 12-bit */
    cmd.sampleTimeMode = kLPADC_SampleTimeADCK3;                        /* shortest */
    cmd.hardwareAverageMode = kLPADC_HardwareAverageCount1;
    LPADC_SetConvCommandConfig(adc, ADC_CMD_ID, &cmd);

    lpadc_conv_trigger_config_t trig;
    LPADC_GetDefaultConvTriggerConfig(&trig);
    trig.targetCommandId = ADC_CMD_ID;
    trig.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(adc, 0U, &trig);

    LPADC_EnableFIFOWatermarkDMA(adc, true);
}

static void analog_pin(PORT_Type *port, uint32_t pin)
{
    const port_pin_config_t cfg = {
        kPORT_PullDisable,         kPORT_LowPullResistor,    kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,  kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength, kPORT_MuxAlt0,            kPORT_InputBufferDisable,
        kPORT_InputNormal,         kPORT_UnlockRegister,
    };
    PORT_SetPinConfig(port, pin, &cfg);
}

void adc_capture_init(void)
{
    /* analog pins */
    CLOCK_EnableClock(kCLOCK_GatePORT1);
    CLOCK_EnableClock(kCLOCK_GatePORT2);
    RESET_ReleasePeripheralReset(kPORT1_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT2_RST_SHIFT_RSTn);
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        analog_pin(s_hw[i].port, s_hw[i].pin);
    }

    /* ADCs: FRO_HF (96 MHz) / 2 = 48 MHz conversion clock */
    CLOCK_AttachClk(kFRO_HF_to_ADC0);
    CLOCK_SetClockDiv(kCLOCK_DivADC0, 2U);
    CLOCK_AttachClk(kFRO_HF_to_ADC1);
    CLOCK_SetClockDiv(kCLOCK_DivADC1, 2U);
    RESET_ReleasePeripheralReset(kADC0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kADC1_RST_SHIFT_RSTn);
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        adc_setup(s_hw[i].adc, s_hw[i].adc_channel);
    }

    /* CTIMER0 match 0 triggers both ADCs */
    RESET_ReleasePeripheralReset(kINPUTMUX0_RST_SHIFT_RSTn);
    CLOCK_EnableClock(kCLOCK_GateINPUTMUX0);
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Ctimer0M0ToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0, 0U, kINPUTMUX_Ctimer0M0ToAdc1Trigger);

    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);
    CLOCK_SetClockDiv(kCLOCK_DivCTIMER0, 1U);
    ctimer_config_t tcfg;
    CTIMER_GetDefaultConfig(&tcfg);
    CTIMER_Init(CTIMER0, &tcfg);

    /* DMA */
    edma_config_t dcfg;
    EDMA_GetDefaultConfig(&dcfg);
    EDMA_Init(DMA0, &dcfg);
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        edma_channel_config_t ccfg;
        memset(&ccfg, 0, sizeof(ccfg));
        ccfg.channelRequestSource = s_hw[i].dma_request;
        ccfg.protectionLevel = kEDMA_ChannelProtectionLevelUser;
        ccfg.channelPreemptionConfig.enablePreemptAbility = true;
#if !(defined(FSL_FEATURE_EDMA_HAS_NO_CH_SBR_SEC) && FSL_FEATURE_EDMA_HAS_NO_CH_SBR_SEC)
        ccfg.securityLevel = kEDMA_ChannelSecurityLevelNonSecure;
#endif
        EDMA_InitChannel(DMA0, s_hw[i].dma_channel, &ccfg);
    }
}

static uint32_t dma_pos(void)
{
    uint32_t ch = s_hw[s_countCh].dma_channel;
    uint32_t citer = EDMA_TCD_CITER(EDMA_TCD_BASE(DMA0, ch), EDMA_TCD_TYPE(DMA0)) & DMA_CITER_ELINKNO_CITER_MASK;
    return s_ringLen - citer; /* CITER counts down from s_ringLen and reloads on wrap */
}

/* Call with interrupts disabled, at least once per ring period. */
static void update(void)
{
    uint32_t pos = dma_pos();
    if (pos < s_lastPos)
    {
        s_wraps++;
    }
    s_lastPos = pos;
    s_count = (uint64_t)s_wraps * s_ringLen + pos;
}

void adc_capture_stop(void)
{
    CTIMER_StopTimer(CTIMER0);
    s_running = false;
}

/*
 * Longest sampling window that still fits the sample period. A longer window lets the
 * ADC's sampling capacitor settle from higher source impedances (more accurate readings),
 * but adds to the conversion time. Measured: a conversion with the shortest window
 * (3 ADCK) takes ~24 ADCK, so the rest of the conversion is ~21 ADCK.
 */
static uint32_t sample_time_for(uint32_t rate)
{
    static const uint8_t cycles[] = {3, 5, 7, 11, 19, 35, 67, 131}; /* CMDH.STS = index */
    const uint32_t overhead = 21U;
    uint32_t period = ADC_CLOCK_HZ / rate;
    uint32_t budget = period - period / 8U; /* keep ~12 % margin */
    uint32_t sts = 0;
    for (uint32_t i = 0; i < sizeof(cycles); i++)
    {
        if (cycles[i] + overhead <= budget)
        {
            sts = i;
        }
    }
    return sts;
}

void adc_capture_start(uint32_t rate, uint16_t mask, unsigned count_ch)
{
    adc_capture_stop();

    unsigned n = 0;
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        n += (mask >> i) & 1U;
        EDMA_DisableChannelRequest(DMA0, s_hw[i].dma_channel);
        s_ring[i] = NULL;
    }
    if (n == 0U)
    {
        return;
    }
    s_ringLen = SAMPLE_BUF_TOTAL / n;
    if (s_ringLen > RING_MAX)
    {
        s_ringLen = RING_MAX;
    }
    s_mask = mask;
    s_countCh = count_ch;

    unsigned slot = 0;
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        if (((mask >> i) & 1U) == 0U)
        {
            continue;
        }
        const chan_hw_t *h = &s_hw[i];
        int16_t *ring = &s_buf[slot++ * s_ringLen];
        s_ring[i] = ring;

        h->adc->CMD[ADC_CMD_ID - 1U].CMDH =
            (h->adc->CMD[ADC_CMD_ID - 1U].CMDH & ~ADC_CMDH_STS_MASK) | ADC_CMDH_STS(sample_time_for(rate));
        LPADC_DoResetFIFO(h->adc);
        LPADC_ClearStatusFlags(h->adc, kLPADC_ResultFIFOOverflowFlag);

        /* 16-bit reads of RESFIFO[15:0] (the conversion result), 16-bit writes into the
         * ring; the destination wraps back to the start after every major loop */
        edma_transfer_config_t t;
        EDMA_PrepareTransfer(&t, (void *)&h->adc->RESFIFO, sizeof(int16_t), ring, sizeof(int16_t),
                             sizeof(int16_t), s_ringLen * sizeof(int16_t), kEDMA_PeripheralToMemory);
        t.dstMajorLoopOffset = -(int32_t)(s_ringLen * sizeof(int16_t));
        EDMA_SetTransferConfig(DMA0, h->dma_channel, &t, NULL);
        EDMA_EnableAutoStopRequest(DMA0, h->dma_channel, false); /* run forever */
        EDMA_ClearChannelStatusFlags(DMA0, h->dma_channel, kEDMA_DoneFlag | kEDMA_InterruptFlag);
        EDMA_EnableChannelRequest(DMA0, h->dma_channel);
    }

    uint32_t primask = DisableGlobalIRQ();
    s_wraps = 0;
    s_lastPos = 0;
    s_count = 0;
    s_stopAt = UINT64_MAX;
    EnableGlobalIRQ(primask);

    /* Match output toggles on every match, and the ADCs trigger on its rising edge, so
     * the match period is half the sample period. */
    ctimer_match_config_t m = {
        .matchValue = TIMER_CLOCK_HZ / (2U * rate) - 1U,
        .enableCounterReset = true,
        .enableCounterStop = false,
        .outControl = kCTIMER_Output_Toggle,
        .outPinInitState = false,
        .enableInterrupt = false,
    };
    CTIMER_Reset(CTIMER0);
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &m);
    s_running = true;
    CTIMER_StartTimer(CTIMER0);
}

void adc_capture_stop_at(uint64_t count)
{
    s_stopAt = count;
}

/*
 * Self-test signal: alternate each input pin's internal pull resistor between up and down,
 * giving a 0..3.3 V square wave (with RC edges from the ~50 kOhm pull and the pin
 * capacitance) on unconnected inputs. CH1 toggles at 1 kHz, CH2 at 500 Hz.
 */
static volatile bool s_testSig;
static uint32_t s_testTicks;

static void set_pull(unsigned ch, int dir) /* dir: 0 off, 1 up, -1 down */
{
    PORT_Type *port = s_hw[ch].port;
    uint32_t pcr = port->PCR[s_hw[ch].pin] & ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
    if (dir != 0)
    {
        pcr |= PORT_PCR_PE_MASK | ((dir > 0) ? PORT_PCR_PS_MASK : 0U);
    }
    port->PCR[s_hw[ch].pin] = pcr;
}

void adc_capture_test_signal(bool on)
{
    s_testSig = on;
    if (!on)
    {
        for (unsigned i = 0; i < NUM_CHANNELS; i++)
        {
            set_pull(i, 0);
        }
    }
}

bool adc_capture_test_signal_on(void)
{
    return s_testSig;
}

void adc_capture_tick(void)
{
    if (s_testSig)
    {
        /* tick is 10 kHz: CH1 period 10 ticks (1 kHz), CH2 period 20 ticks (500 Hz) */
        s_testTicks++;
        set_pull(0, ((s_testTicks / 5U) & 1U) ? 1 : -1);
        set_pull(1, ((s_testTicks / 10U) & 1U) ? 1 : -1);
    }
    if (!s_running)
    {
        return;
    }
    update();
    if (s_count >= s_stopAt)
    {
        adc_capture_stop();
    }
}

bool adc_capture_stopped(void)
{
    if (s_running)
    {
        return false;
    }
    /* let the conversion in flight (< 1 us) reach the ring */
    SDK_DelayAtLeastUs(5U, SystemCoreClock);
    uint32_t primask = DisableGlobalIRQ();
    update();
    EnableGlobalIRQ(primask);
    return true;
}

uint64_t adc_capture_count(void)
{
    uint32_t primask = DisableGlobalIRQ();
    if (s_running)
    {
        update();
    }
    uint64_t c = s_count;
    EnableGlobalIRQ(primask);
    return c;
}

uint32_t adc_capture_ring_len(void)
{
    return s_ringLen;
}

const int16_t *adc_capture_ring(unsigned ch)
{
    return (ch < NUM_CHANNELS) ? s_ring[ch] : NULL;
}

bool adc_capture_overrun(void)
{
    for (unsigned i = 0; i < NUM_CHANNELS; i++)
    {
        if (((s_mask >> i) & 1U) && (LPADC_GetStatusFlags(s_hw[i].adc) & kLPADC_ResultFIFOOverflowFlag))
        {
            return true;
        }
    }
    return false;
}
