/*
 * MCXScope firmware: USB CDC-ACM byte stream.
 *
 * Based on the MCUXpresso SDK usb_device_cdc_vcom (bare metal) example,
 * Copyright 2015-2016 Freescale Semiconductor, Inc. and 2016-2017, 2023 NXP.
 *
 * Differences from the example, as required by PROTOCOL.md §1:
 *   - no DTR gate: data flows as soon as the host has configured the device;
 *   - the OUT endpoint is only re-armed once the previous packet has been consumed,
 *     so a busy device NAKs instead of dropping bytes;
 *   - nothing is ever sent unless the application asks for it.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "fsl_device_registers.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_reset.h"

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "timebase.h"
#include "usb_cdc.h"

#define CONTROLLER_ID      kUSB_ControllerKhci0
#define USB_IRQ_PRIORITY   3U
#define PKT_SIZE           FS_CDC_VCOM_BULK_OUT_PACKET_SIZE
#define TX_BUF_SIZE        1024U /* multiple of the 64-byte packet size */
#define TX_TIMEOUT_MS      1000U /* host stopped reading: stall and drop the rest of the response */

extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;

static usb_status_t cdc_class_cb(class_handle_t handle, uint32_t event, void *param);
static usb_status_t cdc_device_cb(usb_device_handle handle, uint32_t event, void *param);

static usb_device_handle s_device;
static class_handle_t s_cdc;
static volatile bool s_attached;

/* ---- receive: a single packet buffer, re-armed only after it has been consumed ---- */
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_rxPkt[PKT_SIZE];
static volatile uint32_t s_rxLen; /* bytes in s_rxPkt, 0 = receive armed (or not attached) */
static uint32_t s_rxPos;

/* ---- transmit: two staging buffers so one fills while the other is on the wire ---- */
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_txBuf[2][TX_BUF_SIZE];
static uint32_t s_txFill;   /* bytes queued in s_txBuf[s_txCur] */
static uint32_t s_txCur;
static volatile bool s_txBusy;
static volatile bool s_txZlp; /* send a zero-length packet after the current transfer */
static bool s_txDrop;         /* discard output until the next flush (host stalled) */

/* CDC line state: accepted and echoed back, but otherwise ignored (§1). */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_lineCoding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08}; /* 115200 8N1 */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_abstractState[2];
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_countryCode[2];

static usb_device_class_config_struct_t s_classConfig[1] = {{cdc_class_cb, 0, &g_UsbDeviceCdcVcomConfig}};
static usb_device_class_config_list_struct_t s_classConfigList = {s_classConfig, cdc_device_cb, 1};

/* ------------------------------------------------------------------------------------ */

void USB0_IRQHandler(void)
{
    USB_DeviceKhciIsrFunction(s_device);
}

static void rx_arm(void)
{
    s_rxPos = 0;
    s_rxLen = 0;
    USB_DeviceCdcAcmRecv(s_cdc, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_rxPkt, PKT_SIZE);
}

static usb_status_t cdc_class_cb(class_handle_t handle, uint32_t event, void *param)
{
    usb_device_cdc_acm_request_param_struct_t *req = (usb_device_cdc_acm_request_param_struct_t *)param;
    usb_device_endpoint_callback_message_struct_t *ep = (usb_device_endpoint_callback_message_struct_t *)param;

    switch (event)
    {
        case kUSB_DeviceCdcEventSendResponse:
            if (ep->length != USB_CANCELLED_TRANSFER_LENGTH && ep->length != 0U &&
                (ep->length % FS_CDC_VCOM_BULK_IN_PACKET_SIZE) == 0U && s_txZlp)
            {
                /* the transfer ended on a packet boundary: a ZLP tells the host it is complete */
                s_txZlp = false;
                if (USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0) == kStatus_USB_Success)
                {
                    return kStatus_USB_Success;
                }
            }
            s_txBusy = false;
            return kStatus_USB_Success;

        case kUSB_DeviceCdcEventRecvResponse:
            if (ep->length == USB_CANCELLED_TRANSFER_LENGTH)
            {
                return kStatus_USB_Success;
            }
            if (ep->length == 0U)
            {
                rx_arm(); /* empty packet: nothing to hand to the application */
            }
            else
            {
                s_rxPos = 0;
                s_rxLen = ep->length; /* consumed by cdc_read(), which re-arms */
            }
            return kStatus_USB_Success;

        case kUSB_DeviceCdcEventSerialStateNotif:
            ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 0;
            return kStatus_USB_Success;

        case kUSB_DeviceCdcEventSetCommFeature:
        case kUSB_DeviceCdcEventGetCommFeature:
            if (req->setupValue == USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE)
            {
                *(req->buffer) = s_abstractState;
                *(req->length) = sizeof(s_abstractState);
                return kStatus_USB_Success;
            }
            if (req->setupValue == USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING)
            {
                *(req->buffer) = s_countryCode;
                *(req->length) = sizeof(s_countryCode);
                return kStatus_USB_Success;
            }
            return kStatus_USB_InvalidRequest;

        case kUSB_DeviceCdcEventGetLineCoding:
        case kUSB_DeviceCdcEventSetLineCoding:
            if (event == kUSB_DeviceCdcEventGetLineCoding || req->isSetup)
            {
                *(req->buffer) = s_lineCoding;
                *(req->length) = sizeof(s_lineCoding);
            }
            return kStatus_USB_Success;

        case kUSB_DeviceCdcEventSetControlLineState: /* DTR/RTS: deliberately ignored */
        case kUSB_DeviceCdcEventClearCommFeature:
        case kUSB_DeviceCdcEventSendBreak:
            return kStatus_USB_Success;

        default:
            return kStatus_USB_InvalidRequest;
    }
}

static usb_status_t cdc_device_cb(usb_device_handle handle, uint32_t event, void *param)
{
    uint8_t *p8 = (uint8_t *)param;
    uint16_t *p16 = (uint16_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
            s_attached = false;
            s_rxLen = 0;
            s_txBusy = false;
            s_txZlp = false;
            return kStatus_USB_Success;

        case kUSB_DeviceEventSetConfiguration:
            if (*p8 == USB_CDC_VCOM_CONFIGURE_INDEX)
            {
                s_attached = true;
                s_txBusy = false;
                rx_arm();
                return kStatus_USB_Success;
            }
            s_attached = false;
            return (*p8 == 0U) ? kStatus_USB_Success : kStatus_USB_InvalidRequest;

        case kUSB_DeviceEventSetInterface:
        {
            uint8_t alt = (uint8_t)(*p16 & 0xFFU);
            return (s_attached && alt == 0U) ? kStatus_USB_Success : kStatus_USB_InvalidRequest;
        }
        case kUSB_DeviceEventGetConfiguration:
            *p8 = s_attached ? USB_CDC_VCOM_CONFIGURE_INDEX : 0U;
            return kStatus_USB_Success;

        case kUSB_DeviceEventGetInterface:
            *p16 = (*p16 & 0xFF00U); /* alternate setting 0 */
            return kStatus_USB_Success;

        case kUSB_DeviceEventGetDeviceDescriptor:
            return USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
        case kUSB_DeviceEventGetConfigurationDescriptor:
            return USB_DeviceGetConfigurationDescriptor(handle,
                                                        (usb_device_get_configuration_descriptor_struct_t *)param);
        case kUSB_DeviceEventGetStringDescriptor:
            return USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
        default:
            return kStatus_USB_InvalidRequest;
    }
}

void cdc_init(void)
{
    RESET_PeripheralReset(kUSB0_RST_SHIFT_RSTn);
    CLOCK_EnableUsbfsClock();

    if (USB_DeviceClassInit(CONTROLLER_ID, &s_classConfigList, &s_device) != kStatus_USB_Success)
    {
        return;
    }
    s_cdc = s_classConfigList.config->classHandle;

    uint8_t irqs[] = USBFS_IRQS;
    NVIC_SetPriority((IRQn_Type)irqs[0], USB_IRQ_PRIORITY);
    EnableIRQ((IRQn_Type)irqs[0]);

    SDK_DelayAtLeastUs(5000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    USB_DeviceRun(s_device);
}

size_t cdc_read(uint8_t *dst, size_t max)
{
    uint32_t len = s_rxLen;
    if (!s_attached || len == 0U)
    {
        return 0;
    }
    size_t n = len - s_rxPos;
    if (n > max)
    {
        n = max;
    }
    memcpy(dst, &s_rxPkt[s_rxPos], n);
    s_rxPos += n;
    if (s_rxPos >= len)
    {
        rx_arm();
    }
    return n;
}

#define BULK_IN_ADDR (USB_CDC_VCOM_BULK_IN_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT))
#define STALL_WAIT_MS 300U /* how long a new reply waits for the host to clear a stall */

static bool in_stalled(void)
{
    usb_device_endpoint_status_struct_t st = {BULK_IN_ADDR, 0};
    if (USB_DeviceGetStatus(s_device, kUSB_DeviceStatusEndpoint, &st) != kStatus_USB_Success)
    {
        return false;
    }
    return st.endpointStatus == (uint16_t)kUSB_DeviceEndpointStateStalled;
}

/*
 * Wait until the IN endpoint is free. Returns false if the host stopped reading.
 *
 * When the host stops reading (it closed the port or the application hung), the
 * transfer can't simply be cancelled: the KHCI driver leaves the half-sent packet primed
 * in the buffer descriptor, after which the driver's and the hardware's data toggles
 * disagree and the host silently discards every later packet. Stalling the endpoint
 * replaces that packet with a STALL handshake instead; the host's CLEAR_FEATURE(HALT)
 * (Linux cdc-acm sends it as soon as a read hits the stall) resets the toggle on both
 * sides, so the next session starts clean.
 */
static bool tx_wait(void)
{
    uint32_t t0 = timebase_ms();
    while (s_txBusy)
    {
        if (!s_attached)
        {
            return false;
        }
        if (timebase_ms() - t0 > TX_TIMEOUT_MS)
        {
            (void)USB_DeviceStallEndpoint(s_device, BULK_IN_ADDR); /* cancels the transfer */
            s_txBusy = false;
            s_txZlp = false;
            return false;
        }
    }
    return true;
}

/* Wait briefly for the host to clear a stall left by tx_wait(). */
static bool stall_cleared(void)
{
    uint32_t t0 = timebase_ms();
    while (in_stalled())
    {
        if (!s_attached || timebase_ms() - t0 > STALL_WAIT_MS)
        {
            return false;
        }
    }
    return true;
}

static void tx_send(bool last)
{
    uint32_t len = s_txFill;
    uint8_t *buf = s_txBuf[s_txCur];
    s_txFill = 0;
    s_txCur ^= 1U;
    if (s_txDrop || !s_attached || len == 0U)
    {
        return;
    }
    if (!tx_wait() || !stall_cleared())
    {
        s_txDrop = true; /* sending onto a stalled endpoint would overwrite the STALL */
        return;
    }
    s_txZlp = last;
    s_txBusy = true;
    if (USB_DeviceCdcAcmSend(s_cdc, USB_CDC_VCOM_BULK_IN_ENDPOINT, buf, len) != kStatus_USB_Success)
    {
        s_txBusy = false;
        s_txZlp = false;
    }
}

void cdc_write(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    while (len > 0U)
    {
        if (s_txFill == TX_BUF_SIZE)
        {
            /* only send a full buffer once more data follows, so cdc_flush() sends the
             * final buffer with its ZLP */
            tx_send(false);
        }
        size_t n = TX_BUF_SIZE - s_txFill;
        if (n > len)
        {
            n = len;
        }
        /* s_txBuf[s_txCur] is never the buffer on the wire: tx_send() swaps before sending */
        memcpy(&s_txBuf[s_txCur][s_txFill], p, n);
        s_txFill += n;
        p += n;
        len -= n;
    }
}

void cdc_flush(void)
{
    tx_send(true);
    /* wait for the last transfer, so the next response can't overtake it */
    if (!s_txDrop)
    {
        (void)tx_wait();
    }
    s_txDrop = false;
}
