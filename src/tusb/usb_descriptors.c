#include "tusb.h"
#include "uac_descriptors.h"
#include <string.h>

#define USB_VID 0x303A
#define USB_PID 0x81A1

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING_MIC,
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL,
};

enum {
    EPNUM_CDC_NOTIF = 0x81,
    EPNUM_CDC_OUT   = 0x02,
    EPNUM_CDC_IN    = 0x82,
    EPNUM_AUDIO_IN  = 0x83,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_AUDIO_DEVICE_DESC_LEN + TUD_CDC_DESC_LEN)

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // UAC 2.0 microphone: control interface 0, streaming interface 1.
    TUD_AUDIO_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, 4, 0, EPNUM_AUDIO_IN, 0),
    // CDC ACM control/data pair for the versioned control protocol.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 6, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

char const *string_desc_arr[] = {
    (const char[]) {0x09, 0x04},
    "OpenAI Pedal",
    "ESP32-S3 Processed Audio",
    "PEDAL-A-0001",
    "Processed microphone",
    "Processed audio stream",
    "Pedal control",
};

static uint16_t desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t count;
    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char *str = string_desc_arr[index];
        count = (uint8_t)strlen(str);
        if (count > 31) count = 31;
        for (uint8_t i = 0; i < count; ++i) desc_str[1 + i] = str[i];
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * count + 2));
    return desc_str;
}
