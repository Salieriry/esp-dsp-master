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

// The component's stock microphone descriptor advertises connector, mute and
// volume controls that this firmware does not implement. Keep its topology and
// lengths, but advertise no controls for the physical input or Feature Unit.
#define PEDAL_AUDIO_MIC_DESCRIPTOR(_itfnum, _stridx, _epin) \
    TUD_AUDIO_DESC_IAD(_itfnum, NUM_INTERFACES, 0x00), \
    TUD_AUDIO_DESC_STD_AC(_itfnum, 0x00, _stridx), \
    TUD_AUDIO_DESC_CS_AC(0x0200, AUDIO_FUNC_MICROPHONE, TUD_AUDIO_DESC_CS_AC_TOTAL_LEN, \
                         AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS), \
    TUD_AUDIO_DESC_CLK_SRC(UAC2_ENTITY_CLOCK, 1, 1, UAC2_ENTITY_MIC_INPUT_TERMINAL, 0x00), \
    TUD_AUDIO_DESC_INPUT_TERM(UAC2_ENTITY_MIC_INPUT_TERMINAL, AUDIO_TERM_TYPE_IN_GENERIC_MIC, \
                              UAC2_ENTITY_MIC_OUTPUT_TERMINAL, UAC2_ENTITY_CLOCK, MIC_CHANNEL_NUM, \
                              AUDIO_CHANNEL_CONFIG_FRONT_CENTER, 0x00, 0x0000, 0x00), \
    TUD_AUDIO_DESC_OUTPUT_TERM(UAC2_ENTITY_MIC_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING, \
                               UAC2_ENTITY_MIC_INPUT_TERMINAL, UAC2_ENTITY_MIC_FEATURE_TERMINAL, \
                               UAC2_ENTITY_CLOCK, 0x0000, 0x00), \
    TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL(TUD_AUDIO_DESC_MIC_FEATURE_UNIT_N_CHANNEL_LEN, \
                                          UAC2_ENTITY_MIC_FEATURE_TERMINAL, \
                                          UAC2_ENTITY_MIC_INPUT_TERMINAL, 0x00, \
                                          U32_TO_U8S_LE(0), U32_TO_U8S_LE(0)), \
    TUD_AUDIO_DESC_STD_AS_INT((uint8_t)(_itfnum + 1), 0x00, 0x00, _stridx + 1), \
    TUD_AUDIO_DESC_STD_AS_INT((uint8_t)(_itfnum + 1), 0x01, 0x01, _stridx + 1), \
    TUD_AUDIO_DESC_CS_AS_INT(UAC2_ENTITY_MIC_OUTPUT_TERMINAL, AUDIO_CTRL_NONE, \
                             AUDIO_FORMAT_TYPE_I, AUDIO_DATA_FORMAT_TYPE_I_PCM, MIC_CHANNEL_NUM, \
                             AUDIO_CHANNEL_CONFIG_FRONT_CENTER, 0x00), \
    TUD_AUDIO_DESC_TYPE_I_FORMAT(CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX, \
                                 CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX), \
    TUD_AUDIO_DESC_STD_AS_ISO_EP(_epin, \
                                 (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | \
                                  TUSB_ISO_EP_ATT_DATA), \
                                 CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN, 0x01), \
    TUD_AUDIO_DESC_CS_AS_ISO_EP(AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO_CTRL_NONE, \
                                AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0x0000)

// Measure the same local template used below against both TinyUSB's expected
// function length and the component's transfer/parser buffer allocation.
static const uint8_t pedal_audio_descriptor_layout[] = {
    PEDAL_AUDIO_MIC_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, 4, EPNUM_AUDIO_IN)
};
_Static_assert(sizeof(pedal_audio_descriptor_layout) == TUD_AUDIO_DEVICE_DESC_LEN,
               "UAC descriptor template length mismatch");
_Static_assert(sizeof(pedal_audio_descriptor_layout) == CFG_TUD_AUDIO_FUNC_1_DESC_LEN,
               "UAC descriptor length must match TinyUSB audio buffer configuration");

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
    PEDAL_AUDIO_MIC_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, 4, EPNUM_AUDIO_IN),
    // CDC ACM control/data pair for the versioned control protocol.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 6, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};
_Static_assert(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
               "USB composite configuration descriptor length mismatch");

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
