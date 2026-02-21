#include <string.h>
#include "tusb.h"

// HID Report Descriptor (Gamepad) ------------------------------------
static uint8_t const desc_hid_report[] = {
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
    HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
        // 16 buttons (2 bytes)
        HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),
        HID_USAGE_MIN(1),
        HID_USAGE_MAX(16),
        HID_LOGICAL_MIN(0),
        HID_LOGICAL_MAX(1),
        HID_REPORT_COUNT(16),
        HID_REPORT_SIZE(1),
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
        // Hat Switch (1 byte)
        HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
        HID_USAGE(HID_USAGE_DESKTOP_HAT_SWITCH),
        HID_LOGICAL_MIN(0),
        HID_LOGICAL_MAX(7),
        HID_PHYSICAL_MIN(0),
        HID_PHYSICAL_MAX_N(315, 2),
        HID_REPORT_COUNT(1),
        HID_REPORT_SIZE(8),
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END
};

uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance) {
    (void) instance;
    return desc_hid_report;
}

// Device descriptor -------------------------------------------------
static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0xCafe,
    .idProduct = 0x4006,  // MSC+HID composite (GamePad)
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void) {
    return (uint8_t const*) &desc_device;
}

// Configuration descriptor with HID + MSC interfaces -------------------
enum {
    ITF_NUM_HID = 0,
    ITF_NUM_MSC,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_MSC_DESC_LEN)

#define EPNUM_HID       0x81
#define EPNUM_MSC_OUT   0x02
#define EPNUM_MSC_IN    0x82

static uint8_t const desc_fs_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 250),
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, 16, 10),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64)
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index; return desc_fs_configuration;
}

// String descriptors -------------------------------------------------
static char const *string_desc[] = {
    (const char[]) {0x09, 0x04}, // 0: supported language (en-US)
    "Pico",                      // 1: Manufacturer
    "PicoRapidX PAD Mode",      // 2: Product
    "123456",                   // 3: Serial
};

static uint16_t _desc_str[32];

uint16_t const * tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count;
    if (index == 0) {
        memcpy(&_desc_str[1], string_desc[0], 2);
        chr_count = 1; // one LANGID pair
    } else {
        const char *str = string_desc[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
    }
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);
    return _desc_str;
}
