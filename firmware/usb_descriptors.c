#include "tusb.h"
#include "pico/unique_id.h"
#include "usb_descriptors.h"

//--------------------------------------------------------------------
// Device Descriptor
//--------------------------------------------------------------------
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x1209,
    .idProduct          = 0xAC01,
    .bcdDevice          = 0x0300,

    .iManufacturer      = STR_IDX_MANUF,
    .iProduct           = STR_IDX_PRODUCT,
    .iSerialNumber      = STR_IDX_SERIAL,

    .bNumConfigurations = 1
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------
// Configuration Descriptor
//--------------------------------------------------------------------
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN * 3)

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 500),

    // CDC #0: zenoh serial transport
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC0_COMM, STR_IDX_CDC0,
                       EP_CDC0_NOTIF, 8, EP_CDC0_OUT, EP_CDC0_IN, 64),

    // CDC #1: UART1 passthrough (Lidar)
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC1_COMM, STR_IDX_CDC1,
                       EP_CDC1_NOTIF, 8, EP_CDC1_OUT, EP_CDC1_IN, 64),

    // CDC #2: debug log
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC2_COMM, STR_IDX_CDC2,
                       EP_CDC2_NOTIF, 8, EP_CDC2_OUT, EP_CDC2_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

//--------------------------------------------------------------------
// String Descriptors
//--------------------------------------------------------------------
static char const *string_desc_arr[] = {
    [STR_IDX_LANG]    = (const char[]){0x09, 0x04},
    [STR_IDX_MANUF]   = "RoboCore",
    [STR_IDX_PRODUCT] = "Axon ROS Node",
    [STR_IDX_SERIAL]  = NULL,
    [STR_IDX_CDC0]    = "RoboCore Axon Zenoh",
    [STR_IDX_CDC1]    = "RoboCore Axon Lidar",
    [STR_IDX_CDC2]    = "RoboCore Axon Debug",
};

static uint16_t _desc_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == STR_IDX_LANG) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else if (index == STR_IDX_SERIAL) {
        pico_unique_board_id_t id;
        pico_get_unique_board_id(&id);
        chr_count = 0;
        for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES && chr_count < 16; i++) {
            const char hex[] = "0123456789ABCDEF";
            _desc_str[1 + chr_count++] = hex[(id.id[i] >> 4) & 0xF];
            _desc_str[1 + chr_count++] = hex[id.id[i] & 0xF];
        }
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;
        const char *str = string_desc_arr[index];
        if (!str) return NULL;
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
