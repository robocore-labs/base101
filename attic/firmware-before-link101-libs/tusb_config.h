#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "pico.h"

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_DEBUG        0

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_ENABLED       1
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

#define CFG_TUD_ENDPOINT0_SIZE 64

//--------------------------------------------------------------------
// CLASS CONFIGURATION
//--------------------------------------------------------------------

// 3 CDC interfaces: zenoh transport + lidar passthrough + debug log
#define CFG_TUD_CDC           3
// zenoh serial frames are up to ~1.7 KB (MTU 1500 + framing + COBS);
// generous FIFOs keep the byte-wise reader and bulk writer fast.
#define CFG_TUD_CDC_RX_BUFSIZE 2048
#define CFG_TUD_CDC_TX_BUFSIZE 2048

#define CFG_TUD_VENDOR        0
#define CFG_TUD_HID           0
#define CFG_TUD_MIDI          0
#define CFG_TUD_MSC           0

#endif
