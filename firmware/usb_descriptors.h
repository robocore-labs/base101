#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

//--------------------------------------------------------------------
// Axon ROS Node - USB Interface Layout
//--------------------------------------------------------------------
//
// CDC #0 (IF0+1): zenoh serial transport (Pico-ROS node)
// CDC #1 (IF2+3): UART1 passthrough (Lidar)
// CDC #2 (IF4+5): debug log (init info, discovered IDs, status)
//
// Total: 6 interfaces
//--------------------------------------------------------------------

#define ITF_NUM_CDC0_COMM  0   // zenoh
#define ITF_NUM_CDC0_DATA  1
#define ITF_NUM_CDC1_COMM  2   // Lidar passthrough
#define ITF_NUM_CDC1_DATA  3
#define ITF_NUM_CDC2_COMM  4   // Debug log
#define ITF_NUM_CDC2_DATA  5
#define ITF_NUM_TOTAL      6

// CDC #0 endpoints (zenoh)
#define EP_CDC0_NOTIF      0x81
#define EP_CDC0_OUT        0x02
#define EP_CDC0_IN         0x82

// CDC #1 endpoints (Lidar)
#define EP_CDC1_NOTIF      0x83
#define EP_CDC1_OUT        0x04
#define EP_CDC1_IN         0x84

// CDC #2 endpoints (Debug)
#define EP_CDC2_NOTIF      0x85
#define EP_CDC2_OUT        0x06
#define EP_CDC2_IN         0x86

//--------------------------------------------------------------------
// CDC port mapping
//--------------------------------------------------------------------
#define CDC_IDX_ZENOH      0
#define CDC_IDX_LIDAR      1
#define CDC_IDX_DEBUG      2

//--------------------------------------------------------------------
// String descriptor indices
//--------------------------------------------------------------------
#define STR_IDX_LANG       0
#define STR_IDX_MANUF      1
#define STR_IDX_PRODUCT    2
#define STR_IDX_SERIAL     3
#define STR_IDX_CDC0       4   // zenoh
#define STR_IDX_CDC1       5   // Lidar
#define STR_IDX_CDC2       6   // Debug

#endif // USB_DESCRIPTORS_H
