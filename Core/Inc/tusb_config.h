/*
 * tusb_config.h
 *
 *  Created on: Oct 27, 2021
 *      Author: Toni
 */

#ifndef INC_TUSB_CONFIG_H_
#define INC_TUSB_CONFIG_H_

#define CFG_TUSB_MCU                 OPT_MCU_STM32F1
#define CFG_TUSB_OS                  OPT_OS_NONE
#define BOARD_DEVICE_RHPORT_SPEED    OPT_MODE_FULL_SPEED  // 12mbps
#define BOARD_DEVICE_RHPORT_NUM     1
#define CFG_TUSB_RHPORT1_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// CFG_TUSB_DEBUG is defined by compiler in DEBUG build
// 0 : no debug
// 1 : print error
// 2 : print warning
// 3 : print info
#define CFG_TUSB_DEBUG           1


/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              1
#define CFG_TUD_VENDOR            0

// MIDI FIFO size of TX and RX
#define CFG_TUD_MIDI_RX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 512 : 64)
#define CFG_TUD_MIDI_TX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 512 : 64)

#endif /* INC_TUSB_CONFIG_H_ */
