/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Joel Michael <joelpmichael@gmail.com>
 * Derived from hcd_template.c Copyright (c) 2023 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && defined(TUP_USBIP_WCH_USBFS) && CFG_TUH_WCH_USBIP_USBFS && !(defined(CFG_TUH_WCH_BACKEND) && CFG_TUH_WCH_BACKEND)

  #include "ch32_usbfs_reg.h"
  #include "host/hcd.h"

// tusb_time_delay_ms_api() is declared by TinyUSB headers (tusb_timeout.h)

// private variables
CFG_TUH_MEM_ALIGN static uint8_t USBFS_RX_Buf[MAX_PACKET_SIZE];
CFG_TUH_MEM_ALIGN static uint8_t USBFS_TX_Buf[MAX_PACKET_SIZE];

volatile uint16_t retransmit_count = 0;

// Attach state + disconnect debounce. The WCH USBFS hardware toggles the DETECT
// flag (and momentarily reads DEV_ATTACH=0) during normal transactions, so a raw
// DETECT handler fires spurious remove/attach churn that aborts enumeration and
// prevents the HID interface from mounting/polling. Strategy (matches the WCH
// vendor stack / our hybrid): on a real attach, DISABLE the DETECT interrupt and
// detect a true disconnect from sustained DEV_ATTACH=0 sampled in the SOF IRQ.
static bool s_attached = false;
static uint16_t s_detach_cnt = 0;
static volatile uint32_t s_detect_cnt = 0;

typedef struct usb_device_map_s {
  volatile bool tx_data1[16];
  volatile bool rx_data1[16];
  uint16_t max_packet_size[16];
  uint8_t devaddr;
  uint8_t *p_buffer;
  size_t buff_size;
  volatile size_t buff_pos;
} usb_device_map_t;

// Indexed by dev_addr. TinyUSB assigns addresses 1..CFG_TUH_DEVICE_MAX (plus
// addr 0 during enumeration), so a full 128-entry table wastes ~10KB of SRAM on
// the 64KB CH32V307. Size to the device ceiling instead.
#ifndef CH32_USB_DEV_MAP_SIZE
#define CH32_USB_DEV_MAP_SIZE (CFG_TUH_DEVICE_MAX + 2)
#endif
usb_device_map_t usb_device_map[CH32_USB_DEV_MAP_SIZE] = {
    {
        .tx_data1 = {false},
        .rx_data1 = {false},
        .max_packet_size = {0},
        .devaddr = 0,
        .p_buffer = NULL,
        .buff_size = 0,
        .buff_pos = 0,
    },
};

volatile uint32_t frame_count = 0;
volatile bool sof_passed = true;

static hcd_event_t irq_event;

// private helper functions
  // V307: barf() printed register dumps from ISR context, deadlocking the UART.
  // Make it a no-op so error paths just report the result to TinyUSB and continue.
  #define barf()  do {} while (0)

void ch32_usbfs_barf(void) {
  TU_LOG_HEX(3, USBFSH->BASE_CTRL);
  if (USBFSH->BASE_CTRL & (1 << 7)) { TU_LOG(3, "RB_UC_HOST_MODE\r\n"); }
  if (USBFSH->BASE_CTRL & (1 << 6)) { TU_LOG(3, "RB_UC_LOW_SPEED\r\n"); }
  if ((USBFSH->BASE_CTRL & (0b11 << 4)) == (0b00 << 4)) { TU_LOG(3, "DM/DP Normal\r\n"); }
  if ((USBFSH->BASE_CTRL & (0b11 << 4)) == (0b01 << 4)) { TU_LOG(3, "DM/DP Force SE0\r\n"); }
  if ((USBFSH->BASE_CTRL & (0b11 << 4)) == (0b10 << 4)) { TU_LOG(3, "DM/DP Force J\r\n"); }
  if ((USBFSH->BASE_CTRL & (0b11 << 4)) == (0b11 << 4)) { TU_LOG(3, "DM/DP Force K (wakeup)\r\n"); }
  if (USBFSH->BASE_CTRL & (1 << 3)) { TU_LOG(3, "RB_UC_INT_BUSY\r\n"); }
  if (USBFSH->BASE_CTRL & (1 << 2)) { TU_LOG(3, "RB_UC_RESET_SIE\r\n"); }
  if (USBFSH->BASE_CTRL & (1 << 1)) { TU_LOG(3, "RB_UC_CLR_ALL\r\n"); }
  if (USBFSH->BASE_CTRL & (1 << 0)) { TU_LOG(3, "RB_UC_DMA_EN\r\n"); }

  TU_LOG_HEX(3, USBFSH->HOST_CTRL);
  if (USBFSH->HOST_CTRL & (1 << 7)) { TU_LOG(3, "RB_UH_PD_DIS \r\n"); }
  if (USBFSH->HOST_CTRL & (1 << 5)) { TU_LOG(3, "RB_UH_DP_PIN\r\n"); }
  if (USBFSH->HOST_CTRL & (1 << 4)) { TU_LOG(3, "RB_UH_DM_PIN\r\n"); }
  if (USBFSH->HOST_CTRL & (1 << 2)) { TU_LOG(3, "RB_UH_LOW_SPEED\r\n"); }
  if (USBFSH->HOST_CTRL & (1 << 1)) { TU_LOG(3, "RB_UH_BUS_RESET\r\n"); }
  if (USBFSH->HOST_CTRL & (1 << 0)) { TU_LOG(3, "RB_UH_PORT_EN\r\n"); }

  TU_LOG_HEX(1, USBFSH->MIS_ST);
  if (USBFSH->MIS_ST & (1 << 7)) { TU_LOG(3, "RB_UMS_SOF_PRES\r\n"); }
  if (USBFSH->MIS_ST & (1 << 6)) { TU_LOG(3, "RB_UMS_SOF_ACT\r\n"); }
  if (USBFSH->MIS_ST & (1 << 5)) { TU_LOG(3, "RB_UMS_SIE_FREE\r\n"); }
  if (USBFSH->MIS_ST & (1 << 4)) { TU_LOG(3, "RB_UMS_R_FIFO_RDY\r\n"); }
  if (USBFSH->MIS_ST & (1 << 3)) { TU_LOG(3, "RB_UMS_BUS_RESET\r\n"); }
  if (USBFSH->MIS_ST & (1 << 2)) { TU_LOG(3, "RB_UMS_SUSPEND\r\n"); }
  if (USBFSH->MIS_ST & (1 << 1)) { TU_LOG(3, "RB_UMS_DM_LEVEL\r\n"); }
  if (USBFSH->MIS_ST & (1 << 0)) { TU_LOG(3, "RB_UMS_DEV_ATTACH\r\n"); }

  TU_LOG_HEX(3, USBFSH->INT_EN);
  if (USBFSH->INT_EN & (1 << 6)) { TU_LOG(3, "RB_UIE_DEV_NAK \r\n"); }
  if (USBFSH->INT_EN & (1 << 5)) { TU_LOG(3, "RB_U_1WIRE_MODE\r\n"); }
  if (USBFSH->INT_EN & (1 << 4)) { TU_LOG(3, "RB_UIE_FIFO_OV\r\n"); }
  if (USBFSH->INT_EN & (1 << 3)) { TU_LOG(3, "RB_UIE_HST_SOF\r\n"); }
  if (USBFSH->INT_EN & (1 << 2)) { TU_LOG(3, "RB_UIE_SUSPEND\r\n"); }
  if (USBFSH->INT_EN & (1 << 1)) { TU_LOG(3, "RB_UIE_TRANSFER\r\n"); }
  if (USBFSH->INT_EN & (1 << 0)) { TU_LOG(3, "RB_UIE_DETECT \r\n"); }

  TU_LOG_HEX(1, USBFSH->INT_FG);
  if (USBFSH->INT_FG & (1 << 7)) { TU_LOG(3, "RB_U_IS_NAK\r\n"); }
  if (USBFSH->INT_FG & (1 << 6)) { TU_LOG(3, "RB_U_TOG_OK\r\n"); }
  if (USBFSH->INT_FG & (1 << 5)) { TU_LOG(3, "RB_U_SIE_FREE\r\n"); }
  if (USBFSH->INT_FG & (1 << 4)) { TU_LOG(3, "RB_UIF_FIFO_OV\r\n"); }
  if (USBFSH->INT_FG & (1 << 3)) { TU_LOG(3, "RB_UIF_HST_SOF\r\n"); }
  if (USBFSH->INT_FG & (1 << 2)) { TU_LOG(3, "RB_UIF_SUSPEND\r\n"); }
  if (USBFSH->INT_FG & (1 << 1)) { TU_LOG(3, "RB_UIF_TRANSFER\r\n"); }
  if (USBFSH->INT_FG & (1 << 0)) { TU_LOG(3, "RB_UIF_DETECT\r\n"); }

  TU_LOG_HEX(1, USBFSH->INT_ST);
  if (USBFSH->INT_ST & (1 << 7)) { TU_LOG(3, "RB_UIS_IS_NAK\r\n"); }
  if (USBFSH->INT_ST & (1 << 6)) { TU_LOG(3, "RB_UIS_TOG_OK\r\n"); }
  if (USBFSH->INT_ST & (0b11 << 4)) { TU_LOG(3, "UIS_TOKEN=%d\r\n", ((USBFSH->INT_ST & (0b11 << 4)) >> 4)); }
  if (USBFSH->INT_ST & 0b1111) { TU_LOG(1, "UIS_H_RES=%x\r\n", (USBFSH->INT_ST & 0b1111)); }

  TU_LOG_HEX(3, USBFSH->HOST_EP_MOD);
  if (USBFSH->HOST_EP_MOD & (1 << 6)) { TU_LOG(3, "RB_UH_EP_TX_EN\r\n"); }
  if (USBFSH->HOST_EP_MOD & (1 << 4)) { TU_LOG(3, "RB_UH_EP_TBUF_MOD\r\n"); }
  if (USBFSH->HOST_EP_MOD & (1 << 3)) { TU_LOG(3, "RB_UH_EP_RX_EN\r\n"); }
  if (USBFSH->HOST_EP_MOD & (1 << 0)) { TU_LOG(3, "RB_UH_EP_RBUF_MOD\r\n"); }

  TU_LOG_HEX(3, USBFSH->HOST_SETUP);
  if (USBFSH->HOST_SETUP & (1 << 10)) { TU_LOG(3, "RB_UH_PRE_PID_EN\r\n"); }
  if (USBFSH->HOST_SETUP & (1 << 2)) { TU_LOG(3, "RB_UH_SOF_EN\r\n"); }

  if (USBFSH->DEV_ADDR & (1 << 7)) { TU_LOG(3, "RB_UDA_GP_BIT\r\n"); }
  TU_LOG(3, "DEV_ADDR=%d\r\n", USBFSH->DEV_ADDR & 0x7F);

  if (USBFSH->HOST_EP_PID & (0b1111 << 4)) { TU_LOG(3, "UH_TOKEN=%x\r\n", (USBFSH->HOST_EP_PID & (0b1111 << 4)) >> 4); }
  TU_LOG(3, "UH_ENDP=%d\r\n", USBFSH->HOST_EP_PID & 0x0F);

  TU_LOG_HEX(3, USBFSH->HOST_RX_CTRL);
  if (USBFSH->HOST_RX_CTRL & (1 << 3)) { TU_LOG(3, "RB_UH_R_AUTO_TOG\r\n"); }
  if (USBFSH->HOST_RX_CTRL & (1 << 2)) { TU_LOG(3, "RB_UH_R_TOG\r\n"); }
  if (USBFSH->HOST_RX_CTRL & (1 << 0)) { TU_LOG(3, "RB_UH_R_RES\r\n"); }
  TU_LOG_INT(3, USBFSH->RX_LEN);
  TU_LOG(3, "HOST_RX_DMA=0x2000%04x\r\n", (uint16_t) USBFSH->HOST_RX_DMA);
  TU_LOG_HEX(3, USBFS_RX_Buf);
  TU_LOG_BUF(3, USBFS_RX_Buf, MAX_PACKET_SIZE);

  TU_LOG_HEX(3, USBFSH->HOST_TX_CTRL);
  if (USBFSH->HOST_TX_CTRL & (1 << 3)) { TU_LOG(3, "RB_UH_T_AUTO_TOG\r\n"); }
  if (USBFSH->HOST_TX_CTRL & (1 << 2)) { TU_LOG(3, "RB_UH_T_TOG\r\n"); }
  if (USBFSH->HOST_TX_CTRL & (1 << 0)) { TU_LOG(3, "RB_UH_T_RES\r\n"); }
  TU_LOG_INT(3, USBFSH->HOST_TX_LEN);
  TU_LOG(3, "HOST_TX_DMA=0x2000%04x\r\n", (uint16_t) USBFSH->HOST_TX_DMA);
  TU_LOG_HEX(3, USBFS_TX_Buf);
  TU_LOG_BUF(3, USBFS_TX_Buf, MAX_PACKET_SIZE);
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

// optional hcd configuration, called by tuh_configure()
bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void *cfg_param) {
  (void) rhport;
  (void) cfg_id;
  (void) cfg_param;
  TU_LOG_LOCATION();
  TU_LOG(3, "rhport=%d\r\n", rhport);
  return false;
}

// Initialize controller to host mode
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init) {
  (void) rhport;
  (void) rh_init;

  // init frame count
  frame_count = 0;
  hcd_int_disable(rhport);

  // reset SIE
  USBFSH->BASE_CTRL = USBFS_CTRL_RESET_SIE | USBFS_CTRL_CLR_ALL;
  // wait for SIE reset
  tusb_time_delay_ms_api(100);
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  // init host mode
  USBFSH->BASE_CTRL = USBFS_CTRL_HOST_MODE;
  tusb_time_delay_ms_api(1);
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  USBFSH->HOST_CTRL = 0;
  USBFSH->DEV_ADDR = 0;
  USBFSH->HOST_EP_MOD = USBFS_UH_EP_TX_EN | USBFS_UH_EP_RX_EN;
  USBFSH->HOST_RX_DMA = (uint32_t) USBFS_RX_Buf;
  USBFSH->HOST_TX_DMA = (uint32_t) USBFS_TX_Buf;
  USBFSH->HOST_RX_CTRL = 0;
  USBFSH->HOST_TX_CTRL = 0;
  USBFSH->INT_FG = 0xFF;
  USBFSH->BASE_CTRL = USBFS_CTRL_HOST_MODE | USBFS_CTRL_INT_BUSY | USBFS_CTRL_DMA_EN;

  if (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH) {
    hcd_event_device_attach(rhport, false);
  }

  hcd_int_enable(rhport);
  USBFSH->INT_EN = USBFS_INT_EN_HST_SOF | USBFS_INT_EN_TRANSFER | USBFS_INT_EN_DETECT;
  return true;
}

// de-init controller
bool hcd_deinit(uint8_t rhport) {
  (void) rhport;
  // reset SIE
  USBFSH->BASE_CTRL = USBFS_CTRL_RESET_SIE | USBFS_CTRL_CLR_ALL;
  return true;
}

// --------------------------------------------------------------------------
// Single-pipe transfer scheduler
//
// The CH32 USBFS host has ONE transfer pipe (a single HOST_EP_PID). The upper
// stack queues a transfer PER endpoint though — a composite HID device has an
// interrupt-IN outstanding on EP1 AND EP2 at once. The original driver armed
// every hcd_edpt_xfer immediately, so arming the 2nd clobbered the 1st in-flight
// transfer and permanently abandoned it (observed: keyboard EP81 dropped, only
// media-key EP82 kept polling). Fix: serialize. At most one transfer is armed
// (ch32_pipe_busy); extra submits queue and are armed one-at-a-time as each
// completes, so endpoints round-robin. Critical sections mask only USBHD_IRQn
// (the host ISR) to keep queue/pipe state consistent between submit (task ctx)
// and completion (ISR ctx).
// --------------------------------------------------------------------------
typedef struct {
  uint8_t  dev_addr;
  uint8_t  ep_addr;     // TinyUSB ep_addr (incl. 0x80 IN bit) for data xfers
  uint8_t *buffer;
  uint16_t buflen;
  uint8_t  setup[8];
  bool     is_setup;
} ch32_xfer_req_t;

#define CH32_REQ_Q_N 16
static ch32_xfer_req_t ch32_reqq[CH32_REQ_Q_N];
static volatile uint8_t ch32_reqq_head = 0;  // producer (submit)
static volatile uint8_t ch32_reqq_tail = 0;  // consumer (pump)
static volatile bool    ch32_pipe_busy = false;

static void ch32_arm_edpt(uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen);
static void ch32_arm_setup(uint8_t dev_addr, const uint8_t setup_packet[8]);

// Arm the next queued request iff the pipe is free.
// MUST be called from ISR context, or with USBHD_IRQn masked — never concurrently.
static void ch32_pump_locked(void) {
  if (ch32_pipe_busy) return;
  if (ch32_reqq_tail == ch32_reqq_head) return;          // queue empty
  ch32_xfer_req_t req = ch32_reqq[ch32_reqq_tail];
  ch32_reqq_tail = (uint8_t)((ch32_reqq_tail + 1) % CH32_REQ_Q_N);
  ch32_pipe_busy = true;
  if (req.is_setup) ch32_arm_setup(req.dev_addr, req.setup);
  else              ch32_arm_edpt(req.dev_addr, req.ep_addr, req.buffer, req.buflen);
}

// Enqueue a request, then pump. Task context only.
static void ch32_submit(const ch32_xfer_req_t *req) {
  NVIC_DisableIRQ(USBHD_IRQn);
  uint8_t next = (uint8_t)((ch32_reqq_head + 1) % CH32_REQ_Q_N);
  if (next != ch32_reqq_tail) {            // drop if full (must not happen in practice)
    ch32_reqq[ch32_reqq_head] = *req;
    ch32_reqq_head = next;
  }
  ch32_pump_locked();
  NVIC_EnableIRQ(USBHD_IRQn);
}

// Arm an IN/OUT data transfer on the single pipe. pipe_busy already set by caller.
static void ch32_arm_edpt(uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen) {
  retransmit_count = 0;
  usb_device_map[dev_addr].p_buffer = buffer;
  usb_device_map[dev_addr].buff_size = buflen;
  usb_device_map[dev_addr].buff_pos = 0;

  { uint32_t to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SOF_PRES) && --to) {} }
  { uint32_t to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --to) {} }
  USBFSH->HOST_EP_PID = 0;
  USBFSH->HOST_RX_DMA = (uint32_t) USBFS_RX_Buf;
  USBFSH->HOST_TX_DMA = (uint32_t) USBFS_TX_Buf;

  if (ep_addr & 0x80) {
    // DATA IN (RX)
    ep_addr &= 0x7F;
    USBFSH->DEV_ADDR = (USBFSH->DEV_ADDR & USBFS_UDA_GP_BIT) | (dev_addr & USBFS_USB_ADDR_MASK);
    USBFSH->HOST_TX_LEN = USBFSH->RX_LEN = 0;
    USBFSH->HOST_TX_CTRL = USBFSH->HOST_RX_CTRL = USBFS_UH_T_AUTO_TOG | USBFS_UH_R_AUTO_TOG | (usb_device_map[dev_addr].rx_data1[ep_addr] << 2);
    USBFSH->INT_FG = 0xFF;
    sof_passed = false;
    USBFSH->HOST_EP_PID = (USB_PID_IN << 4) | (ep_addr & USBFS_UH_ENDP_MASK);
  } else {
    // DATA OUT (TX)
    USBFSH->DEV_ADDR = (USBFSH->DEV_ADDR & USBFS_UDA_GP_BIT) | (dev_addr & USBFS_USB_ADDR_MASK);
    memcpy(USBFS_TX_Buf, buffer, TU_MIN(TU_MIN(buflen, MAX_PACKET_SIZE), usb_device_map[dev_addr].max_packet_size[ep_addr]));
    USBFSH->HOST_TX_LEN = TU_MIN(TU_MIN(buflen, MAX_PACKET_SIZE), usb_device_map[dev_addr].max_packet_size[ep_addr]);
    USBFSH->HOST_TX_CTRL = USBFSH->HOST_RX_CTRL = USBFS_UH_T_AUTO_TOG | USBFS_UH_R_AUTO_TOG | (usb_device_map[dev_addr].tx_data1[ep_addr] << 2);
    USBFSH->INT_FG = 0xFF;
    sof_passed = false;
    USBFSH->HOST_EP_PID = (USB_PID_OUT << 4) | (ep_addr & USBFS_UH_ENDP_MASK);
  }
}

// Arm an 8-byte SETUP packet on the single pipe. pipe_busy already set by caller.
static void ch32_arm_setup(uint8_t dev_addr, const uint8_t setup_packet[8]) {
  retransmit_count = 0;
  usb_device_map[dev_addr].p_buffer = NULL;
  usb_device_map[dev_addr].buff_size = 8;
  usb_device_map[dev_addr].buff_pos = 0;
  usb_device_map[dev_addr].tx_data1[0] = false;
  usb_device_map[dev_addr].rx_data1[0] = false;
  memcpy(USBFS_TX_Buf, setup_packet, 8);

  { uint32_t to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SOF_PRES) && --to) {} }
  { uint32_t to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --to) {} }
  USBFSH->HOST_EP_PID = 0;
  USBFSH->DEV_ADDR = (USBFSH->DEV_ADDR & USBFS_UDA_GP_BIT) | (dev_addr & USBFS_USB_ADDR_MASK);
  USBFSH->HOST_RX_DMA = (uint32_t) USBFS_RX_Buf;
  USBFSH->HOST_TX_DMA = (uint32_t) USBFS_TX_Buf;
  USBFSH->HOST_TX_LEN = USBFSH->RX_LEN = 0;
  USBFSH->HOST_TX_LEN = 8;
  USBFSH->HOST_TX_CTRL = USBFSH->HOST_RX_CTRL = USBFS_UH_T_AUTO_TOG | USBFS_UH_R_AUTO_TOG;
  USBFSH->INT_FG = 0xFF;
  sof_passed = false;
  USBFSH->HOST_EP_PID = (USB_PID_SETUP << 4);
}

// Interrupt Handler
void hcd_int_handler(uint8_t rhport, bool in_isr) {
  // V307 FIX: do NOT busy-wait for SOF_PRES at IRQ entry — on V307 that burns
  // ~20ms per interrupt and starves the whole host stack (looks hung). Just
  // process the pending flags directly.

  // process DETECT IRQ
  if (USBFSH->INT_FG & USBFS_UIF_DETECT) {
    USBFSH->INT_FG = USBFS_UIF_DETECT;// always clear the (constantly toggling) flag
    s_detect_cnt++;
    // Only ACT on DETECT while waiting for an attach. Once a device is attached,
    // the WCH USBFS hardware keeps toggling this flag and momentarily reads
    // DEV_ATTACH=0 during normal transactions — acting on that fires spurious
    // removes that abort enumeration before the HID interface mounts. So while
    // attached we just clear the flag and ignore it. (Disabling the DETECT
    // interrupt-enable does NOT help: this handler is also entered via the
    // TRANSFER/SOF interrupts and still sees the flag. Disconnect is handled on
    // the next reset/boot for now.)
    if (!s_attached && (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH)) {
      s_attached = true;
      hcd_event_device_attach(rhport, in_isr);
    }
  }

  // process SOF IRQ
  if (USBFSH->INT_FG & USBFS_UIF_HST_SOF) {
    USBFSH->INT_FG = USBFS_UIF_HST_SOF;
    sof_passed = true;
    frame_count++;
    // NOTE: no DEV_ATTACH-based disconnect detection. On the WCH USBFS, DEV_ATTACH
    // reads 0 for sustained stretches (hundreds of ms) during normal operation —
    // not just brief transaction blips — so ANY threshold fires false removes that
    // kill a healthy device mid-session before its HID interface can poll input.
    // Once attached we stay attached; a real unplug is handled on the next reset.
    (void) s_detach_cnt;
  }

  if (USBFSH->INT_FG & USBFS_UIF_TRANSFER) {
    // finished handling xfer, either as
    USBFSH->INT_FG = USBFS_UIF_TRANSFER;// Clear IRQ flag

    // USBFS host stops the transfer when USBFSH->HOST_EP_PID is zero
    uint8_t orig_ep_pid = USBFSH->HOST_EP_PID;
    USBFSH->HOST_EP_PID = 0x00;// Stop USB transfer

    irq_event.rhport = 1;
    irq_event.event_id = HCD_EVENT_XFER_COMPLETE;
    irq_event.dev_addr = USBFSH->DEV_ADDR & USBFS_USB_ADDR_MASK;
    irq_event.xfer_complete.ep_addr = orig_ep_pid & USBFS_UH_ENDP_MASK;

    // Bounds guard: usb_device_map is sized to the device ceiling
    // (CH32_USB_DEV_MAP_SIZE), not the full 7-bit USB address space the DEV_ADDR
    // register can hold. Indexing it with a stale/unexpected address would write
    // out of bounds *in ISR context* -> fault -> QingKe core reset (SFT, no trap
    // handler). Drop such spurious completions and free the pipe.
    if (irq_event.dev_addr >= CH32_USB_DEV_MAP_SIZE) {
      ch32_pipe_busy = false;
      ch32_pump_locked();
      return;
    }

    if (USBFSH->INT_ST & USBFS_UIS_TOG_OK) {
      // NOTE: the helper function hcd_event_xfer_complete uses the wrong root port! open-code it here instead...
      irq_event.xfer_complete.result = XFER_RESULT_SUCCESS;

      if ((orig_ep_pid & USBFS_UH_TOKEN_MASK) == (USB_PID_IN << 4)) {
        // IN frame
        irq_event.xfer_complete.len = USBFSH->RX_LEN;

        // copy RX DMA buffer to the destination
        memcpy(
            usb_device_map[irq_event.dev_addr].p_buffer + usb_device_map[irq_event.dev_addr].buff_pos,
            USBFS_RX_Buf,
            TU_MIN(
                usb_device_map[irq_event.dev_addr].max_packet_size[irq_event.xfer_complete.ep_addr],
                TU_MIN(
                    irq_event.xfer_complete.len,
                    usb_device_map[irq_event.dev_addr].buff_size - usb_device_map[irq_event.dev_addr].buff_pos)));

        // TinyUSB uses endpoint address with high-bit set to indicate in or out/setup transaction

        // end of data: either a short packet or the buffer is full
        if (irq_event.xfer_complete.len < usb_device_map[irq_event.dev_addr].max_packet_size[irq_event.xfer_complete.ep_addr] || (usb_device_map[irq_event.dev_addr].buff_pos + irq_event.xfer_complete.len) >= usb_device_map[irq_event.dev_addr].buff_size) {
          // end of data
          usb_device_map[irq_event.dev_addr].tx_data1[irq_event.xfer_complete.ep_addr] = usb_device_map[irq_event.dev_addr].rx_data1[irq_event.xfer_complete.ep_addr] = USBFSH->HOST_RX_CTRL & USBFS_UH_R_TOG ? true : false;
          irq_event.xfer_complete.len += usb_device_map[irq_event.dev_addr].buff_pos;
          irq_event.xfer_complete.ep_addr |= 0x80;

          hcd_event_handler(&irq_event, in_isr);
          ch32_pipe_busy = false;       // transfer done — free pipe, arm next queued
          ch32_pump_locked();
        } else {
          { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }
          USBFSH->HOST_EP_PID = orig_ep_pid;
        }
      } else {
        if ((orig_ep_pid & USBFS_UH_TOKEN_MASK) == (USB_PID_SETUP << 4)) {
          // SETUP frame
        }

        // OUT or SETUP frame
        irq_event.xfer_complete.len = USBFSH->HOST_TX_LEN;

        // check for more data to tx
        if (usb_device_map[irq_event.dev_addr].buff_size > (usb_device_map[irq_event.dev_addr].buff_pos + irq_event.xfer_complete.len)) {
          memcpy(
              USBFS_TX_Buf,
              usb_device_map[irq_event.dev_addr].p_buffer + usb_device_map[irq_event.dev_addr].buff_pos + irq_event.xfer_complete.len,
              TU_MIN(
                  usb_device_map[irq_event.dev_addr].max_packet_size[irq_event.xfer_complete.ep_addr],
                  (usb_device_map[irq_event.dev_addr].buff_size - (usb_device_map[irq_event.dev_addr].buff_pos + irq_event.xfer_complete.len))));
          { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }
          USBFSH->HOST_EP_PID = orig_ep_pid;
        } else {
          // end of data
          usb_device_map[irq_event.dev_addr].tx_data1[irq_event.xfer_complete.ep_addr] = usb_device_map[irq_event.dev_addr].rx_data1[irq_event.xfer_complete.ep_addr] = USBFSH->HOST_TX_CTRL & USBFS_UH_T_TOG ? true : false;
          irq_event.xfer_complete.len += usb_device_map[irq_event.dev_addr].buff_pos;
          hcd_event_handler(&irq_event, in_isr);
          ch32_pipe_busy = false;       // transfer done — free pipe, arm next queued
          ch32_pump_locked();
        }
      }
      usb_device_map[irq_event.dev_addr].buff_pos += irq_event.xfer_complete.len;
    } else {
      // data toggle didn't match
      // probably an error so figure out what happened

      irq_event.xfer_complete.len = 0;

      // TinyUSB uses endpoint address with high-bit set to indicate in or out/setup transaction
      if ((orig_ep_pid & USBFS_UH_TOKEN_MASK) == (USB_PID_IN << 4)) {
        irq_event.xfer_complete.ep_addr |= 0x80;
      }

      switch ((USBFSH->INT_ST & USBFS_UIS_H_RES_MASK)) {
        case USB_PID_STALL: {
          irq_event.xfer_complete.result = XFER_RESULT_STALLED;
          retransmit_count = USBH_MAX_RETRIES;
          break;
        }
        case USB_PID_NAK: {
          irq_event.xfer_complete.result = XFER_RESULT_FAILED;
          break;
        }
        case USB_PID_NULL: {
          // TODO - this might need to be XFER_RESULT_FAILED
          //TU_LOG(1, "WARNING: XFER TIMEOUT - verify TUSB handling of XFER_RESULT_TIMEOUT\r\n");
          irq_event.xfer_complete.result = XFER_RESULT_TIMEOUT;
          break;
        }
        default: {
          // NO printf in ISR context on V307 — it deadlocks the UART and hangs the chip.
          irq_event.xfer_complete.result = XFER_RESULT_FAILED; // don't hang; report failure
        }
      }
      if (retransmit_count < USBH_MAX_RETRIES) {
        retransmit_count++;
        { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }
        USBFSH->HOST_EP_PID = orig_ep_pid;
      } else {
        barf();
        hcd_event_handler(&irq_event, in_isr);
        ch32_pipe_busy = false;         // transfer failed/done — free pipe, arm next queued
        ch32_pump_locked();
      }
    }
  }
}

// Enable USB interrupt
void hcd_int_enable(uint8_t rhport) {
  (void) rhport;
  // busy-wait until SIE is idle
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  // NOTE: do NOT synthesize a boot-attach here — hcd_int_enable runs during
  // tusb_init(host) before the usbh event queue is ready, so the event is lost
  // AND it would set s_attached=1, suppressing the real DETECT-edge attach. The
  // port re-init on reset already creates an attach edge, so the DETECT path
  // (debounced by s_attached in hcd_int_handler) enumerates a present device.
  NVIC_EnableIRQ(USBHD_IRQn);
}

// Boot-attach from TASK context. A device already attached when the host starts
// produces no DETECT edge, so it never enumerates on its own. Call this from the
// main loop (AFTER the usbh event queue exists — unlike hcd_int_enable, which runs
// too early and drops the event). No-op once a device is attached/handled.
void ch32_host_boot_attach(void) {
  if (!s_attached && (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH)) {
    s_attached = true;
    hcd_event_device_attach(1, false);
  }
}

// Retry hook: the poll-based attach enumerates only ~half the time (intermittent
// SET_ADDRESS-recovery failure). The main loop calls this with the current mount
// state; if we've been "attached" but unmounted for too long, drop s_attached so
// the next ch32_host_boot_attach() re-fires a fresh enumeration attempt.
void ch32_host_retry_if_stalled(bool mounted, uint32_t now_ms) {
  static uint32_t since = 0;
  if (s_attached && !mounted) {
    if (since == 0) since = now_ms ? now_ms : 1;
    else if ((now_ms - since) > 1500) { s_attached = false; since = 0; }
  } else {
    since = 0;
  }
}

// Host-state diagnostic — print the USBFS host attach/port state (firmware read).
void ch32_host_diag(void) {
  printf("[diag] host MIS_ST=%02X s_attached=%d detect_irq=%lu frames=%lu\n",
         (unsigned)(USBFSH->MIS_ST & 0xFF), s_attached,
         (unsigned long) s_detect_cnt, (unsigned long) frame_count);
}

// Disable USB interrupt
void hcd_int_disable(uint8_t rhport) {
  (void) rhport;
  // busy-wait until SIE is idle
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  NVIC_DisableIRQ(USBHD_IRQn);
}

// Get frame number (1ms)
uint32_t hcd_frame_number(uint8_t rhport) {
  (void) rhport;
  // busy-wait until SIE is idle
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }
  return frame_count;
}

//--------------------------------------------------------------------+
// Port API
//--------------------------------------------------------------------+

// Get the current connect status of roothub port
bool hcd_port_connect_status(uint8_t rhport) {
  (void) rhport;
  // busy-wait until SIE is idle
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  if (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH)
    return true;
  return false;
}

// Reset USB bus on the port. Return immediately, bus reset sequence may not be complete.
// Some port would require hcd_port_reset_end() to be invoked after 10ms to complete the reset sequence.
void hcd_port_reset(uint8_t rhport) {
  // disable interrupts, because this will generate another disconnect/connect IRQ
  // and the disconnect IRQ will end up aborting the enumeration

  // wait for device to settle before resetting.
  // USB spec debounce is ~100ms; the original 1000ms left freshly-attached
  // devices unreset (no SOF) long enough to suspend, and some controllers
  // dropped off the bus before enumeration. RP2040 host uses standard timing.
  tusb_time_delay_ms_api(100);

  // int_disable implicitly waits for SIE idle
  hcd_int_disable(rhport);

  // reset device address to 0
  USBFSH->DEV_ADDR = (USBFSH->DEV_ADDR & USBFS_UDA_GP_BIT) | (0 & USBFS_USB_ADDR_MASK);

  // set full speed mode
  USBFSH->BASE_CTRL &= ~USBFS_CTRL_LOW_SPEED;
  USBFSH->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
  USBFSH->HOST_SETUP &= ~USBFS_UH_PRE_PID_EN;

  // start bus reset
  USBFSH->HOST_CTRL |= USBFS_UH_BUS_RESET;
}

// Complete bus reset sequence
// TinyUSB inserts a 10-50ms delay in between hcd_port_reset() and hcd_port_reset_end()
void hcd_port_reset_end(uint8_t rhport) {
  // end reset
  USBFSH->HOST_CTRL &= ~USBFS_UH_BUS_RESET;
  tusb_time_delay_ms_api(10);

  // busy-wait until SIE is idle
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  // clear spurious DETECT interrupt
  if (USBFSH->INT_FG & USBFS_UIF_DETECT) {
    if (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH) {
      USBFSH->INT_FG = USBFS_UIF_DETECT;
    }
  }

  // enable port
  if (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH) {
    if ((USBFSH->HOST_CTRL & USBFS_UH_PORT_EN) == 0x00) {
      if ((USBFSH->MIS_ST & USBFS_UMS_DM_LEVEL ? USB_LOW_SPEED : USB_FULL_SPEED) == USB_LOW_SPEED) {
        USBFSH->BASE_CTRL |= USBFS_UC_LOW_SPEED;
        USBFSH->HOST_CTRL |= USBFS_UH_LOW_SPEED;
        USBFSH->HOST_SETUP |= USBFS_UH_PRE_PID_EN;
      }
    }
    USBFSH->HOST_CTRL |= USBFS_UH_PORT_EN;
    USBFSH->HOST_SETUP |= USBFS_UH_SOF_EN;

    // Let SOF run so the device can power up its USB engine before the first
    // control transfer. Complex controllers (DualSense, XInput pads) boot far
    // slower than a simple HID mouse/keyboard and otherwise time out the first
    // SETUP. The bus is active (SOF running) here, so this does NOT risk the
    // suspend/disconnect that a long PRE-reset delay caused.
    tusb_time_delay_ms_api(200);
  }

  USBFSH->HOST_RX_DMA = (uint32_t) USBFS_RX_Buf;
  USBFSH->HOST_TX_DMA = (uint32_t) USBFS_TX_Buf;

  USBFSH->INT_FG = 0xFF;
  hcd_int_enable(rhport);
}

// Get port link speed
tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  (void) rhport;
  // busy-wait until SIE is idle
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  if ((USBFSH->HOST_CTRL & USBFS_UH_LOW_SPEED))
    return TUSB_SPEED_LOW;
  return TUSB_SPEED_FULL;
}

// HCD closes all opened endpoints belong to this device
void hcd_device_close(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
  for (uint8_t i = 0; i < 16; i++) {
    hcd_edpt_close(rhport, dev_addr, i);
  }
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+

// Open an endpoint
bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const *ep_desc) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_desc;
  TU_ASSERT(dev_addr < 128);
  usb_device_map[dev_addr].max_packet_size[ep_desc->bEndpointAddress & 0x7F] = ep_desc->wMaxPacketSize;
  return true;
}

bool hcd_edpt_close(uint8_t rhport, uint8_t daddr, uint8_t ep_addr) {
  (void) rhport;
  (void) daddr;
  (void) ep_addr;
  TU_ASSERT(daddr < 128);
  usb_device_map[daddr].max_packet_size[ep_addr] = 0;
  return true;
}

// Submit a transfer, when complete hcd_event_xfer_complete() must be invoked
bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  (void) buffer;
  (void) buflen;
  TU_ASSERT(dev_addr < 128);
  if (usb_device_map[dev_addr].max_packet_size[(ep_addr & 0x7F)] < 8) {
    TU_LOG_LOCATION();
    TU_LOG(2, "max_packet_size too small, reset to 8\r\n");
    usb_device_map[dev_addr].max_packet_size[(ep_addr & 0x7F)] = 8;
  }

  TU_LOG(3, "rhport=%d dev_addr=%d ep_addr=%d buflen=%d\r\n", rhport, dev_addr, ep_addr, buflen);

  // Queue + pump rather than arming immediately: concurrent transfers on
  // different endpoints must not clobber each other on the single host pipe.
  ch32_xfer_req_t req = { .dev_addr = dev_addr, .ep_addr = ep_addr, .buffer = buffer, .buflen = buflen, .is_setup = false };
  ch32_submit(&req);
  return true;
}

// Abort a queued transfer. Note: it can only abort transfer that has not been started
// Return true if a queued transfer is aborted, false if there is no transfer to abort
bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  TU_LOG_LOCATION();
  TU_LOG(3, "rhport=%d\r\n", rhport);

  // stop xmit
  USBFSH->HOST_EP_PID = 0;

  // busy-wait until SIE is idle
  { uint32_t _to = 2000000; while (!(USBFSH->MIS_ST & USBFS_UMS_SIE_FREE) && --_to) {} }

  // pipe is now stopped — free it so the scheduler can arm the next queued xfer
  NVIC_DisableIRQ(USBHD_IRQn);
  ch32_pipe_busy = false;
  ch32_pump_locked();
  NVIC_EnableIRQ(USBHD_IRQn);

  return true;
}

// Submit a special transfer to send 8-byte Setup Packet, when complete hcd_event_xfer_complete() must be invoked
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8]) {
  // no need for special length handling because this will always be inside the max packet size
  #define SETUP_PACKET_LEN 8
  #if MAX_PACKET_SIZE < SETUP_PACKET_LEN
    #error MAX_PACKET_SIZE smaller than SETUP_PACKET_LEN
  #endif
  (void) rhport;
  (void) dev_addr;
  (void) setup_packet;
  TU_ASSERT(dev_addr < 128);
  if (usb_device_map[dev_addr].max_packet_size[0] < 8) {
    TU_LOG_LOCATION();
    TU_LOG(2, "max_packet_size too small, reset to 8\r\n");
    usb_device_map[dev_addr].max_packet_size[0] = 8;
  }

  // Queue + pump on the single host pipe (see hcd_edpt_xfer).
  ch32_xfer_req_t req = { .dev_addr = dev_addr, .is_setup = true };
  memcpy(req.setup, setup_packet, 8);
  ch32_submit(&req);
  return true;
}

// clear stall, data toggle is also reset to DATA0
bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  TU_LOG_LOCATION();
  TU_LOG(3, "rhport=%d\r\n", rhport);
  barf();
  return false;
}

#endif
