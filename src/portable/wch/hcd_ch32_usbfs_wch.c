/*
 * hcd_ch32_usbfs_wch.c — TinyUSB HCD for CH32 USBFS host, backed by WCH's own
 * transaction layer (wch_usbfs_ll.c). TinyUSB's full host stack (enumeration,
 * HID parsing, class drivers, hub) runs on top; the chip-specific transaction
 * timing/toggle/reset is WCH's proven code (it enumerates DualSense/Xbox/hubs).
 *
 * Model: SYNCHRONOUS. Control transfers run via USBFSH_CtrlTransfer inside the
 * host task; device attach/detach come from the USBFS DETECT interrupt.
 *
 * MIT (Joypad OS). Transaction layer (c) WCH.
 */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && defined(TUP_USBIP_WCH_USBFS) && defined(CFG_TUH_WCH_BACKEND) && CFG_TUH_WCH_BACKEND

#include "host/hcd.h"
#include "wch_usbfs_ll.h"

// USBFS host interrupt vector (shared with device-mode USBHD on V307)
#ifndef CH32_USBFS_IRQn
  #define CH32_USBFS_IRQn  USBFS_IRQn   // == OTG_FS_IRQn == 83 on CH32V307
#endif

#define WCH_RHPORT  0

// ---- per-device state ------------------------------------------------------
typedef struct {
  uint8_t  speed;                 // USB_FULL_SPEED / USB_LOW_SPEED
  uint8_t  ep0_size;
  uint8_t  in_tog[16];            // data toggle per IN endpoint
  uint8_t  out_tog[16];           // data toggle per OUT endpoint
  uint16_t max_packet[16];
} wch_dev_t;

static wch_dev_t s_dev[16];
static uint8_t   s_root_speed = USB_FULL_SPEED;
static bool      s_attached   = false;   // debounce DETECT — only fire on change

// control-transfer staging: SETUP is buffered, the whole transfer runs on the
// first EP0 data/status call via USBFSH_CtrlTransfer.
static bool      s_ctrl_pending = false;
static uint8_t   s_ctrl_daddr   = 0;

// ---- busy-loop delays used by the WCH transaction layer --------------------
// (~SystemCoreClock-calibrated; USB timing is tolerant. SysTick is owned by the BSP.)
void Delay_Us(uint32_t n) {
  volatile uint32_t i = n * (SystemCoreClock / 3000000u);
  while (i--) { __asm volatile ("nop"); }
}
void Delay_Ms(uint32_t n) { while (n--) Delay_Us(1000); }

// ---- helpers ---------------------------------------------------------------
static inline void wch_select_dev(uint8_t daddr) {
  USBFSH_SetSelfAddr(daddr);
  USBFSH_SetSelfSpeed(s_dev[daddr].speed);
}

// ===========================================================================
// Controller init
// ===========================================================================
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init) {
  (void) rhport; (void) rh_init;
  tu_memclr(s_dev, sizeof(s_dev));
  for (int i = 0; i < 16; i++) s_dev[i].speed = USB_FULL_SPEED;

  // NOTE: do NOT call USBFS_RCC_Init() — the TinyUSB BSP board_init already sets
  // a working USBFS 48MHz clock (the async HCD relied on it). Re-running WCH's
  // RCC setup here clobbers it and the SIE goes silent.
  USBFS_Host_Init(ENABLE);

  // enable only the DETECT (attach/disconnect) interrupt; transfers are sync
  USBFSH->INT_EN = USBFS_UIE_DETECT;
  USBFSH->INT_FG = 0xFF;
  return true;
}

bool hcd_deinit(uint8_t rhport) {
  (void) rhport;
  USBFS_Host_Init(DISABLE);
  return true;
}

bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void *cfg_param) {
  (void) rhport; (void) cfg_id; (void) cfg_param;
  return true;
}

uint32_t hcd_frame_number(uint8_t rhport) { (void) rhport; return 0; }

void hcd_int_enable(uint8_t rhport)  { (void) rhport; NVIC_EnableIRQ(CH32_USBFS_IRQn); }
void hcd_int_disable(uint8_t rhport) { (void) rhport; NVIC_DisableIRQ(CH32_USBFS_IRQn); }

// ===========================================================================
// Root port
// ===========================================================================
bool hcd_port_connect_status(uint8_t rhport) {
  (void) rhport;
  return (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH) ? true : false;
}

void hcd_port_reset(uint8_t rhport) {
  (void) rhport;
  // No-op: WCH's reset is a single blocking pulse done in reset_end (matching
  // HOST_KM's USBFSH_ResetRootHubPort(0) -> EnableRootHubPort -> enumerate).
}

void hcd_port_reset_end(uint8_t rhport) {
  (void) rhport;
  uint8_t speed = USB_FULL_SPEED;
  // Full blocking reset exactly like WCH HOST_KM: assert + 11ms + de-assert + 2ms.
  USBFSH_ResetRootHubPort(0);
  // enable the port + latch the speed (device must still be attached)
  for (uint32_t to = 0; to < 100; to++) {
    if (USBFSH_EnableRootHubPort(&speed) == ERR_SUCCESS) break;
    Delay_Ms(1);
  }
  s_root_speed = speed;
  s_dev[0].speed = speed;
  Delay_Ms(5);   // reset-recovery before the first control transfer
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  (void) rhport;
  return (s_root_speed == USB_LOW_SPEED) ? TUSB_SPEED_LOW : TUSB_SPEED_FULL;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  if (dev_addr < 16) tu_memclr(&s_dev[dev_addr], sizeof(wch_dev_t));
}

// ===========================================================================
// Interrupt handler — DETECT only (attach / remove)
// ===========================================================================
void hcd_int_handler(uint8_t rhport, bool in_isr) {
  if (USBFSH->INT_FG & USBFS_UIF_DETECT) {
    USBFSH->INT_FG = USBFS_UIF_DETECT;
    bool attached = (USBFSH->MIS_ST & USBFS_UMS_DEV_ATTACH) != 0;
    // Only emit on a real state change. WCH's reset/transaction code toggles the
    // DETECT flag, so an un-debounced handler would spew spurious attach/remove
    // events mid-enumeration and storm the usbh queue.
    if (attached && !s_attached) {
      s_attached = true;
      s_dev[0].speed = s_root_speed;
      hcd_event_device_attach(rhport, in_isr);
    } else if (!attached && s_attached) {
      s_attached = false;
      hcd_event_device_remove(rhport, in_isr);
    }
  }
}

// ===========================================================================
// Endpoint management
// ===========================================================================
bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const *ep_desc) {
  (void) rhport;
  uint8_t epnum = tu_edpt_number(ep_desc->bEndpointAddress);
  uint16_t mps  = tu_edpt_packet_size(ep_desc);
  s_dev[dev_addr].max_packet[epnum] = mps;
  if (epnum == 0) {
    s_dev[dev_addr].ep0_size = (mps < 8) ? 8 : (uint8_t) mps;
    // a freshly-attached device inherits the root speed until addressed
    if (s_dev[dev_addr].speed == 0 && dev_addr != 0) s_dev[dev_addr].speed = s_dev[0].speed;
  }
  return true;
}

bool hcd_edpt_close(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  s_dev[dev_addr].max_packet[tu_edpt_number(ep_addr)] = 0;
  return true;
}

bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  uint8_t epnum = tu_edpt_number(ep_addr);
  if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) s_dev[dev_addr].in_tog[epnum]  = 0;
  else                                     s_dev[dev_addr].out_tog[epnum] = 0;
  return true;
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport; (void) dev_addr; (void) ep_addr;
  return true;
}

// ===========================================================================
// SETUP — buffer it; the actual transfer runs on the next EP0 call
// ===========================================================================
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8]) {
  (void) rhport;
  if (s_dev[dev_addr].ep0_size == 0) s_dev[dev_addr].ep0_size = 8;

  // build the SETUP into the TX buffer (pUSBFS_SetupRequest aliases USBFS_TX_Buf)
  memcpy((void *) pUSBFS_SetupRequest, setup_packet, 8);
  s_ctrl_pending = true;
  s_ctrl_daddr   = dev_addr;

  // tell usbh the SETUP stage "completed" so it advances to data/status
  hcd_event_xfer_complete(dev_addr, 0, 8, XFER_RESULT_SUCCESS, false);
  return true;
}

// ===========================================================================
// Transfers
// ===========================================================================
static xfer_result_t wch_err_to_result(uint8_t s) {
  if (s == ERR_SUCCESS) return XFER_RESULT_SUCCESS;
  if ((s & 0x0F) == (USB_PID_STALL & 0x0F)) return XFER_RESULT_STALLED;
  return XFER_RESULT_FAILED;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen) {
  (void) rhport;
  uint8_t epnum = tu_edpt_number(ep_addr);

  if (epnum == 0) {
    // ---- control endpoint ----
    if (s_ctrl_pending && s_ctrl_daddr == dev_addr) {
      // run the whole SETUP+DATA+STATUS via WCH's proven control transfer.
      // Mask the USB IRQ across it: WCH's polled code toggles the DETECT flag,
      // and an interrupt mid-transaction would fire a spurious remove/attach and
      // restart enumeration. A REAL disconnect is reported by CtrlTransfer's
      // return code, and any pending DETECT fires once we re-enable below.
      wch_select_dev(dev_addr);
      NVIC_DisableIRQ(CH32_USBFS_IRQn);
      uint16_t got = 0;
      uint8_t  s   = USBFSH_CtrlTransfer(s_dev[dev_addr].ep0_size, buffer, &got);
      USBFSH->INT_FG = USBFS_UIF_DETECT;   // swallow any DETECT toggled by the xfer
      NVIC_EnableIRQ(CH32_USBFS_IRQn);
      s_ctrl_pending = false;
      hcd_event_xfer_complete(dev_addr, ep_addr, got, wch_err_to_result(s), false);
    } else {
      // redundant status stage — already done inside CtrlTransfer
      hcd_event_xfer_complete(dev_addr, ep_addr, 0, XFER_RESULT_SUCCESS, false);
    }
    return true;
  }

  // ---- interrupt / bulk endpoint (MILESTONE 2: stubbed) ----
  // TODO: SOF-driven polling. For now queue without completing so enumeration
  //       and mount proceed; reports are added in the next pass.
  (void) buffer; (void) buflen;
  return true;
}

#endif
