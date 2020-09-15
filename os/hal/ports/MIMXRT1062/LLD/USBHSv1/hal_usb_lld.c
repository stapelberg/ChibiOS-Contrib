/*
    ChibiOS - Copyright (C) 2015 RedoX https://github.com/RedoXyde/
                        (C) 2015-2016 flabbergast <s3+flabbergast@sdfeu.org>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    USBHSv1/hal_usb_lld.c
 * @brief   MIMXRT1062 USB subsystem low level driver source.
 * @note page 2211 in https://www.pjrc.com/teensy/IMXRT1060RM_rev2.pdf
 * @note page 2203: chapter 42 USB
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USB1 driver identifier.*/
#if MIMXRT1062_USB_USE_USB1 || defined(__DOXYGEN__)

// TODO: does this end up in a non-writable flash segment maybe?
// 0x20001508, resulting in MMSR MemoryManage Hard Fault

USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   IN EP0 state.
 */
USBInEndpointState ep0in;

/**
 * @brief   OUT EP0 state.
 */
USBOutEndpointState ep0out;

/**
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  .ep_mode = USB_EP_MODE_TYPE_CTRL,
  .setup_cb = _usb_ep0setup,
  .in_cb = _usb_ep0in,
  .out_cb = _usb_ep0out,
  .in_maxsize = 64,
  .out_maxsize = 64,
  .in_state = &ep0in,
  .out_state = &ep0out,
  .ep_buffers = 1,
  .setup_buf = ep0setup_buffer,
};

// → page 2344, “Device Data Structures”

typedef struct transfer_struct transfer_t;
struct transfer_struct {
  uint32_t next;
  volatile uint32_t status;
  uint32_t pointer0;
  uint32_t pointer1;
  uint32_t pointer2;
  uint32_t pointer3;
  uint32_t pointer4;
  uint32_t callback_param;
};

typedef struct endpoint_struct endpoint_t;

// → page 2346, “Table 42-57 Endpoint Queue Head (dQH)”
struct endpoint_struct {
  uint32_t config;
  uint32_t current;
  uint32_t next;
  uint32_t status;
  uint32_t pointer0;
  uint32_t pointer1;
  uint32_t pointer2;
  uint32_t pointer3;
  uint32_t pointer4;
  uint32_t reserved;
  uint32_t setup0;
  uint32_t setup1;

  // These extra fields make up the remainder of the 64 bytes on which Queue
  // Heads must be aligned:
  transfer_t *first_transfer;
  transfer_t *last_transfer;
  void (*callback_function)(transfer_t *completed_transfer);
  uint32_t unused1;
};

// Two Queue Heads (1 rx, 1 tx) per endpoint:
endpoint_t endpoint_queue_head[(MIMXRT1062_USB_ENDPOINTS+1)*2] __attribute__ ((used, aligned(4096)));

transfer_t endpoint0_transfer_data __attribute__ ((used, aligned(32)));
transfer_t endpoint0_transfer_ack  __attribute__ ((used, aligned(32)));

typedef union {
  struct {
    union {
      struct {
        uint8_t bmRequestType;
        uint8_t bRequest;
      };
      uint16_t wRequestAndType;
    };
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
  };
  struct {
    uint32_t word1;
    uint32_t word2;
  };
  uint64_t bothwords;
} setup_t;



/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// delay sleeps for |cycles| (e.g. sleeping for F_CPU will sleep 1s).
// delay’s precision is ± 166 ns (due to 30 cycles of overhead).
static void delay(const uint32_t cycles) {
  // Reset cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  while (DWT->CYCCNT < cycles) {
    // busy-loop until time has passed
  }
}

void usb_transfer_schedule(transfer_t *t, uint32_t addr, size_t n, int queue_head, uint32_t primebit, int notify) {
    // → page 2371, “Building a transfer descriptor”
    // TODO: initialize first 7 dwords to 0
    t->next = 1;
    t->status = (n << 16) | (notify ? (1 << 15) : 0) | (1<<7);
    t->pointer0 = addr;
    t->pointer1 = (addr + 0x1000) & 0xfffff000;
    t->pointer2 = (addr + 0x2000) & 0xfffff000;
    t->pointer3 = (addr + 0x3000) & 0xfffff000;
    t->pointer4 = (addr + 0x4000) & 0xfffff000;

    // → page 2373, “Executing a transfer descriptor”
    
    // Case 1: Link list is empty
    endpoint_queue_head[queue_head].next = (uint32_t)t;
    endpoint_queue_head[queue_head].status = 0;
    // Parse a new transfer descriptor from the queue head and prepare a
    // transmit/receive buffer:
    USB1->ENDPTPRIME |= primebit;
    while (USB1->ENDPTPRIME);
}

/* Called from locked ISR. */
void usb_packet_transmit(USBDriver *usbp, usbep_t ep, size_t n)
{
  const USBEndpointConfig *epc = usbp->epc[ep];
  USBInEndpointState *isp = epc->in_state;

  // → page 2364, “Data Phase”
  // TODO: set up descriptor

  printf_debug("usb_packet_transmit(n=%d)\n", n);
  transfer_t *t = &endpoint0_transfer_data;
  int notify = epc->in_cb != NULL;
  if (n == 0) {
    t = &endpoint0_transfer_ack;
  }
  usb_transfer_schedule(t, (uint32_t)isp->txbuf, n, 1, USB_ENDPTPRIME_PETB(1 << ep), notify);
}

/* Called from locked ISR. */
void usb_packet_receive(USBDriver *usbp, usbep_t ep, size_t n)
{
  const USBEndpointConfig *epc = usbp->epc[ep];
  USBOutEndpointState *osp = epc->out_state;

  printf_debug("usb_packet_receive(n=%d)\n", n);
  transfer_t *t = &endpoint0_transfer_data;
  int notify = epc->out_cb != NULL;
  if (n == 0) {
    t = &endpoint0_transfer_ack;
  }
  usb_transfer_schedule(t, (uint32_t)osp->rxbuf, n, 0, USB_ENDPTPRIME_PERB(1 << ep), notify);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*============================================================================*/

#if MIMXRT1062_USB_USE_USB1 || defined(__DOXYGEN__)
/**
 * @brief   USB interrupt handler.

 * @note → page 2210, “Interrupts”
 *
 * @isr
 */
OSAL_IRQ_HANDLER(MIMXRT1062_USB_IRQ_VECTOR) {
  USBDriver *usbp = &USBD1;
  OSAL_IRQ_PROLOGUE();

  printf_debug("usb interrupt handler\n");

  uint32_t status = USB1->USBSTS;
  USB1->USBSTS = status;

  // → page 2375, “Servicing Interrupts”
  if (status & USB_USBSTS_UI_MASK /* token done */) {
    // Execution Order 1a: check ENDPTSETUPSTAT
    uint32_t setupstat = USB1->ENDPTSETUPSTAT;
    while (setupstat) {
      printf_debug("  Copy and acknowledge setup buffer, setupstat=%d\n", setupstat);

      for (uint8_t ep = 0; ep < 6; ep++) {
	const USBEndpointConfig *epc = usbp->epc[ep];

	if (setupstat & USB_ENDPTSETUPSTAT_ENDPTSETUPSTAT(1 << ep)) {
	  printf_debug("  setup stat for endpoint %d\n", ep);	  
	  /* Clear receiving in the chibios state machine */
	  (usbp)->receiving &= ~(1 << ep);

	  /* Call SETUP function (ChibiOS core), which prepares
	   * for send or receive and releases the buffer
	   */
	  _usb_isr_invoke_setup_cb(usbp, ep);
	  // -> will call usb_lld_read_setup
	  // -> will call e.g. usb_lld_start_in
	}
      }

      // Repeat if a new setup packet has been received:
      setupstat = USB1->ENDPTSETUPSTAT;
    }

    // Execution Order 1b: Handle completion of dTD
    uint32_t complete = USB1->ENDPTCOMPLETE;
    if (complete) {
      USB1->ENDPTCOMPLETE = complete;
      printf_debug("  completion of dtd, complete = %d\n", complete);

      for (uint8_t ep = 0; ep < 6; ep++) {
	const USBEndpointConfig *epc = usbp->epc[ep];	
	if (complete & USB_ENDPTCOMPLETE_ERCE(1 << ep)) {
	  printf_debug("  endpoint receive complete for endpoint %d (transfer direction OUT)\n", ep);
	  (usbp)->receiving &= ~(1 << ep);
	  /* Endpoint Receive Complete Event */
	  /* Transfer Direction OUT */
	  if (epc->out_cb != NULL) {
	    printf_debug("  invoking out_cb\n");	
	    _usb_isr_invoke_out_cb(usbp, ep);
	  }
	}

	if (complete & USB_ENDPTCOMPLETE_ETCE(1 << ep)) {
	  printf_debug("  endpoint transmit complete for endpoint %d (transfer direction IN)\n", ep);	  
	  /* Endpoint Transmit Complete Event */
	  /* Transfer Direction IN */
	  if (epc->out_cb != NULL) {
	    (usbp)->transmitting &= ~1;      

	    if (epc->in_cb != NULL) {
	      printf_debug("  invoking in_cb\n");
	      _usb_isr_invoke_in_cb(usbp, ep);
	    }
	  }
	}
      }
    }
  }

  if (status & USB_USBSTS_URI_MASK) {
    printf_debug("  USB reset interrupt\n");
    _usb_reset(usbp);
  }
  
  if (status & USB_USBSTS_PCI_MASK) {
    printf_debug("  Port Change Interrupt\n");
    if (!(USB1->PORTSC1 & USB_PORTSC1_PR_MASK)) {
      if (USB1->PORTSC1 & USB_PORTSC1_HSP_MASK) {
	printf_debug("    USB High Speed :)\n");
      } else {
	printf_debug("    USB Full Speed :(\n");
      }
    }
  }
  
  if (status & USB_USBSTS_UEI_MASK) {
    printf_debug("  USB Error Interrupt\n");
  }

  if (status & USB_USBSTS_SLI_MASK) {
    printf_debug("  USB Suspend Interrupt\n");
  }

  printf_debug("usb interrupt done\n\n");  
  
  OSAL_IRQ_EPILOGUE();
}
#endif /* MIMXRT1062_USB_USE_USB1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
  /* Driver initialization.*/
  usbObjectInit(&USBD1);

  printf_debug("usb_lld_init()\n");  
  
#if MIMXRT1062_USB_USE_USB1

#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
  if (usbp->state != USB_STOP) {
    return; // already started
  }
  
#if MIMXRT1062_USB_USE_USB1
  if (usbp != &USBD1) {
    return; // unknown usbp
  }

  printf_debug("usb_lld_start() enter\n");

  // TODO: enable required clocks
#if 1 /* PJRC_CLOCKS */
  PMU->REG_3P0 = PMU_REG_3P0_OUTPUT_TRG(0x0F) |
    PMU_REG_3P0_BO_OFFSET(6) |
    PMU_REG_3P0_ENABLE_LINREG(1);
  
  //usb_init_serialnumber();
  
  // assume PLL3 is already running - already done by usb_pll_start() in hal_lld.c
  CCM->CCGR6 |= CCM_CCGR6_CG0(1) /* usboh3 clock */;
  
  printf_debug("BURSTSIZE=%08lX\n", USB1->BURSTSIZE);
  //USB1_BURSTSIZE = USB_BURSTSIZE_TXPBURST(4) | USB_BURSTSIZE_RXPBURST(4);
  USB1->BURSTSIZE = 0x0404;
  printf_debug("BURSTSIZE=%08lX\n", USB1->BURSTSIZE);
  printf_debug("USB1_TXFILLTUNING=%08lX\n", USB1->TXFILLTUNING);
#endif

#if 1 /* PJRC_RESET */
  if ((USBPHY1->PWD & (USBPHY_PWD_RXPWDRX_MASK |
		       USBPHY_PWD_RXPWDDIFF_MASK |
		       USBPHY_PWD_RXPWD1PT1_MASK |
		      USBPHY_PWD_RXPWDENV_MASK |
		       USBPHY_PWD_TXPWDV2I_MASK |
		       USBPHY_PWD_TXPWDIBIAS_MASK |
		      USBPHY_PWD_TXPWDFS_MASK)) ||
      (USB1->USBMODE & USB_USBMODE_CM_MASK)) {
    // USB controller is turned on from previous use
    // reset needed to turn it off & start from clean slate
    // → page 3292, “USBPHY1_CTRL”
    USBPHY1->CTRL_SET = USBPHY_CTRL_SFTRST(1); 
    USB1->USBCMD |= USB_USBCMD_RST(1); // reset controller
    
    int count=0;
    while (USB1->USBCMD & USB_USBCMD_RST_MASK) count++;
    
    nvicEnableVector(USB_OTG1_IRQn, MIMXRT1062_USB_USB1_IRQ_PRIORITY);
    
    USBPHY1->CTRL_CLR = USBPHY_CTRL_SFTRST(1); // reset PHY
    
    printf_debug("USB reset took %d loops\n", count);
    delay(25);
  }
#endif

  // TODO(docs): add a reference to the USBPHY1 initialization procedure
  USBPHY1->CTRL_CLR = USBPHY_CTRL_CLKGATE(1);
  USBPHY1->PWD = 0;
  
  // → page 2351, “Device Controller Initialization”
  
  // Set Controller Mode in the USB.USBMODE register to device mode:
  USB1->USBMODE = USB_USBMODE_CM(2 /* 0b10 = device controller */) |
    USB_USBMODE_SLOM(1);

  // Allocate and Initialize device queue heads in system memory:
  // Minimum: Initialize device queue heads 0 Tx & 0 Rx
  // All Device Queue Heads for control endpoints must be initialized before the endpoint is enabled.
  memset(endpoint_queue_head, 0, sizeof(endpoint_queue_head));
  endpoint_queue_head[0].config = (64 << 16) /* Maximum Packet Length. wMaxPacketSize */ |
				 (1 << 15) /* Interrupt On Setup (IOS) */;
  endpoint_queue_head[1].config = (64 << 16);

  // Configure ENDPOINTLISTADDR pointer:
  USB1->ENDPTLISTADDR = (uint32_t)&endpoint_queue_head;

  // Enable the microprocessor interrupt associated with the USB core:
  USB1->USBINTR = USB_USBINTR_UE(1) /* USB Interrupt Enable */ |
    USB_USBINTR_UEE(1) /* USB Error Interrupt Enable */ |
    USB_USBINTR_PCE(1) /* Port Change Detect */ |
    USB_USBINTR_URE(1) /* USB Reset Received */ |
    USB_USBINTR_SLE(1) /* Sleep Interrupt Enable */;

  // Set Run/Stop bit to Run Mode:
  USB1->USBCMD = USB_USBCMD_RS(1 /* 0b1 = run */);

  printf_debug("usb_lld_start() exit\n");

#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {
  /* TODO: If in ready state then disables the USB clock.*/
  if (usbp->state == USB_STOP) {
#if MIMXRT1062_USB_USE_USB1
    if (&USBD1 == usbp) {
      printf_debug("TODO: usb_lld_stop()\n");
    }
#endif /* MIMXRT1062_USB_USE_USB1 */
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
  // FIXME, dyn alloc
  //_usbbn = 0;

#if MIMXRT1062_USB_USE_USB1
      printf_debug("TODO: usb_lld_reset()\n");
      usbp->epc[0] = &ep0config;      
#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

#if MIMXRT1062_USB_USE_USB1
  printf_debug("usb_lld_set_address(%d)\n", usbp->address);

  // → page 2417, “Device Address”
  USB1->DEVICEADDR = USB_DEVICEADDR_USBADR(usbp->address) |
    USB_DEVICEADDR_USBADRA(1);
#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {

  if(ep > MIMXRT1062_USB_ENDPOINTS)
    return;
  printf_debug("TODO: usb_lld_init_endpoint()\n");

#if MIMXRT1062_USB_USE_USB1

#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  (void)usbp;
  uint8_t i;
  printf_debug("TODO: usb_lld_disable_endpoints()\n");  
#if MIMXRT1062_USB_USE_USB1
#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
#if MIMXRT1062_USB_USE_USB1
  printf_debug("TODO: usb_lld_get_status_out()\n");
  return EP_STATUS_DISABLED;
#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  if(ep > USB_MAX_ENDPOINTS)
    return EP_STATUS_DISABLED;
#if MIMXRT1062_USB_USE_USB1
  printf_debug("TODO: usb_lld_get_status_in()\n");  
  return EP_STATUS_DISABLED;
#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  /* Get the BDT entry */
  USBOutEndpointState *os = usbp->epc[ep]->out_state;

  printf_debug("usb_lld_read_setup\n");
  
  uint32_t setupstat = USB1->ENDPTSETUPSTAT;
  if (!setupstat) {
    return; // ENDPTSETUPSTAT unexpectedly cleared?!
  }

  // → page 2364, “Control Endpoint Operation Model > Setup Phase”

  // Clear setup bit and wait for clear to complete:
  USB1->ENDPTSETUPSTAT = (1 << ep);
  while (USB1->ENDPTSETUPSTAT & (1 << ep));

  setup_t s;
  do {
    // Setup TripWire (SUTW):
    USB1->USBCMD |= USB_USBCMD_SUTW(1);

    // Duplicate contents of dQH.SetupBuffer:
    s.word1 = endpoint_queue_head[0].setup0;
    s.word2 = endpoint_queue_head[0].setup1;

    // Repeat until Setup TripWire (SUTW) bit is set:
  } while (!(USB1->USBCMD & USB_USBCMD_SUTW_MASK));

  // Clear Setup TripWire (SUTW) bit:
  USB1->USBCMD &= ~USB_USBCMD_SUTW_MASK;

  // Note: After receiving a new setup packet the status and/or handshake
  // phases may still be pending from a previous control sequence. Flush
  // these before linking a new status/and or handshake dTD for the most
  // recent setup packet:
  USB1->ENDPTFLUSH = USB_ENDPTFLUSH_FETB(1 << ep) |
    USB_ENDPTFLUSH_FERB(1 << ep);
  while (USB1->ENDPTFLUSH & (USB_ENDPTFLUSH_FETB_MASK |
			     USB_ENDPTFLUSH_FERB_MASK));

  // drop pending callbacks within this interrupt on the floor:
  // pjrc accomplishes this by setting endpoint0_notify_mask=0;
  uint32_t complete = USB1->ENDPTCOMPLETE;
  USB1->ENDPTCOMPLETE = complete;

  // TODO(cleanup): more elegant way to copy the setup data to |buf|
  *(uint16_t *)buf = s.wRequestAndType;
  buf += 2;
  *(uint16_t *)buf = s.wValue;
  buf += 2;
  *(uint16_t *)buf = s.wIndex;
  buf += 2;
  *(uint16_t *)buf = s.wLength;
  buf += 2;
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  printf_debug("usb_lld_start_out()\n");  
  usb_packet_receive(usbp, ep, osp->rxsize);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @note      Called from ISR and locked zone.
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  printf_debug("usb_lld_start_in()\n");
  usb_packet_transmit(usbp, ep, usbp->epc[ep]->in_state->txsize);
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  printf_debug("TODO: usb_lld_stall_out()\n");      
#if MIMXRT1062_USB_USE_USB1

#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  printf_debug("TODO: usb_lld_stall_in()\n");        
#if MIMXRT1062_USB_USE_USB1

#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  printf_debug("TODO: usb_lld_clear_out()\n");          
#if MIMXRT1062_USB_USE_USB1

#endif /* MIMXRT1062_USB_USE_USB1 */
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  printf_debug("TODO: usb_lld_clear_in()\n");
#if MIMXRT1062_USB_USE_USB1

#endif /* MIMXRT1062_USB_USE_USB1 */
}

#endif /* HAL_USE_USB */

/** @} */
