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

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_dci.h"

usb_device_handle handle;

#define printf_init()
#define printf(...)
#define printf_debug_init()
#define printf_debug(...)

#include "chprintf.h"
#undef printf_debug
extern void printf_debug(const char *format, ...);

#include "fsl_clock.h"


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
USBInEndpointState ep0in __attribute__ ((used, aligned(32)));

/**
 * @brief   OUT EP0 state.
 */
USBOutEndpointState ep0out __attribute__ ((used, aligned(32)));

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


// TODO: use per-driver state instead

// Queue Heads must be laid out in memory in this order:
#define QH_OFFSET_OUT 0 /* receive, i.e. host to device */
#define QH_OFFSET_IN 1  /* transmit, i.e. device to host */

// → page 2345, 42.5.5, “Device Data Structures”
// must be aligned to a 2k boundary
// Two Queue Heads (1 rx, 1 tx) per endpoint:
endpoint_t endpoint_queue_head[(MIMXRT1062_USB_ENDPOINTS)*2] __attribute__ ((used, aligned(4096)));

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

void usb_transfer_schedule(endpoint_t *endpoint, transfer_t *t, uint32_t addr, size_t n, uint32_t primebit, int notify) {
    if ((uint32_t)t % 32 != 0) {
      chSysHalt("ERROR: hal_usb_lld: transfer not memory-aligned\n\n");
    }
  
    // The buffer associated with this transfer must be virtually contiguous. The
    // buffer may start on any byte boundary; however, for optimal utilization of
    // on-chip busses it is recommended to align the buffers on a 32-byte boundary

    printf_debug("    usb_transfer_schedule(endpoint=%x, transfer=%x, n=%d, primebit=%d, notify=%d)",
	       endpoint, t, n, primebit, notify);
    // → page 2371, “Building a transfer descriptor”
    // TODO: initialize first 7 dwords to 0
    t->next = (transfer_t*)1;
    t->status = (n << 16) | (notify ? (1 << 15) /* interrupt on complete */ : 0) | (1<<7) /* status = active */;
    t->pointer0 = addr;
    t->pointer1 = (addr + 0x1000) & 0xfffff000;
    t->pointer2 = (addr + 0x2000) & 0xfffff000;
    t->pointer3 = (addr + 0x3000) & 0xfffff000;
    t->pointer4 = (addr + 0x4000) & 0xfffff000;


    // → page 2373, “Executing a transfer descriptor”

    if (endpoint->head != NULL) {
      printf_debug("\n  ERROR: linked list unexpectedly not empty\n");
      while (1) { delay(500); };
    }

    // Case 1: Link list is empty
    endpoint->head = t;
    endpoint->tail = t;
    endpoint->next = (uint32_t)t;
    endpoint->status = 0;

    // Parse a new transfer descriptor from the queue head and prepare a
    // transmit/receive buffer:

    // See 42.5.6.4.2.2 Data Phase
    // NXP: USB_DeviceEhciTransfer()
    if (USB1->ENDPTPRIME) {
      printf_debug("\n  ERROR: overwriting prime bit\n");
    }
    USB1->ENDPTPRIME = primebit; // TODO: should this be |= instead of =?
    int primeTimesCount = 0;
    while (!(USB1->ENDPTSTAT & primebit)) {
      primeTimesCount++;
      if (USB1->ENDPTCOMPLETE & primebit) {
	break; // TODO: does this mean another transfer came in?
      } else {
	USB1->ENDPTPRIME = primebit; // retry priming
      }
    }
    printf_debug("    primeTimesCount=%d", primeTimesCount);
}

static endpoint_t *epqh(usbep_t ep, int offset)
{
  //printf_debug("epqh(ep=%d, offset=%d): index %d\n", ep, offset, index);
  return &(endpoint_queue_head[ep*2 + offset]);
}

/* Called from locked ISR. */
void usb_packet_transmit(USBDriver *usbp, usbep_t ep, size_t n, int notify)
{
  const USBEndpointConfig *epc = usbp->epc[ep];
  USBInEndpointState *isp = epc->in_state;

  printf_debug("  usb_packet_transmit(n=%d)", n);
  //   endpoint_queue_head[1].config = (64 << 16);
  endpoint_t *endpoint = epqh(ep, QH_OFFSET_IN);
  transfer_t *t = &isp->transfer;

  if (ep == 0) {
    endpoint_t *ep0tx = epqh(0, QH_OFFSET_IN);
    ep0tx->last_setup_offset = QH_OFFSET_IN;
  }

  printf_debug("    txbuf=%x", isp->txbuf);
  usbp->ep0_dir = USB_REQUEST_TYPE_DIR_IN;
  USB_DeviceSendRequest(handle, 0 /* endpointAddress */, isp->txbuf, n);
  //usb_transfer_schedule(endpoint, t, (uint32_t)isp->txbuf, n, USB_ENDPTPRIME_PETB(1 << ep), notify);
}

/* Called from locked ISR. */
void usb_packet_receive(USBDriver *usbp, usbep_t ep, size_t n, int notify)
{
  const USBEndpointConfig *epc = usbp->epc[ep];
  USBOutEndpointState *osp = epc->out_state;

  printf_debug("  usb_packet_receive(n=%d)", n);
  endpoint_t *endpoint = epqh(ep, QH_OFFSET_OUT);
  transfer_t *t = &osp->transfer;

  if (ep == 0) {
    endpoint_t *ep0rx = epqh(0, QH_OFFSET_OUT);
    ep0rx->last_setup_offset = QH_OFFSET_OUT;
  }

  printf_debug("    rxbuf=%x", osp->rxbuf);
  usbp->ep0_dir = USB_REQUEST_TYPE_DIR_OUT;
  USB_DeviceRecvRequest(handle, 0 /* endpointAddress */, osp->rxbuf, n);
  //usb_transfer_schedule(endpoint, t, (uint32_t)osp->rxbuf, n, USB_ENDPTPRIME_PERB(1 << ep), notify);
}

static void clear_out_queue(USBDriver *usbp, usbep_t ep, int offset)
{
  printf_debug("    clear_out_queue(ep=%d)", ep);

  const USBEndpointConfig *epc = usbp->epc[ep];
  endpoint_t *endpoint = epqh(ep, offset);
  transfer_t *t = endpoint->head;
  printf_debug("    clear_out_queue, endpoint = %x (offset %d), t = %x", endpoint, offset, t);

  while (t) {
    // Check if transfer is still active or already completed:
    // TODO: compute how many bytes were transfered (store bytes at beginning of transfer)
    // TODO: set rxcnt accordingly
    const uint16_t bytes_left = (t->status & 0xFFFF0000) >> 16;
    const int active = (t->status & 0x80U);

    printf_debug("      status = %d, total_bytes=%d",
		 t->status,
		 bytes_left);

    if (active) {
      printf_debug("      dtd is still active!");
    } else {
      printf_debug("      dtd completed!");
    }

    if (endpoint->head == endpoint->tail) {
      // Arrived at last element of the queue, set pointers to NULL
      endpoint->head = NULL;
      endpoint->tail = NULL;

      endpoint->next = 1; // terminate
      endpoint->status = 0;
    } else {
      // Advance our queue head pointer
      endpoint->head = (transfer_t*)(t->next);
    }
    transfer_t *next = endpoint->head;
    const bool at_end_of_queue = next == NULL;

    // Call event handlers and update ChibiOS USB driver state
    if (/* TODO(correctness): if ioc == 1 */ at_end_of_queue) {
      if (offset == QH_OFFSET_OUT) {
	printf_debug("    clear_out_complete ep=%d (OUT), rxbuf=%x", ep, epc->out_state->rxbuf);
	(usbp)->receiving &= ~(1 << ep);

	USBOutEndpointState *osp = epc->out_state;
	osp->rxcnt = osp->rxsize - bytes_left;

	printf_debug("    received %d bytes", osp->rxcnt);

	/* Endpoint Receive Complete Event */
	/* Transfer Direction OUT */
	if (epc->out_cb != NULL) {
	  printf_debug("    invoking out_cb for ep %d", ep);
	  _usb_isr_invoke_out_cb(usbp, ep);
	}
      }

      if (offset == QH_OFFSET_IN) {
	printf_debug("    clear_out_complete ep=%d (IN), txbuf=%x", ep, epc->in_state->txbuf);
	(usbp)->transmitting &= ~(1 << ep);
	/* Endpoint Transmit Complete Event */
	/* Transfer Direction IN */
	if (epc->in_cb != NULL) {
	  printf_debug("    invoking in_cb for ep %d", ep);
	  _usb_isr_invoke_in_cb(usbp, ep);
	}
      }
    } else {
      printf_debug("\n    ERROR: not at end of linked list\n");
    }

    t->status = 0;
    t = next;
    if (t != NULL) {
      printf_debug("\n   ERROR: more than 1 queue entry!\n");
    }

    // TODO: this is where we would need to prime descriptors if we had a queue
    // with multiple descriptors!
  }
}


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*============================================================================*/

#if MIMXRT1062_USB_USE_USB1 || defined(__DOXYGEN__)


static void cancel_control_pipe(USBDriver *usbp, usbep_t ep, int offset)
{
  printf_debug("    cancel_control_pipe(ep=%d, offset=%d)", ep, offset);

  endpoint_t *endpoint = epqh(ep, offset);
  transfer_t *currentDtd = endpoint->head;

  while (currentDtd) {
    /* Move the dtd head pointer to next. */
    /* If the pointer of the head equals to the tail, set the dtd queue to null. */
    if (endpoint->head == endpoint->tail) {
      endpoint->head = NULL;
      endpoint->tail = NULL;
      endpoint->next = 1;
      endpoint->status = 0;
    } else {
      endpoint->head = currentDtd->next;
    }

#if 0
    /* When the ioc is set or the dtd queue is empty, the up layer will be notified. */
      {
	message.code = endpoint | (uint8_t)((uint32_t)direction << 0x07U);
	message.isSetup = 0U;
	USB_DeviceNotificationTrigger(ehciState->deviceHandle, &message);
	message.buffer = NULL;
	message.length = 0U;
      }
#endif

    /* Clear the token field of the dtd. */
    currentDtd->status = 0;
    currentDtd->next = (transfer_t*)1;

    /* Get the next in-used dtd. */
    currentDtd = endpoint->head;
  }
}


/**
 * @brief   USB interrupt handler.

 * @note → page 2210, “Interrupts”
 *
 * @isr
 */
OSAL_IRQ_HANDLER(MIMXRT1062_USB0_IRQ_VECTOR) {
  USBDriver *usbp = &USBD1;
  OSAL_IRQ_PROLOGUE();

  printf_debug("ISR\n");

  USB_DeviceEhciIsrFunction(handle);
  
#if 0
  uint32_t status = USB1->USBSTS;
  USB1->USBSTS = status;

  // → page 2375, “Servicing Interrupts”
  if (status & USB_USBSTS_UI_MASK /* token done */) {
    printf_debug("\ntokendone");
    // Execution Order 1a: check ENDPTSETUPSTAT
    uint32_t setupstat = USB1->ENDPTSETUPSTAT;
    while (setupstat) {
      for (uint8_t ep = 0; ep < MIMXRT1062_USB_ENDPOINTS; ep++) {
	const USBEndpointConfig *epc = usbp->epc[ep];

	if (setupstat & USB_ENDPTSETUPSTAT_ENDPTSETUPSTAT(1 << ep)) {
	  printf_debug("  setup ep=%d", ep);

	  // The NXP stack cancels the data/status phase transfers,
	  // presumably because they can safely be discarderd
	  // (zero-length status confirmation packets)
	  // when a new setup is received.

	  /* Clear receiving in the chibios state machine */
	  (usbp)->receiving &= ~(1 << ep);
	  (usbp)->transmitting &= ~(1 << ep);

	  // TODO: is the order in which we are clearing (direction in/out) relevant?
	  cancel_control_pipe(usbp, ep, QH_OFFSET_OUT);
	  cancel_control_pipe(usbp, ep, QH_OFFSET_IN);

	  /* Call SETUP function (ChibiOS core), which prepares
	   * for send or receive and releases the buffer
	   */
	  if (epc->setup_cb != NULL) {
	    _usb_isr_invoke_setup_cb(usbp, ep);
	    // -> will call usb_lld_read_setup
	    // -> will call e.g. usb_lld_start_in
	  }
	}
      }

      break;
      // Repeat if a new setup packet has been received:
      setupstat = USB1->ENDPTSETUPSTAT;
    }

    // Execution Order 1b: Handle completion of dTD
    const uint32_t complete = USB1->ENDPTCOMPLETE;
    if (complete) {
      USB1->ENDPTCOMPLETE = complete;

      for (uint8_t ep = 0; ep < MIMXRT1062_USB_ENDPOINTS; ep++) {
	if (complete & USB_ENDPTCOMPLETE_ERCE(1 << ep)) { // bit 0 to 16
	  printf_debug("  complete ep=%d (OUT)", ep);
	  clear_out_queue(usbp, ep, QH_OFFSET_OUT);
	}

	if (complete & USB_ENDPTCOMPLETE_ETCE(1 << ep)) { // bit 16 and higher
	  printf_debug("  complete ep=%d (IN)", ep);
	  clear_out_queue(usbp, ep, QH_OFFSET_IN);
	}
      }
    }
    printf_debug("tokendone done\n");
  }

  if (status & USB_USBSTS_URI_MASK) {
    printf_debug("  USB reset interrupt");
    _usb_reset(usbp);
  }

  if (status & USB_USBSTS_PCI_MASK) {
    printf_debug("  Port Change Interrupt");
    if (!(USB1->PORTSC1 & USB_PORTSC1_PR_MASK)) {
      if (USB1->PORTSC1 & USB_PORTSC1_HSP_MASK) {
	printf_debug("    USB High Speed :)");
      } else {
	printf_debug("    USB Full Speed :(");
      }
    }
  }

  if (status & USB_USBSTS_UEI_MASK) {
    printf_debug("  USB Error Interrupt");
  }

  if (status & USB_USBSTS_SLI_MASK) {
    printf_debug("  USB Suspend Interrupt");
  }

  if (status & USB_USBSTS_SRI_MASK) {
    //printf_debug("  USB Start Of Frame (SOF) interrupt\n");
    _usb_isr_invoke_sof_cb(usbp);
  }

  //printf_debug("end-of-ISR\n\n");
#endif
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

#if MIMXRT1062_USB_USE_USB1

#endif /* MIMXRT1062_USB_USE_USB1 */
}

usb_status_t usb_device_callback(usb_device_handle handle, uint32_t callbackEvent, void *eventParam) {
  USBDriver *usbp = &USBD1;
  printf_debug("usb_device_callback(event=%d)", callbackEvent);

  switch (callbackEvent) {
  case kUSB_DeviceEventBusReset:
    printf_debug("  bus reset");

    // Initialize the control pipes via usb_lld_reset():
    _usb_reset(usbp);
    break;

  case kUSB_DeviceEventSetConfiguration:
    printf_debug("  set configuration");

    /* When the application receives this event, the host has sent a set
       configuration request. The configuration value can be received from the
       parameter eventParam. In the event, the application configuration can be
       set. Initialize each interface in the current configuration by using zero
       as an alternate setting. */
    break;

  case kUSB_DeviceEventSetInterface:
    printf_debug("  set interface");

/* • kUsbDeviceEventSetInterface */
/* When the application receives this event, the host sent a set alternate setting request of an interface. */
/* The interface and alternate setting value can be received from the parameter eventParam. The event- */
/* Param points to a uint16_t variable. The high 8-bit is interface value and the low 8-bit is alternate */
/* setting. In the event, the application changes the alternate setting of this interface if the new alternate */
/* setting is not equal to the current setting. */
/* Normally, change the steps as follows: */
/* 1. Cancel all transfers of the current alternate setting in this interface. */
/* 2. De-initialize all pipes of the current alternate setting in this interface. */
/* 3. Initialize all pipes of the new alternate setting in this interface. */
/* 4. Prime the transfers of the new setting. */
/* For example, */
/* uint16_t* */
/* temp16 = (uint16_t*)eventParam; */
/* uint8_t */
/* interface = (uint8_t)((*temp16&0xFF00)>>0x08); */
/* currentAlternateSetting[interface] = (uint8_t)(*temp16&0x00FF); */
/* The device callback event work flow: */
    break;
  }
  
  return kStatus_USB_Success;
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
      printf_debug("already started!");
    return; // already started
  }

#if MIMXRT1062_USB_USE_USB1
  if (usbp != &USBD1) {
    return; // unknown usbp
  }

  printf_debug("  usb_lld_start() enter");

#if 1 /* NXP_EXTRA_CLOCKS */
  /* USB PHY configuration */
#ifndef BOARD_USB_PHY_D_CAL
#define BOARD_USB_PHY_D_CAL (0x0CU)
#endif
#ifndef BOARD_USB_PHY_TXCAL45DP
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#endif
#ifndef BOARD_USB_PHY_TXCAL45DM
#define BOARD_USB_PHY_TXCAL45DM (0x06U)
#endif
  /* usb_phy_config_struct_t phyConfig = { */
  /*   BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM, */
  /* }; */
  uint32_t notUsed = 0;

  CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
  CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);

  //USB_EhciPhyInit(USB_DEVICE_CONTROLLER_ID, notUsed, &phyConfig);

#endif



  if (USB_DeviceInit(kUSB_ControllerEhci0, usb_device_callback, &handle) != kStatus_USB_Success) {
    printf_debug("allocating handle failed");
    return;
  }
  printf_debug("handle allocated");

  USB_DeviceRun(handle);
  printf_debug("device running");

  nvicEnableVector(USB_OTG1_IRQn, MIMXRT1062_USB_USB1_IRQ_PRIORITY);
  printf_debug("interrupt enabled");
  
  return;
  
#if 1 /* PJRC_CLOCKS */
  PMU->REG_3P0 = PMU_REG_3P0_OUTPUT_TRG(0x0F) |
    PMU_REG_3P0_BO_OFFSET(6) |
    PMU_REG_3P0_ENABLE_LINREG(1);

  //usb_init_serialnumber();

  // assume PLL3 is already running - already done by usb_pll_start() in hal_lld.c
  CCM->CCGR6 |= CCM_CCGR6_CG0(1) /* usboh3 clock */;

  //USB1_BURSTSIZE = USB_BURSTSIZE_TXPBURST(4) | USB_BURSTSIZE_RXPBURST(4);
  USB1->BURSTSIZE = 0x0404;
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
    
    USBPHY1->CTRL_CLR = USBPHY_CTRL_SFTRST(1); // reset PHY

    printf_debug("USB reset took %d loops", count);
    delay(25);
  }
#endif

  // TODO(docs): add a reference to the USBPHY1 initialization procedure
  USBPHY1->CTRL_CLR = USBPHY_CTRL_CLKGATE(1);
  USBPHY1->PWD = 0;

  // → page 2351, “Device Controller Initialization”

  // Set Controller Mode in the USB.USBMODE register to device mode, and disable
  // setup lockout (old mechanism, results in USB compliance issues, unnecessary
  // when using the setup tripwire mechanism instead (SUTW).
  // → page 2364, “Control Endpoint Operation Model > Setup Phase”
  USB1->USBMODE = USB_USBMODE_CM(2 /* 0b10 = device controller */) |
    USB_USBMODE_SLOM(1);

  //USB1->PORTSC1 |= (1 << 24) /* Force to Full speed */;

  // Allocate and Initialize device queue heads in system memory:
  // Minimum: Initialize device queue heads 0 Tx & 0 Rx
  // All Device Queue Heads for control endpoints must be initialized before the endpoint is enabled.
  memset(endpoint_queue_head, 0, sizeof(endpoint_queue_head));

  // → page 2347, 42.5.5.1.1 Endpoint Capabilities/Characteristics
  // tamago/soc/imx6/usb sets only eqh[1] ios=1
  // NXP sets eqh[0] ios=1 and eqh[1] ios=1
  // PJRC sets only eqh[0] ios=1
  endpoint_queue_head[0].config = (64 << 16) /* Maximum Packet Length. wMaxPacketSize */ |
				 (1 << 15) /* Interrupt On Setup (IOS) */;
  endpoint_queue_head[1].config = (64 << 16) | (1 << 15);

  // Configure ENDPOINTLISTADDR pointer:
  USB1->ENDPTLISTADDR = (uint32_t)&endpoint_queue_head;

  // Enable the microprocessor interrupt associated with the USB core:
  USB1->USBINTR = USB_USBINTR_UE(1) /* USB Interrupt Enable */ |
    USB_USBINTR_UEE(1) /* USB Error Interrupt Enable */ |
    USB_USBINTR_PCE(1) /* Port Change Detect */ |
    USB_USBINTR_URE(1) /* USB Reset Received */ |
    USB_USBINTR_SLE(1) /* Sleep Interrupt Enable */ |
    USB_USBINTR_SRE(1) /* Start Of Frame */;

  nvicEnableVector(USB_OTG1_IRQn, MIMXRT1062_USB_USB1_IRQ_PRIORITY);

  // Set Run/Stop bit to Run Mode:
  USB1->USBCMD = USB_USBCMD_RS(1 /* 0b1 = run */);

  printf_debug("usb_lld_start() exit");

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
      printf_debug("usb_lld_stop()");
      nvicDisableVector(USB_OTG1_IRQn);
    }
#endif /* MIMXRT1062_USB_USE_USB1 */
  }
}

static usb_status_t device_ep0_control_callback(usb_device_handle handle,
						usb_device_endpoint_callback_message_struct_t *message,
						void *callbackParam) {
  USBDriver *usbp = &USBD1;
  printf_debug("device_ep0_control_callback");
  if (message->isSetup) {
    printf_debug("setup message for endpoint 0");
    uint8_t ep = 0; // TODO: take from message->code
    const USBEndpointConfig *epc = usbp->epc[ep];
    // TODO: handle setup message
    /* Clear receiving in the chibios state machine */
    (usbp)->receiving &= ~(1 << ep);
    (usbp)->transmitting &= ~(1 << ep);

    usb_setup_struct_t *ds = (usb_setup_struct_t*)message->buffer;
    printf_debug("native wValue = %d / wIndex = %d / wLength = %d",
		 ds->wValue,
		 ds->wIndex,
		 ds->wLength);
    printf_debug("swapped wValue = %d / wIndex = %d / wLength = %d",
		 USB_SHORT_FROM_LITTLE_ENDIAN(ds->wValue),
		 USB_SHORT_FROM_LITTLE_ENDIAN(ds->wIndex),
		 USB_SHORT_FROM_LITTLE_ENDIAN(ds->wLength));
    memcpy(usbp->setup, message->buffer, message->length);
    
    /* Call SETUP function (ChibiOS core), which prepares
     * for send or receive and releases the buffer
     */
    if (epc->setup_cb != NULL) {
      _usb_isr_invoke_setup_cb(usbp, ep);
      // -> will call usb_lld_read_setup
      // -> will call e.g. usb_lld_start_in
    }
    return;
  }
  // TODO: describe under which circumstances this callback is called. seems
  // like there are a number of situations, e.g. transfer completion (but what
  // about e.g. message->length == 0?)
  printf_debug("non-setup message control callback for buffer %x, len=%d!", message->buffer, message->length);

  /* if (message->length == 0) { */
  /*   return; // no action required */
  /* } */
  
  uint8_t ep = 0; // TODO
  const USBEndpointConfig *epc = usbp->epc[ep];

  if (usbp->ep0_dir == USB_REQUEST_TYPE_DIR_OUT) {
    printf_debug("    complete, OUT ep=%d (OUT), rxbuf=%x", ep, epc->out_state->rxbuf);
    (usbp)->receiving &= ~(1 << ep);

    USBOutEndpointState *osp = epc->out_state;
    const uint32_t bytes_left = 0; // TODO
    osp->rxcnt = osp->rxsize - bytes_left;

    printf_debug("    received %d bytes", osp->rxcnt);

    /* Endpoint Receive Complete Event */
    /* Transfer Direction OUT */
    if (epc->out_cb != NULL) {
      printf_debug("    invoking out_cb for ep %d", ep);
      _usb_isr_invoke_out_cb(usbp, ep);
    }
  } else {
    printf_debug("    complete, IN ep=%d (IN), txbuf=%x", ep, epc->in_state->txbuf);
    (usbp)->transmitting &= ~(1 << ep);
    /* Endpoint Transmit Complete Event */
    /* Transfer Direction IN */
    if (epc->in_cb != NULL) {
      printf_debug("    invoking in_cb for ep %d", ep);
      _usb_isr_invoke_in_cb(usbp, ep);
    }
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

  usbp->epc[0] = &ep0config;
  
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t epCallback;
    usb_status_t status;

    epCallback.callbackFn    = device_ep0_control_callback;
    epCallback.callbackParam = NULL;

    // NXP sets zlt = 1, but that makes enumeration hang when a packet with 64
    // bytes is transmitted:
    // https://community.nxp.com/t5/MQX-Software-Solutions/MQX-USBHS-Control-Endpoint-Zero-Length-Termination/m-p/375730
    epInitStruct.zlt             = 0; 
    epInitStruct.transferType    = USB_ENDPOINT_CONTROL;
    epInitStruct.interval        = 0;
    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    epInitStruct.maxPacketSize   = USB_CONTROL_MAX_PACKET_SIZE;
    /* Initialize the control IN pipe */
    status = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

    if (kStatus_USB_Success != status)
    {
      printf_debug("USB_DeviceInitEndpoint() failed: %d", status);
      return;
    }
    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    /* Initialize the control OUT pipe */
    status = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

    if (kStatus_USB_Success != status)
    {
        (void)USB_DeviceDeinitEndpoint(
            handle, USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
	printf_debug("USB_DeviceInitEndpoint() failed: %d", status);
        return;
    }

    return;
  
#if 0 /*MIMXRT1062_USB_USE_USB1*/
      printf_debug("usb_lld_reset()");
      usbp->epc[0] = &ep0config;

      // → page 2354, 42.5.6.2.1, “Bus Reset”

      // Clear the setup flag:
      uint32_t status = USB1->ENDPTSETUPSTAT;
      USB1->ENDPTSETUPSTAT = status;
      // Clear the endpoint complete flag:
      status = USB1->ENDPTCOMPLETE;
      USB1->ENDPTCOMPLETE = status;

      do {
	// Flush pending transfers
	USB1->ENDPTFLUSH = USB_ENDPTFLUSH_FERB_MASK | USB_ENDPTFLUSH_FETB_MASK;
      } while (USB1->ENDPTPRIME & (USB_ENDPTPRIME_PERB_MASK | USB_ENDPTPRIME_PETB_MASK));

      for (int i = 0; i < (MIMXRT1062_USB_ENDPOINTS)*2; i++) {
	endpoint_t *endpoint = &(endpoint_queue_head[i]);
	endpoint->head = NULL;
	endpoint->tail = NULL;
	//uint16_t maxsize = (i%2 == QH_OFFSET_OUT ? usbp->epc[i/2]->out_maxsize : usbp->epc[i/2]->in_maxsize);
	uint16_t maxsize = (i == 0 || i == 1 ? 64 : 512);
	uint32_t config = (maxsize << 16);
	if (i == 0 || i == 1 /* endpoint 0, IN or OUT */) {
	  config |= (1 << 15) /* Interrupt On Setup (IOS) */;
	}
	config |= (1 << 29) /* disable automatic zlt */;
	endpoint->config = config;
	endpoint->next = 1;
      }

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
  printf_debug("usb_lld_set_address(%d)", usbp->address);

  usb_status_t error = kStatus_USB_InvalidRequest;
  uint8_t state      = 0U;

  USB_DeviceGetStatus(handle, kUSB_DeviceStatusDeviceState, &state);

  printf_debug("  DeviceStatusDeviceState=%x", state);

  if (((uint8_t)kUSB_DeviceStateAddressing != state) && ((uint8_t)kUSB_DeviceStateAddress != state) &&
      ((uint8_t)kUSB_DeviceStateDefault != state) && ((uint8_t)kUSB_DeviceStateConfigured != state))
    {
      printf_debug("  invalid state!");
      return;
    }

  if ((uint8_t)kUSB_DeviceStateAddressing != state)
    {
      /* If the device address is not setting, pass the address and the device state will change to
       * kUSB_DeviceStateAddressing internally. */
      state = (uint8_t)(usbp->address & 0xFF);
      error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusAddress, &state);
    }
  else
    {
      /* If the device address is setting, set device address and the address will be write into the controller
       * internally. */
      error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusAddress, NULL);
      /* And then change the device state to kUSB_DeviceStateAddress. */
      if (kStatus_USB_Success == error)
        {
	  state = (uint8_t)kUSB_DeviceStateAddress;
	  error = USB_DeviceSetStatus(handle, kUSB_DeviceStatusDeviceState, &state);
        }
    }

  return error;


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
  printf_debug("usb_lld_init_endpoint(ep=%d)", ep);

#if MIMXRT1062_USB_USE_USB1
  const USBEndpointConfig *epc = usbp->epc[ep];

  // → page 2357, 42.5.6.3.1, Endpoint Initialization

  if (epc->out_state == NULL && epc->in_state == NULL) {
    printf_debug("  BUG: unknown endpoint type?!");
    return;
  }

  uint32_t mask = 0;
  if (epc->out_state != NULL) {
    endpoint_t *endpoint = epqh(ep, QH_OFFSET_OUT);
    // Set up Endpoint Queue Head (dQH)
    endpoint->config = (epc->out_maxsize << 16) | (1 << 29) /* disable automatic zlt */;
    endpoint->next = 1;

    /* OUT endpoint, enable OUT direction */
    mask |= USB_ENDPTCTRL_RXE(1) |
      USB_ENDPTCTRL_RXR(1) |
      USB_ENDPTCTRL_RXT(epc->ep_mode);
    printf_debug("  OUT endpoint (RX enable), mode=%d, maxsize=%d", epc->ep_mode, epc->out_maxsize);
  }

  if (epc->in_state != NULL) {
    endpoint_t *endpoint = epqh(ep, QH_OFFSET_IN);
    // Set up Endpoint Queue Head (dQH)
    endpoint->config = (epc->in_maxsize << 16)  | (1 << 29) /* disable automatic zlt */;;
    endpoint->next = 1;

    /* IN endpoint, enable IN direction */
    mask |= USB_ENDPTCTRL_TXE(1) |
      USB_ENDPTCTRL_TXR(1) |
      USB_ENDPTCTRL_TXT(epc->ep_mode);
    printf_debug("  IN endpoint (TX enable), mode=%d, maxsize=%d", epc->ep_mode, epc->in_maxsize);
  }

  USB1->ENDPTCTRL[ep-1] = mask;

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
  printf_debug("TODO: usb_lld_disable_endpoints()");
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
  if(ep > USB_MAX_ENDPOINTS)
    return EP_STATUS_DISABLED;
#if MIMXRT1062_USB_USE_USB1
  uint32_t ctrl = USB1->ENDPTCTRL[ep-1];
  printf_debug("usb_lld_get_status_out(), ctrl=%d", ctrl);
  if (!(ctrl & USB_ENDPTCTRL_RXE_MASK)) {
    return EP_STATUS_DISABLED;
  }
  if (ctrl & USB_ENDPTCTRL_RXS_MASK) {
    return EP_STATUS_STALLED;
  }
  return EP_STATUS_ACTIVE;
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
  if(ep > USB_MAX_ENDPOINTS)
    return EP_STATUS_DISABLED;
#if MIMXRT1062_USB_USE_USB1
  uint32_t ctrl = USB1->ENDPTCTRL[ep-1];
  printf_debug("usb_lld_get_status_in(), ctrl=%d", ctrl);
  if (!(ctrl & USB_ENDPTCTRL_TXE_MASK)) {
    return EP_STATUS_DISABLED;
  }
  if (ctrl & USB_ENDPTCTRL_TXS_MASK) {
    return EP_STATUS_STALLED;
  }
  return EP_STATUS_ACTIVE;
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
  printf_debug("usb_lld_read_setup NOOP");
  return;

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
    endpoint_t *endpoint = epqh(ep, QH_OFFSET_OUT);
    s.word1 = endpoint->setup0;
    s.word2 = endpoint->setup1;

    // Repeat until Setup TripWire (SUTW) bit is set:
  } while (!(USB1->USBCMD & USB_USBCMD_SUTW_MASK));

  // Clear Setup TripWire (SUTW) bit:
  USB1->USBCMD &= ~USB_USBCMD_SUTW_MASK;

  // → I confirmed that the NXP stack does not do ENDPTFLUSH somewhere
#if 0 /* This is the wrong kind of flushing, I think. */
  // Note: After receiving a new setup packet the status and/or handshake
  // phases may still be pending from a previous control sequence. Flush
  // these before linking a new status/and or handshake dTD for the most
  // recent setup packet:
  USB1->ENDPTFLUSH = USB_ENDPTFLUSH_FETB(1 << ep) |
    USB_ENDPTFLUSH_FERB(1 << ep);
  while (USB1->ENDPTFLUSH & (USB_ENDPTFLUSH_FETB_MASK |
			     USB_ENDPTFLUSH_FERB_MASK));

  usbp->receiving &= ~((uint16_t)((unsigned)1U << (unsigned)ep));
  usbp->transmitting &= ~((uint16_t)((unsigned)1U << (unsigned)ep));
#endif

  printf_debug("bmRequestType = %x, bRequest = %x, wValue = %x, wIndex = %x, wLength = %x",
	       s.bmRequestType, s.bRequest, s.wValue, s.wIndex, s.wLength);

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
  const USBEndpointConfig *epc = usbp->epc[ep];
  USBOutEndpointState *osp = epc->out_state;

  printf_debug("  usb_lld_start_out(ep=%d) (receive)", ep);

  if (osp->rxsize > epc->out_maxsize) {
    osp->rxsize = epc->out_maxsize;
  }

  usb_packet_receive(usbp, ep, osp->rxsize, epc->out_cb != NULL);
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
  const USBEndpointConfig *epc = usbp->epc[ep];
  USBInEndpointState *isp = epc->in_state;
  printf_debug("  usb_lld_start_in(ep=%d) (transmit)", ep);

  size_t n = isp->txsize;
  if (n > epc->in_maxsize) {
    printf_debug("  txsize=%d EXCEEDS MAX SIZE %d", isp->txsize, epc->in_maxsize);
    // continue anyway, otherwise the device does not enumerate
    //n = epc->in_maxsize;
  }
  usb_packet_transmit(usbp, ep, n, epc->in_cb != NULL);
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
  printf_debug("usb_lld_stall_out(ep=%d)", ep);
#if MIMXRT1062_USB_USE_USB1
  USB1->ENDPTCTRL[ep-1] |= USB_ENDPTCTRL_RXS(1);
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
  printf_debug("usb_lld_stall_in(ep=%d)", ep);

#if MIMXRT1062_USB_USE_USB1
  USB1->ENDPTCTRL[ep-1] |= USB_ENDPTCTRL_TXS(1);
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
  printf_debug("usb_lld_clear_out(ep=%d)", ep);
#if MIMXRT1062_USB_USE_USB1
  USB1->ENDPTCTRL[ep-1] &= ~USB_ENDPTCTRL_RXS_MASK;
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
  printf_debug("usb_lld_clear_in(ep=%d)", ep);
#if MIMXRT1062_USB_USE_USB1
  USB1->ENDPTCTRL[ep-1] &= ~USB_ENDPTCTRL_TXS_MASK;
#endif /* MIMXRT1062_USB_USE_USB1 */
}

#endif /* HAL_USE_USB */

/** @} */
