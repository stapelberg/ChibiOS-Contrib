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
#include "usb_device_ehci.h"

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

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint8_t epaddr(usbep_t ep, uint8_t direction) {
  return ep | (direction << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
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
OSAL_IRQ_HANDLER(MIMXRT1062_USB0_IRQ_VECTOR) {
  USBDriver *usbp = &USBD1;
  OSAL_IRQ_PROLOGUE();

  // Do not print as long as SOF interrupt is enabled to avoid slowing down the
  // application due to serial print latency:
  //printf_debug("ISR\n");

  // inlined from USB_DeviceEhciIsrFunction(handle) so that we can dispatch the
  // SOF (start of frame) interrupt:
  usb_device_struct_t *dev_handle = (usb_device_struct_t *)handle;
  usb_device_ehci_state_struct_t *ehciState;
  uint32_t status;

  ehciState = (usb_device_ehci_state_struct_t *)(dev_handle->controllerHandle);

  status = ehciState->registerBase->USBSTS;
  status &= ehciState->registerBase->USBINTR;

  USB_DeviceEhciIsrFunction(handle);

  // Some ChibiOS components require the SOF interrupt, such as the USB serial.
  if ((status & USBHS_USBSTS_SRI_MASK)) {
    _usb_isr_invoke_sof_cb(usbp);
  }
  
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
  (void)handle;
  (void)eventParam;

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
  //uint32_t notUsed = 0;

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
  if (usbp->state == USB_STOP) {
#if MIMXRT1062_USB_USE_USB1
    if (&USBD1 == usbp) {
      printf_debug("usb_lld_stop()");
      USB_DeviceStop(handle);
      nvicDisableVector(USB_OTG1_IRQn);
    }
#endif /* MIMXRT1062_USB_USE_USB1 */
  }
}

static usb_status_t device_ep0_control_callback(usb_device_handle handle,
						usb_device_endpoint_callback_message_struct_t *message,
						void *callbackParam,
						uint8_t direction) {
  (void)handle;
  (void)callbackParam;

  USBDriver *usbp = &USBD1;
  printf_debug("device_ep0_control_callback");
  const uint8_t ep = 0;
  const USBEndpointConfig *epc = usbp->epc[ep];
  if (message->isSetup) {
    printf_debug("setup message for endpoint 0");

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
    return kStatus_USB_Success;
  }
  // TODO: describe under which circumstances this callback is called. seems
  // like there are a number of situations, e.g. transfer completion (but what
  // about e.g. message->length == 0?)
  printf_debug("non-setup message control callback for buffer %x, len=%d!", message->buffer, message->length);

  if (message->length == USB_CANCELLED_TRANSFER_LENGTH) {
    printf_debug("USB transfer cancelled");
    return kStatus_USB_InvalidRequest;
  }
  
  /* if (message->length == 0) { */
  /*   return; // no action required */
  /* } */
 

  if (direction == USB_OUT) {
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
  } else if (direction == USB_IN) {
    printf_debug("    complete, IN ep=%d (IN), txbuf=%x", ep, epc->in_state->txbuf);
    (usbp)->transmitting &= ~(1 << ep);
    /* Endpoint Transmit Complete Event */
    /* Transfer Direction IN */
    if (epc->in_cb != NULL) {
      printf_debug("    invoking in_cb for ep %d", ep);
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  } else if (message->buffer == usbp->ep0next) {
    printf_debug("TODO: setup complete");
  } else {
    printf_debug("BUG BUG BUG: ep0 unexpected setup");
  }

  return kStatus_USB_Success;
}

static usb_status_t device_ep0_control_callback_in(usb_device_handle handle,
						usb_device_endpoint_callback_message_struct_t *message,
						void *callbackParam) {
  return device_ep0_control_callback(handle, message, callbackParam, USB_IN);
}

static usb_status_t device_ep0_control_callback_out(usb_device_handle handle,
						usb_device_endpoint_callback_message_struct_t *message,
						void *callbackParam) {
  return device_ep0_control_callback(handle, message, callbackParam, USB_OUT);
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

    epCallback.callbackFn    = device_ep0_control_callback_in;
    epCallback.callbackParam = NULL;

    // NXP sets zlt = 1, but that makes enumeration hang when a packet with 64
    // bytes is transmitted:
    // https://community.nxp.com/t5/MQX-Software-Solutions/MQX-USBHS-Control-Endpoint-Zero-Length-Termination/m-p/375730
    epInitStruct.zlt             = 0; 
    epInitStruct.transferType    = USB_ENDPOINT_CONTROL;
    epInitStruct.interval        = 0;
    epInitStruct.endpointAddress = epaddr(USB_CONTROL_ENDPOINT, USB_IN);
    epInitStruct.maxPacketSize   = USB_CONTROL_MAX_PACKET_SIZE;
    /* Initialize the control IN pipe */
    status = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

    if (kStatus_USB_Success != status)
    {
      printf_debug("USB_DeviceInitEndpoint() failed: %d", status);
      return;
    }

    epCallback.callbackFn    = device_ep0_control_callback_out;
    epCallback.callbackParam = NULL;
    epInitStruct.endpointAddress = epaddr(USB_CONTROL_ENDPOINT, USB_OUT);
    /* Initialize the control OUT pipe */
    status = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

    if (kStatus_USB_Success != status)
    {
        (void)USB_DeviceDeinitEndpoint(
            handle, USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
	printf_debug("USB_DeviceInitEndpoint() failed: %d", status);
        return;
    }
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

  return;
#endif /* MIMXRT1062_USB_USE_USB1 */
}

static usb_status_t device_epN_control_callback(usb_device_handle handle,
						usb_device_endpoint_callback_message_struct_t *message,
						void *callbackParam,
						uint8_t direction) {
  (void)handle;
  USBDriver *usbp = &USBD1;
  uint8_t ep = (uint8_t)callbackParam;
  printf_debug("device_epN_control_callback");
  if (message->isSetup) {
    printf_debug("discarding unexpected setup message for endpoint ep=%d", ep);
    return kStatus_USB_Error;
  }
  // TODO: describe under which circumstances this callback is called. seems
  // like there are a number of situations, e.g. transfer completion

  if (message->length == USB_CANCELLED_TRANSFER_LENGTH) {
    printf_debug("USB transfer cancelled");
    return kStatus_USB_InvalidRequest;
  }

  printf_debug("transfer done: buffer %x, len=%d!", message->buffer, message->length);

  const USBEndpointConfig *epc = usbp->epc[ep];

  if (direction == USB_OUT) {
    printf_debug("    complete, OUT ep=%d (OUT), rxbuf=%x", ep, epc->out_state->rxbuf);
    (usbp)->receiving &= ~(1 << ep);

    USBOutEndpointState *osp = epc->out_state;
    osp->rxcnt = message->length;
    printf_debug("    received %d bytes", osp->rxcnt);

    /* Endpoint Receive Complete Event */
    /* Transfer Direction OUT */
    if (epc->out_cb != NULL) {
      printf_debug("    invoking out_cb for ep %d", ep);
      _usb_isr_invoke_out_cb(usbp, ep);
    }
  } else if (direction == USB_IN) {
    printf_debug("    complete, IN ep=%d (IN), txbuf=%x", ep, epc->in_state->txbuf);
    (usbp)->transmitting &= ~(1 << ep);
    /* Endpoint Transmit Complete Event */
    /* Transfer Direction IN */
    if (epc->in_cb != NULL) {
      printf_debug("    invoking in_cb for ep %d", ep);
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  }
  return kStatus_USB_Success;
}

static usb_status_t device_epN_control_callback_out(usb_device_handle handle,
						    usb_device_endpoint_callback_message_struct_t *message,
						    void *callbackParam) {
  return device_epN_control_callback(handle, message, callbackParam, USB_OUT);
}

static usb_status_t device_epN_control_callback_in(usb_device_handle handle,
						    usb_device_endpoint_callback_message_struct_t *message,
						    void *callbackParam) {
  return device_epN_control_callback(handle, message, callbackParam, USB_IN);
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

  const USBEndpointConfig *epc = usbp->epc[ep];

  if (epc->out_state == NULL && epc->in_state == NULL) {
    printf_debug("  BUG: unknown endpoint type?!");
    return;
  }

  usb_device_endpoint_init_struct_t epInitStruct;
  usb_device_endpoint_callback_struct_t epCallback;
  usb_status_t status;

  epCallback.callbackFn    = device_epN_control_callback_in;
  epCallback.callbackParam = (void*)ep;

  epInitStruct.transferType = epc->ep_mode;
  epInitStruct.zlt = 0;
  epInitStruct.interval = 0;
  
  if (epc->in_state != NULL) {
    epInitStruct.endpointAddress = epaddr(ep, USB_IN);
    epInitStruct.maxPacketSize   = epc->in_maxsize;
    /* Initialize the endpoint IN pipe */
    if ((status = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback)) != kStatus_USB_Success) {
      printf_debug("USB_DeviceInitEndpoint() failed: %d", status);
      return;
    }
  }
    
  if (epc->out_state != NULL) {
    epCallback.callbackFn    = device_epN_control_callback_out;

    epInitStruct.endpointAddress = epaddr(ep, USB_OUT);
    epInitStruct.maxPacketSize = epc->out_maxsize;
    /* Initialize the endpoint OUT pipe */
    if ((status = USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback)) != kStatus_USB_Success) {
      USB_DeviceDeinitEndpoint(handle, USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
      printf_debug("USB_DeviceInitEndpoint() failed: %d", status);
      return;
    }
  }

  return;

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
  printf_debug("TODO: usb_lld_disable_endpoints()");
#if MIMXRT1062_USB_USE_USB1
  // TODO: find active endpoints and disable
  // USB_DeviceDeinitEndpoint(usb_device_handle handle, uint8_t endpointAddress);
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
  (void)usbp;

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
  (void)usbp;
  (void)ep;
  (void)buf;

  // We already populated usbp->setup in device_ep0_control_callback(), so no
  // extra steps are required in this function.
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

  USB_DeviceRecvRequest(handle, epaddr(ep, USB_OUT), osp->rxbuf, osp->rxsize);
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
  printf_debug("  usb_lld_start_in(ep=%d, txbuf=%x) (transmit)", ep, isp->txbuf);

  // Notably, at least with Linux, isp->txsize may exceed epc->in_maxsize, and
  // capping it makes device enumeration fail.
  USB_DeviceSendRequest(handle, epaddr(ep, USB_IN), isp->txbuf, isp->txsize);
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
  usb_status_t status;
  if ((status = USB_DeviceStallEndpoint(handle, epaddr(ep, USB_OUT))) != kStatus_USB_Success) {
    printf_debug("  stalling failed: %d", status);
  }
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
  usb_status_t status;
  if ((status = USB_DeviceStallEndpoint(handle, epaddr(ep, USB_IN))) != kStatus_USB_Success) {
    printf_debug("  stalling failed: %d", status);
  }
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
  usb_status_t status;
  if ((status = USB_DeviceUnstallEndpoint(handle, epaddr(ep, USB_OUT))) != kStatus_USB_Success) {
    printf_debug("  unstalling failed: %d", status);
  }
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
  usb_status_t status;
  if ((status = USB_DeviceUnstallEndpoint(handle, epaddr(ep, USB_IN))) != kStatus_USB_Success) {
    printf_debug("  unstalling failed: %d", status);
  }
#endif /* MIMXRT1062_USB_USE_USB1 */
}

#endif /* HAL_USE_USB */

/** @} */
