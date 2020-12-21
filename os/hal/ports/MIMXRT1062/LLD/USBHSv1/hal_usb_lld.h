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
 * @file    USBHSv1/hal_usb_lld.h
 * @brief   MIMXRT1062 USB subsystem low level driver header.
 *
 * @addtogroup USB
 * @{
 */

#ifndef HAL_USB_LLD_H_
#define HAL_USB_LLD_H_

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Maximum endpoint address.
 */
#define USB_MAX_ENDPOINTS                   7

/**
 * @brief   Status stage handling method.
 */
#define USB_EP0_STATUS_STAGE                USB_EP0_STATUS_STAGE_SW

/**
 * @brief   Address ack handling
 */
#define USB_SET_ADDRESS_ACK_HANDLING        USB_SET_ADDRESS_ACK_SW

/**
 * @brief   This device requires the address change after the status packet.
 */
#define USB_SET_ADDRESS_MODE                USB_EARLY_SET_ADDRESS

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   USB1 driver enable switch.
 * @details If set to @p TRUE the support for USB1 is included.
 * @note    The default is @p TRUE.
 */
#if !defined(MIMXRT1062_USB_USE_USB1) || defined(__DOXYGEN__)
#define MIMXRT1062_USB_USE_USB1                  TRUE
#endif

#if !defined(MIMXRT1062_USB1_IS_USBOTG)
#define MIMXRT1062_USB1_IS_USBOTG TRUE
#endif

/**
 * @brief   USB1 interrupt priority level setting.
 */
#if !defined(MIMXRT1062_USB_USB1_IRQ_PRIORITY)|| defined(__DOXYGEN__)
#define MIMXRT1062_USB_USB1_IRQ_PRIORITY      3
#endif

#if !defined(MIMXRT1062_USB_ENDPOINTS) || defined(__DOXYGEN__)
#define MIMXRT1062_USB_ENDPOINTS (USB_MAX_ENDPOINTS+1)
#endif

/**
 * @brief   Host wake-up procedure duration.
 */
#if !defined(USB_HOST_WAKEUP_DURATION) || defined(__DOXYGEN__)
#define USB_HOST_WAKEUP_DURATION            2
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if MIMXRT1062_USB_USE_USB1 && !MIMXRT1062_HAS_USB
#error "USB not present in the selected device"
#endif

#if !MIMXRT1062_USB_USE_USB1
#error "USB driver activated but no USB peripheral assigned"
#endif

#if MIMXRT1062_USB_USE_USB1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(MIMXRT1062_USB_USB1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to MIMXRT1062_USB_USB1_IRQ_PRIORITY"
#endif

#if !defined(MIMXRT1062_USB_IRQ_VECTOR)
#error "MIMXRT1062_USB_IRQ_VECTOR not defined"
#endif

#if (USB_HOST_WAKEUP_DURATION < 2) || (USB_HOST_WAKEUP_DURATION > 15)
#error "invalid USB_HOST_WAKEUP_DURATION setting, it must be between 2 and 15"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/


// → page 2344, 42.5.5 “Device Data Structures”

// → page 2348, 42.5.5.2 Endpoint Transfer Descriptor (dTD)
// NXP: usb_device_ehci_dtd_struct_t
typedef struct transfer_struct transfer_t;
struct transfer_struct {
  // Next dTD Pointer
  //
  // 1 = invalid pointer, or address of the next transfer element descriptor
  // (aligned to a 32-byte boundary, i.e. lowest 4 bits are all zero).
  //
  // This is the only field that can be modified in an active dTD,
  // but only as described in “Managing Transfers with Transfer Descriptors”.
  uint32_t next;

  // TODO: should be called |token|, not |status|
  //
  // IOC = Interrupt On Complete: USBINT will be set in response to the device
  // controller (hardware) being finished with this dTD.
  //
  // bit 7 = active
  // bit 6 = halted
  // bit 5 = data buffer error
  // bit 3 = transaction error
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
// NXP: usb_device_ehci_qh_struct_t
struct endpoint_struct {
  // → page 2347, 42.5.5.1.1 Endpoint Capabilities/Characteristics
  volatile uint32_t config;

  // → page 2348, 42.5.5.1.3 Current dTD Pointer
  // for use by hardware only, should not be modified.
  // NXP: currentDtdPointer
  volatile uint32_t current;

  // → page 2347, 42.5.5.1.1, Transfer Overlay-Endpoint Queue Head
  // working space for the device controller
  // NXP: nextDtdPointer
  volatile uint32_t next;
  // NXP: dtdToken
  volatile uint32_t status;
  volatile uint32_t pointer0;
  volatile uint32_t pointer1;
  volatile uint32_t pointer2;
  volatile uint32_t pointer3;
  volatile uint32_t pointer4;
  
  volatile uint32_t reserved;

  // → page 2348, 42.5.5.1.4 Set-up Buffer
  // NXP: setupBuffer
  uint32_t setup0;
  uint32_t setup1;

  // -- 48 byte consumed, need to fill 64, i.e. 4 x uint32 --
  //
  // 56.4.5.1 Endpoint Queue Head (dQH)
  //
  // The device Endpoint Queue Head (dQH) is where all transfers for a given
  // endpoint are managed. The dQH is a 48-byte data structure, but must be
  // aligned on 64-byte boundaries.
  //
  // NXP: setupBufferBack[2]
  // NXP: endpointStatusUnion
  // NXP: reserved2

  transfer_t *head;
  transfer_t *tail;
  void (*callback_function)(transfer_t *completed_transfer);
  uint32_t last_setup_offset;
};


/**
 * @brief   Type of an IN endpoint state structure.
 */
typedef struct {
  // Must come first so that we can align the USBInEndpointState:
  transfer_t transfer;
  
  /**
   * @brief   Requested transmit transfer size.
   */
  size_t                        txsize;
  /**
   * @brief   Transmitted bytes so far.
   */
  size_t                        txcnt;
  /**
   * @brief   Pointer to the transmission linear buffer.
   */
  const uint8_t                 *txbuf;
#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t            thread;
#endif

} USBInEndpointState;

/**
 * @brief   Type of an OUT endpoint state structure.
 */
typedef struct {
  // Must come first so that we can align the USBOutEndpointState:
  transfer_t transfer;
  
  /**
   * @brief   Requested receive transfer size.
   */
  size_t                        rxsize;
  /**
   * @brief   Received bytes so far.
   */
  size_t                        rxcnt;
  /**
   * @brief   Pointer to the receive linear buffer.
   */
  uint8_t                       *rxbuf;
#if (USB_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t            thread;
#endif
  /* End of the mandatory fields.*/

} USBOutEndpointState;

/**
 * @brief   Type of an USB endpoint configuration structure.
 * @note    Platform specific restrictions may apply to endpoints.
 */
typedef struct {
  /**
   * @brief   Type and mode of the endpoint.
   */
  uint32_t                      ep_mode;
  /**
   * @brief   Setup packet notification callback.
   * @details This callback is invoked when a setup packet has been
   *          received.
   * @post    The application must immediately call @p usbReadPacket() in
   *          order to access the received packet.
   * @note    This field is only valid for @p USB_EP_MODE_TYPE_CTRL
   *          endpoints, it should be set to @p NULL for other endpoint
   *          types.
   */
  usbepcallback_t               setup_cb;
  /**
   * @brief   IN endpoint notification callback.
   * @details This field must be set to @p NULL if callback is not required.
   */
  usbepcallback_t               in_cb;
  /**
   * @brief   OUT endpoint notification callback.
   * @details This field must be set to @p NULL if callback is not required.
   */
  usbepcallback_t               out_cb;
  /**
   * @brief   IN endpoint maximum packet size.
   * @details This field must be set to zero if the IN endpoint is not used.
   */
  uint16_t                      in_maxsize;
  /**
   * @brief   OUT endpoint maximum packet size.
   * @details This field must be set to zero if the OUT endpoint is not used.
   */
  uint16_t                      out_maxsize;
  /**
   * @brief   @p USBEndpointState associated to the IN endpoint.
   * @details This field must be set to @p NULL if the IN endpoint is not
   *          used.
   */
  USBInEndpointState            *in_state;
  /**
   * @brief   @p USBEndpointState associated to the OUT endpoint.
   * @details This field must be set to @p NULL if the OUT endpoint is not
   *          used.
   */
  USBOutEndpointState           *out_state;
  /* End of the mandatory fields.*/
  /**
   * @brief   Reserved field, not currently used.
   * @note    Initialize this field to 1 in order to be forward compatible.
   */
  uint16_t                      ep_buffers;
  /**
   * @brief   Pointer to a buffer for setup packets.
   * @details Setup packets require a dedicated 8-bytes buffer, set this
   *          field to @p NULL for non-control endpoints.
   */
  uint8_t                       *setup_buf;
} USBEndpointConfig;

/**
 * @brief   Type of an USB driver configuration structure.
 */
typedef struct {
  /**
   * @brief   USB events callback.
   * @details This callback is invoked when an USB driver event is registered.
   */
  usbeventcb_t                  event_cb;
  /**
   * @brief   Device GET_DESCRIPTOR request callback.
   * @note    This callback is mandatory and cannot be set to @p NULL.
   */
  usbgetdescriptor_t            get_descriptor_cb;
  /**
   * @brief   Requests hook callback.
   * @details This hook allows to be notified of standard requests or to
   *          handle non standard requests.
   */
  usbreqhandler_t               requests_hook_cb;
  /**
   * @brief   Start Of Frame callback.
   */
  usbcallback_t                 sof_cb;
  /* End of the mandatory fields.*/
} USBConfig;

/**
 * @brief   Structure representing an USB driver.
 */
struct USBDriver {
  /**
   * @brief   Driver state.
   */
  usbstate_t                    state;
  /**
   * @brief   Current configuration data.
   */
  const USBConfig               *config;
  /**
   * @brief   Bit map of the transmitting IN endpoints.
   */
  uint16_t                      transmitting;
  /**
   * @brief   Bit map of the receiving OUT endpoints.
   */
  uint16_t                      receiving;
  /**
   * @brief   Active endpoints configurations.
   */
  const USBEndpointConfig       *epc[USB_MAX_ENDPOINTS + 1];
  /**
   * @brief   Fields available to user, it can be used to associate an
   *          application-defined handler to an IN endpoint.
   * @note    The base index is one, the endpoint zero does not have a
   *          reserved element in this array.
   */
  void                          *in_params[USB_MAX_ENDPOINTS];
  /**
   * @brief   Fields available to user, it can be used to associate an
   *          application-defined handler to an OUT endpoint.
   * @note    The base index is one, the endpoint zero does not have a
   *          reserved element in this array.
   */
  void                          *out_params[USB_MAX_ENDPOINTS];
  /**
   * @brief   Endpoint 0 state.
   */
  usbep0state_t                 ep0state;
  /**
   * @brief   Next position in the buffer to be transferred through endpoint 0.
   */
  uint8_t                       *ep0next;
  /**
   * @brief   Number of bytes yet to be transferred through endpoint 0.
   */
  size_t                        ep0n;
  /**
   * @brief   Endpoint 0 end transaction callback.
   */
  usbcallback_t                 ep0endcb;
  /**
   * @brief   Setup packet buffer.
   */
  uint8_t                       setup[8];
  /**
   * @brief   Current USB device status.
   */
  uint16_t                      status;
  /**
   * @brief   Assigned USB address.
   */
  uint8_t                       address;
  /**
   * @brief   Current USB device configuration.
   */
  uint8_t                       configuration;
  /**
   * @brief   State of the driver when a suspend happened.
   */
  usbstate_t                    saved_state;
#if defined(USB_DRIVER_EXT_FIELDS)
  USB_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Pointer to the next address in the packet memory.
   */
  uint32_t                      pmnext;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the current frame number.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The current frame number.
 *
 * @notapi
 */
#define usb_lld_get_frame_number(usbp) ((USB1->FRMNUMH<<8)|USB1->FRMNUML)

/**
 * @brief   Returns the exact size of a receive transaction.
 * @details The received size can be different from the size specified in
 *          @p usbStartReceiveI() because the last packet could have a size
 *          different from the expected one.
 * @pre     The OUT endpoint must have been configured in transaction mode
 *          in order to use this function.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              Received data size.
 *
 * @notapi
 */
#define usb_lld_get_transaction_size(usbp, ep)                              \
  ((usbp)->epc[ep]->out_state->rxcnt)

/**
 * @brief   Connects the USB device.
 *
 * @api
 */
#if !defined(usb_lld_connect_bus)
#define usb_lld_connect_bus(usbp) USB1->USBCMD |= USB_USBCMD_RS(1)
#endif

/**
 * @brief   Disconnect the USB device.
 *
 * @api
 */
#if !defined(usb_lld_disconnect_bus)
/* Writing to USB1->CONTROL causes an unhandled exception when USB module is not clocked. */
#define usb_lld_disconnect_bus(usbp) do { if (CCM->CCGR6 & CCM_CCGR6_CG0_MASK) { USB1->USBCMD |= USB_USBCMD_RS(0); } } while (0)
#endif

/**
 * @brief   Start of host wake-up procedure.
 *
 * @notapi
 */
/* #define usb_lld_wakeup_host(usbp)                                     \ */
/*   do{                                                                 \ */
/*     USB1->CTL |= USBx_CTL_RESUME;                                     \ */
/*     osalThreadSleepMilliseconds(USB_HOST_WAKEUP_DURATION);            \ */
/*     USB1->CTL &= ~USBx_CTL_RESUME;                                    \ */
/*   } while (false) */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if MIMXRT1062_USB_USE_USB1 && !defined(__DOXYGEN__)
extern USBDriver USBD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void usb_lld_init(void);
  void usb_lld_start(USBDriver *usbp);
  void usb_lld_stop(USBDriver *usbp);
  void usb_lld_reset(USBDriver *usbp);
  void usb_lld_set_address(USBDriver *usbp);
  void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep);
  void usb_lld_disable_endpoints(USBDriver *usbp);
  usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep);
  usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf);
  void usb_lld_start_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_start_in(USBDriver *usbp, usbep_t ep);
  void usb_lld_stall_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_stall_in(USBDriver *usbp, usbep_t ep);
  void usb_lld_clear_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_clear_in(USBDriver *usbp, usbep_t ep);
  void usb_lld_end_setup(USBDriver *usbp, usbep_t ep);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_USB */

#endif /* HAL_USB_LLD_H_ */

/** @} */
