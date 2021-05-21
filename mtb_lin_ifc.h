/***************************************************************************//**
 * \file mtb_lin_ifc.h
 * \version 1.0
 *
 * \brief
 * Provides the LIN middleware signal interaction and notification API
 * declarations.
 *
 *******************************************************************************
 * (c) (2020-2021), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software") is owned by Cypress Semiconductor Corporation or one of its
 * affiliates ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/

#if !defined(MTB_LIN_IFC_H)
#define MTB_LIN_IFC_H

#include "mtb_lin_types.h"

/*******************************************************************************
*            Interface API
*******************************************************************************/
/**
 * \addtogroup group_lin_core_api_interface_management_function
 * \{
 */
l_bool l_ifc_init(l_ifc_handle iii, mtb_stc_lin_context_t* context, cyhal_gpio_t tx,
                  cyhal_gpio_t rx);
void l_ifc_wake_up(l_ifc_handle iii, const mtb_stc_lin_context_t* context);
mtb_lin_status_t l_ifc_ioctl(l_ifc_handle iii, l_ioctl_op op, void* pv,
                             mtb_stc_lin_context_t* context);
void l_ifc_rx(l_ifc_handle iii, mtb_stc_lin_context_t* context);
void l_ifc_tx(l_ifc_handle iii, mtb_stc_lin_context_t* context);
void l_ifc_aux(l_ifc_handle iii, mtb_stc_lin_context_t* context);
l_u16 l_ifc_read_status(l_ifc_handle iii, mtb_stc_lin_context_t* context);

mtb_lin_status_t mtb_lin_config_check_nad(l_u8 nad, const mtb_stc_lin_config_t* config);
/** \} group_lin_core_api_interface_management_function */

bool mtb_lin_enable_timer(const mtb_stc_lin_context_t* context);
void mtb_lin_disable_timer(const mtb_stc_lin_context_t* context);

/**
 * \addtogroup group_lin_macros
 * \{
 */
/******************************************************************************
*           l_ifc_ioctl() Parameters
******************************************************************************/
/**
 * \addtogroup group_lin_l_ifc_ioctl_parameters_macros Parameters of l_ifc_ioctl
 * \brief Constants to be used for select operation in call of \ref
 * l_ifc_ioctl().
 * \{
 */
/** Optional status indicators.
 *
 * The first bit in this byte is the flag that indicates there was no
 * signalling on the bus for a certain elapsed time (available when the
 * Bus Inactivity Timeout Detection option is enabled). If the elapsed time
 * passes a certain threshold, this flag is set.
 *
 * The second bit is the flag that indicates that a Targeted Reset service
 *  request (0xB5) is received (when J2602-1 Compliance is enabled).
 *
 * Calling this API clears all the status bits after they are returned.
 *
 * Parameters: The pointer to l_u16, where the status will be written.
 */
#define MTB_LIN_IOCTL_READ_STATUS                         (0x00U)

/** Set a baud rate.
 *
 * Parameters: l_u16*.
 */
#define MTB_LIN_IOCTL_SET_BAUD_RATE                       (0x01U)

/** Prepare the device for the Low-power mode entry.
 *
 * Parameters: None.
 */
#define MTB_LIN_IOCTL_SLEEP                               (0x02U)

/** Restore the LIN middleware state after a wakeup.
 *
 * Parameters: None.
 */
#define MTB_LIN_IOCTL_WAKEUP                              (0x03U)

/** Return the current number of the sync field timer counts in "pv".
 *
 * Parameters: The pointer to l_u8, where the number will be written.
 */
#define MTB_LIN_IOCTL_SYNC_COUNTS                         (0x04U)

/** Update the pointer to the serial number
 *
 * Parameters: The pointer to l_u8, where the serial number will be written.
 */
#define MTB_LIN_IOCTL_SET_SERIAL_NUMBER                   (0x05U)

/** Return the configured NAD
 *
 * Parameters: The pointer to l_u8, where the NAD will be written.
 */
#define MTB_LIN_IOCTL_GET_NAD                             (0x06U)

/** Set the configured NAD to any value except 00, 7E or 7F. In J2602 compliance
 * mode, frame PIDs are also updated according to the new NAD.
 *
 * Parameters: The pointer to l_u8, where the NAD will be read from.
 */
#define MTB_LIN_IOCTL_SET_NAD                             (0x07U)

/** Returns the frame PID by the frame table index.
 *
 * Reports an error if the frame PIDs cannot be updated due to the NAD and frame count
 * mismatch (see SEA J2602-1 specification 5.7.2.2 Message ID Assignment,
 * Table 1).
 *
 * The NAD is not updated if a message PID update fails.
 *
 * Parameters: The pointer to mtb_stc_lin_new_pid_by_msg_t, where the NAD will be
 * written.
 */
#define MTB_LIN_IOCTL_GET_FRAME_PID                       (0x08U)

/** Sets the frame PID by the frame table index
 *
 * Parameters: The pointer to mtb_stc_lin_new_pid_by_msg_t, where the NAD will be
 * read from.
 */
#define MTB_LIN_IOCTL_SET_FRAME_PID                       (0x09U)


/** \} group_lin_l_ifc_ioctl_parameters_macros */


/******************************************************************************
*           Variable MTB_LIN_ioctlStatus flags
******************************************************************************/
/**
 * \addtogroup group_lin_l_l_ioctl_read_status_flag_macros Status flag of MTB_LIN_IOCTL_READ_STATUS
 * operation
 * \brief Status flag of \ref MTB_LIN_IOCTL_READ_STATUS operation
 * \{
 */
/** Indicates that No signal was detected on the bus for a certain elapsed
 * time.
 */
#define MTB_LIN_IOCTL_STS_BUS_INACTIVITY   (0x0001U)

/** Indicates that a Targeted Reset service request (0xB5) was received */
#define MTB_LIN_IOCTL_STS_TARGET_RESET     (0x0002U)
/** \} group_lin_l_l_ioctl_read_status_flag_macros */

/** \cond INTERNAL */

#define MTB_LIN_NO_RESPONSE_REQUIRED       (0x00U)
#define MTB_LIN_RESPONSE_REQUIRED          (0x01U)
#define MTB_LIN_RECEIVE_CONTINUES          (0x02U)
#define MTB_LIN_TRANSMISSION_CONTINUES     (0x04U)
#define MTB_LIN_ERRONEOUS_TL_FRAME         (0x10U)


/******************************************************************************
*                       UART State Machine                                    *
******************************************************************************/

/* Sync Field detection(UDB) / IDLE state (SCB) */
#define MTB_LIN_UART_ISR_STATE_0_IDLE      (0x00U)

/* Receive PID was detected. Analyze the received PID and determine the action */
#define MTB_LIN_UART_ISR_STATE_1_PID       (0x01U)

/* Transmit the data and checksum byte to LIN Master */
#define MTB_LIN_UART_ISR_STATE_2_TX        (0x02U)

/* Read the data from LIN Master */
#define MTB_LIN_UART_ISR_STATE_3_RX        (0x03U)

/* Checksum verification */
#define MTB_LIN_UART_ISR_STATE_4_CHS       (0x04U)

/* PID Parity Error Mask */
#define MTB_LIN_PID_PARITY_MASK            (0x3FU)


#define MTB_LIN_FRAME_DATA_SIZE_1          (0x01U)
#define MTB_LIN_FRAME_DATA_SIZE_2          (0x02U)
#define MTB_LIN_FRAME_DATA_SIZE_3          (0x03U)
#define MTB_LIN_FRAME_DATA_SIZE_4          (0x04U)
#define MTB_LIN_FRAME_DATA_SIZE_5          (0x05U)
#define MTB_LIN_FRAME_DATA_SIZE_6          (0x06U)
#define MTB_LIN_FRAME_DATA_SIZE_7          (0x07U)
#define MTB_LIN_FRAME_DATA_SIZE_8          (0x08U)


/******************************************************************************
*                                Interface Status
*                          context->ifc_comm_status
******************************************************************************/
/* An error in response */
#define MTB_LIN_IFC_STS_ERROR_IN_RESPONSE  (0x0001U)

/* Successful frame transfer */
#define MTB_LIN_IFC_STS_SUCCESSFUL_TRANSFER (0x0002U)

/* Overrun */
#define MTB_LIN_IFC_STS_OVERRUN            (0x0004U)

/* Go to sleep */
#define MTB_LIN_IFC_STS_GO_TO_SLEEP        (0x0008U)

/* Bus activity */
#define MTB_LIN_IFC_STS_BUS_ACTIVITY       (0x0010U)

/* Event-triggered frame collision */
#define MTB_LIN_IFC_STS_EVTRIG_COLLISION   (0x0020U)

/* Save configuration */
#define MTB_LIN_IFC_STS_SAVE_CONFIG        (0x0040u)

/* Last frame PID mask */
#define MTB_LIN_IFC_STS_PID_MASK           (0xFF00U)

/* Status mask */
#define MTB_LIN_IFC_STS_MASK               (0xFFFFU)

/******************************************************************************
*                          Internal LIN Slave Status
*                           MTB_LIN_status
******************************************************************************/

/* This bit indicates that there is a response for ACRH service
 *  and it is ready to be sent to Master node.
 */
#define MTB_LIN_STATUS_SRVC_RSP_RDY        (0x80U)
#define MTB_LIN_STATUS_RESPONSE_ACTIVE     (0x10U)
/* Indicates that a response pending frame (NRC 0x78 code) was issued. */
#define MTB_LIN_STATUS_RESPONSE_PENDING    (0x20U)

/******************************************************************************
*                       mtb_lin_finalize_frame(l_u16 status)
******************************************************************************/
/** Reset the FSM error status */
#define MTB_LIN_HANDLING_RESET_FSM_ERR     (0x01U)
/** Reset FSM break stage */
#define MTB_LIN_HANDLING_RESET_BREAK       (0x02U)
/** Don't save the PID status */
#define MTB_LIN_HANDLING_DONT_SAVE_PID     (0x10U)


/******************************************************************************
*                           MTB_LIN_fsmFlags
******************************************************************************/

/* UART receives at least 1 byte of data */
#define MTB_LIN_FSM_DATA_RECEIVE           (0x04U)

/* UART enable flag */
#define MTB_LIN_FSM_UART_ENABLE_FLAG       (0x08U)

/* Overrun flag is used for status word */
#define MTB_LIN_FSM_OVERRUN                (0x40U)

/* Framing error */
#define MTB_LIN_FSM_FRAMING_ERROR_FLAG     (0x80U)


/* UART oversampling rate */
#define MTB_LIN_OVERSAMPLE_RATE                        (16U)

/* 8 bits with the 16x oversampling rate */
#define MTB_LIN_EXPECTED_TIME_COUNTS                   (8U * MTB_LIN_OVERSAMPLE_RATE)

/* Clock max deviation is less than +/- (14.0 + 0.5) %, so max deviation is
 * (14.5 * MTB_LIN_EXPECTED_TIME_COUNTS / 100) = 18,56 (0x13).
 */
#define MTB_LIN_AUTO_BAUD_TIME_COUNTS_MAX_DEVIATION    (0x13U)

/* Clock max deviation is less than +/- (1.5 + 0.5) %, so max deviation is
 * (2 * 128 / MTB_LIN_EXPECTED_TIME_COUNTS) = 2.56 (0x03).
 */
#define MTB_LIN_TIME_COUNTS_MAX_DEVIATION              (0x03U)

#define MTB_LIN_BR_BASE_SHIFT                          (7U)


#define MTB_LIN_FRAME_PID_MRF_J2602        (0xFEU)

/* Wakeup signal length in us */
#define MTB_LIN_WAKE_UP_SIGNAL_LENGTH      (300U)

/* Wakeup fake symbol length in ms */
#define MTB_LIN_WAKEUP_TX_DELAY            (11U)

#define MTB_LIN_INVALID_FRAME_PID          (0xFFU)
/** \endcond */
/** \} group_lin_macros */

#endif /* MTB_LIN_IFC_H */

/* [] END OF FILE */
