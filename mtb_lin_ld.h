/***************************************************************************//**
 * \file mtb_lin_ld.h
 * \version 1.10
 *
 * \brief
 * Provides the LIN middleware node configuration and identification API
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

#if !defined(MTB_LIN_LD_H)
#define MTB_LIN_LD_H

#include "mtb_lin_types.h"

/*******************************************************************************
*            Node Configuration API
*******************************************************************************/
/**
 * \addtogroup group_lin_node_configuration_function
 * \{
 */
/* Transport Layer Functions: Node Configuration Functions */
l_u8 ld_read_configuration(l_ifc_handle iii, l_u8* pData, l_u8* const length,
                           const mtb_stc_lin_context_t* context);
l_u8 ld_set_configuration(l_ifc_handle iii, const l_u8* const pData, l_u16 length,
                          mtb_stc_lin_context_t* context);

l_u8 ld_read_by_id_callout(l_ifc_handle iii, l_u8 id, l_u8* frame_data,
                           l_u8* data_length, const mtb_stc_lin_context_t* context);
/** \} group_lin_node_configuration_function */

/*******************************************************************************
*            Transport Layer API
*******************************************************************************/
/**
 * \addtogroup group_lin_transport_layer_initialization_function
 * \{
 */
/* Transport Layer Functions: Initialization */
void ld_init(l_ifc_handle iii, mtb_stc_lin_context_t* context);
/** \} group_lin_transport_layer_initialization_function */

/**
 * \addtogroup group_lin_transport_layer_raw_function
 * \{
 */
/* Transport Layer Functions: Raw Transport Layer API */
void ld_put_raw(l_ifc_handle iii, const l_u8* const ld_data, mtb_stc_lin_context_t* context);
void ld_get_raw(l_ifc_handle iii, l_u8* const ld_data, mtb_stc_lin_context_t* context);
l_u8 ld_raw_tx_status(l_ifc_handle iii, const mtb_stc_lin_context_t* context);
l_u8 ld_raw_rx_status(l_ifc_handle iii, const mtb_stc_lin_context_t* context);
/** \} group_lin_transport_layer_raw_function */

/**
 * \addtogroup group_lin_transport_layer_cooked_function
 * \{
 */
/* Transport Layer Functions: Cooked Transport Layer API */
void ld_send_message(l_ifc_handle iii, l_u16 length, l_u8 nad, const l_u8* const ld_data,
                     mtb_stc_lin_context_t* context);
void ld_receive_message(l_ifc_handle iii, l_u16* const length, l_u8* const nad, l_u8* const ld_data,
                        mtb_stc_lin_context_t* context);
l_u8 ld_tx_status(l_ifc_handle iii, mtb_stc_lin_context_t* context);
l_u8 ld_rx_status(l_ifc_handle iii, mtb_stc_lin_context_t* context);
/** \} group_lin_transport_layer_cooked_function */


/*******************************************************************************
*            Transport Layer Internal API
*******************************************************************************/
/**
 * \addtogroup group_lin_functions
 * \{
 */
/** \cond INTERNAL */
void mtb_lin_clear_rx_buffer(mtb_stc_lin_node_state_stimulus_t stimulus,
                             mtb_stc_lin_context_t* context);
void mtb_lin_clear_tx_buffer(mtb_stc_lin_node_state_stimulus_t stimulus,
                             mtb_stc_lin_context_t* context);
l_u8 mtb_lin_process_mrf(mtb_stc_lin_context_t* context);
l_u8 mtb_lin_transmit_tl_frame(mtb_stc_lin_node_state_stimulus_t stimulus,
                               mtb_stc_lin_context_t* context);

l_bool mtb_lin_check_product_id(volatile const l_u8 frame_data[],
                                const mtb_stc_lin_context_t* context);
/** \endcond */
/** \} group_lin_functions */

/**
 * \addtogroup group_lin_macros
 * \{
 */
/**
 * \addtogroup group_lin_ld_note_identification_status_macros Node Identification Status
 * \{
 */
/** If Slave node responds with a negative response, the data
 * area is not considered.
 */
#define LD_NEGATIVE_RESPONSE                        (0x00U)

/** Slave node sets up a positive response using the data provided by
 * the application.
 */
#define LD_NO_RESPONSE                              (0x01U)

/** Slave node does not answer */
#define LD_POSITIVE_RESPONSE                        (0x02U)
/** \} group_lin_ld_note_identification_status_macros */


/**
 * \addtogroup group_lin_ld_note_configuration_status_macros Node Configuration Status
 * \{
 */
/** Read configuration completed */
#define LD_READ_OK                 (0x01U)

/** Read configuration failed. The configuration size is greater than the length. */
#define LD_LENGTH_TOO_SHORT        (0x02U)

/** Set configuration completed. */
#define LD_SET_OK                  (0x01U)

/** Set configuration failed. The required configuration size is not equal to
 * the given length.
 */
#define LD_LENGTH_NOT_CORRECT      (0x02U)

/** Set of configuration could not be set. An error occurred while setting
 * the configuration and the read back configuration settings do not match the
 * required settings.
 */
#define LD_DATA_ERROR              (0x04U)
/** \} group_lin_ld_note_configuration_status_macros */


/*******************************************************************************
*       config->tl_tx_status and config->tl_rx_status
*******************************************************************************/

/**
 * \addtogroup group_lin_tl_return_status_macros Transport Layer Status
 * \{
 */
/** Reception or transmission not yet completed */
#define LD_IN_PROGRESS                              (0x01U)

/** Reception or transmission completed successfully. This value is also
 * returned after initialization of Transport Layer.
 */
#define LD_COMPLETED                                (0x02U)

/** Reception or transmission ended in an error. The data was only partially
 * sent or received (should not be trusted). The transport layer is
 * reinitialized before processing further messages. To find out why reception
 * or transmission failed, check the status management function
 * \ref ld_rx_status() or \ref ld_tx_status().
 */
#define LD_FAILED                                   (0x03U)

/** Transmission failed because of a N_As timeout */
#define LD_N_AS_TIMEOUT                             (0x04U)

/** Reception failed because of a N_Cr timeout */
#define LD_N_CR_TIMEOUT                             (0x05U)

/** Reception failed because of an unexpected sequence number */
#define LD_WRONG_SN                                 (0x06U)

/** The transmit queue is empty. In case of previous calls to ld_put_raw, all
 * frames in the queue were transmitted.
 */
#define LD_QUEUE_EMPTY                              (0x07U)

/** The transmit queue contains entries, but is not full */
#define LD_QUEUE_AVAILABLE                          (0x08U)

/** The transmit queue is full and cannot accept further frames */
#define LD_QUEUE_FULL                               (0x09U)

/** LIN protocol errors occurred during a transfer; initialize and redo the transfer. */
#define LD_TRANSMIT_ERROR                           (0x0AU)

/** The receive queue is empty. */
#define LD_NO_DATA                                  (0x0BU)

/** The receive queue contains data that can be read */
#define LD_DATA_AVAILABLE                           (0x0CU)

/** LIN protocol errors occurred during a transfer; initialize and redo the transfer. */
#define LD_RECEIVE_ERROR                            (0x0DU)
/** \} group_lin_tl_return_status_macros */

/** \cond INTERNAL */

/* Node Configuration Services */
#define MTB_LIN_NCS_ASSIGN_NAD             (0xB0U)
#define MTB_LIN_NCS_ASSIGN_FRAME_ID        (0xB1U) /* Used only in LIN 2.0 */
#define MTB_LIN_NCS_READ_BY_ID             (0xB2U)
#define MTB_LIN_NCS_COND_CHANGE_NAD        (0xB3U)
#define MTB_LIN_NCS_DATA_DUMP              (0xB4U) /* Not supported */
#define MTB_LIN_NCS_ASSIGN_NAD_SNPD        (0xB5U) /* Not supported */
#define MTB_LIN_NCS_SAVE_CONFIG            (0xB6U)
#define MTB_LIN_NCS_ASSIGN_FRAME_ID_RANGE  (0xB7U)

/*******************************************************************************
*                           MTB_LIN_tlFlags
*******************************************************************************/

/* The requested service is disabled and diagnostic frame will
 *  be "passed" to Transport Layer.
 */
#define MTB_LIN_TL_CS_SERVICE_DISABLED     (0x01U)

/* SID is not an ACRH but a simple diagnostic SID */
#define MTB_LIN_TL_DIAG_FRAME_DETECTED     (0x02U)

/* The last PID that occurred is SRF PID */
#define MTB_LIN_TL_TX_DIRECTION            (0x04U)

/* The last PID that occurred is MRF PID */
#define MTB_LIN_TL_RX_DIRECTION            (0x08U)

/* Indicates that Cooked API requested transmit data */
#define MTB_LIN_TL_TX_REQUESTED            (0x10U)

/* Indicates that Cooked API requested receive data */
#define MTB_LIN_TL_RX_REQUESTED            (0x20U)

/* Indicates that the N_AS timeout monitoring is in progress */
#define MTB_LIN_TL_N_AS_TIMEOUT_ON         (0x40U)

/* Indicates that the N_CR timeout monitoring is in progress */
#define MTB_LIN_TL_N_CR_TIMEOUT_ON         (0x80U)

/*******************************************************************************
*                           Transport Layer
*******************************************************************************/
/* Wildcard ID understandable for every slave node */
#define MTB_LIN_CS_SUPPLIER_ID_WILDCARD    (0x7FFFU)
#define MTB_LIN_CS_FUNCTION_ID_WILDCARD    (0xFFFFU)

#define MTB_LIN_CS_BYTE_SUPPLIER_ID1       (0x01U)
#define MTB_LIN_CS_BYTE_SUPPLIER_ID2       (0x02U)
#define MTB_LIN_CS_BYTE_FUNCTION_ID1       (0x03U)
#define MTB_LIN_CS_BYTE_FUNCTION_ID2       (0x04U)
#define MTB_LIN_CS_BYTE_VARIANT            (0x05U)

/* Specifies the value for the TL timeouts  */
#define MTB_LIN_TL_N_AS_TIMEOUT_VALUE      (1000U)
#define MTB_LIN_TL_N_CR_TIMEOUT_VALUE      (1000U)
#define MTB_LIN_TL_P2_TIMEOUT_VALUE        (500U)

/* Specifies the Frame buffer length for Transport Layer */
#define MTB_LIN_FRAME_BUFF_LEN             (8U)

/* Specifies the Frame length for Transport Layer */
#define MTB_LIN_FRAME_LEN                  (8U)

/* Packet Data Unit (PDU) Offsets */
#define MTB_LIN_PDU_NAD_IDX                (0U)
#define MTB_LIN_PDU_PCI_IDX                (1U)
#define MTB_LIN_PDU_SID_IDX                (2U)
#define MTB_LIN_PDU_LEN_IDX                (2U)
#define MTB_LIN_PDU_D1_IDX                 (3U)
#define MTB_LIN_PDU_D1_START_IDX           (3U)
#define MTB_LIN_PDU_D1_ID_IDX              (3U)
#define MTB_LIN_PDU_D2_IDX                 (4U)
#define MTB_LIN_PDU_D2_PID_IDX             (4U)
#define MTB_LIN_PDU_D2_BYTE_IDX            (4U)
#define MTB_LIN_PDU_D3_IDX                 (5U)
#define MTB_LIN_PDU_D3_MASK_IDX            (5U)
#define MTB_LIN_PDU_D4_IDX                 (6U)
#define MTB_LIN_PDU_D4_INVERT_IDX          (6U)
#define MTB_LIN_PDU_D5_IDX                 (7U)
#define MTB_LIN_PDU_D5_NEW_NAD_IDX         (7U)

/* Single Frame data length  */
#define MTB_LIN_PDU_SF_DATA_LEN            (6U)

/* Protocol Control Information (PCI) Types */
#define MTB_LIN_PDU_PCI_TYPE_SF            (0x00U)     /* Single Frame */
#define MTB_LIN_PDU_PCI_TYPE_FF            (0x10U)     /* First Frame */
#define MTB_LIN_PDU_PCI_TYPE_CF            (0x20U)     /* Consecutive Frame */
#define MTB_LIN_PDU_PCI_TYPE_UNKNOWN       (0xFFU)
#define MTB_LIN_PDU_PCI_TYPE_MASK          (0xF0U)
#define MTB_LIN_PDU_PCI_LENGTH_MASK        (0x0FU)

#define MTB_LIN_NAD_J2602_BASE             (0x60U)
#define MTB_LIN_NAD_MULTIPLE2_MASK         (0x01U)
#define MTB_LIN_NAD_MULTIPLE4_MASK         (0x03U)
#define MTB_LIN_NAD_MULTIPLE8_MASK         (0x07U)
#define MTB_LIN_NAD_DNN_MASK               (0x0FU)
#define MTB_LIN_NAD_UNINITIALIZED          (0x6FU)
#define MTB_LIN_NAD_FUNCTIONAL             (0x7EU)

/* Wildcard NAD */
#define MTB_LIN_NAD_BROADCAST              (0x7Fu)

/* "Go to sleep" command ID */
#define MTB_LIN_NAD_GO_TO_SLEEP            (0x00U)

/* Max and min Config Services IDs */
#define MTB_LIN_SID_CONF_MIN               (0xB0U)
#define MTB_LIN_SID_CONF_MAX               (0xB7U)

/* Used with tempStatusErr to Return Negative response for Config Service */
#define MTB_LIN_NCS_NEGATIVE_RESPONSE_REQUIRED      (0x01U)

/* Other LIN TL constants */
#define MTB_LIN_NCS_READ_BY_ID_ID          (0x00U)
#define MTB_LIN_NCS_READ_BY_ID_SERIAL      (0x01U)
#define MTB_LIN_NCS_READ_BY_ID_FILE_REVISION (0x03U)
#define MTB_LIN_NCS_RSID_NEG_REPLY         (0x7FU)
#define MTB_LIN_NCS_MAX_FRAME_ID_RANGE     (0x04U)

#define MTB_LIN_FRAME_PID_MRF              (0x3CU)
#define MTB_LIN_FRAME_PID_SRF              (0x7DU)
#define MTB_LIN_FRAME_PID_BROADCAST_MIN    (0x38U)
#define MTB_LIN_FRAME_PID_BROADCAST_MAX    (0x3BU)
#define MTB_LIN_FRAME_PID_UNASSIGNED       (0x40U)
#define MTB_LIN_MESSAGE_ID_UNASSIGNED      (0xFFFFU)
/** \endcond */
/** \} group_lin_macros */

#endif /* MTB_LIN_LD_H */

/* [] END OF FILE */
