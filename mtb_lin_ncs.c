/***************************************************************************//**
 * \file mtb_lin_ncs.c
 * \version 1.0
 *
 * \brief
 * Provides the LIN middleware node configuration services API
 * implementation.
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

#include "mtb_lin_ld.h"
#include "mtb_lin_ifc.h"
#include "mtb_lin_ncs.h"


/*******************************************************************************
*            Internal API
*******************************************************************************/
__STATIC_INLINE l_bool mtb_lin_service_read_by_product_id(mtb_stc_lin_context_t* context);
__STATIC_INLINE l_bool mtb_lin_service_read_by_serial_number(mtb_stc_lin_context_t* context);
__STATIC_INLINE l_bool mtb_lin_service_read_by_ldf_ncf_id(mtb_stc_lin_context_t* context);
__STATIC_INLINE l_bool mtb_lin_service_read_by_user_id(mtb_stc_lin_context_t* context);
__STATIC_INLINE void mtb_lin_service_read_by_id_negative_resp(mtb_stc_lin_context_t* context);


/*******************************************************************************
* Function Name: mtb_lin_service_assign_nad
****************************************************************************//**
*
* The internal function implements the "Assign NAD" service.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
*******************************************************************************/
void mtb_lin_service_assign_nad(mtb_stc_lin_context_t* context)
{
    /* Checks the data length. Do not respond if the PCI length does not match this service's data
       length. */
    if (MTB_LIN_PDU_SF_DATA_LEN ==
        (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_LENGTH_MASK))
    {
        /* Validates if preserved Node Address values are not used as NAD for slave node */
        if (mtb_lin_config_check_nad(context->mrf_buffer[MTB_LIN_PDU_D5_NEW_NAD_IDX],
                                     context->config) == MTB_LIN_STATUS_SUCCESS)
        {
            /* Checks LIN Product ID and if MRF has NAD equal to the initial NAD. */
            if (mtb_lin_check_product_id(&context->mrf_buffer[3U], context) &&
                (context->config->initial_nad == context->mrf_buffer[MTB_LIN_PDU_NAD_IDX]))
            {
                /* Saves the received NAD */
                context->configured_nad = context->mrf_buffer[MTB_LIN_PDU_D5_NEW_NAD_IDX];

                /* Fills SRF Buffer with a response to the service.
                 * The NAD field should contain the initial NAD.
                 */
                context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->config->initial_nad;

                /* PCI is 0, so only the length is required. */
                context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 1U;

                /* RSID for a positive response is always SID + 0x40. */
                context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_POS_RESP_ASSIGN_NAD;

                /* Fills unused data bytes with 0xFFs. */
                for (l_u8 i = 3U; i < MTB_LIN_FRAME_LEN; i++)
                {
                    context->srf_buffer[i] = 0xFFu;
                }

                /* Sets a service response bit that indicates that a response is ready
                 * to be sent to Master node. */
                context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;
            }
        }
    }
}


/*******************************************************************************
* Function Name: mtb_lin_service_read_by_id
****************************************************************************//**
*
* The internal function implements the "Read by identifier" 0xB2 service.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
*******************************************************************************/
void mtb_lin_service_read_by_id(mtb_stc_lin_context_t* context)
{
    /* Ignore the request if the PCI length does not match this service data length */
    if ((MTB_LIN_PDU_SF_DATA_LEN ==
         (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_LENGTH_MASK)))
    {
        if (mtb_lin_check_product_id(&context->mrf_buffer[4U], context))
        {
            l_bool positive_response = false;

            /* LIN Product Identification (only identifier is supported) */
            if (MTB_LIN_NCS_READ_BY_ID_ID == context->mrf_buffer[MTB_LIN_PDU_D1_IDX])
            {
                positive_response = mtb_lin_service_read_by_product_id(context);
            }
            /* Serial number identification */
            else if (MTB_LIN_NCS_READ_BY_ID_SERIAL == context->mrf_buffer[MTB_LIN_PDU_D1_IDX])
            {
                positive_response = mtb_lin_service_read_by_serial_number(context);
            }
            /* Version of LDF/NCF identification */
            else if (MTB_LIN_NCS_READ_BY_ID_FILE_REVISION ==
                     context->mrf_buffer[MTB_LIN_PDU_D1_IDX])
            {
                positive_response = mtb_lin_service_read_by_ldf_ncf_id(context);
            }
            /* User-defined identifiers (32 - 63) */
            else if ((context->mrf_buffer[MTB_LIN_PDU_D1_IDX] >= 32U) &&
                     (context->mrf_buffer[MTB_LIN_PDU_D1_IDX] <= 63U))
            {
                positive_response = mtb_lin_service_read_by_user_id(context);
            }
            else
            {
                /* The plan for a negative response for the unsupported identifier */
                positive_response = false;
            }

            /* If no positive response was generated - generate a negative response */
            if (!positive_response)
            {
                mtb_lin_service_read_by_id_negative_resp(context);
            }
        }
    }
}


/*******************************************************************************
* Function Name: mtb_lin_service_read_by_product_id
****************************************************************************//**
*
* Fills the SRF Buffer with a response to the "LIN Product Identification"
* request.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
* \return
* Returns "true" if a positive response is required.
*
*******************************************************************************/
__STATIC_INLINE l_bool mtb_lin_service_read_by_product_id(mtb_stc_lin_context_t* context)
{
    /* Fills SRF Buffer with a response to the service. The NAD field should contain the
     * current NAD.
     */
    context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

    /* PCI is 0, so only the length is required. */
    context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 6U;

    /* RSID for a positive response is always SID + 0x40 */
    context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_POS_RESP_READ_BY_ID;

    /* Fills the data fields with the Supplier and function IDs */
    context->srf_buffer[MTB_LIN_PDU_D1_IDX] = CY_LO8(context->config->supplier_id);
    context->srf_buffer[MTB_LIN_PDU_D2_IDX] = CY_HI8(context->config->supplier_id);
    context->srf_buffer[MTB_LIN_PDU_D3_IDX] = CY_LO8(context->config->function_id);
    context->srf_buffer[MTB_LIN_PDU_D4_IDX] = CY_HI8(context->config->function_id);
    context->srf_buffer[MTB_LIN_PDU_D5_IDX] = context->config->variant;

    /* Sets the service response bit that indicates that the response is
     * ready to be sent to Master node.
     */
    context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;

    return true;
}


/*******************************************************************************
* Function Name: mtb_lin_service_read_by_serial_number
****************************************************************************//**
*
* Fills the SRF Buffer with a response to the "Serial number" request.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
* \return
* Returns "true" if a positive response is required.
*
*******************************************************************************/
__STATIC_INLINE l_bool mtb_lin_service_read_by_serial_number(mtb_stc_lin_context_t* context)
{
    l_bool result = false;

    if (NULL != context->serial_number)
    {
        /* The NAD field should contain the current NAD. */
        context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

        /* PCI is 0, so only the length is required. */
        context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 5U;

        /* RSID for a positive response is always SID + 0x40 */
        context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_POS_RESP_READ_BY_ID;

        /* Fills unused data bytes with the serial number ID. */
        for (l_u8 i = 3U; i < (MTB_LIN_FRAME_LEN - 1U); i++)
        {
            context->srf_buffer[i] = context->serial_number[i - 3U];
        }

        /* The serial number is the 4-byte length, sets the last unused byte to 0xFF. */
        context->srf_buffer[MTB_LIN_PDU_D5_IDX] = 0xFFU;

        /* Sets the service response bit that indicates that the response is
         * ready to be sent to Master node.
         */
        context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;

        result = true;
    }

    return (result);
}


/*******************************************************************************
* Function Name: mtb_lin_service_read_by_ldf_ncf_id
****************************************************************************//**
*
* Fills the SRF Buffer with a response to the "LDF/NCF identification" request.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
* \return
* Returns "true" if a positive response is required.
*
*******************************************************************************/
__STATIC_INLINE l_bool mtb_lin_service_read_by_ldf_ncf_id(mtb_stc_lin_context_t* context)
{
    l_bool result = false;

    if ((context->config->file_rev_defined) && (context->config->spec == MTB_LIN_SPEC_2_2) &&
        (context->config->iso17987))
    {
        /* The NAD field should contain the current NAD. */
        context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

        /* PCI is 0, so only the length is required. */
        context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 5U;

        /* RSID for a positive response is always SID + 0x40 */
        context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_POS_RESP_READ_BY_ID;

        /* Fills data bytes with revision numbers. */
        context->srf_buffer[MTB_LIN_PDU_D1_IDX] = context->config->file_rev->major;
        context->srf_buffer[MTB_LIN_PDU_D2_IDX] = context->config->file_rev->minor;
        context->srf_buffer[MTB_LIN_PDU_D3_IDX] = context->config->file_rev->sub;
        context->srf_buffer[MTB_LIN_PDU_D4_IDX] = (l_u8)context->config->file_rev->source;

        /* Reserved for future use and transmitted with value 0x00 */
        context->srf_buffer[MTB_LIN_PDU_D5_IDX] = 0x00U;

        /* Sets a service response bit that indicates that the response is
         * ready to be sent to Master node.
         */
        context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;

        result = true;
    }

    return (result);
}


/*******************************************************************************
* Function Name: mtb_lin_service_read_by_user_id
****************************************************************************//**
*
* Fills the SRF Buffer with a response to the "User Defined Identification"
* request.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
* \return
* Returns "true" if a positive response is required.
*
*******************************************************************************/
__STATIC_INLINE l_bool mtb_lin_service_read_by_user_id(mtb_stc_lin_context_t* context)
{
    l_bool result = false;
    l_u8   temp_status;

    /* If the user does not reassign the status of ld_read_by_id_callout(),
     * LD_NEGATIVE_RESPONSE is always returned by ld_read_by_id_callout().
     * This indicates to the master that the service by the user defined
     * identification is not supported. temp_status is used to hold the status of
     * ld_read_by_id_callout().
     */
    CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 18.4', 2,
                            'Advisory. Usage of arithmetic on pointer. This approach gives flexibility in data access. Checked manually, no possible issues expected.');
    temp_status = ld_read_by_id_callout(0U, context->mrf_buffer[MTB_LIN_PDU_D1_IDX],
                                        (context->mrf_buffer + MTB_LIN_FRAME_DATA_SIZE_3),
                                        (context->mrf_buffer + MTB_LIN_PDU_PCI_IDX),
                                        context);
    CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.4');

    if (temp_status == LD_POSITIVE_RESPONSE)
    {
        /* Fills SRF Buffer with a response to the service. The NAD field should contain the current
           NAD. */
        context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

        /* PCI is 0, so only the length is required. Plus one for RSID. */
        context->srf_buffer[MTB_LIN_PDU_PCI_IDX] =
            (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_LENGTH_MASK) + 1U;

        /* RSID for a positive response is always SID + 0x40 */
        context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_POS_RESP_READ_BY_ID;

        /* Fills unused data bytes with user-defined information */
        for (l_u8 i = 3U; i < MTB_LIN_FRAME_LEN; i++)
        {
            /* User data is located at the offset */
            if (i <
                ((context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_LENGTH_MASK) +
                 MTB_LIN_PDU_D1_IDX))
            {
                context->srf_buffer[i] = context->mrf_buffer[i];
            }
            else
            {
                context->srf_buffer[i] =  0xFFU;
            }
        }

        /* Sets the service response bit that indicates that the response is
         * ready to be sent to Master node.
         */
        context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;

        result = true;
    }

    return (result);
}


/*******************************************************************************
* Function Name: mtb_lin_service_read_by_id_negative_resp
****************************************************************************//**
*
* Fills the SRF Buffer with a negative response.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
*******************************************************************************/
__STATIC_INLINE void mtb_lin_service_read_by_id_negative_resp(mtb_stc_lin_context_t* context)
{
    /* The NAD field should contain the current NAD. */
    context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

    /* PCI is 0 so only the length is required */
    context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 3U;

    /* RSID for a negative response is always 0x7F */
    context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_RSID_NEG_REPLY;

    /* D1 holds the service ID */
    context->srf_buffer[MTB_LIN_PDU_D1_ID_IDX] = MTB_LIN_NCS_READ_BY_ID;

    /* D2 contains error code */
    context->srf_buffer[MTB_LIN_PDU_D2_IDX] = 0x12U;

    /* Fill unused data bytes with 0xFFs */
    for (l_u8 i=  5U; i < MTB_LIN_FRAME_LEN; i++)
    {
        context->srf_buffer[i] = 0xFFu;
    }

    /* Set service response bit that indicates that the response is
     * ready to be sent to Master node.
     */
    context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;
}


/*******************************************************************************
* Function Name: mtb_lin_service_cond_change_nad
****************************************************************************//**
*
* The internal function implements the "Conditional Change NAD" 0xB3 service.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
*******************************************************************************/
void mtb_lin_service_cond_change_nad(mtb_stc_lin_context_t* context)
{
    l_u8 id_byte;
    l_u8 id_mask;
    l_u8 id_invert;
    l_u8 temp_status = 0U;
    l_bool response_flag = false;

    /* Checks the data length. Does not respond if the PCI length does not match this service data
       length. */
    if (MTB_LIN_PDU_SF_DATA_LEN ==
        (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_LENGTH_MASK))
    {
        /* Validates if preserved Node Address values are not used as NAD for slave node */
        if (mtb_lin_config_check_nad(context->mrf_buffer[MTB_LIN_PDU_D5_NEW_NAD_IDX],
                                     context->config) == MTB_LIN_STATUS_SUCCESS)
        {
            if (MTB_LIN_NCS_READ_BY_ID_ID == context->mrf_buffer[MTB_LIN_PDU_D1_ID_IDX])
            {
                /* LIN Product Identification */
                if ((6U > context->mrf_buffer[MTB_LIN_PDU_D2_BYTE_IDX]) &&
                    (0U != context->mrf_buffer[MTB_LIN_PDU_D2_BYTE_IDX]))
                {
                    switch (context->mrf_buffer[MTB_LIN_PDU_D2_BYTE_IDX])
                    {
                        /* temp_status is used as a temporary variable to store ID byte */
                        case MTB_LIN_CS_BYTE_SUPPLIER_ID1:
                            temp_status = CY_LO8(context->config->supplier_id);
                            break;

                        case MTB_LIN_CS_BYTE_SUPPLIER_ID2:
                            temp_status = CY_HI8(context->config->supplier_id);
                            break;

                        case MTB_LIN_CS_BYTE_FUNCTION_ID1:
                            temp_status = CY_LO8(context->config->function_id);
                            break;

                        case MTB_LIN_CS_BYTE_FUNCTION_ID2:
                            temp_status = CY_HI8(context->config->function_id);
                            break;

                        case MTB_LIN_CS_BYTE_VARIANT:
                            temp_status = context->config->variant;
                            break;

                        default:
                            /* Never use this state. */
                            break;
                    }

                    if (0U == ((temp_status ^ (context->mrf_buffer[MTB_LIN_PDU_D4_INVERT_IDX])) &
                               context->mrf_buffer[MTB_LIN_PDU_D3_MASK_IDX]))
                    {
                        /* Fills SRF Buffer with a response to the service. The NAD field should
                           contain
                         * the current NAD.
                         */
                        context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

                        /* Changes the NAD to a new NAD. */
                        context->configured_nad =
                            context->mrf_buffer[MTB_LIN_PDU_D5_NEW_NAD_IDX];

                        response_flag = true;
                    }
                    else
                    {
                        /* Does nothing, ignores an erroneous request. */
                    }
                }
                else
                {
                    /* Does nothing, ignores an erroneous request. */
                }
            }
            else if (MTB_LIN_NCS_READ_BY_ID_SERIAL == context->mrf_buffer[MTB_LIN_PDU_D1_ID_IDX])
            {
                if ((5U < context->mrf_buffer[MTB_LIN_PDU_D2_BYTE_IDX]) &&
                    (0U != context->mrf_buffer[MTB_LIN_PDU_D2_BYTE_IDX]))
                {
                    /* Byte = 1 corresponds to first byte (context->serial_number[0]) */
                    if (0U ==
                        ((context->serial_number[context->mrf_buffer[MTB_LIN_PDU_D2_BYTE_IDX] - 1U]
                          ^ context->mrf_buffer[MTB_LIN_PDU_D4_INVERT_IDX]) &
                         context->mrf_buffer[MTB_LIN_PDU_D3_MASK_IDX]))
                    {
                        /* Fills SRF Buffer with a response to the service. The NAD field should
                           contain
                         * the current NAD.
                         */
                        context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;
                        /* Changes the NAD to a new NAD. */

                        context->configured_nad =
                            context->mrf_buffer[MTB_LIN_PDU_D5_NEW_NAD_IDX];

                        response_flag = true;
                    }
                    else
                    {
                        /* Does nothing, ignores an erroneous request. */
                    }
                }
                else
                {
                    /* Does nothing, ignores an erroneous request. */
                }
            }
            else if ((context->mrf_buffer[MTB_LIN_PDU_D1_IDX] >= 32U) &&
                     (context->mrf_buffer[MTB_LIN_PDU_D1_IDX] <= 63U)) /* User-defined
                                                                          identification */
            {
                /* Byte, Invert, and Mask must be stored in variables for user-defined
                 * identification as frame[] should contain user data after execution of
                 * ld_read_by_id_callout();
                 */
                id_byte   = context->mrf_buffer[MTB_LIN_PDU_D2_BYTE_IDX] - 1U;
                id_invert = context->mrf_buffer[MTB_LIN_PDU_D4_INVERT_IDX];
                id_mask   = context->mrf_buffer[MTB_LIN_PDU_D3_MASK_IDX];

                /* If the user does not re-assign the status of ld_read_by_id_callout(),
                 * LD_NEGATIVE_RESPONSE is always returned by ld_read_by_id_callout(). This
                 * indicates to the master that the service by the user defined identification is
                 * not supported. temp_status is used to hold the status of ld_read_by_id_callout().
                 */
                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 18.4', 2,
                                        'Advisory. Usage of arithmetic on pointer. This approach gives flexibility in data access. Checked manually, no possible issues expected.');
                temp_status = ld_read_by_id_callout(0U, context->mrf_buffer[MTB_LIN_PDU_D1_IDX],
                                                    (context->mrf_buffer + MTB_LIN_PDU_D1_IDX),
                                                    (context->mrf_buffer + MTB_LIN_PDU_PCI_IDX),
                                                    context);
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.4');

                if ((temp_status == LD_NEGATIVE_RESPONSE) || (temp_status == LD_NO_RESPONSE))
                {
                    /* Does nothing as there is no response from the user. */
                }
                else
                {
                    if (0U ==
                        ((context->mrf_buffer[id_byte + MTB_LIN_PDU_D1_IDX] ^ id_invert) & id_mask))
                    {
                        /* Changes the NAD to a new NAD. */
                        context->configured_nad =
                            context->mrf_buffer[MTB_LIN_PDU_D5_NEW_NAD_IDX];

                        response_flag = true;

                        /* Fills SRF Buffer with a response to the service. The NAD field should
                         * contain the changed NAD.
                         */
                        context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;
                    }
                    else
                    {
                        /* Does nothing, ignores an erroneous request */
                    }
                }
            }
            else
            {
                /* Does nothing, ignores an erroneous request */
            }

            if (response_flag)
            {
                /* PCI is 0, so only the length is required. */
                context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 1U;

                /* RSID for a positive response is always SID + 0x40 */
                context->srf_buffer[MTB_LIN_PDU_SID_IDX] =
                    MTB_LIN_NCS_POS_RESP_COND_CHANGE_NAD;

                /* Fills unused bytes with 0xFF. */
                for (l_u8 i = 3U; i < MTB_LIN_FRAME_LEN; i++)
                {
                    context->srf_buffer[i] = 0xFFU;
                }

                /* Sets a service response bit that indicates that the response is
                 * ready to be sent to Master node.
                 */
                context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;
            }
        }
    }
}


/*******************************************************************************
* Function Name: mtb_lin_service_save_config
****************************************************************************//**
*
* The internal function implements the "Save Configuration" 0xB6 service.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
*******************************************************************************/
void mtb_lin_service_save_config(mtb_stc_lin_context_t* context)
{
    /* Checks the data length. Does not respond if the PCI length does not match this service data
       length. */
    /* Only Save configuration services B5 and B6 have PCI.length=1U */
    if (1U == (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_LENGTH_MASK))
    {
        /* Sets the save configuration bit in status register. */
        context->ifc_comm_status |= MTB_LIN_IFC_STS_SAVE_CONFIG;

        /* Fills SRF Buffer with a response to the service. NAD field should contain the current
           NAD. */
        context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

        /* PCI is 0, so only the length is required. */
        context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 1U;

        /* RSID for a positive response is always SID + 0x40. */
        context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_POS_RESP_SAVE_CONFIG;

        /* Fills unused data bytes with 0xFFs. */
        for (l_u8 i = 3U; i < MTB_LIN_FRAME_LEN; i++)
        {
            context->srf_buffer[i] = 0xFFu;
        }

        /* Set the service response bit that indicates that response is ready to be sent to Master
         * node.
         */
        context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;
    }
}


/*******************************************************************************
* Function Name: mtb_lin_service_frame_id_range
****************************************************************************//**
*
* The internal function implements the "Assign frame identifier range" 0xB7
* service.
*
* \param context
*  The pointer to the LIN Slave context structure.
*  See \ref mtb_stc_lin_context_t.
*
*******************************************************************************/
void mtb_lin_service_frame_id_range(mtb_stc_lin_context_t* context)
{
    l_u8 temp_status = 0U;

    /* Checks the data length. Does not respond if the PCI length does not match this service data
       length. */
    if (MTB_LIN_PDU_SF_DATA_LEN ==
        (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_LENGTH_MASK))
    {
        /* Zeroes out the temp status. Used as an error counter. */
        temp_status = 0U;
        for (l_u8 i = 0U; i < MTB_LIN_NCS_MAX_FRAME_ID_RANGE; i++)
        {
            if ((i + context->mrf_buffer[MTB_LIN_PDU_D1_START_IDX]) <
                context->config->num_of_frames)
            {
                if ((context->mrf_buffer[i + MTB_LIN_PDU_D2_PID_IDX] != MTB_LIN_INVALID_FRAME_PID)
                    && ((context->mrf_buffer[i + MTB_LIN_PDU_D2_PID_IDX] & MTB_LIN_PID_PARITY_MASK)
                        < MTB_LIN_FRAME_PID_MRF))
                {
                    /* Unassigned value "0" is used to invalidate this frame for transportation
                     * to the bus. Set new received PID value */
                    context->config->frames_context[i +
                                                    context->mrf_buffer[MTB_LIN_PDU_D1_START_IDX]].
                    configured_pid =
                        context->mrf_buffer[i + MTB_LIN_PDU_D2_PID_IDX];
                }
                else if (context->mrf_buffer[i + MTB_LIN_PDU_D2_PID_IDX] == 0xFFu)
                {
                    /* Does nothing. */
                }
                else
                {
                    /* Indicates an error by changing the status other than 0, if Frame ID is
                       reserved. */
                    temp_status++;
                }
            }
            else
            {
                if (context->mrf_buffer[i + MTB_LIN_PDU_D2_PID_IDX] != 0xFFu)
                {
                    temp_status++;  /* Indicates an error by changing the status other than 0. */
                }
            }
        }

        if (temp_status == 0U) /* No errors condition check */
        {
            /* Fills SRF Buffer with a response to the service. The NAD field should contain the
             *  current NAD.
             */
            context->srf_buffer[MTB_LIN_PDU_NAD_IDX] = context->configured_nad;

            /* PCI is 0, so only the length is required. */
            context->srf_buffer[MTB_LIN_PDU_PCI_IDX] = 1U;

            /* RSID for a positive response is always SID + 0x40. */
            context->srf_buffer[MTB_LIN_PDU_SID_IDX] = MTB_LIN_NCS_POS_RESP_ASSIGN_FRAME_ID_RANGE;

            /* Fills unused data bytes with 0xFFs. */
            for (l_u8 i = 3U; i < MTB_LIN_FRAME_LEN; i++)
            {
                context->srf_buffer[i] = 0xFFu;
            }

            /* Sets a service response bit that indicates that the response is
             * ready to be sent to master node.
             */
            context->status |= MTB_LIN_STATUS_SRVC_RSP_RDY;
        }
        else
        {
            /* Does nothing, ignores an erroneous request */
        }
    }
}


/* [] END OF FILE */
