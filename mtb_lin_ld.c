/***************************************************************************//**
 * \file mtb_lin_ld.c
 * \version 1.10
 *
 * \brief
 * Provides the LIN middleware node configuration and identification API
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

static l_bool mtb_lin_mrf_accept_nad(const mtb_stc_lin_context_t* context);
static void mtb_lin_mrf_sleep(mtb_stc_lin_context_t* context);
static l_u8 mtb_lin_mrf_single_frame(mtb_stc_lin_context_t* context);
static void mtb_lin_mrf_single_frame_tl(mtb_stc_lin_context_t* context);
static l_u8 mtb_lin_mrf_first_frame(mtb_stc_lin_context_t* context);
static l_u8 mtb_lin_mrf_consecutive_frame(mtb_stc_lin_context_t* context);


/*******************************************************************************
* Function Name: ld_init
****************************************************************************//**
*
* This call will (re)initialize the raw and the cooked layers
* on the interface iii.
*
* All transport layer buffers will be initialized.
*
* If there is an ongoing diagnostic frame transporting a cooked or raw message
* on the bus, it will not be aborted.
*
* This API must be called before using any Transport Layer API functions.
* It must also be called before the slave node can do any Transport Layer
* communication.
*
* If the API is called in the middle of an ongoing diagnostic frame transporting
* a cooked or raw message on the bus, the function waits until the message
* is completed.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void ld_init(l_ifc_handle iii, mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    l_u32 interrupt_state;

    if ((context->config->tl_enabled) &&
        (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED))
    {
        while (0U != (context->tl_flags & MTB_LIN_TL_TX_DIRECTION))
        {
            /* Waits until the current message is processed. */
        }

        /* Saves the interrupt state and disable interrupts. */
        interrupt_state = Cy_SysLib_EnterCriticalSection();

        /* Initializes the TX and RX status variables correctly */
        context->tl_tx_status = LD_COMPLETED;
        context->tl_rx_status = LD_COMPLETED;

        context->tl_length_pointer = NULL;

        /* Resets the frame counters */
        context->tl_rx_message_length = 0U;
        context->tl_tx_message_length = 0U;

        /* Resets the frame counters */
        context->tl_tx_frame_counter = 0U;
        context->tl_rx_frame_counter = 0U;

        context->tl_rx_data_pointer = NULL;
        context->tl_rx_initial_data_pointer = NULL;

        /* The previous PCI requires to be unknown after initialization */
        context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_UNKNOWN;
        context->tl_tx_prev_pci = MTB_LIN_PDU_PCI_TYPE_UNKNOWN;
    }
    else
    {
        /* Saves the interrupt state and disables the interrupts */
        interrupt_state = Cy_SysLib_EnterCriticalSection();

        /* Resets the buffers depth to 0 to indicate that the buffers are empty. */
        context->tl_raw_tx_buf_depth = 0U;
        context->tl_raw_rx_buf_depth = 0U;

        /* Raw API buffers initialization */
        context->tl_raw_tx_write_index = 0U;
        context->tl_raw_tx_read_index = 0U;

        context->tl_raw_rx_write_index = 0U;
        context->tl_raw_rx_read_index = 0U;

        context->tl_tx_status = LD_QUEUE_EMPTY;
        context->tl_rx_status = LD_NO_DATA;
    }

    /* Sets the initial NAD as the current active NAD before initializing TL */
    context->configured_nad = context->config->initial_nad;

    context->tl_flags = 0U;

    /* Changes the node state to the next state */
    context->node_state = MTB_LIN_NODE_STATE_IDLE;

    /* Enables the interrupts */
    Cy_SysLib_ExitCriticalSection(interrupt_state);
}


/*******************************************************************************
* Function Name: ld_read_by_id_callout
****************************************************************************//**
*
*  This function calls out when Master node transmits a "read" by an identifier
*  request with the identifier in the user-defined area. Slave node
*  application is called from the driver when such a request is received.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param id
*  The ID parameter is the identifier in the user--defined area (32 to 63),
*  from the "read" by an identifier configuration request.
*
* \param frame_data
*  The data pointer points to a data area with 5 bytes. This area
*  is used by the application to set up a positive response.
*
*  \param data_length
*  The data pointer, which points to a data length in response. The length is 4 LSB
*  bits in PCI ([bit0 - bit3]). The RSID byte is added by FW, the user must set
*  just the data size.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* \ref group_lin_ld_note_identification_status_macros
*
*******************************************************************************/
l_u8 ld_read_by_id_callout(l_ifc_handle iii, l_u8 id, l_u8* frame_data,
                           l_u8* data_length, const mtb_stc_lin_context_t* context)
{
    l_u8 result = LD_NEGATIVE_RESPONSE;

    if (context->read_by_id_callback != NULL)
    {
        result = context->read_by_id_callback(iii, id, frame_data, data_length);
    }

    return (result);
}


/*******************************************************************************
* Function Name: ld_read_configuration
****************************************************************************//**
*
*  This function reads the NAD and PID values from the volatile memory.
*  This function reads the current configuration data, and
*  saves this data into the non-volatile (flash) memory. The application
*  saves the configuration data to the flash when the "Save Configuration" bit
*  is set in the LIN status register (returned by
*  l_ifc_read_status_LINS).
*  The configuration data that is read is a series of bytes. The first byte is
*  the current NAD of slave. The next bytes are the current PID values for
*  the frames that the Slave responds to. The PID values are in the order in
*  which the frames appear in the LDF or NCF file.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param pData
*  The pointer to data.
*
* \param length
*  The length of the configuration bytes.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* \ref group_lin_ld_note_configuration_status_macros
*
*******************************************************************************/
l_u8 ld_read_configuration(l_ifc_handle iii, l_u8* pData, l_u8* const length,
                           const mtb_stc_lin_context_t* context)
{
    l_u8 result = LD_READ_OK;

    CY_UNUSED_PARAMETER(iii);

    if (*length < (context->config->num_of_frames + 1U))
    {
        /* Returns with no action when the requested length is smaller
         *  than the configuration data length.
         */
        result = LD_LENGTH_TOO_SHORT;
    }
    else
    {
        /* Copy the configured NAD */
        pData[0U] = context->configured_nad;

        /* Copy the data from the PID array to the data array */
        for (l_u8 i = 0U; i < context->config->num_of_frames; i++)
        {
            pData[i + 1U] = context->config->frames_context[i].configured_pid;
        }

        /* Set the length parameter to the actual length of the configuration data */
        *length = context->config->num_of_frames + 1U;
    }

    /* Return the status */
    return (result);
}


/*******************************************************************************
* Function Name: ld_set_configuration
****************************************************************************//**
*
*  This call does not transport anything to the bus.
*
*  The function configures NAD and PIDs accordingly to the
*  configuration given by data. The intended usage is to restore the saved
*  configuration or set initial configuration (e.g. coded by I/O pins).
*
*  The function is called after calling l_ifc_init.
*
*  The caller must set the size of the data area before calling the function.
*
*  The data contains NAD and PIDs each occupying one byte.
*  The data structure is: NAD and all PIDs for the frames.
*  The PIDs order is the same as the frame list in LDF,
*  Section 9.2.2.2, and NCF, Section 8.2.5.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param pData
*  The pointer to data.
*
* \param length
*  The length of the configuration bytes.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* \ref group_lin_ld_note_configuration_status_macros
*
*******************************************************************************/
l_u8 ld_set_configuration(l_ifc_handle iii, const l_u8* const pData, l_u16 length,
                          mtb_stc_lin_context_t* context)
{
    l_u8 result = LD_SET_OK;

    CY_UNUSED_PARAMETER(iii);


    if (length != ((l_u16)context->config->num_of_frames + 1U))
    {
        /* Returns an error if the length is not correct. */
        result = LD_LENGTH_NOT_CORRECT;
    }
    else
    {
        /* Copies the NAD to the volatile memory */
        context->configured_nad = pData[0U];

        /* Data read back */
        if (context->configured_nad != pData[0U])
        {
            /* Indicates a data error if the NAD is not set correctly. */
            result = LD_DATA_ERROR;
        }

        /* Copies the Frame PIDs to the volatile memory */
        for (l_u8 i = 0U; i < context->config->num_of_frames; i++)
        {
            context->config->frames_context[i].configured_pid = pData[i + 1U];

            /* Data read back */
            if (context->config->frames_context[i].configured_pid != pData[i + 1U])
            {
                /* Indicates a data error if the NAD is not set correctly. */
                result = LD_DATA_ERROR;
            }
        }
    }

    /* Returns success code if the copy is completed. */
    return (result);
}


/*******************************************************************************
* Function Name: mtb_lin_check_product_id
****************************************************************************//**
*
*  Verifies that the received LIN product identification matches.
*
* \param frame_data[]
*  The pointer to the 4 bytes that hold LIN product ID.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  0 - If the LIN product IDs do not match.
*  1 - If the LIN product IDs match.
*
*******************************************************************************/
l_bool mtb_lin_check_product_id(volatile const l_u8 frame_data[],
                                const mtb_stc_lin_context_t* context)
{
    l_bool i = true;


    if ((frame_data[0U] != CY_LO8(context->config->supplier_id)) &&
        (frame_data[0U] != CY_LO8(MTB_LIN_CS_SUPPLIER_ID_WILDCARD)))
    {
        i = false;        /* The data is not for this Slave */
    }

    if ((frame_data[1U] != CY_HI8(context->config->supplier_id)) &&
        (frame_data[1U] != CY_HI8(MTB_LIN_CS_SUPPLIER_ID_WILDCARD)))
    {
        i = false;        /* Data is not for this Slave */
    }

    if ((frame_data[2U] != CY_LO8(context->config->function_id)) &&
        (frame_data[2U] != CY_LO8(MTB_LIN_CS_FUNCTION_ID_WILDCARD)))
    {
        i = false;        /* Data is not for this Slave */
    }

    if ((frame_data[3U] != CY_HI8(context->config->function_id)) &&
        (frame_data[3U] != CY_HI8(MTB_LIN_CS_FUNCTION_ID_WILDCARD)))
    {
        i = false;        /* Data is not for this Slave */
    }

    return (i);
}


/*******************************************************************************
* Function Name: mtb_lin_transmit_tl_frame
****************************************************************************//**
*
*  Transmits the frame of a segmented message.
*
* \param stimulus
*  See \ref mtb_stc_lin_node_state_stimulus_t for details.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* - MTB_LIN_NO_RESPONSE_REQUIRED
* - MTB_LIN_TRANSMISSION_CONTINUES
*
*******************************************************************************/
l_u8 mtb_lin_transmit_tl_frame(mtb_stc_lin_node_state_stimulus_t stimulus,
                               mtb_stc_lin_context_t* context)
{
    l_u8 result = MTB_LIN_NO_RESPONSE_REQUIRED;
    CY_UNUSED_PARAMETER(stimulus);
    CY_UNUSED_PARAMETER(context);

    /* Clears CR Timeout after answering SRF. */
    context->tl_flags &= ((l_u8) ~MTB_LIN_TL_N_CR_TIMEOUT_ON);

    if (0U == (context->status & MTB_LIN_STATUS_SRVC_RSP_RDY))
    {
        if ((0U != context->tl_tx_message_length) &&
            (0U != (context->tl_flags & MTB_LIN_TL_TX_REQUESTED)))
        {
            result = MTB_LIN_TRANSMISSION_CONTINUES;
        }
    }

    return (result);
}


/*******************************************************************************
* Function Name: mtb_lin_clear_rx_buffer
****************************************************************************//**
*
*  Transmits the frame of a segmented message.
*  This function initializes the receive part of Transport Layer variables .
*
* \param stimulus
*  See \ref mtb_stc_lin_node_state_stimulus_t for details.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  None.
*
*******************************************************************************/
void mtb_lin_clear_rx_buffer(mtb_stc_lin_node_state_stimulus_t stimulus,
                             mtb_stc_lin_context_t* context)
{
    /* The previous PCI requires to be unknown after the initialization */
    context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_UNKNOWN;

    /* Resets the frame counters. */
    context->tl_rx_message_length = 0U;
    context->tl_rx_frame_counter  = 0U;

    if ((context->config->tl_enabled) &&
        (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED))
    {
        if (context->tl_length_pointer != NULL)
        {
            /* Clears the length of the erroneous frame. */
            *context->tl_length_pointer = 0U;
        }

        if (stimulus == MTB_LIN_NODE_STIMULUS_RX_TIMEOUT)
        {
            /* Sets the error status as a timeout occurs. */
            context->tl_rx_status = LD_N_CR_TIMEOUT;
        }
        else if (stimulus == MTB_LIN_NODE_STIMULUS_MRF_ALIEN_NAD)
        {
            context->tl_rx_status = LD_FAILED;
        }
        else
        {
            if ((context->tl_rx_status != LD_FAILED) &&
                (context->tl_rx_status != LD_WRONG_SN) &&
                (context->tl_rx_status != LD_N_CR_TIMEOUT))
            {
                /* Initializes the RX status variable properly. */
                context->tl_rx_status = LD_COMPLETED;
            }
        }

        context->tl_rx_data_pointer = NULL;
        context->tl_rx_initial_data_pointer = NULL;
    }
    else
    {
        CY_UNUSED_PARAMETER(stimulus);

        /* Resets the buffers depth to 0 to indicate the buffers are empty. */
        context->tl_raw_rx_buf_depth = 0U;

        /* Raw API buffers initialization */
        context->tl_raw_rx_write_index = 0U;
        context->tl_raw_rx_read_index = 0U;
        context->tl_rx_status   = LD_NO_DATA;
    }

    /* Clears the Service Response ready status bit */
    context->status &= ((l_u8) ~MTB_LIN_STATUS_SRVC_RSP_RDY);

    context->tl_flags &= (l_u8)(~(MTB_LIN_TL_RX_REQUESTED | MTB_LIN_TL_N_CR_TIMEOUT_ON));

    context->tl_timeout_counter = 0U;
}


/*******************************************************************************
* Function Name: mtb_lin_clear_tx_buffer
****************************************************************************//**
*
*  Transmits the frame of a segmented message.
*  This function initializes the transmit part of the Transport Layer variables.
*
* \param stimulus
*  See \ref mtb_stc_lin_node_state_stimulus_t for details.
*
* \param context
*  Pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void mtb_lin_clear_tx_buffer(mtb_stc_lin_node_state_stimulus_t stimulus,
                             mtb_stc_lin_context_t* context)
{
    /* The previous PCI requires to be unknown after the initialization */
    context->tl_tx_prev_pci = MTB_LIN_PDU_PCI_TYPE_UNKNOWN;

    if ((context->config->tl_enabled) &&
        (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED))
    {
        /* Resets the frame counters */
        context->tl_tx_message_length = 0U;

        /* Resets the frame counters */
        context->tl_tx_frame_counter = 0U;

        if (stimulus == MTB_LIN_NODE_STIMULUS_TX_TIMEOUT)
        {
            /* Sets the error status as a timeout occurs  */
            context->tl_tx_status = LD_N_AS_TIMEOUT;
        }
        else if (stimulus == MTB_LIN_NODE_STIMULUS_MRF_ALIEN_NAD)
        {
            context->tl_tx_status = LD_FAILED;
        }
        else
        {
            if ((context->tl_tx_status != LD_FAILED) && (context->tl_tx_status != LD_N_AS_TIMEOUT))
            {
                /* Initializes the TX status variable properly. */
                context->tl_tx_status = LD_COMPLETED;
            }
        }
        context->tl_flags &= (l_u8)(~(MTB_LIN_TL_TX_REQUESTED | MTB_LIN_TL_N_AS_TIMEOUT_ON |
                                      MTB_LIN_TL_N_CR_TIMEOUT_ON));
    }
    else
    {
        CY_UNUSED_PARAMETER(stimulus);

        if (0U == (context->status & MTB_LIN_STATUS_RESPONSE_PENDING))
        {   /* Regular frame */
            /* Resets the frame counters */
            context->tl_tx_message_length = 0U;

            /* Resets the frame counters */
            context->tl_tx_frame_counter = 0U;

            /* Resets the buffers depth to 0 to indicate the buffers are empty. */
            context->tl_raw_tx_buf_depth = 0U;

            /* Raw API buffers initialization */
            context->tl_raw_tx_write_index = 0U;
            context->tl_raw_tx_read_index = 0U;
            context->tl_tx_status   = LD_QUEUE_EMPTY;
            context->tl_flags     &=
                (l_u8)(~(MTB_LIN_TL_TX_REQUESTED | MTB_LIN_TL_N_AS_TIMEOUT_ON |
                         MTB_LIN_TL_N_CR_TIMEOUT_ON));
        }
        else /* Response pending frame */
        {
            /* Disables N_AS and N_CR timers, LIN Master controls P2* timeout for the response
               pending
               frame. */
            context->tl_flags &= (l_u8)(~(MTB_LIN_TL_N_AS_TIMEOUT_ON | MTB_LIN_TL_N_CR_TIMEOUT_ON));
        }
    }

    /* Clears the Service Response Ready status bit. */
    context->status         &= ((l_u8) ~MTB_LIN_STATUS_SRVC_RSP_RDY);
    context->tl_timeout_counter = 0U;
}


/*******************************************************************************
* Function Name: mtb_lin_mrf_accept_nad
****************************************************************************//**
*
* Checks if MRF frame will be accepted by Slave node.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* Returns "true" if the NAD is valid for Slave node.
*
*******************************************************************************/
static l_bool mtb_lin_mrf_accept_nad(const mtb_stc_lin_context_t* context)
{
    l_bool result = false;
    l_u8 nad;

    nad = context->mrf_buffer[MTB_LIN_PDU_NAD_IDX];

    if ((context->configured_nad == nad) || (MTB_LIN_NAD_BROADCAST == nad))
    {
        result = true;
    }
    else if ((MTB_LIN_NAD_FUNCTIONAL == nad) && (context->config->spec == MTB_LIN_SPEC_2_2))
    {
        /* The NAD 0x7E is functional node address only for LIN 2.2 */
        result = true;
    }
    else if ((context->config->initial_nad == nad) &&
             (MTB_LIN_NCS_ASSIGN_NAD == context->mrf_buffer[MTB_LIN_PDU_SID_IDX]))
    {
        result = true;
    }
    else
    {
        result = false;
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_lin_mrf_sleep
****************************************************************************//**
*
* The process master request frame for Sleep NAD.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_mrf_sleep(mtb_stc_lin_context_t* context)
{
    /* Enable MRF with ID=0x3E for J2602 */
    context->ifc_comm_status |= MTB_LIN_IFC_STS_GO_TO_SLEEP;
}


/*******************************************************************************
* Function Name: mtb_lin_mrf_single_frame_tl
****************************************************************************//**
*
* Processes the transport layer of the MRF single frame.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_mrf_single_frame_tl(mtb_stc_lin_context_t* context)
{
    if ((context->config->tl_enabled) &&
        (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED))
    {
        /* Gets one frame of a message if there is a message pending. */
        if (context->tl_rx_status == LD_IN_PROGRESS)
        {
            /* Makes sure the pointer points to the receive buffer beginning. */
            context->tl_rx_data_pointer = context->tl_rx_initial_data_pointer;
            /* Copies data to the user buffer. */
            for (l_u8 i = 0U; i < context->mrf_buffer[MTB_LIN_PDU_PCI_IDX]; i++)
            {
                *context->tl_rx_data_pointer = context->mrf_buffer[i + 2U];
                context->tl_rx_data_pointer++;
            }
            /* Stores the NAD */
            *context->tl_nad_pointer = context->mrf_buffer[MTB_LIN_PDU_NAD_IDX];
            /* Gets the data bytes length. */
            *context->tl_length_pointer =
                (l_u16)context->mrf_buffer[MTB_LIN_PDU_PCI_IDX];
            /* Updates the length pointer properly. */
            context->tl_rx_message_length = 0U;
            /* SF message is received, so sets the proper status */
            context->tl_rx_status = LD_COMPLETED;
        }
    }
    else
    {
        if (context->tl_raw_rx_buf_depth < (context->config->tl_rx_queue_len / 8U))
        {
            /* Fills the RX queue from the MRF buffer. */
            for (l_u8 i = 0U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
            {
                context->config->tl_raw_rx_queue[context->tl_raw_rx_write_index] =
                    context->mrf_buffer[i];
                context->tl_raw_rx_write_index++;
            }
            /* The Read index points to the next byte in MRF. */
            if (context->tl_raw_rx_write_index == context->config->tl_rx_queue_len)
            {
                context->tl_raw_rx_write_index = 0U;
            }
            /* The 8 Bytes copied to the MRF - increment buffer depth */
            context->tl_raw_rx_buf_depth++;
            /* Specification does not require status "queue full"
             * so unconditionally set the status to the data available
             */
            context->tl_rx_status         = LD_DATA_AVAILABLE;
            context->tl_rx_message_length = 0U;
        }
    }
}


#define MTB_LIN_NCS_PROCESS_ASSIGN_NAD(c)   \
    ((MTB_LIN_NCS_ASSIGN_NAD == (c)->mrf_buffer[MTB_LIN_PDU_SID_IDX]) && \
    ((c)->config->service_assign_nad))

#define MTB_LIN_NCS_PROCESS_READ_BY_ID(c)   \
    ((MTB_LIN_NCS_READ_BY_ID == (c)->mrf_buffer[MTB_LIN_PDU_SID_IDX]) && \
    ((c)->config->service_read_by_id))

#define MTB_LIN_NCS_PROCESS_COND_CHANGE_NAD(c)   \
    ((MTB_LIN_NCS_COND_CHANGE_NAD == (c)->mrf_buffer[MTB_LIN_PDU_SID_IDX]) && \
    ((c)->config->service_cond_change_nad))

#define MTB_LIN_NCS_PROCESS_SAVE_CONFIG(c)   \
    ((MTB_LIN_NCS_SAVE_CONFIG == (c)->mrf_buffer[MTB_LIN_PDU_SID_IDX]) && \
    ((c)->config->service_save_config))

#define MTB_LIN_NCS_PROCESS_ASSIGN_FRAME_ID_RANGE(c)   \
    ((MTB_LIN_NCS_ASSIGN_FRAME_ID_RANGE == (c)->mrf_buffer[MTB_LIN_PDU_SID_IDX]) && \
    ((c)->config->service_assign_frame_id_range))

/*******************************************************************************
* Function Name: mtb_lin_mrf_single_frame
****************************************************************************//**
*
* Process single frame master request frame.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* MTB_LIN_RESPONSE_REQUIRED
* MTB_LIN_NO_RESPONSE_REQUIRED
*
*******************************************************************************/
static l_u8 mtb_lin_mrf_single_frame(mtb_stc_lin_context_t* context)
{
    l_u8 result = MTB_LIN_NO_RESPONSE_REQUIRED;
    l_bool handle_in_tl = false;

    /* SID used for node configuration */
    if (MTB_LIN_NCS_PROCESS_ASSIGN_NAD(context))
    {
        mtb_lin_service_assign_nad(context);
    }
    else if (MTB_LIN_NCS_PROCESS_READ_BY_ID(context))
    {
        mtb_lin_service_read_by_id(context);
    }
    else if (MTB_LIN_NCS_PROCESS_COND_CHANGE_NAD(context))
    {
        mtb_lin_service_cond_change_nad(context);
    }
    else if (MTB_LIN_NCS_PROCESS_SAVE_CONFIG(context))
    {
        mtb_lin_service_save_config(context);
    }
    else if (MTB_LIN_NCS_PROCESS_ASSIGN_FRAME_ID_RANGE(context))
    {
        mtb_lin_service_frame_id_range(context);
    }
    else
    {
        handle_in_tl = true;
    }

    if (handle_in_tl)
    {
        /* SID used for diagnostics */
        if (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] <= MTB_LIN_PDU_SF_DATA_LEN)
        {
            mtb_lin_mrf_single_frame_tl(context);
            result = MTB_LIN_RESPONSE_REQUIRED;
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_lin_mrf_first_frame
****************************************************************************//**
*
* Process first frame master request frame.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* - MTB_LIN_NO_RESPONSE_REQUIRED
* - MTB_LIN_RECEIVE_CONTINUES
*
*******************************************************************************/
static l_u8 mtb_lin_mrf_first_frame(mtb_stc_lin_context_t* context)
{
    l_u8 result = MTB_LIN_NO_RESPONSE_REQUIRED;
    l_u8 tmp;
    l_u16 size;

    size = (l_u16)context->mrf_buffer[MTB_LIN_PDU_PCI_IDX];
    size &= MTB_LIN_PDU_PCI_LENGTH_MASK;
    size <<= 8U;
    size |= context->mrf_buffer[MTB_LIN_PDU_LEN_IDX];
    if (size >= MTB_LIN_FRAME_DATA_SIZE_7)
    {
        if ((context->config->tl_enabled) &&
            (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED))
        {
            l_u16 tmpWord;

            /* Gets one frame of a message if there is message pending and PCI is valid. */
            context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_UNKNOWN;

            /* Get First Frame Length with the following two operations */
            tmp =
                (context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & ((l_u8) ~MTB_LIN_PDU_PCI_TYPE_MASK));
            tmpWord =
                ((l_u16)((l_u16)tmp <<
                         8U)) | ((l_u16)context->mrf_buffer[MTB_LIN_PDU_LEN_IDX]);

            if ((context->tl_rx_status == LD_IN_PROGRESS) &&
                (context->config->tl_buf_len_max >= tmpWord))
            {
                context->tl_rx_message_length = tmpWord;
                /* Copy the Length to the current length variable */
                *context->tl_length_pointer = context->tl_rx_message_length;
                for (l_u8 i = 3U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
                {
                    *context->tl_rx_data_pointer = context->mrf_buffer[i];   /* Get Frame
                                                                                Data */
                    context->tl_rx_data_pointer++;
                }
                /* Updates the length pointer properly. */
                context->tl_rx_message_length -= MTB_LIN_FRAME_DATA_SIZE_5;

                /* Saves the state of the Frame Counter to monitor future possible errors.
                 */
                context->tl_rx_frame_counter = 0U;

                /* Saves the PCI type */
                context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_FF;
                result       = MTB_LIN_RECEIVE_CONTINUES;
            }
        }
        else /* ((context->config->tl_enabled) && (context->config->tl_api_format ==
                MTB_LIN_TL_FORMAT_COOKED)) */
        {
            if (context->tl_raw_rx_buf_depth < (context->config->tl_rx_queue_len / 8U))
            {
                /* Copy the Length to the current length variable */
                context->tl_rx_message_length =
                    (l_u16)((((l_u16)context->mrf_buffer[MTB_LIN_PDU_PCI_IDX]) &
                             ((l_u16)((l_u8) ~
                                      MTB_LIN_PDU_PCI_TYPE_MASK))) << 8U);
                context->tl_rx_message_length |=
                    ((l_u16)context->mrf_buffer[MTB_LIN_PDU_LEN_IDX]);

                /* Fill MRF from the frame buffer */
                for (l_u8 i = 0U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
                {
                    context->config->tl_raw_rx_queue[context->tl_raw_rx_write_index] =
                        context->mrf_buffer[i];
                    context->tl_raw_rx_write_index++;
                }
                /* The "read" index points to the next byte in MRF */
                if (context->tl_raw_rx_write_index == context->config->tl_rx_queue_len)
                {
                    context->tl_raw_rx_write_index = 0U;
                }
                /* The 8 Bytes copied to MRF - increment the buffer depth */
                context->tl_raw_rx_buf_depth++;
                /* Specification does not require status "queue full",
                 * so unconditionally set the status to the data available
                 */
                context->tl_rx_status = LD_DATA_AVAILABLE;

                /* Updates the length pointer properly. */
                context->tl_rx_message_length -= MTB_LIN_FRAME_DATA_SIZE_5;

                /* Saves the state of the Frame Counter to monitor future possible errors.
                 */
                context->tl_rx_frame_counter = 0U;

                /* Saves the PCI type */
                context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_FF;
                result       = MTB_LIN_RECEIVE_CONTINUES;
            }
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_lin_mrf_consecutive_frame
****************************************************************************//**
*
* Processes a consecutive frame master request frame.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* - MTB_LIN_RESPONSE_REQUIRED
* - MTB_LIN_NO_RESPONSE_REQUIRED
* - MTB_LIN_ERRONEOUS_TL_FRAME
*
*******************************************************************************/
static l_u8 mtb_lin_mrf_consecutive_frame(mtb_stc_lin_context_t* context)
{
    l_u8 result = MTB_LIN_NO_RESPONSE_REQUIRED;
    l_u8 tmp;

    /* Gets one frame of a message if there is a message pending and the PCI is valid. */
    if ((context->tl_rx_prev_pci == MTB_LIN_PDU_PCI_TYPE_FF) ||
        (context->tl_rx_prev_pci == MTB_LIN_PDU_PCI_TYPE_CF))
    {
        tmp = context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & ((l_u8) ~MTB_LIN_PDU_PCI_TYPE_MASK);

        /* Checks if the frame counter is valid, the counter is always less than 16. */
        if (((context->tl_rx_frame_counter + 1U) & 0x0Fu) == tmp)
        {
            if ((context->config->tl_enabled) &&
                (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED))
            {
                /* Checks if a message is requested from the application. */
                if (context->tl_rx_status == LD_IN_PROGRESS)
                {
                    for (l_u8 i = 2U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
                    {
                        *context->tl_rx_data_pointer = context->mrf_buffer[i];    /* Get
                                                                                        Frame
                                                                                        Data */
                        context->tl_rx_data_pointer++;
                    }

                    /* Saves the current Frame Counter. */
                    context->tl_rx_frame_counter = context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] &
                                                   ((l_u8) ~MTB_LIN_PDU_PCI_TYPE_MASK);

                    /* Saves the PCI type */
                    context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_CF;

                    /* Updates the length pointer properly. */
                    if (context->tl_rx_message_length > MTB_LIN_FRAME_DATA_SIZE_6)
                    {
                        context->tl_rx_message_length -= MTB_LIN_FRAME_DATA_SIZE_6;
                        result                 = MTB_LIN_RECEIVE_CONTINUES;
                    }
                    else
                    {
                        context->tl_rx_message_length = 0U;
                        result                = MTB_LIN_RESPONSE_REQUIRED;
                    }
                }
            }
            else
            {
                if (context->tl_raw_rx_buf_depth < (context->config->tl_rx_queue_len / 8U))
                {
                    /* Fills the MRF from the frame buffer. */
                    for (l_u8 i = 0U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
                    {
                        context->config->tl_raw_rx_queue[context->tl_raw_rx_write_index] =
                            context->mrf_buffer[i];
                        context->tl_raw_rx_write_index++;
                    }
                    /* The Read index points to the next byte in MRF. */
                    if (context->tl_raw_rx_write_index == context->config->tl_rx_queue_len)
                    {
                        context->tl_raw_rx_write_index = 0U;
                    }
                    /* The 8 Bytes copied to MRF - increment the buffer depth */
                    context->tl_raw_rx_buf_depth++;
                    /* Specification does not require the status of "queue full"
                     * so unconditionally set the status to the data available
                     */
                    context->tl_rx_status = LD_DATA_AVAILABLE;

                    /* Saves the current Frame Counter. */
                    context->tl_rx_frame_counter = context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] &
                                                   ((l_u8) ~MTB_LIN_PDU_PCI_TYPE_MASK);

                    /* Saves the PCI type */
                    context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_CF;

                    /* Updates the length pointer properly. */
                    if (context->tl_rx_message_length > MTB_LIN_FRAME_DATA_SIZE_6)
                    {
                        context->tl_rx_message_length -= MTB_LIN_FRAME_DATA_SIZE_6;
                        result                 = MTB_LIN_RECEIVE_CONTINUES;
                    }
                    else
                    {
                        context->tl_rx_message_length = 0U;
                        result                = MTB_LIN_RESPONSE_REQUIRED;
                    }
                }
            }
        }
        else
        {
            /* Indicates an an error if the frame counter is invalid. */
            context->tl_rx_status = LD_WRONG_SN;
            result        = MTB_LIN_ERRONEOUS_TL_FRAME;
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_lin_process_mrf
****************************************************************************//**
*
*  Transmits the frame of a segmented message.
*  This API is called from ISR. It is responsible for parsing RX frames
*  that come from the LIN Master. This API handles Automatic Configuration
*  requests and receives both Raw and Cooked API frames.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* - MTB_LIN_RESPONSE_REQUIRED
* - MTB_LIN_NO_RESPONSE_REQUIRED
* - MTB_LIN_ERRONEOUS_TL_FRAME
* - MTB_LIN_RECEIVE_CONTINUES
*
*******************************************************************************/
l_u8 mtb_lin_process_mrf(mtb_stc_lin_context_t* context)
{
    l_u8 result = MTB_LIN_NO_RESPONSE_REQUIRED;

    if (MTB_LIN_NAD_GO_TO_SLEEP == context->mrf_buffer[MTB_LIN_PDU_NAD_IDX])
    {
        mtb_lin_mrf_sleep(context);
    }
    else if (mtb_lin_mrf_accept_nad(context))
    {
        l_u8 pci = context->mrf_buffer[MTB_LIN_PDU_PCI_IDX] & MTB_LIN_PDU_PCI_TYPE_MASK;

        if (0U == pci)
        {
            /* Single Frame is detected */
            result = mtb_lin_mrf_single_frame(context);
        }
        else if (pci == MTB_LIN_PDU_PCI_TYPE_FF)
        {
            /* First Frame is detected */
            result = mtb_lin_mrf_first_frame(context);
        }
        else if (pci == MTB_LIN_PDU_PCI_TYPE_CF)
        {
            /* Consecutive Frames are detected */
            result = mtb_lin_mrf_consecutive_frame(context);
        }
        else
        {
            /* Does nothing. SID is invalid. */
            result = MTB_LIN_NO_RESPONSE_REQUIRED;
        }

        if (0U != (context->status & MTB_LIN_STATUS_SRVC_RSP_RDY))
        {
            /* Changes the node state to the next state. */
            result = MTB_LIN_RESPONSE_REQUIRED;
        }
    }
    else  /* Alien NAD. Indicate an error */
    {
        /* Clears the service response ready status bit. */
        context->status &= ((l_u8) ~MTB_LIN_STATUS_SRVC_RSP_RDY);

        if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED)
        {
            /* Reception failed */
            if (0U != (context->tl_flags & MTB_LIN_TL_RX_REQUESTED))
            {
                context->tl_rx_status = LD_FAILED;
                context->tl_tx_status = LD_FAILED;
                context->tl_flags   &= ((l_u8) ~MTB_LIN_TL_RX_REQUESTED);
            }
        }
    }

    if (MTB_LIN_NAD_FUNCTIONAL != context->mrf_buffer[MTB_LIN_PDU_NAD_IDX])
    {
        if ((result == MTB_LIN_RESPONSE_REQUIRED) ||
            (result == MTB_LIN_RECEIVE_CONTINUES))
        {
            context->tl_flags        |= MTB_LIN_TL_N_CR_TIMEOUT_ON;
            context->tl_timeout_counter = 0U;
        }
    }

    if ((result == MTB_LIN_RESPONSE_REQUIRED) && (0U == context->tl_rx_message_length))
    {
        /* Resets the frame counter. */
        context->tl_rx_frame_counter = 0U;

        /* The previous PCI is required to be unknown at the beginning of a new message. */
        context->tl_rx_prev_pci = MTB_LIN_PDU_PCI_TYPE_UNKNOWN;

        if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED)
        {
            if (0U != (context->tl_flags & MTB_LIN_TL_RX_REQUESTED))
            {
                /* Indicates that a message is received. */
                context->tl_rx_status = LD_COMPLETED;

                /* Clears the  RX requested flag as a message was received. */
                context->tl_flags &= ((l_u8) ~MTB_LIN_TL_RX_REQUESTED);
            }
        }
    }

    return (result);
}


/*******************************************************************************
* Function Name: ld_send_message
****************************************************************************//**
*
*  The call packs the information specified by the data and length into one or
*  multiple diagnostic frames. If the call is made in Master node application,
*  the frames are transmitted to Slave node with the address NAD. If the
*  call is made in a Slave node application, the frames are transmitted to
*  Master node with the address NAD. The parameter NAD is not used in Slave
*  nodes.
*
*  The value of the SID (or RSID) is the first byte in the data area.
*
*  The length must be in the range from 1 to 4095 bytes. The length also
*  includes the SID (or RSID) value, i.e. message length plus one.
*
*  The call is asynchronous, i.e. is not suspended until the message is
*  sent, and the buffer cannot be changed by the application as long as calls
*  to ld_tx_status returns LD_IN_PROGRESS.
*
*  The data is transmitted in suitable frames (the master request frame for
*  master nodes and the Slave response frame for Slave nodes).
*
*  If there is a message in progress, the call returns with no action.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param length
*  The size of data to be sent in bytes.
*
* \param nad
*  The address of Slave node to which data is sent.
*
* \param ld_data
*  The array of data to be sent. The value of RSID is the first byte in the data area.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void ld_send_message(l_ifc_handle iii, l_u16 length, l_u8 nad, const l_u8* const ld_data,
                     mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED)
    {
        volatile l_u32 interrupt_state;

        /* NAD is not used in Slave node but the function definition is the same for Master and
           Slave */
        CY_UNUSED_PARAMETER(nad);

        if (context->tl_tx_status != LD_IN_PROGRESS)
        {
            /* Interrupts can be disabled as global variables used by LIN ISR are used below */
            interrupt_state = Cy_SysLib_EnterCriticalSection();

            context->tl_tx_data_pointer = ld_data;

            /* Resets the data count */
            context->tl_tx_data_count = 0U;

            /* Sets up the length pointer. The length must not be greater than
               context->config->tl_buf_len_max */
            context->tl_tx_message_length = length;

            /* Indicates that there is a message in progress */
            context->tl_tx_status = LD_IN_PROGRESS;

            /* Indicates that a Cooked API request transmits data */
            context->tl_flags |= MTB_LIN_TL_TX_REQUESTED;

            /* Restores the interrupt state */
            Cy_SysLib_ExitCriticalSection(interrupt_state);
        }
    }
}


/*******************************************************************************
* Function Name: ld_receive_message
****************************************************************************//**
*
*  The call prepares the LIN diagnostic module to receive one message and store
*  it in the buffer pointed to by data. During the call, the length
*  specifies the maximum length allowed. When the reception is completed, the length
*  changes to the actual length and NAD to the NAD in the message.
*
*  SID (or RSID) are the first byte in the data area.
*
*  The length are in the range from 1 to 4095 bytes, but never more than the
*  value originally set in the call. SID (or RSID) is included in the length.
*
*  The parameter NAD is not used in Slave nodes.
*
*  The call is asynchronous, i.e. it is suspended only after the message is
*  received, and the buffer cannot be changed by the application as long as
*  a call to ld_rx_status returns LD_IN_PROGRESS. If the call is made after the
*  message transmission has started on the bus (i.e. the SF or FF is already
*  transmitted), this message is received. Instead, the function
*  waits until a next message starts.
*
*  The data is received from the succeeding suitable frames (the master request
*  frame for Slave nodes and the Slave response frame for master nodes).
*
*  The application monitors the ld_rx_status and does not call this
*  function until the status is LD_COMPLETED. Otherwise, this function may
*  return inconsistent data in the parameters.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param length
*  The size of data to be received in bytes.
*
* \param nad
*  The address of Slave node from which data is received.
*
* \param ld_data
*  The array of data to be received. The value of the SID is the first byte in the data area.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void ld_receive_message(l_ifc_handle iii, l_u16* const length, l_u8* const nad, l_u8* const ld_data,
                        mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    l_u32 interrupt_state;

    if (context->tl_rx_status != LD_IN_PROGRESS)
    {
        /* Interrupts can be disabled as global variables used by LIN ISR are used below. */
        interrupt_state = Cy_SysLib_EnterCriticalSection();

        /* Sets the user status bits */
        context->tl_rx_status = LD_IN_PROGRESS;

        /* Sets up the data pointer */
        context->tl_rx_data_pointer = ld_data;

        /* Sets up the initial data pointer that always points to the user buffer beginning.
         */
        context->tl_rx_initial_data_pointer = ld_data;

        /* Sets up a NAD pointer. */
        context->tl_nad_pointer = nad;

        /* Sets up the length pointer. */
        context->tl_length_pointer = length;
        context->tl_rx_message_length = *length;

        /* Indicates that Cooked API requests receive data */
        context->tl_flags |= MTB_LIN_TL_RX_REQUESTED;

        /* Restores the interrupt state */
        Cy_SysLib_ExitCriticalSection(interrupt_state);
    }
}


/*******************************************************************************
* Function Name: ld_tx_status
****************************************************************************//**
*
*  The call returns the status of the last made call to ld_send_message.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  See: \ref LD_IN_PROGRESS \ref LD_COMPLETED \ref LD_FAILED
*  \ref LD_N_AS_TIMEOUT
*
*  \note Failed status (e.g. \ref LD_FAILED) can be read only once by this
*  function. On the next read, the status will be turned to \ref LD_COMPLETED by the
*  transport layer re-initialization.
*
*******************************************************************************/
l_u8 ld_tx_status(l_ifc_handle iii, mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    l_u8 status = context->tl_tx_status;

    if (context->tl_tx_status != LD_IN_PROGRESS)
    {
        context->tl_tx_status = LD_COMPLETED;
    }

    return (status);
}


/*******************************************************************************
* Function Name: ld_rx_status
****************************************************************************//**
*
*  The call returns the status of the last made call to ld_receive_message.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  See: \ref LD_IN_PROGRESS \ref LD_COMPLETED \ref LD_FAILED
*  \ref LD_N_CR_TIMEOUT \ref LD_WRONG_SN
*
*  \note Failed status (e.g. \ref LD_WRONG_SN) can be read only once by this
*  function. On the next read, the status will be turned to \ref LD_COMPLETED
*  by the transport layer re-initialization.
*
*******************************************************************************/
l_u8 ld_rx_status(l_ifc_handle iii, mtb_stc_lin_context_t* context)
{
    l_u8 status = context->tl_rx_status;
    CY_UNUSED_PARAMETER(iii);

    if (context->tl_rx_status != LD_IN_PROGRESS)
    {
        context->tl_rx_status = LD_COMPLETED;
    }

    return (status);
}


/*******************************************************************************
* Function Name: ld_put_raw
****************************************************************************//**
*
* This function is used to allow the application code to send data using the
* Transport Layer.
*
* It essentially copies 8 bytes of data from a user application array to a frame
* buffer array. The data is sent in the next suitable frame (Slave response
* frame).
*
* The data area will be copied in the call, the pointer will not be memorized.
*
* This function is used to send one frame of a complete message at a time.
* Therefore, a multiframe message requires multiple calls to this function.
*
* Always check \ref ld_raw_tx_status() to see if there is a place for
* the frame in the buffer before calling this function. If no more queue
* resources are available, the data may be jettisoned and the appropriate error
* status will be set.
*
* \note In J2602 compliance mode, the J2602 Status is sent in the first byte of each
* frame when:
*  - using 0x3C messages with NADs in the User reserved range of 0x80 - 0xFF
*  - using 0x3E messages with any NAD, so only 7 bytes of the data can be sent.
*    There is no need to place NAD as first byte of user data.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param ld_data
*  The array of data to be sent.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void ld_put_raw(l_ifc_handle iii, const l_u8* const ld_data, mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_RAW)
    {
        l_u32 interrupt_state;

        interrupt_state = Cy_SysLib_EnterCriticalSection();

        /* Copies data only when the buffer is not full */
        if (context->tl_raw_tx_buf_depth < (context->config->tl_tx_queue_len / 8U))
        {
            /* Copies the 8 bytes of data to Raw TX queue buffer */
            for (l_u8 i = 0U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
            {
                /* Copies one byte of data to SRF buffer */
                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 18.4', 1,
                                        'Advisory. Usage of arithmetic on pointer. This approach gives flexibility in data access. Checked manually, no possible issues expected.');
                context->config->tl_raw_tx_queue[context->tl_raw_tx_write_index] = *(ld_data + i);
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.4');
                context->tl_raw_tx_write_index++;
            }

            /* If the buffer end is reached, resets the write index */
            if (context->tl_raw_tx_write_index == context->config->tl_tx_queue_len)
            {
                context->tl_raw_tx_write_index = 0U;
            }

            /* The 8 bytes of data are copied, so increment the buffer depth */
            context->tl_raw_tx_buf_depth++;

            /* Updates the status properly */
            if (context->tl_raw_tx_buf_depth == (context->config->tl_tx_queue_len / 8U))
            {
                context->tl_tx_status = LD_QUEUE_FULL;
            }
            else
            {
                context->tl_tx_status = LD_QUEUE_AVAILABLE;
            }

            context->tl_flags |= MTB_LIN_TL_TX_REQUESTED;
        }

        Cy_SysLib_ExitCriticalSection(interrupt_state);
    }
}


/*******************************************************************************
* Function Name: ld_get_raw
****************************************************************************//**
*
* This function is used to allow the application code to receive data using the
* Transport Layer. It essentially copies some data from a frame buffer array to
* a user application array.
*
* This function is used to receive one frame of a complete Transport Layer
* message at a time. Therefore, a multiframe Transport Layer message requires
* multiple calls to this API function.
*
* If the receive queue is empty, no data will be copied. Always check
* \ref ld_raw_rx_status() to ensure there is data in the queue before calling
* this function.
*
* \param iii
*  The interface handle. See \ref l_ifc_handle.
*
* \param ld_data
*  The array to which the oldest received diagnostic frame data is copied.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void ld_get_raw(l_ifc_handle iii, l_u8* const ld_data, mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_RAW)
    {
        l_u32 interrupt_state;

        interrupt_state = Cy_SysLib_EnterCriticalSection();

        /* If the queue is empty, does not copy anything. */
        if (context->tl_raw_rx_buf_depth != 0U)
        {
            /* Copies the 8 bytes of data from Raw RX queue buffer. */
            for (l_u8 i = 0U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
            {
                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 18.4', 1,
                                        'Advisory. Usage of arithmetic on pointer. This approach gives flexibility in data access. Checked manually, no possible issues expected.');
                *(ld_data + i) = context->config->tl_raw_rx_queue[context->tl_raw_rx_read_index];
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.4');
                context->tl_raw_rx_read_index++;
            }

            /* The 8 bytes of data are copied. Decrement the buffer depth */
            context->tl_raw_rx_buf_depth--;

            /* If the buffer end is reached, go to the start */
            if (context->tl_raw_rx_read_index == context->config->tl_rx_queue_len)
            {
                context->tl_raw_rx_read_index = 0U;
            }

            /* Update the status properly */
            if (context->tl_raw_rx_buf_depth == 0U)
            {
                context->tl_rx_status = LD_NO_DATA;
            }
            else
            {
                context->tl_rx_status = LD_DATA_AVAILABLE;
            }
        }

        Cy_SysLib_ExitCriticalSection(interrupt_state);
    }
}


/*******************************************************************************
* Function Name: ld_raw_tx_status
****************************************************************************//**
*
* This call returns the status of the last performed frame transmission on the
* bus when a raw API was used.
*
* \param iii
*  The interface handle. See \ref l_ifc_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* \ref group_lin_tl_return_status_macros
*
*******************************************************************************/
l_u8 ld_raw_tx_status(l_ifc_handle iii, const mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    return (context->tl_tx_status);
}


/*******************************************************************************
* Function Name: ld_raw_rx_status
****************************************************************************//**
*
*  The call returns the status of the raw frame receive function.
*
* \param iii
*  The name of the interface handle. The parameter is not used within the
*  middleware as the interface is defined by the context parameter.
*  See \ref l_ifc_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* \ref group_lin_tl_return_status_macros
*
*******************************************************************************/
l_u8 ld_raw_rx_status(l_ifc_handle iii, const mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    return (context->tl_rx_status);
}


/* [] END OF FILE */
