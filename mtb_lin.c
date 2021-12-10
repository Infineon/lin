/***************************************************************************//**
 * \file mtb_lin.c
 * \version 1.10
 *
 * \brief
 * Provides the LIN middleware common API implementation.
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

#include <string.h>  /* For memset() */

#include "cy_systick.h"
#include "mtb_lin.h"


#define LIN_SIGNAL_LENGTH_MAX_BYTES        (8U)
#define LIN_SIGNAL_BITS_IN_BYTE            (8U)
#define LIN_ONE_BIT_SIGNAL_LENGTH          (1U)

#define LIN_BAUDRATE_MIN                   (1000U)
#define LIN_BAUDRATE_MAX                   (20000U)

#define LIN_AUTO_BAUD_RATE_TOLERANCE       (140U)
#define LIN_BAUD_RATE_TOLERANCE            (15U)

#define LIN_BAUD_RATE_MULT_RATE            (10U)

/** The basic sanity check for LIN configuration */
static mtb_lin_status_t mtb_lin_config_check(const mtb_stc_lin_config_t* config);
static mtb_lin_status_t mtb_lin_config_check_signal_handle(const mtb_stc_lin_config_t* config);
static mtb_lin_status_t mtb_lin_config_check_signal_frame_id(const mtb_stc_lin_config_t* config);
static mtb_lin_status_t mtb_lin_config_check_signal_bit_offset(const mtb_stc_lin_config_t* config);
static mtb_lin_status_t mtb_lin_config_check_signal_size(const mtb_stc_lin_config_t* config);
static mtb_lin_status_t mtb_lin_config_check_signal_placing(const mtb_stc_lin_config_t* config);
static mtb_lin_status_t mtb_lin_config_check_common(const mtb_stc_lin_config_t* config);
static l_bool mtb_lin_signal_trace_error(volatile const mtb_stc_lin_signal_t* p_signal,
                                         l_bool* p_scratchpad, l_u8 frame_size_bits);
static mtb_lin_status_t mtb_lin_calculate_init_divider(mtb_stc_lin_context_t* context);


/*******************************************************************************
* Function Name: l_sys_init
****************************************************************************//**
*
*  Initializes the LIN core. Initializes context with the:
*  - the pointer to the configuration structure
*  - the pointer to the communication and inactivity interrupt service routine (ISR)
*  - the priority of the communication interrupt
*  - the calculated peripheral clock divider
*
* \param config
*  The pointer to the LIN Slave configuration structure.
*  See \ref mtb_stc_lin_config_t.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \param comm_isr
*  The pointer to the communication ISR.
*
* \param comm_isr_priority
* The interrupt priority number for the communication interrupt handler.
*
* \param inactivity_isr
*  The pointer to the inactivity ISR.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
mtb_lin_status_t l_sys_init(const mtb_stc_lin_config_t* config, mtb_stc_lin_context_t* context,
                            cy_israddress comm_isr, l_u32 comm_isr_priority,
                            cy_israddress inactivity_isr)
{
    mtb_lin_status_t result = MTB_LIN_STATUS_BAD_PARAM;

    if ((NULL != config) && (NULL != context) && (NULL != comm_isr) && (NULL != inactivity_isr))
    {
        context->config            = config;
        context->comm_isr          = comm_isr;
        context->comm_isr_priority = comm_isr_priority;
        context->inactivity_isr    = inactivity_isr;
        result                     = mtb_lin_calculate_init_divider(context);
    }

    /** The sanity check of LIN configuration */
    if (MTB_LIN_STATUS_SUCCESS == result)
    {
        result = mtb_lin_config_check(config);
    }

    return result;
}


/*******************************************************************************
* Function Name: l_sys_irq_disable
****************************************************************************//**
*
*  Disables the LIN communication interrupts.
*
* \note The function shall only be called after \ref l_ifc_init().
*
* \note If the bus inactivity feature is enabled, this function disables the
* SysTick interrupt. This will affect all the applications that use SysTick.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  The state of the LIN communication interrupts.
*
*******************************************************************************/
l_irqmask l_sys_irq_disable(const mtb_stc_lin_context_t* context)
{
    l_irqmask result = 0U;

    if (1U == NVIC_GetEnableIRQ(context->comm_isr_config.intrSrc))
    {
        NVIC_DisableIRQ(context->comm_isr_config.intrSrc);
        result |= MTB_LIN_COMMUNICATION_IRQ_ENABLED;
    }

    if (context->config->inactivity_enabled)
    {
        if (0U != (SYSTICK_CTRL & SysTick_CTRL_TICKINT_Msk))
        {
            Cy_SysTick_DisableInterrupt();
            result |= MTB_LIN_INACTIVITY_IRQ_ENABLED;
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: l_sys_irq_restore
****************************************************************************//**
*
*  Restores the interrupt level identified by the provided parameter.
*
* \note The function shall only be called after \ref l_ifc_init().
*
* \param previous
*  The interrupt state reported by \ref l_sys_irq_disable().
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_sys_irq_restore(l_irqmask previous, const mtb_stc_lin_context_t* context)
{
    if (0U != (previous & MTB_LIN_COMMUNICATION_IRQ_ENABLED))
    {
        NVIC_EnableIRQ(context->comm_isr_config.intrSrc);
    }

    if (context->config->inactivity_enabled)
    {
        if (0U != (previous & MTB_LIN_INACTIVITY_IRQ_ENABLED))
        {
            Cy_SysTick_EnableInterrupt();
        }
    }
}


/*******************************************************************************
* Function Name: mtb_lin_config_check
****************************************************************************//**
*
* Executes a basic sanity check for the LIN configuration structure.
*
* \param config
* The pointer to the LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_config_check(const mtb_stc_lin_config_t* config)
{
    mtb_lin_status_t  result;

    if (NULL != config)
    {
        result = mtb_lin_config_check_signal_handle(config);
        if (MTB_LIN_STATUS_SUCCESS == result)
        {
            result = mtb_lin_config_check_signal_frame_id(config);
            if (MTB_LIN_STATUS_SUCCESS == result)
            {
                result = mtb_lin_config_check_signal_bit_offset(config);
                if (MTB_LIN_STATUS_SUCCESS == result)
                {
                    result = mtb_lin_config_check_signal_size(config);
                    if (MTB_LIN_STATUS_SUCCESS == result)
                    {
                        result = mtb_lin_config_check_signal_placing(config);
                        if (MTB_LIN_STATUS_SUCCESS == result)
                        {
                            result = mtb_lin_config_check_nad(config->initial_nad, config);
                            if (MTB_LIN_STATUS_SUCCESS == result)
                            {
                                result = mtb_lin_config_check_common(config);
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        result = MTB_LIN_STATUS_BAD_PARAM;
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_lin_config_check_common
****************************************************************************//**
*
* Validates common configuration parameters
*
* \param config
* The pointer to a LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_config_check_common(const mtb_stc_lin_config_t* config)
{
    mtb_lin_status_t  result;

    if (config->iso17987 && (config->spec != MTB_LIN_SPEC_2_2))
    {
        result = MTB_LIN_STATUS_PROTOCOL_SPECIFICATION;
    }
    else if ((config->baud_rate < LIN_BAUDRATE_MIN) || (config->baud_rate > LIN_BAUDRATE_MAX))
    {
        result = MTB_LIN_STATUS_BAD_BAUDRATE;
    }
    else
    {
        result = MTB_LIN_STATUS_SUCCESS;
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_lin_config_check_signal_handle
****************************************************************************//**
*
* Validates the size of signals with the same handle.
*
* \param config
* The pointer to the LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_config_check_signal_handle(const mtb_stc_lin_config_t* config)
{
    l_bool err_flag = false;
    l_u8 signal_handle;
    l_u8 signal_size_bits;
    l_u8 i, j;

    for (i = 0; i < config->num_of_signals; i++)
    {
        /* Exit from the external loop in case of error */
        if (err_flag == true)
        {
            break;
        }

        signal_handle = config->signals[i].handle;
        signal_size_bits = config->signals[i].size;

        /* Find other signals with the same handle */
        for (j = i + 1U; j < config->num_of_signals; j++)
        {
            if (signal_handle == config->signals[j].handle)
            {
                /* Check the signal size, it must be equal for signals with the same handle */
                if (signal_size_bits != config->signals[j].size)
                {
                    /* Exit from internal loop in case of error */
                    err_flag = true;
                    break;
                }
            }
        }
    }

    return (i == config->num_of_signals) ? MTB_LIN_STATUS_SUCCESS : MTB_LIN_STATUS_SIGNAL_HANDLE;
}


/*******************************************************************************
* Function Name: mtb_lin_config_check_signal_frame_id
****************************************************************************//**
*
* Validates the value of the frame_index field in the signals array with
* the value of the overall number of frames.
*
* \param config
* The pointer to the LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_config_check_signal_frame_id(const mtb_stc_lin_config_t* config)
{
    l_u8 i;

    for (i = 0; i < config->num_of_signals; i++)
    {
        if (config->signals[i].frame_index >= config->num_of_frames)
        {
            break;
        }
    }

    return (i == config->num_of_signals) ? MTB_LIN_STATUS_SUCCESS : MTB_LIN_STATUS_SIGNAL_FRAME_ID;
}


/*******************************************************************************
* Function Name: mtb_lin_config_check_signal_bit_offset
****************************************************************************//**
*
* Validates the value of the bit_offset field in the signals array.
*
* \param config
* The pointer to the LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_config_check_signal_bit_offset(const mtb_stc_lin_config_t* config)
{
    l_u8 i;

    for (i = 0; i < config->num_of_signals; i++)
    {
        /* The signal bits offset must be less than 64 */
        if (config->signals[i].bit_offset >
            ((LIN_SIGNAL_LENGTH_MAX_BYTES * LIN_SIGNAL_BITS_IN_BYTE) - 1U))
        {
            break;
        }
    }

    return (i ==
            config->num_of_signals) ? MTB_LIN_STATUS_SUCCESS : MTB_LIN_STATUS_SIGNAL_BIT_OFFSET;
}


/*******************************************************************************
* Function Name: mtb_lin_config_check_signal_size
****************************************************************************//**
*
* Validates that the value of the size field in the signals array is correct
* for the corresponding signal type.
*
* \param config
* The pointer to the LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_config_check_signal_size(const mtb_stc_lin_config_t* config)
{
    mtb_stc_lin_signal_type_t signal_type;
    l_u8 signal_size_bits;
    l_u8 i;

    for (i = 0; i < config->num_of_signals; i++)
    {
        signal_type = config->signals[i].type;
        signal_size_bits = config->signals[i].size;

        if (
            /* The size of BOOL signal must be 1 bit */
            ((signal_type == MTB_LIN_SIGNAL_TYPE_SCALAR_BOOL) &&
             (signal_size_bits > LIN_ONE_BIT_SIGNAL_LENGTH)) ||
            /* The size of U8 signal must be 2 - 8 bits */
            ((signal_type == MTB_LIN_SIGNAL_TYPE_SCALAR_U8) &&
             ((signal_size_bits < (LIN_ONE_BIT_SIGNAL_LENGTH + 1U)) ||
              (signal_size_bits > LIN_SIGNAL_BITS_IN_BYTE))) ||
            /* The size of U16 signal must be 9 - 16 bits */
            ((signal_type == MTB_LIN_SIGNAL_TYPE_SCALAR_U16) &&
             ((signal_size_bits < (LIN_SIGNAL_BITS_IN_BYTE + 1U)) ||
              (signal_size_bits > (LIN_SIGNAL_BITS_IN_BYTE * 2U)))) ||
            /* The size of ARRAY signal must be 1 - 8 bytes */
            ((signal_type == MTB_LIN_SIGNAL_TYPE_BYTE_ARRAY) &&
             ((signal_size_bits < LIN_SIGNAL_BITS_IN_BYTE) ||
              (signal_size_bits > (LIN_SIGNAL_BITS_IN_BYTE * LIN_SIGNAL_LENGTH_MAX_BYTES))))
            )
        {
            break;
        }
    }

    return (i == config->num_of_signals) ? MTB_LIN_STATUS_SUCCESS : MTB_LIN_STATUS_SIGNAL_SIZE;
}


/*******************************************************************************
* Function Name: mtb_lin_config_check_signal_placing
****************************************************************************//**
*
* Validates the possibility of storing all mapped signals within a frame.
*
* \param config
* The pointer to the LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_config_check_signal_placing(const mtb_stc_lin_config_t* config)
{
    l_bool scratchpad[LIN_SIGNAL_LENGTH_MAX_BYTES * LIN_SIGNAL_BITS_IN_BYTE];

    l_bool err_flag = false;
    l_u8 signals_size_bits;
    l_u8 frame_size_bits;
    l_u8 frame_idx;
    l_u8 i, j;

    for (i = 0; i < config->num_of_signals; i++)
    {
        /* Get the size of the frame, bits */
        frame_idx = config->signals[i].frame_index;
        frame_size_bits = config->frames[frame_idx].size * LIN_SIGNAL_BITS_IN_BYTE;

        /* A trace current signal in an empty scratchpad */
        (void)memset(scratchpad, 0x0, sizeof(scratchpad));
        if (mtb_lin_signal_trace_error(&config->signals[i], scratchpad,
                                       frame_size_bits) || err_flag)
        {
            break;
        }

        /* The collective size of the signals within a frame, bits */
        signals_size_bits = config->signals[i].size;

        /* Find other signals within the same frame */
        for (j = i + 1U; j < config->num_of_signals; j++)
        {
            if (frame_idx == config->signals[j].frame_index)
            {
                /* Add the size of another signal from the same frame to the collective size of the
                   signals */
                signals_size_bits += config->signals[j].size;

                /* Check the possibility of tracing this signal without overlapping with the other
                   signals in
                   the frame */
                if (mtb_lin_signal_trace_error(&config->signals[j], scratchpad,
                                               frame_size_bits) ||
                    (frame_size_bits < signals_size_bits))
                {
                    /* Exit from the internal loop in case of error */
                    err_flag = true;
                    break;
                }
            }
        }
    }

    return (!err_flag &&
            (i == config->num_of_signals)) ? MTB_LIN_STATUS_SUCCESS : MTB_LIN_STATUS_SIGNAL_PLACING;
}


/*******************************************************************************
* Function Name: mtb_lin_signal_trace_error
****************************************************************************//**
*
* Detects error conditions for placing mapped signals within a frame
*
* \param p_signal
* The pointer to a particular signal in the signals array.
*
* \param p_scratchpad
* The pointer to the array, which represents the maximum allowable space
* for placing signals within a frame.
*
* \param frame_size_bits
* The real size of the frame in bits
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static l_bool mtb_lin_signal_trace_error(volatile const mtb_stc_lin_signal_t* p_signal,
                                         l_bool* p_scratchpad, l_u8 frame_size_bits)
{
    l_bool err_flag = false;

    l_u8 signal_offset_bits = p_signal->bit_offset;
    l_u8 signal_size_bits   = p_signal->size;
    l_u8 i;

    /* The starting point for a tracing signal */
    i = signal_offset_bits;

    /* The trace signal in the scratchpad */
    for (; i < signal_size_bits; i++)
    {
        /* Check if the signal is within the frame and the space is NOT occupied with another signal
         */
        if ((frame_size_bits < i) || p_scratchpad[i])
        {
            err_flag = true;
            break;
        }
        else
        {
            /* The trace current signal */
            p_scratchpad[i] = true;
        }
    }

    return (err_flag ? true : false);
}


/*******************************************************************************
* Function Name: mtb_lin_calculate_init_divider
****************************************************************************//**
*
*  Calculates the peripheral clock divider. The peripheral clock divider will be
*  calculated correctly only when the function returns
*  \ref MTB_LIN_STATUS_SUCCESS.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
static mtb_lin_status_t mtb_lin_calculate_init_divider(mtb_stc_lin_context_t* context)
{
    mtb_lin_status_t status = MTB_LIN_STATUS_SUCCESS;

    l_u32 desired_freq = context->config->baud_rate * MTB_LIN_OVERSAMPLE_RATE;
    l_u32 hfclk_freq = Cy_SysClk_ClkHfGetFrequency();

    if (desired_freq <= hfclk_freq)
    {
        /* Calculate clock divider value */
        context->initial_clock_divider = CY_SYSLIB_DIV_ROUND(hfclk_freq, desired_freq);

        /* Calculate real baudrate. Multiply high-frequency frequency by 10 to increase accuracy.
         * Baudrate = frequency of high-frequency clock / clock divider / oversample.
         */
        l_u32 real_baudrate = CY_SYSLIB_DIV_ROUND((hfclk_freq * LIN_BAUD_RATE_MULT_RATE),
                                                  context->initial_clock_divider);
        real_baudrate = CY_SYSLIB_DIV_ROUND(real_baudrate, MTB_LIN_OVERSAMPLE_RATE);

        /* Multiply initial baudrate by 10 to have the same dimension of both baudrate */
        l_u32 mult_initial_baud_rate = context->config->baud_rate * LIN_BAUD_RATE_MULT_RATE;

        /* Find difference between initial and real baudrate */
        l_u32 baudrate_diff = (real_baudrate > mult_initial_baud_rate) ?
                              (real_baudrate - mult_initial_baud_rate) :
                              (mult_initial_baud_rate - real_baudrate);

        /* Calculate tolerance in percent */
        l_u32 tolerance = CY_SYSLIB_DIV_ROUND((baudrate_diff * 100U), context->config->baud_rate);

        /* Check tolerance */
        if (tolerance > (context->config->auto_baud_rate_sync ? LIN_AUTO_BAUD_RATE_TOLERANCE :
                         LIN_BAUD_RATE_TOLERANCE))
        {
            status = MTB_LIN_STATUS_BAD_TOLERANCE;
        }
    }
    else
    {
        status = MTB_LIN_STATUS_TOO_LOW_FREQUENCY;
    }

    return status;
}


/* [] END OF FILE */
