/***************************************************************************//**
 * \file mtb_lin_ifc.c
 * \version 1.10
 *
 * \brief
 * Provides the LIN middleware signal interaction and notification API
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

#include "cyhal_scb_common.h"

#include "mtb_lin_ld.h"
#include "mtb_lin_ifc.h"
#include "mtb_lin_sig.h"

#define MTB_LIN_SCB_INTR_RX_ALL                    \
    (CY_SCB_RX_INTR_UART_BREAK_DETECT | CY_SCB_RX_INTR_BAUD_DETECT |                                             \
                                                 CY_SCB_RX_INTR_NOT_EMPTY)


/*******************************************************************************
*            Internal API
*******************************************************************************/
static void mtb_lin_update_frame_index(mtb_stc_lin_context_t* context);
static void mtb_lin_uart_write_tx_data(l_u32 txData, const mtb_stc_lin_context_t* context);
static void mtb_lin_update_flags(const mtb_stc_lin_context_t* context);
static void mtb_lin_finalize_frame(l_u8 status, mtb_stc_lin_context_t* context);
static void mtb_lin_update_node_state(mtb_stc_lin_node_state_stimulus_t stimulus,
                                      mtb_stc_lin_context_t* context);
static void mtb_lin_set_clock_divider(l_u32 div_val, const mtb_stc_lin_context_t* context);
__STATIC_INLINE l_bool mtb_lin_is_clock_valid(l_u16 sync_counts,
                                              const mtb_stc_lin_context_t* context);

/* Rx sub-routines */
__STATIC_INLINE l_bool l_ifc_rx_break_field(mtb_stc_lin_context_t* context);
__STATIC_INLINE l_bool l_ifc_rx_bad_sync(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_0_idle(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_mrf(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_srf(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_srf_raw(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_srf_cooked(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_err(l_u8 status, mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_publish(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_subscribe(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_2_tx(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_3_rx(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_4_chs(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_4_chs_tl(mtb_stc_lin_context_t* context);
__STATIC_INLINE void l_ifc_rx_fsm_4_chs_err(mtb_stc_lin_context_t* context);
__STATIC_INLINE l_bool mtb_is_current_frame_event_triggered(const mtb_stc_lin_context_t* context);


/*******************************************************************************
* Function Name: mtb_lin_update_frame_index
****************************************************************************//**
*
* Updates context->current_frame_index based on context->current_frame_pid.
* Sets MTB_LIN_INVALID_FRAME_PID if it is not found.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_update_frame_index(mtb_stc_lin_context_t* context)
{
    context->current_frame_index = MTB_LIN_INVALID_FRAME_PID;

    for (l_u8 i = 0U; i < context->config->num_of_frames; i++)
    {
        if (context->current_frame_pid == context->config->frames_context[i].configured_pid)
        {
            context->current_frame_index = i;
            break;
        }
    }
}


/*******************************************************************************
* Function Name: mtb_is_current_frame_event_triggered
****************************************************************************//**
*
* Check if the context->current_frame_pid frame is event-triggered.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  "True" if the frame is event-triggered.
*
*******************************************************************************/
__STATIC_INLINE l_bool mtb_is_current_frame_event_triggered(const mtb_stc_lin_context_t* context)
{
    l_bool result = false;

    if (MTB_LIN_INVALID_FRAME_PID != context->current_frame_index)
    {
        if (MTB_LIN_FRAME_TYPE_EVENT_TRIGGERED ==
            context->config->frames[context->current_frame_index].type)
        {
            result = true;
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_lin_finalize_frame
****************************************************************************//**
*
*  Finishes the frame transmission.
*
* \param status
*  Takes MTB_LIN_HANDLING_RESET_FSM_ERR, MTB_LIN_HANDLING_RESET_BREAK, or
*  MTB_LIN_HANDLING_DONT_SAVE_PID.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_finalize_frame(l_u8 status, mtb_stc_lin_context_t* context)
{
    l_u32 interrupt_state;

    /* Clears the data received flag */
    context->uart_fsm_flags &= ((l_u8) ~MTB_LIN_FSM_DATA_RECEIVE);

    if (context->config->tl_enabled)
    {
        /* Clears the TL TX direction flag */
        context->tl_flags &= ((l_u8) ~MTB_LIN_TL_TX_DIRECTION);
    }

    if (status == MTB_LIN_HANDLING_DONT_SAVE_PID)
    {
        /* Check uart overrun flag */
        if (0U != (context->uart_fsm_flags & MTB_LIN_FSM_OVERRUN))
        {
            /* Sets an overrun */
            context->ifc_comm_status |= MTB_LIN_IFC_STS_OVERRUN;
        }
    }

    if ((status == MTB_LIN_HANDLING_RESET_FSM_ERR) ||
        (status == MTB_LIN_HANDLING_DONT_SAVE_PID) ||
        (status == MTB_LIN_HANDLING_RESET_BREAK))
    {
        /* Clears the UART enable flag */
        context->uart_fsm_flags &=
            ((l_u8) ~(MTB_LIN_FSM_UART_ENABLE_FLAG | MTB_LIN_FSM_FRAMING_ERROR_FLAG));

        interrupt_state = Cy_SysLib_EnterCriticalSection();

        /* Clears RX and TX FIFOs after a Frame or Overrun error */
        Cy_SCB_ClearTxFifo(context->scb_uart_obj.base);
        Cy_SCB_ClearRxFifo(context->scb_uart_obj.base);

        Cy_SCB_SetRxInterrupt(context->scb_uart_obj.base, CY_SCB_RX_INTR_NOT_EMPTY);

        /* Sets IDLE FSM State */
        context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_0_IDLE;

        Cy_SysLib_ExitCriticalSection(interrupt_state);
    }

    /* Auto Baud Rate Sync Enabled and break was detected correctly. */
    if ((context->config->auto_baud_rate_sync) && (status != MTB_LIN_HANDLING_RESET_BREAK))
    {
        /* Restores the initial clock divider */
        mtb_lin_set_clock_divider((context->initial_clock_divider - 1U), context);
    }
}


/*******************************************************************************
* Function Name: l_ifc_init
****************************************************************************//**
*
*  The function initializes the LIN Slave middleware instance specified
*  by the context.
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
* \param tx
*  The TX pin name.
*
* \param rx
*  The RX pin name.
*
* \return
*  "false" if initialization succeeds, and "true" if it fails.
*
*******************************************************************************/
l_bool l_ifc_init(l_ifc_handle iii, mtb_stc_lin_context_t* context, cyhal_gpio_t tx,
                  cyhal_gpio_t rx)
{
    CY_UNUSED_PARAMETER(iii);
    l_u32  interrupt_state;
    l_bool result = false;
    cy_rslt_t rslt;

    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits      = 8U,
        .stop_bits      = 1U,
        .parity         = CYHAL_UART_PARITY_NONE,
        .rx_buffer      = NULL,
        .rx_buffer_size = 0U,
    };


    interrupt_state = Cy_SysLib_EnterCriticalSection();

    /* Allocate the peripheral clock divider for SCB */
    rslt = cyhal_clock_allocate(&context->scb_uart_clk, CYHAL_CLOCK_BLOCK_PERIPHERAL_16BIT);
    if (CY_RSLT_SUCCESS != rslt)
    {
        result = true;
    }
    else
    {
        #if CYHAL_API_VERSION >= 2
        rslt =
            cyhal_uart_init(&context->scb_uart_obj, tx, rx, NC, NC, &context->scb_uart_clk,
                            &uart_config);
        #else
        rslt =
            cyhal_uart_init(&context->scb_uart_obj, tx, rx, &context->scb_uart_clk, &uart_config);
        #endif
        if (CY_RSLT_SUCCESS != rslt)
        {
            /* Release the peripheral divider */
            cyhal_clock_free(&context->scb_uart_clk);

            result = true;
        }
        else
        {
            NVIC_DisableIRQ(_CYHAL_SCB_IRQ_N[context->scb_uart_obj.resource.block_num]);

            /* Disable SCB block before reconfiguring it by Cy_SCB_UART_Init */
            Cy_SCB_UART_Disable(context->scb_uart_obj.base, &context->scb_uart_obj.context);

            /* The run time copy of the clock divider */
            context->configured_clock_divider = context->initial_clock_divider;

            /* Sets the internal clock divider with effect on the end of a cycle.
             * It should be only after cyhal_uart_init().
             */
            mtb_lin_set_clock_divider((context->initial_clock_divider - 1U), context);

            /* Save the RX and TX pins in the context structure */
            context->tx_pin = tx;
            context->rx_pin = rx;

            cy_stc_sysint_t irqCfg =
            {
                .intrSrc      = _CYHAL_SCB_IRQ_N[context->scb_uart_obj.resource.block_num],
                .intrPriority = context->comm_isr_priority,
            };

            context->comm_isr_config = irqCfg;

            /* Change some default values of PDL UART config structure */
            context->scb_uart_obj.config.breakWidth    = context->config->break_threshold;
            context->scb_uart_obj.config.enableLinMode = true;
            context->scb_uart_obj.config.oversample    = MTB_LIN_OVERSAMPLE_RATE;

            #ifdef CY_IP_M0S8SCB
            /* Enable bits error detection (CY_SCB_RX_INTR_UART_FRAME_ERROR). */
            context->scb_uart_obj.config.stopBits = CY_SCB_UART_STOP_BITS_1_5;
            #endif /* CY_IP_M0S8SCB */

            /* No need to check the return value of the function, all passed parameters are not NULL
               pointer. */
            (void)Cy_SCB_UART_Init(context->scb_uart_obj.base, &context->scb_uart_obj.config,
                                   &context->scb_uart_obj.context);

            Cy_SCB_SetRxInterruptMask(context->scb_uart_obj.base, MTB_LIN_SCB_INTR_RX_ALL);

            /* Initialize runtime fields of the frames structure  */
            for (l_u8 i = 0U; i < context->config->num_of_frames; i++)
            {
                /* Copy PIDs from NVRAM to VRAM */
                context->config->frames_context[i].configured_pid =
                    context->config->frames[i].initial_pid;

                context->config->frames_context[i].flag = false;

                context->config->frames_context[i].et_flag = false;
            }

            if (context->config->tl_enabled)
            {
                context->configured_nad = context->config->initial_nad;
            }

            /* Initialize runtime fields of the signals structure  */
            for (l_u8 i = 0U; i < context->config->num_of_signals; i++)
            {
                context->config->signals_context[i].flag = false;
                context->config->signals_context[i].associated_frame = NULL;
            }


            /* Sets the flag for the event-triggered frame that is associated with the
             * unconditional frame where the specified signal is placed.
             */
            for (l_u8 fi = 0U; fi < context->config->num_of_frames; fi++)
            {
                if (context->config->frames[fi].type == MTB_LIN_FRAME_TYPE_EVENT_TRIGGERED)
                {
                    for (l_u8 si = 0U; si < context->config->num_of_signals; si++)
                    {
                        /* The unconditional frame index associated with the specified
                            event-triggered
                         * frame.
                         */
                        l_u8 associated_frame_index =
                            context->config->frames[fi].associated_frame_index;

                        /* The frame index with the specified signal */
                        l_u8 frame_index = context->config->signals[si].frame_index;

                        /* Initialize pointer to the context of the event-triggered frame that
                            has the associated
                         * unconditional frame contains the specified signal.
                         */
                        if (associated_frame_index == frame_index)
                        {
                            context->config->signals_context[si].associated_frame =
                                &context->config->frames_context[fi];
                        }
                    }
                }
            }


            /* Initialize the pointer to the serial number with NULL. Later it
             *  can be changed with the MTB_LIN_IOCTL_SET_SERIAL_NUMBER operation of
             * the \ref l_ifc_ioctl().
             */
            context->serial_number = NULL;

            /* Clears the interface status */
            context->ifc_comm_status &= (l_u16) ~MTB_LIN_IFC_STS_MASK;

            /* Sets IDLE FSM State as default state */
            context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_0_IDLE;

            if (context->config->inactivity_enabled)
            {
                /* Clears the period timer counter */
                context->period_counter = 0U;
            }

            /* Clears the ioctl status register */
            context->ifc_ioctl_status = 0U;

            Cy_SCB_UART_Enable(context->scb_uart_obj.base);

            NVIC_ClearPendingIRQ(context->comm_isr_config.intrSrc);

            /* No need to check the return value of the function, all passed parameters are not NULL
               pointer. */
            (void)Cy_SysInt_Init(&context->comm_isr_config, context->comm_isr);
            NVIC_EnableIRQ(context->comm_isr_config.intrSrc);

            /* Bus inactivity block configuration */
            if (context->config->inactivity_enabled)
            {
                /* Starts SysTick timer with the default period = 1mS */
                if (!mtb_lin_enable_timer(context))
                {
                    /* Release the UART peripheral and peripheral divider */
                    cyhal_uart_free(&context->scb_uart_obj);
                    cyhal_clock_free(&context->scb_uart_clk);

                    /* Reports an error because all SysTick callback slots are busy */
                    result = true;
                }
            }

            /* Sets the max time counts deviation */
            if (context->config->auto_baud_rate_sync)
            {
                context->time_counts_max_deviation = MTB_LIN_AUTO_BAUD_TIME_COUNTS_MAX_DEVIATION;
            }
            else
            {
                context->time_counts_max_deviation = MTB_LIN_TIME_COUNTS_MAX_DEVIATION;
            }
        }
    }

    Cy_SysLib_ExitCriticalSection(interrupt_state);

    return (result);
}


/*******************************************************************************
* Function Name: l_ifc_wake_up
****************************************************************************//**
*
*  This function transmits one wake-up signal. The wake-up signal is transmitted
*  directly when this function is called. When you call this API function, the
*  application is blocked until the wake-up signal is transmitted to the LIN bus.
*  The Cy_SysLib_Delay() function is used as the timing source. The delay is
*  calculated based on the clock configuration.
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
void l_ifc_wake_up(l_ifc_handle iii, const mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    /* For LIN 2.x: Force TXD low by transmission of N dominant bits
     *  (N depends on baud_rate)
     *  To calculate how much of bit-interval takes 300uS wake-up pulse:
     *  1) Calculate the bit period: 1000000(uS) divide by BAUD_RATE(bps)
     *  2) Divide WAKE_UP_SIGNAL_LENGTH by result of 1)
     *  3) Add 1 to compensate the rounding error
     *  4) Shift the 0xFF constant by the number of the bit obtained in 3)
     *  from the dominant level pulse with duration proportional to Baud rate
     */
    mtb_lin_uart_write_tx_data((l_u8)(0xFFU << ((MTB_LIN_WAKE_UP_SIGNAL_LENGTH /
                                                 (1000000U / context->config->baud_rate)) + 1U)),
                               context);

    /* Waits until symbol transmission ends, Tdelay > 11mS ,
     *  11 bit-times at min. baud_rate 1000bps,for  8N1.5 symbol */
    Cy_SysLib_Delay(MTB_LIN_WAKEUP_TX_DELAY /*ms*/);
}


/******************************************************************************
 * Function Name: l_ifc_ioctl
 ***************************************************************************//**
 *
 *  This function controls functionality not covered by the other API
 *  calls. It is used for protocol specific parameters or hardware specific
 *  functionality. An example of such functionality can be switching on/off the
 *  wake-up signal detection.
 *
 * \param iii
 *  The name of the interface handle. The parameter is not used within the
 *  middleware as the interface is defined by the context parameter.
 *  See \ref l_ifc_handle.
 *
 * \param op
 *  The operation to be applied \ref group_lin_l_ifc_ioctl_parameters_macros.
 *
 * \param pv
 *  The pointer to the optional parameter.
 *  See \ref group_lin_l_ifc_ioctl_parameters_macros.
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
mtb_lin_status_t l_ifc_ioctl(l_ifc_handle iii, l_ioctl_op op, void* pv,
                             mtb_stc_lin_context_t* context)
{
    mtb_lin_status_t result = MTB_LIN_STATUS_SUCCESS;
    CY_UNUSED_PARAMETER(iii);

    l_u32 interrupt_state;

    switch (op)
    {
        /***********************************************************************
        * Read Status
        ***********************************************************************/
        case MTB_LIN_IOCTL_READ_STATUS:
            CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                    'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
            *((l_u16*)pv) = context->ifc_ioctl_status;
            CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');
            break;

        /***********************************************************************
        * Set Baud Rate
        ***********************************************************************/
        case MTB_LIN_IOCTL_SET_BAUD_RATE:
        {
            l_u32 clock_divider;

            interrupt_state = Cy_SysLib_EnterCriticalSection();

            CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                    'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
            clock_divider = *(((l_u32*)pv));
            CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');

            if (context->config->auto_baud_rate_sync)
            {
                context->configured_clock_divider = clock_divider;
            }

            /* Set new internal clock divider with effect on the end of a cycle. */
            mtb_lin_set_clock_divider((clock_divider - 1U), context);

            Cy_SysLib_ExitCriticalSection(interrupt_state);
            break;
        }

        /***********************************************************************
        * Prepare for low-power modes
        ***********************************************************************/
        case MTB_LIN_IOCTL_SLEEP:
            /* Stop the timer */
            mtb_lin_disable_timer(context);

            /* Stop UART */
            cyhal_uart_free(&context->scb_uart_obj);

            /* Release the peripheral divider */
            cyhal_clock_free(&context->scb_uart_clk);
            break;

        /***********************************************************************
        * Restore after wakeup from low-power modes
        ***********************************************************************/
        case MTB_LIN_IOCTL_WAKEUP:
            if (l_ifc_init(0U, context, context->tx_pin, context->rx_pin))
            {
                result = MTB_LIN_STATUS_IFC_IOCTL_WAKEUP;
            }
            break;

        case MTB_LIN_IOCTL_SYNC_COUNTS:
            if (context->config->auto_baud_rate_sync)
            {
                /* Returns the current number of the sync field timer counts */
                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                        'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
                *((l_u16*)pv) = (l_u16)context->sync_counts;
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');
            }
            else
            {
                result = MTB_LIN_STATUS_AUTO_BAUD_RATE_SYNC_DISABLED;
            }
            break;

        case MTB_LIN_IOCTL_SET_SERIAL_NUMBER:
            if (context->config->tl_enabled && context->config->auto_config_request_handling)
            {
                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                        'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
                context->serial_number = (l_u8*)pv;
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');
            }
            else
            {
                result = MTB_LIN_STATUS_IFC_IOCTL_UNSUPPORTED_COMMAND;
            }
            break;

        /***********************************************************************
        * Gets configured NAD. NAD must not be 00, 7E nor 7F.
        ***********************************************************************/
        case MTB_LIN_IOCTL_GET_NAD:
            if (context->config->tl_enabled && context->config->auto_config_request_handling)
            {
                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                        'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
                *((l_u8*)pv) = context->configured_nad;
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');
            }
            else
            {
                result = MTB_LIN_STATUS_IFC_IOCTL_UNSUPPORTED_COMMAND;
            }
            break;

        /***********************************************************************
        * Sets configured NAD. NAD must not be 00, 7E nor 7F.
        ***********************************************************************/
        case MTB_LIN_IOCTL_SET_NAD:
            if (context->config->tl_enabled && context->config->auto_config_request_handling)
            {
                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 3,
                                        'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
                /* Validates if preserved Node Address values are not used as NAD for slave node */
                if (mtb_lin_config_check_nad(*((l_u8*)pv),
                                             context->config) == MTB_LIN_STATUS_SUCCESS)
                {
                    CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');

                    CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                            'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
                    context->configured_nad = *((l_u8*)pv);
                    CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');
                }
                else
                {
                    result = MTB_LIN_STATUS_INVALID_NAD;
                }
            }
            else
            {
                result = MTB_LIN_STATUS_IFC_IOCTL_UNSUPPORTED_COMMAND;
            }
            break;

        /***********************************************************************
        * Gets frame PIDs by the frame table index.
        * Uses the pointer to the MTB_LIN_NEW_PID structure
        * as a parameter. Input MTB_LIN_NEW_PID.index.
        * Returns MTB_LIN_NEW_PID.PID
        ***********************************************************************/
        case MTB_LIN_IOCTL_GET_FRAME_PID:
            if (context->config->tl_enabled && context->config->auto_config_request_handling)
            {
                mtb_stc_lin_pid_t* new_pid;

                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                        'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
                new_pid = (mtb_stc_lin_pid_t*)pv;
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');
                if (new_pid->index < context->config->num_of_frames)
                {
                    new_pid->pid = context->config->frames_context[new_pid->index].configured_pid;
                }
                else
                {
                    result = MTB_LIN_STATUS_INVALID_PID_INDEX;
                    new_pid->pid = MTB_LIN_INVALID_FRAME_PID;
                }
            }
            else
            {
                result = MTB_LIN_STATUS_IFC_IOCTL_UNSUPPORTED_COMMAND;
            }
            break;

        /***********************************************************************
        * Sets frame PIDs by the frame table index.
        * Uses the MTB_LIN_NEW_PID structure
        * as an input parameter.
        ***********************************************************************/
        case MTB_LIN_IOCTL_SET_FRAME_PID:
            if (context->config->tl_enabled && context->config->auto_config_request_handling)
            {
                const mtb_stc_lin_pid_t* new_pid;

                CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 11.5', 1,
                                        'Advisory. The pointer to void is cast to pointer to object depends on context. This approach produce more efficient code. Checked manually, no possible issues expected.');
                new_pid = (mtb_stc_lin_pid_t*)pv;
                CY_MISRA_BLOCK_END('MISRA C-2012 Rule 11.5');
                if ((new_pid->index < context->config->num_of_frames) &&
                    ((new_pid->pid & MTB_LIN_PID_PARITY_MASK) < MTB_LIN_FRAME_PID_MRF))
                {
                    context->config->frames_context[new_pid->index].configured_pid = new_pid->pid;
                }
                else
                {
                    result = MTB_LIN_STATUS_INVALID_PID_INDEX;
                }
            }
            else
            {
                result = MTB_LIN_STATUS_IFC_IOCTL_UNSUPPORTED_COMMAND;
            }
            break;

        /***********************************************************************
        * Unknown operation
        ***********************************************************************/
        default:
            result = MTB_LIN_STATUS_IFC_IOCTL_UNSUPPORTED_COMMAND;
            break;
    }

    return (result);
}


/*******************************************************************************
* Function Name: l_ifc_aux
****************************************************************************//**
*
*  This function may be used to synchronize the break/sync field sequence
*  transmitted by Master node on the interface specified by iii. It may, for
*  example, be called from a user-defined interrupt handler raised upon a flank
*  detection on a hardware pin connected to the interface iii.
*
* \param iii
*  The name of the interface handle
*  See \ref l_ifc_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_ifc_aux(l_ifc_handle iii, mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);

    /**************************************************************************
    *                       Bus Inactivity Interrupt Detected
    **************************************************************************/
    if (context->config->inactivity_enabled)
    {
        if (context->config->tl_enabled)
        {
            /* If the timeout is enabled, proceeds the timeout manage */
            if (0U != (context->tl_flags & MTB_LIN_TL_N_AS_TIMEOUT_ON))
            {
                /* Increments the timeout */
                context->tl_timeout_counter++;

                if ((MTB_LIN_TL_N_AS_TIMEOUT_VALUE) <= context->tl_timeout_counter)
                {
                    mtb_lin_update_node_state(MTB_LIN_NODE_STIMULUS_TX_TIMEOUT, context);
                }
            }
            else if (0U != (context->tl_flags & MTB_LIN_TL_N_CR_TIMEOUT_ON))
            {
                /* Increments the timeout */
                context->tl_timeout_counter++;

                if ((MTB_LIN_TL_N_CR_TIMEOUT_VALUE) <= context->tl_timeout_counter)
                {
                    mtb_lin_update_node_state(MTB_LIN_NODE_STIMULUS_RX_TIMEOUT, context);
                }
            }
            else
            {
                /* Resets the timeout counter */
                context->tl_timeout_counter = 0U;
            }
        }

        if ((context->config->inactivity_threshold) <= context->period_counter)
        {
            /* The inactivity threshold is achieved */

            /* Sets the bus inactivity ioctl status bit */
            context->ifc_ioctl_status |= MTB_LIN_IOCTL_STS_BUS_INACTIVITY;
        }
        else
        {
            context->period_counter++;
        }
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx
****************************************************************************//**
*
*  The application program is responsible for binding the interrupt and for
*  setting the correct interface handle. It may be called from a user-defined
*  interrupt handler triggered by a UART when it receives one character of data.
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
void l_ifc_rx(l_ifc_handle iii, mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(iii);
    l_bool nothing_to_do_flag = false;

    /* Sets the bus activity interface status bit */
    context->ifc_comm_status |= MTB_LIN_IFC_STS_BUS_ACTIVITY;

    if (context->config->inactivity_enabled)
    {
        /* Clears the period timer counter */
        context->period_counter = 0x00U;

        /* Clears the bus inactivity ioctl status bit */
        context->ifc_ioctl_status &= (l_u16)(~(l_u16)MTB_LIN_IOCTL_STS_BUS_INACTIVITY);
    }

    /* Checks for the RX UART framing error and overflow */
    if (0U != (Cy_SCB_GetRxInterruptStatus(context->scb_uart_obj.base) &
               (CY_SCB_RX_INTR_UART_FRAME_ERROR | CY_SCB_RX_INTR_OVERFLOW)))
    {
        Cy_SCB_ClearRxInterrupt(context->scb_uart_obj.base,
                                (CY_SCB_RX_INTR_UART_FRAME_ERROR | CY_SCB_RX_INTR_OVERFLOW));

        /* Set a framing error */
        context->uart_fsm_flags |= MTB_LIN_FSM_FRAMING_ERROR_FLAG;
    }

    /***************************************************************************
    *                       Break Field Detected
    ***************************************************************************/
    if (0U != (Cy_SCB_GetRxInterruptStatus(context->scb_uart_obj.base) &
               (CY_SCB_RX_INTR_UART_BREAK_DETECT)))
    {
        nothing_to_do_flag = l_ifc_rx_break_field(context);
    }

    /***********************************************************************
    *                       Sync Field Complete                            *
    ***********************************************************************/
    if (!nothing_to_do_flag &&
        (0U != (Cy_SCB_GetRxInterruptStatus(context->scb_uart_obj.base) &
                (CY_SCB_RX_INTR_BAUD_DETECT))))
    {
        nothing_to_do_flag = l_ifc_rx_bad_sync(context);
    }

    /* Handle the RX/TX state machine */
    if (!nothing_to_do_flag &&
        (0U != (Cy_SCB_GetRxInterruptStatus(context->scb_uart_obj.base) &
                (CY_SCB_RX_INTR_NOT_EMPTY))))
    {
        switch (context->uart_fsm_state)
        {
            /***********************************************************************
            *                       IDLE state
            * State description:
            *  - Receives a sporadic byte not predicted by the BREAK/SYNC sequence,
            *    so does not set Response Error
            ***********************************************************************/
            case MTB_LIN_UART_ISR_STATE_0_IDLE:
                l_ifc_rx_fsm_0_idle(context);
                break;

            /***********************************************************************
            *                       PID Field Byte Receive
            * State description:
            *  - Receives protected identifier (PID)
            *  - Checks PID parity
            *  - Set flags
            *  - Determine next state (RX or TX)
            ***********************************************************************/
            case MTB_LIN_UART_ISR_STATE_1_PID:
                l_ifc_rx_fsm_1_pid(context);
                break;


            /***********************************************************************
            *                       TX response (Publish)
            * State description:
            *  - Transmits data to LIN Master.
            *  - Transmits the next data byte if no errors has occurred.
            *  - Transmits checksum when data is send correctly.
            ***********************************************************************/
            case MTB_LIN_UART_ISR_STATE_2_TX:
                l_ifc_rx_fsm_2_tx(context);
                break;

            /***********************************************************************
            *                       RX response (Subscribe)
            * State description:
            *  - Receives data from LIN Master.
            *  - Received data saved to the temporary buffer.
            ***********************************************************************/
            case MTB_LIN_UART_ISR_STATE_3_RX:
                l_ifc_rx_fsm_3_rx(context);
                break;

            /***********************************************************************
            *                              Checksum
            ***********************************************************************/
            case MTB_LIN_UART_ISR_STATE_4_CHS:
                l_ifc_rx_fsm_4_chs(context);
                break;

            default:
                /* Never use this state. */
                break;
        }
    }

    /* Clears the SCB UART interrupt */
    Cy_SCB_ClearRxInterrupt(context->scb_uart_obj.base, CY_SCB_RX_INTR_NOT_EMPTY);
}


/*******************************************************************************
* Function Name: l_ifc_tx
****************************************************************************//**
*
*  The application program is responsible for binding the interrupt and for
*  setting the correct interface handle. It may be called from a user-defined
*  interrupt handler triggered by a UART when it has transmitted one character
*  of data. In this case, the function will perform necessary operations on the
*  UART control registers
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
void l_ifc_tx(l_ifc_handle iii, mtb_stc_lin_context_t* context)
{
    l_ifc_rx(iii, context);
}


/*******************************************************************************
* Function Name: mtb_lin_update_flags
****************************************************************************//**
*
*  Updates flags for the frame, its signals and associated event-triggered frame.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_update_flags(const mtb_stc_lin_context_t* context)
{
    if (MTB_LIN_INVALID_FRAME_PID != context->current_frame_index)
    {
        /* Set the flag attached to the frame */
        context->config->frames_context[context->current_frame_index].flag = true;

        /* Set flags for all the signals placed onto the frame */
        for (l_u32 i = 0U; i < context->config->num_of_signals; i++)
        {
            if (context->config->signals[i].frame_index == context->current_frame_index)
            {
                context->config->signals_context[i].flag = true;
            }
        }


        /* Clears the flag for the event-triggered frame that is associated with the
         * current unconditional frame that is denoted by the context->current_frame_index.
         */
        if (context->config->frames[context->current_frame_index].type ==
            MTB_LIN_FRAME_TYPE_UNCONDITIONAL)
        {
            for (l_u8 i = 0U; i < context->config->num_of_frames; i++)
            {
                /* Satisfying of the MISRA 2012, Rule 13.5 */
                if (context->config->frames[i].type == MTB_LIN_FRAME_TYPE_EVENT_TRIGGERED)
                {
                    if (context->config->frames[i].associated_frame_index ==
                        context->current_frame_index)
                    {
                        context->config->frames_context[i].et_flag = false;
                        break;
                    }
                }
            }
        }
    }
}


/*******************************************************************************
* Function Name: mtb_lin_update_node_state
****************************************************************************//**
*
*  This function implements LIN Slave Node state machine as defined in LIN2.2.a,
*  item 5.5 SLAVE NODE TRANSMISSION HANDLER.
*
*  Refer to \ref mtb_stc_lin_node_state_t for the defined states.
*
* \param stimulus
*  The event passed from LIN Protocol Layer.
*  See mtb_stc_lin_node_state_stimulus_t
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_update_node_state(mtb_stc_lin_node_state_stimulus_t stimulus,
                                      mtb_stc_lin_context_t* context)
{
    CY_UNUSED_PARAMETER(stimulus);
    CY_UNUSED_PARAMETER(context);

    l_u8 result;

    switch (context->node_state)
    {
        case MTB_LIN_NODE_STATE_IDLE:
            switch (stimulus)
            {
                case MTB_LIN_NODE_STIMULUS_MRF_ALIEN_NAD:
                case MTB_LIN_NODE_STIMULUS_SRF:
                case MTB_LIN_NODE_STIMULUS_RX_TIMEOUT:
                case MTB_LIN_NODE_STIMULUS_TX_TIMEOUT:
                    context->tl_flags &=
                        (l_u8)(~(MTB_LIN_TL_N_CR_TIMEOUT_ON | MTB_LIN_TL_N_AS_TIMEOUT_ON));
                    break;

                case MTB_LIN_NODE_STIMULUS_MRF_FUNC_NAD:
                    (void)mtb_lin_process_mrf(context);
                    mtb_lin_clear_tx_buffer(stimulus, context);
                    break;

                case MTB_LIN_NODE_STIMULUS_MRF_OWN_NAD:
                    /* Receives an incoming TL frame (FF or SF) */
                    mtb_lin_clear_tx_buffer(stimulus, context);

                    result = mtb_lin_process_mrf(context);

                    if (MTB_LIN_RESPONSE_REQUIRED == result)
                    {
                        /* Changes the node state to TX state */
                        context->node_state = MTB_LIN_NODE_STATE_TX_PHY_RESPONSE;
                    }
                    else if (MTB_LIN_RECEIVE_CONTINUES == result)
                    {
                        /* Changes the node state to RX state */
                        context->node_state = MTB_LIN_NODE_STATE_RX_PHY_REQUEST;
                    }
                    else
                    {
                        /* Does nothing. */
                    }
                    break;

                default:
                    /* Does nothing. */
                    break;
            }
            break;

        case MTB_LIN_NODE_STATE_RX_PHY_REQUEST:
            switch (stimulus)
            {
                case MTB_LIN_NODE_STIMULUS_MRF_ALIEN_NAD:
                case MTB_LIN_NODE_STIMULUS_SRF:
                case MTB_LIN_NODE_STIMULUS_RX_TIMEOUT:
                case MTB_LIN_NODE_STIMULUS_TX_TIMEOUT:
                    mtb_lin_clear_tx_buffer(stimulus, context);
                    mtb_lin_clear_rx_buffer(stimulus, context);
                    context->node_state = MTB_LIN_NODE_STATE_IDLE;
                    break;

                case MTB_LIN_NODE_STIMULUS_MRF_FUNC_NAD:
                    /* Misses the functional request */
                    break;

                case MTB_LIN_NODE_STIMULUS_MRF_OWN_NAD:
                    /* Receives the incoming TL frame (CF) */
                    result = mtb_lin_process_mrf(context);

                    if (MTB_LIN_RESPONSE_REQUIRED == result)
                    {
                        /* Changes the node state to TX state */
                        context->node_state = MTB_LIN_NODE_STATE_TX_PHY_RESPONSE;
                    }
                    else if (MTB_LIN_RECEIVE_CONTINUES == result)
                    {
                        /* Changes the node state to RX state */
                        context->node_state = MTB_LIN_NODE_STATE_RX_PHY_REQUEST;
                    }
                    else    /* Does nothing. */
                    {
                        mtb_lin_clear_tx_buffer(stimulus, context);
                        mtb_lin_clear_rx_buffer(stimulus, context);
                        context->node_state = MTB_LIN_NODE_STATE_IDLE;
                    }
                    break;

                default:
                    /* Does nothing. */
                    break;
            }
            break;

        case MTB_LIN_NODE_STATE_TX_PHY_RESPONSE:
            switch (stimulus)
            {
                case MTB_LIN_NODE_STIMULUS_MRF_ALIEN_NAD:
                case MTB_LIN_NODE_STIMULUS_RX_TIMEOUT:
                case MTB_LIN_NODE_STIMULUS_TX_TIMEOUT:
                    mtb_lin_clear_tx_buffer(stimulus, context);
                    context->node_state = MTB_LIN_NODE_STATE_IDLE;
                    break;

                case MTB_LIN_NODE_STIMULUS_MRF_FUNC_NAD:
                    /* Does nothing, misses the functional request here. */
                    break;

                case MTB_LIN_NODE_STIMULUS_SRF:
                    /* Transmit RESPONSE message */
                    result = mtb_lin_transmit_tl_frame(stimulus, context);
                    if (MTB_LIN_TRANSMISSION_CONTINUES != result)
                    {
                        mtb_lin_clear_tx_buffer(stimulus, context);
                        if (0U == (context->status & MTB_LIN_STATUS_RESPONSE_PENDING))
                        {
                            /* Changes the node state to idle state. */
                            context->node_state = MTB_LIN_NODE_STATE_IDLE;
                        }
                    }
                    break;

                case MTB_LIN_NODE_STIMULUS_MRF_OWN_NAD:
                    /* MRF arrives during the RESPONSE transmission , drop
                       transmitted message and receive a new diagnostic message */

                    mtb_lin_clear_tx_buffer(stimulus, context);

                    result = mtb_lin_process_mrf(context);

                    if (MTB_LIN_RESPONSE_REQUIRED == result)
                    {
                        /* Changes the node state to TX state. */
                        context->node_state = MTB_LIN_NODE_STATE_TX_PHY_RESPONSE;
                    }
                    else if (MTB_LIN_RECEIVE_CONTINUES == result)
                    {
                        /* Changes the node state to RX state. */
                        context->node_state = MTB_LIN_NODE_STATE_RX_PHY_REQUEST;
                    }
                    else
                    {
                        mtb_lin_clear_rx_buffer(stimulus, context);
                        mtb_lin_clear_tx_buffer(stimulus, context);
                        context->node_state = MTB_LIN_NODE_STATE_IDLE;
                    }
                    break;

                default:
                    /* Does nothing. */
                    break;
            }
            break;

        default:
            /* Does nothing. */
            break;
    }
}


/*******************************************************************************
* Function Name: mtb_lin_set_clock_divider
****************************************************************************//**
*
*  Set the divider value.
*
* \param div_val Divider value to be set.

* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_set_clock_divider(l_u32 div_val, const mtb_stc_lin_context_t* context)
{
    if (Cy_SysClk_PeriphGetDivider(CY_SYSCLK_DIV_16_BIT,
                                   context->scb_uart_obj.clock.channel) != div_val)
    {
        /* No need to check return value of functions, all parameters which pass is in
         * valid range, checked by cyhal_clock_allocate().
         */
        (void)Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT,
                                             context->scb_uart_obj.clock.channel);
        (void)Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT,
                                         context->scb_uart_obj.clock.channel,
                                         div_val);
        (void)Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT,
                                            context->scb_uart_obj.clock.channel);
    }
}


/*******************************************************************************
* Function Name: mtb_lin_uart_write_tx_data
****************************************************************************//**
*
*  Places a data entry into the transmit buffer to be sent at the next available
*  bus time.
*  This function is blocking and waits until there is space available to put the
*  requested data in the transmit buffer.
*
* \param txData
*   The amount of data bits to be transmitted depends on TX data bits selection
*   (the data bit counting starts from LSB of txData).
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
static void mtb_lin_uart_write_tx_data(l_u32 txData, const mtb_stc_lin_context_t* context)
{
    /* Wait until TX FIFO has space to put a data element */
    while (8U == Cy_SCB_GetNumInTxFifo(context->scb_uart_obj.base))
    {
    }

    (void)Cy_SCB_UART_Put(context->scb_uart_obj.base, txData);
}


/*******************************************************************************
* Function Name: l_ifc_read_status
****************************************************************************//**
*
*  This function is defined by the LIN specification. This returns the status of
*  the specified LIN interface and clears all status bits for that
*  interface. See Section 7.2.5.8 of the LIN 2.1 specification.
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
*  The status bits of the specified LIN interface are returned. These bits have
*  the following meanings:
*
*  Bits         | Meaning
*  -------------|--------------------------
*    [15:8]     | Last Received PID
*    [7]        | 0
*    [6]        | Save Configuration flag
*    [5]        | 0
*    [4]        | Bus Activity flag
*    [3]        | Go To Sleep flag
*    [2]        | Overrun flag
*    [1]        | Successful Transfer flag
*    [0]        | Error in Response flag
*
*******************************************************************************/
l_u16 l_ifc_read_status(l_ifc_handle iii, mtb_stc_lin_context_t* context)
{
    l_u16 result;
    l_u32 interrupt_state;
    CY_UNUSED_PARAMETER(iii);

    interrupt_state = Cy_SysLib_EnterCriticalSection();

    /* Copy the global status variable to the local temp variable. */
    result = context->ifc_comm_status;

    /* Clears the status variable */
    context->ifc_comm_status &= (l_u16) ~MTB_LIN_IFC_STS_MASK;

    Cy_SysLib_ExitCriticalSection(interrupt_state);

    return (result);
}


/*******************************************************************************
* Function Name: mtb_lin_is_clock_valid
****************************************************************************//**
*
*  Checks the baud rate deviation from the target.
*
* \param sync_counts
*  The measured time counts.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  "true" if the current deviation is less than the maximum deviation, and false if
*  the current deviation is more than the maximum deviation.
*
*******************************************************************************/
__STATIC_INLINE l_bool mtb_lin_is_clock_valid(l_u16 sync_counts,
                                              const mtb_stc_lin_context_t* context)
{
    l_u16 max_deviation = context->time_counts_max_deviation;

    return (max_deviation > ((sync_counts > MTB_LIN_EXPECTED_TIME_COUNTS) ?
                             (sync_counts - MTB_LIN_EXPECTED_TIME_COUNTS) :
                             (MTB_LIN_EXPECTED_TIME_COUNTS - sync_counts)));
}


/*******************************************************************************
* Function Name: mtb_lin_enable_timer
****************************************************************************//**
*
*  Enables Systick timer interrupts and configures the SystemTick timer callback
*  slot to the middleware function that updates the timestamp value.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  True if timer successfully configured, and false in case of failure.
*
*******************************************************************************/
bool mtb_lin_enable_timer(const mtb_stc_lin_context_t* context)
{
    bool result = false;

    if (!Cy_SysTick_IsEnabled())
    {
        /* Configure SysTick timer to fire every 1 ms and set a callback */
        l_u32 freq = Cy_SysClk_ClkSysGetFrequency();
        Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, freq/1000U);
        (void)Cy_SysTick_SetCallback(0U, context->inactivity_isr);
        result = true;
    }
    else
    {
        /* If SysTick timer is configured, checks if its own callback exists */
        for (l_u32 i = 0U; i < CY_SYS_SYST_NUM_OF_CALLBACKS; i++)
        {
            if (Cy_SysTick_GetCallback(i) == context->inactivity_isr)
            {
                result = true;
                break;
            }
        }

        /* Looks for an unused callback slot to fill it with its own callback */
        if (!result)
        {
            for (l_u32 i = 0U; i < CY_SYS_SYST_NUM_OF_CALLBACKS; i++)
            {
                /* Makes sure that the callback slot is not used. */
                if (Cy_SysTick_GetCallback(i) == NULL)
                {
                    (void)Cy_SysTick_SetCallback(i, context->inactivity_isr);
                    result = true;
                    break;
                }
            }
        }
    }

    return (result);
}


/*******************************************************************************
* Function Name: mtb_lin_disable_timer
****************************************************************************//**
*
*  Disables Systick timer interrupts.
*
* \sideeffect
*  Stops the SysTick timer if there are no active callback slots left and
*  disables the SysTick interrupt.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void mtb_lin_disable_timer(const mtb_stc_lin_context_t* context)
{
    l_u32 null_pointers = 0U;

    /* Find a used callback slot */
    for (l_u32 i = 0U; i < CY_SYS_SYST_NUM_OF_CALLBACKS; i++)
    {
        if (Cy_SysTick_GetCallback(i) == context->inactivity_isr)
        {
            /* Free callback */
            (void)Cy_SysTick_SetCallback(i, NULL);
            null_pointers++;
        }
        else if (Cy_SysTick_GetCallback(i) == NULL)
        {
            /* This callback slot is unused */
            null_pointers++;
        }
        else
        {
            /* This callback slot is used by some function */
        }
    }

    if (null_pointers == CY_SYS_SYST_NUM_OF_CALLBACKS)
    {
        /* If there are no used callback slots - disable SysTick timer */
        Cy_SysTick_Disable();
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_break_field
****************************************************************************//**
*
*  Handles the receive of the break field.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* Always returns "true".
*
*******************************************************************************/
__STATIC_INLINE l_bool l_ifc_rx_break_field(mtb_stc_lin_context_t* context)
{
    /* Set flag that Break detected correctly. In this case we don't need to
     * restore clock divider in mtb_lin_finalize_frame() as it will be done
     * after entire packet is transmitted.
     */
    l_u8 end_frame_status = MTB_LIN_HANDLING_RESET_BREAK;

    /* Resets the pending flag */
    Cy_SCB_ClearRxInterrupt(context->scb_uart_obj.base, CY_SCB_RX_INTR_UART_BREAK_DETECT);

    /* A framing error or data transfer is aborted */
    if (0U != (MTB_LIN_FSM_DATA_RECEIVE & context->uart_fsm_flags))
    {
        /* Sets the response error */
        context->ifc_comm_status |= MTB_LIN_IFC_STS_ERROR_IN_RESPONSE;

        l_bool_wr(context->config->resp_error_signal_handle, true, context);

        /* Set flag that Break detected incorrectly. In this case we need to
         * restore clock divider in mtb_lin_finalize_frame().
         */
        end_frame_status = MTB_LIN_HANDLING_RESET_FSM_ERR;
    }   /* No response error, continue */


    context->uart_fsm_flags &=
        ((l_u8) ~(MTB_LIN_FSM_DATA_RECEIVE | MTB_LIN_FSM_FRAMING_ERROR_FLAG));

    /* Sets UART ISR FSM to IDLE state */
    context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_0_IDLE;

    /* Resets the UART state machine */
    mtb_lin_finalize_frame(end_frame_status, context);

    /* Starts the BREAK to SYNC timeout counter for SYNC failure detection*/
    context->break_to_sync_counts = 1U; /* Counter increments the value if non-zero. */

    return true;
}


/*******************************************************************************
* Function Name: l_ifc_rx_bad_sync
****************************************************************************//**
*
*  Processes the incorrect sync field received.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
* "true" if the frame receive is aborted.
*
*******************************************************************************/
__STATIC_INLINE l_bool l_ifc_rx_bad_sync(mtb_stc_lin_context_t* context)
{
    l_bool result = false;

    /* Clears SYNC source first */
    Cy_SCB_ClearRxInterrupt(context->scb_uart_obj.base, CY_SCB_RX_INTR_BAUD_DETECT);

    /* Stops and disables the BREAK to SYNC timeout counter */
    context->break_to_sync_counts = 0U; /* counter does not increment if zero */

    if (context->config->auto_baud_rate_sync)    /* Auto Baud Rate Sync Enabled */
    {
        context->sync_counts = (l_u16)Cy_SCB_UART_GetBaudRateCount(context->scb_uart_obj.base);

        /* Checks if correction is necessary */
        if (context->sync_counts != MTB_LIN_EXPECTED_TIME_COUNTS)
        {
            /* Check if we can get into the valid range, and drop the packet if we cannot */
            if (mtb_lin_is_clock_valid(context->sync_counts, context))
            {
                l_u32 tmp;

                /* BR_COUNTER uses 0x80 as the base to measure the frequency,
                 * so divide its constant by 0x80 and round off the result checking the last
                 * thrown bit */
                tmp =
                    ((l_u32)context->configured_clock_divider * (l_u32)context->sync_counts);
                context->corrected_clock_divider  = (l_u32)(tmp >> MTB_LIN_BR_BASE_SHIFT);
                context->corrected_clock_divider +=
                    (l_u16)((0U !=
                             (tmp & (l_u8)(1U << (MTB_LIN_BR_BASE_SHIFT - 1U)))) ? 1U : 0U);

                /* Restores the initial clock divider. */
                mtb_lin_set_clock_divider((context->corrected_clock_divider - 1U), context);
            }
            else
            {
                /* Sets a response error */
                context->ifc_comm_status |= MTB_LIN_IFC_STS_ERROR_IN_RESPONSE;

                /* Reset the UART state machine */
                mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);

                /* Nothing to process in the RX/TX state machine due to bad SYNC */
                result = true;
            }
        }
    }

    if (false == result)
    {
        /* Sets the UART ISR FSM to state 1 (PID awaiting) */
        context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_1_PID;

        /* Sets the UART enabled flag */
        context->uart_fsm_flags |= MTB_LIN_FSM_UART_ENABLE_FLAG;

        /* Clears any pending UART interrupt and RX FIFO */
        Cy_SCB_ClearRxFifo(context->scb_uart_obj.base);

        /* Clears all UART pending interrupts */
        Cy_SCB_ClearRxInterrupt(context->scb_uart_obj.base, MTB_LIN_SCB_INTR_RX_ALL);
    }

    return result;
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_0_idle
****************************************************************************//**
*
*  Handles the idle state of the RX finite-state machine.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_0_idle(mtb_stc_lin_context_t* context)
{
    Cy_SCB_ClearRxFifo(context->scb_uart_obj.base);
    Cy_SCB_ClearRxInterrupt(context->scb_uart_obj.base, CY_SCB_RX_INTR_NOT_EMPTY);

    /* Reset the UART state machine */
    mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid
****************************************************************************//**
*
*  Handles the state of the PID receive in the finite-state machine.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid(mtb_stc_lin_context_t* context)
{
    /******************************************************************************
    *                            Parity Lookup Table
    * A 6-bit identifier is given as an index.
    * The indexed value provides the correct value with the parity bit-set.
    ******************************************************************************/
    const l_u8 mtb_lin_parity_table[] =
    {
        0x80U, 0xC1U, 0x42U, 0x03U, 0xC4U, 0x85U, 0x06U, 0x47U, 0x08U, 0x49U, 0xCAU,
        0x8BU, 0x4CU, 0x0DU, 0x8EU, 0xCFU, 0x50U, 0x11U, 0x92U, 0xD3U, 0x14U, 0x55U,
        0xD6U, 0x97U, 0xD8U, 0x99U, 0x1AU, 0x5BU, 0x9CU, 0xDDU, 0x5EU, 0x1FU, 0x20U,
        0x61U, 0xE2U, 0xA3U, 0x64U, 0x25U, 0xA6U, 0xE7U, 0xA8U, 0xE9U, 0x6AU, 0x2BU,
        0xECU, 0xADU, 0x2EU, 0x6FU, 0xF0U, 0xB1U, 0x32U, 0x73U, 0xB4U, 0xF5U, 0x76U,
        0x37U, 0x78U, 0x39U, 0xBAU, 0xFBU, 0x3CU, 0x7DU, 0xFEU, 0xBFU
    };

    if (0U == (MTB_LIN_FSM_FRAMING_ERROR_FLAG & context->uart_fsm_flags))
    {
        /* Saves PID */
        context->current_frame_pid = (l_u8)Cy_SCB_UART_Get(context->scb_uart_obj.base);

        /* Resets the number of transferred bytes */
        context->bytes_transferred = 0U;

        /* Clears the checksum byte */
        context->interim_checksum = 0U;

        /* Clears Frame Error after PID flag - such a condition must be processed separately,
         * as it can be Frame Error caused by the Break field of the next frame
         */
        context->frame_error_after_pid = false;

        /* Verifies the PID parity */
        if (mtb_lin_parity_table[context->current_frame_pid & MTB_LIN_PID_PARITY_MASK] ==
            context->current_frame_pid)
        {
            /* Update the context->current_frame_index based on context->current_frame_pid */
            mtb_lin_update_frame_index(context);

            /* Checks whether it is an MRF or SRF frame */
            if ((MTB_LIN_FRAME_PID_MRF == context->current_frame_pid) ||
                (MTB_LIN_FRAME_PID_SRF == context->current_frame_pid))
            {
                /*  Transport Layer section. MRF and SRF detection */
                if (context->config->tl_enabled)
                {
                    l_ifc_rx_fsm_1_pid_tl(context);
                }
                else
                {
                    /* These are invalid PIDs when TL is disabled - reset the UART state machine */
                    l_ifc_rx_fsm_1_pid_err(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
                }
            }
            else    /* Not MRF or SRF */
            {
                if (MTB_LIN_INVALID_FRAME_PID != context->current_frame_index)
                {
                    /* Valid ID */
                    /* Starts enhanced checksum calculation  */
                    context->interim_checksum = context->current_frame_pid;

                    /* Gets the size of the frame */
                    context->frame_size =
                        context->config->frames[context->current_frame_index].size;

                    /* TX response (publish action) is requested by Master */
                    if (MTB_LIN_FRAME_DIR_PUBLISH ==
                        context->config->frames[context->current_frame_index].direction)
                    {
                        l_ifc_rx_fsm_1_pid_publish(context);
                    }
                    else    /* RX response (subscribe action) is requested by Master */
                    {
                        l_ifc_rx_fsm_1_pid_subscribe(context);
                    }
                }
                else    /* Invalid ID */
                {
                    /* Resets the UART State Machine */
                    mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
                }
            }
        }
        else    /* PID parity is incorrect */
        {
            l_ifc_rx_fsm_1_pid_err(MTB_LIN_HANDLING_DONT_SAVE_PID, context);
        }
    }
    else
    {
        /* Reset the UART State Machine */
        mtb_lin_finalize_frame(MTB_LIN_HANDLING_DONT_SAVE_PID, context);
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_tl
****************************************************************************//**
*
*  Handles the state of the PID receive in the context of the transport layer.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl(mtb_stc_lin_context_t* context)
{
    if (MTB_LIN_FRAME_PID_MRF == context->current_frame_pid)
    {
        l_ifc_rx_fsm_1_pid_tl_mrf(context);
    }

    if (MTB_LIN_FRAME_PID_SRF == context->current_frame_pid)
    {
        l_ifc_rx_fsm_1_pid_tl_srf(context);
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_tl_mrf
****************************************************************************//**
*
*  Handles the PID receive in the context of the transport layer, when Master
*  Request Frame (MRF) PID received.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_mrf(mtb_stc_lin_context_t* context)
{
    /* Indicates that Slave is required to receive data. */
    context->tl_flags |= MTB_LIN_TL_RX_DIRECTION;

    /*******************************************************
    *               Cooked & RAW API
    *******************************************************/
    /* If MRF PID is detected, passes the pointer to the start of
     * Frame Buffer and size of data to RX state to handle data receiving */

    /* The frame equals 8 bytes */
    context->frame_size = MTB_LIN_FRAME_DATA_SIZE_8;

    /* Sets the frame data pointer to start the frame buffer */
    context->frame_data = context->mrf_buffer;

    /* Switches to subscribe data state */
    context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_3_RX;
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_tl_srf
****************************************************************************//**
*
*  Handles the PID receive in the context of the transport layer, when Slave
*  Response Frame (SRF) PID received.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_srf(mtb_stc_lin_context_t* context)
{
    if (0U != (context->status & MTB_LIN_STATUS_SRVC_RSP_RDY))
    {
        /* The frame is always equal to 8 bytes for TL */
        context->frame_size = MTB_LIN_FRAME_DATA_SIZE_8;

        /* Sets the frame data pointer to the frame buffer start. */
        context->frame_data = context->srf_buffer;

        /* Sends the first byte to LIN Master. */
        context->tmp_data = *context->frame_data;
        context->frame_data++;

        mtb_lin_uart_write_tx_data((l_u32)context->tmp_data, context);
        context->bytes_transferred = 1U;

        /* Switch to publish data state. */
        context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_2_TX;

        /* One or more data bytes have been received */
        context->uart_fsm_flags |= MTB_LIN_FSM_DATA_RECEIVE;

        /* Indicates to Transport layer that Slave transmits the frame */
        context->tl_flags |= MTB_LIN_TL_TX_DIRECTION;
    }
    else
    {
        /* This part of code handles LIN Transport Layer. Sends one frame of
           a message if
         * applications prepared the message in the TX message buffer or TX
         * queue and LIN Slave node
         * state machine has the appropriate state (is ready for physical
         * response transmission).
         */
        if ((0U != (context->tl_flags & MTB_LIN_TL_TX_REQUESTED)) &&
            (context->node_state == MTB_LIN_NODE_STATE_TX_PHY_RESPONSE))
        {
            if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_RAW)
            {
                l_ifc_rx_fsm_1_pid_tl_srf_raw(context);
            }

            /* Handles PDU packing for Cooked API. */
            /* Checks the length so it shows if the message is already sent.
             */
            if (context->tl_tx_message_length == 0U)
            {
                context->tl_flags &= (l_u8) ~MTB_LIN_TL_TX_REQUESTED;

                /* Resets UART State Machine */
                mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
            }
            /* Processes message sending */
            else
            {
                l_ifc_rx_fsm_1_pid_tl_srf_cooked(context);
            }
        }
        else
        {
            /* Reset the UART State Machine */
            mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
        }
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_tl_srf_raw
****************************************************************************//**
*
*  Handles the PID receive in the context of the transport layer in Raw mode,
*  when Slave Response Frame (SRF) PID received.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_srf_raw(mtb_stc_lin_context_t* context)
{
    l_u8 i;

    if (0U != context->tl_raw_tx_buf_depth)
    {
        i = 0U;

        for (; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
        {
            context->srf_buffer[i] =
                context->config->tl_raw_tx_queue[context->tl_raw_tx_read_index];

            /* Updates the index to TX queue */
            context->tl_raw_tx_read_index++;
        }

        /* The Read index must point to the next byte in MRF
         */
        if (context->config->tl_tx_queue_len == context->tl_raw_tx_read_index)
        {
            context->tl_raw_tx_read_index = 0U;
        }

        /* 8 bytes were read from the SRF, so decrement the
           buffer depth. */
        context->tl_raw_tx_buf_depth--;

        /* Updates the status properly */
        context->tl_tx_status =
            (0U == context->tl_raw_tx_buf_depth) ? LD_QUEUE_EMPTY : LD_QUEUE_AVAILABLE;

        if (MTB_LIN_PDU_PCI_TYPE_UNKNOWN == context->tl_tx_prev_pci)
        {
            if (context->srf_buffer[MTB_LIN_PDU_PCI_IDX] <= MTB_LIN_PDU_SF_DATA_LEN)
            {
                /* Get the length of the message from PCI field of SF */
                context->tl_tx_message_length |=
                    ((l_u16)context->srf_buffer[MTB_LIN_PDU_PCI_IDX]);
            }
            else
            {
                /* Get the length of Segmented message from PCI and
                   LEN fields of FF
                 * NOTE The PCI field contains four MSb of the
                 * length (Length / 256).
                 */
                context->tl_tx_message_length =
                    (l_u16)((((l_u16)context->srf_buffer[MTB_LIN_PDU_PCI_IDX]) &
                             ((l_u16)((l_u8) ~MTB_LIN_PDU_PCI_TYPE_MASK))) << 8U);
                context->tl_tx_message_length |=
                    ((l_u16)context->srf_buffer[MTB_LIN_PDU_LEN_IDX]);
            }
        }
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_tl_srf_cooked
****************************************************************************//**
*
*  Handles the PID receive in the context of the transport layer in Cooked mode,
*  when Slave Response Frame (SRF) PID received.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_tl_srf_cooked(mtb_stc_lin_context_t* context)
{
    /* Analyzes the length to find the type of frame message to be sent */
    if (context->tl_tx_message_length > MTB_LIN_FRAME_DATA_SIZE_6)
    {
        /* Process the FF Frame */
        if (context->tl_tx_prev_pci == MTB_LIN_PDU_PCI_TYPE_UNKNOWN)
        {
            /* Fill Frame PCI field */
            /* Save the previous PCI */
            context->tl_tx_prev_pci = MTB_LIN_PDU_PCI_TYPE_FF;
            context->tl_flags |= MTB_LIN_TL_N_AS_TIMEOUT_ON;
            context->tl_timeout_counter = 0U;

            if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED)
            {
                context->srf_buffer[0U] = context->configured_nad;

                context->srf_buffer[1U] =
                    (MTB_LIN_PDU_PCI_TYPE_FF | (CY_HI8(context->tl_tx_message_length)));

                /* Fill the Frame LEN field */
                context->srf_buffer[2U] = CY_LO8(context->tl_tx_message_length);

                /* Fill Frame Data fields */
                for (l_u8 i = 3U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
                {
                    context->srf_buffer[i] =
                        context->tl_tx_data_pointer[(i + context->tl_tx_data_count) - 3U];
                }

                /* Update the user buffer pointer */
                context->tl_tx_data_count += MTB_LIN_FRAME_DATA_SIZE_5;
                context->tl_tx_message_length -= MTB_LIN_FRAME_DATA_SIZE_5;
            }
        }
        else    /* Process the CF Frame */
        {
            if (context->config->tl_api_format == MTB_LIN_TL_FORMAT_COOKED)
            {
                /* Fills the Frame PCI field */
                context->srf_buffer[1U] =
                    (MTB_LIN_PDU_PCI_TYPE_CF | context->tl_tx_frame_counter);

                /* Fills the Frame Data fields */
                for (l_u8 i = 2U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
                {
                    context->srf_buffer[i] =
                        context->tl_tx_data_pointer[(i + context->tl_tx_data_count) - 2U];
                }

                /* Updates the user buffer pointer */
                context->tl_tx_data_count += MTB_LIN_FRAME_DATA_SIZE_6;
            }

            /* Saves the previous PCI */
            context->tl_tx_prev_pci = MTB_LIN_PDU_PCI_TYPE_CF;

            /* Update the length pointer properly */
            context->tl_tx_message_length -= MTB_LIN_FRAME_DATA_SIZE_6;
            context->tl_flags |= MTB_LIN_TL_N_AS_TIMEOUT_ON;
            context->tl_timeout_counter = 0U;
        }
    }
    else    /* Processes SF Frame or last CF Frame */
    {
        /* Checks if the Previous frame is in "Unknown" state,
           which indicates that
         * the current frame is SF, otherwise it is the last CF
         * frame.
         * Fill the Frame PCI field properly.
         */
        context->tl_flags |= MTB_LIN_TL_N_AS_TIMEOUT_ON;
        context->tl_timeout_counter = 0U;

        if (MTB_LIN_PDU_PCI_TYPE_UNKNOWN == context->tl_tx_prev_pci)
        {
            if (MTB_LIN_TL_FORMAT_COOKED == context->config->tl_api_format)
            {
                context->srf_buffer[0U] = context->configured_nad;
                context->srf_buffer[1U] = (l_u8)context->tl_tx_message_length;
            }

            /* Save the previous PCI */
            context->tl_tx_prev_pci = MTB_LIN_PDU_PCI_TYPE_SF;
        }
        else
        {
            if (MTB_LIN_TL_FORMAT_COOKED == context->config->tl_api_format)
            {
                /* Fill the Frame NAD field */
                context->srf_buffer[0U] = context->configured_nad;
                context->srf_buffer[1U] = (MTB_LIN_PDU_PCI_TYPE_CF | context->tl_tx_frame_counter);
            }

            /* Save the previous PCI */
            context->tl_tx_prev_pci = MTB_LIN_PDU_PCI_TYPE_CF;
        }

        if (MTB_LIN_TL_FORMAT_COOKED == context->config->tl_api_format)
        {
            /* Fill the Frame Data fields */
            for (l_u8 i = 2U; i < MTB_LIN_FRAME_DATA_SIZE_8; i++)
            {
                if (context->tl_tx_message_length >= ((l_u8)(i - 1U)))
                {
                    context->srf_buffer[i] =
                        context->tl_tx_data_pointer[(i + context->tl_tx_data_count) - 2U];
                }
                else
                {
                    /* Fill unused data bytes with FFs */
                    context->srf_buffer[i] = 0xFFU;
                }
            }

            /* Update the user buffer pointer */
            context->tl_tx_data_count += MTB_LIN_FRAME_DATA_SIZE_6;
        }

        /* Update the length pointer properly */
        if (context->tl_tx_message_length > MTB_LIN_FRAME_DATA_SIZE_6)
        {
            context->tl_tx_message_length -= MTB_LIN_FRAME_DATA_SIZE_6;
        }
        else
        {
            context->tl_tx_message_length = 0U;
        }
    }

    /* Update the frame counter */
    if (context->tl_tx_frame_counter != 15U)
    {
        context->tl_tx_frame_counter++;
    }
    else
    {
        /* If the frame counter is larger than 15, reset it */
        context->tl_tx_frame_counter = 0U;
    }

    /* Frame equals 8 bytes */
    context->frame_size = MTB_LIN_FRAME_DATA_SIZE_8;

    /* Set the frame data pointer to a start of a frame buffer */
    context->frame_data = context->srf_buffer;

    /* Send the first byte to the LIN Master */
    context->tmp_data = *context->frame_data;
    context->frame_data++;

    mtb_lin_uart_write_tx_data((l_u32)context->tmp_data, context);
    context->bytes_transferred = 1U;

    /* Switch to the publish data state. */
    context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_2_TX;

    /* One or more data bytes are received */
    context->uart_fsm_flags |= MTB_LIN_FSM_DATA_RECEIVE;

    /* Indicates to transport layer that Slave is transmitting
       the frame */
    context->tl_flags |= MTB_LIN_TL_TX_DIRECTION;

    /* Checks if the SRF is the pending response frame */
    if ((context->srf_buffer[MTB_LIN_PDU_PCI_IDX] == 0x03U) &&
        (context->srf_buffer[MTB_LIN_PDU_SID_IDX] == 0x7FU) &&
        (context->srf_buffer[MTB_LIN_PDU_D2_IDX] == 0x78U))
    {
        context->status |= MTB_LIN_STATUS_RESPONSE_PENDING;
    }
    else
    {
        context->status &= (l_u8) ~MTB_LIN_STATUS_RESPONSE_PENDING;
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_err
****************************************************************************//**
*
*  Resets the UART finite-state machine upon invalid PID receive.
*
* \param status
*  - MTB_LIN_HANDLING_DONT_SAVE_PID - PID parity is incorrect.
*  - MTB_LIN_HANDLING_RESET_FSM_ERR - Invalid PID.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_err(l_u8 status, mtb_stc_lin_context_t* context)
{
    /* Sets the response error */
    context->ifc_comm_status |= MTB_LIN_IFC_STS_ERROR_IN_RESPONSE;

    /* Resets the UART State Machine */
    mtb_lin_finalize_frame(status, context);
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_publish
****************************************************************************//**
*
*  Processes the PID of the publish frame.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_publish(mtb_stc_lin_context_t* context)
{
    l_bool event_triggered_flag = true;

    /* The function is called only when context->current_frame_index is not
     * equal to MTB_LIN_INVALID_FRAME_PID, so it is safe to access data.
     */
    mtb_stc_lin_frame_t frame = context->config->frames[context->current_frame_index];

    /* This frame is event-triggered */
    if (MTB_LIN_FRAME_TYPE_EVENT_TRIGGERED == frame.type)
    {
        /* Checks whether to process an event-triggered frame */
        if (!(context->config->frames_context[context->
                                              current_frame_index].
              et_flag))
        {
            /* Resets the UART State Machine */
            mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR,
                                   context);

            /* Finalization of data handling */
            event_triggered_flag = false;
        }
    }

    if (true == event_triggered_flag)
    {
        /* Copy the frame data to the buffer and use it for TX */
        for (l_u8 i = 0U; i < frame.size; i++)
        {
            context->tmp_tx_frame_data[i] = frame.data[i];
        }
        context->frame_data = context->tmp_tx_frame_data;

        /* Sends the first byte to LIN master */
        context->tmp_data = *context->frame_data;

        /* This frame is event-triggered */
        if (MTB_LIN_FRAME_TYPE_EVENT_TRIGGERED == frame.type)
        {
            /* Send the associated unconditional frame's PID as the first
                byte */
            l_u8 unconditional_frame_index = frame.associated_frame_index;
            context->tmp_data =
                context->config->frames_context[unconditional_frame_index].
                configured_pid;
        }
        else
        {
            context->uart_fsm_flags |= MTB_LIN_FSM_DATA_RECEIVE;
        }

        context->frame_data++;

        mtb_lin_uart_write_tx_data((l_u32)context->tmp_data, context);
        context->bytes_transferred = 1U;

        /* Switches to publish data state. */
        context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_2_TX;

        /* Sets DATA RECEIVE flag */
        context->uart_fsm_flags |= MTB_LIN_FSM_DATA_RECEIVE;
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_1_pid_subscribe
****************************************************************************//**
*
*  Processes the PID of the subscribe frame.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_1_pid_subscribe(mtb_stc_lin_context_t* context)
{
    context->bytes_transferred = 0U;

    /* Gets the pointer to the temp RX frame data buffer */
    context->frame_data = context->tmp_rx_frame_data;

    /* Switches to subscribe the data state. */
    context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_3_RX;
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_2_tx
****************************************************************************//**
*
*  Handles data transmit.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_2_tx(mtb_stc_lin_context_t* context)
{
    /* Sets the response active flag */
    context->status |= MTB_LIN_STATUS_RESPONSE_ACTIVE;

    /* Previously transmitted and read back bytes are not equal */
    if ((context->tmp_data != (l_u8)Cy_SCB_UART_Get(context->scb_uart_obj.base)) ||
        (0U != (MTB_LIN_FSM_FRAMING_ERROR_FLAG & context->uart_fsm_flags)))
    {
        /* Mismatch Error */

        /* Skips the event-triggered frame */
        if (!mtb_is_current_frame_event_triggered(context))
        {
            /* Readback error - sets the response error flag */
            context->ifc_comm_status |= MTB_LIN_IFC_STS_ERROR_IN_RESPONSE;

            l_bool_wr(context->config->resp_error_signal_handle, true, context);
        }

        /* Checks for framing error */
        if (0U == (context->uart_fsm_flags & MTB_LIN_FSM_FRAMING_ERROR_FLAG))
        {
            /* Saves the last processed on bus PID to status variable. */
            context->ifc_comm_status &= ((l_u16) ~MTB_LIN_IFC_STS_PID_MASK);
            context->ifc_comm_status |=
                ((l_u16)(((l_u16)context->current_frame_pid) << 8U));
        }

        /* End frame with response error */
        mtb_lin_finalize_frame(MTB_LIN_HANDLING_DONT_SAVE_PID, context);
    }
    else    /* If the readback is successful, continue transmitting */
    {
        /* Adds the transmitted byte to interim checksum. */
        context->interim_checksum += context->tmp_data;

        if (context->interim_checksum >= 256U)
        {
            context->interim_checksum -= 255U;
        }

        /* Checks to see if all data bytes are sent. */
        if (context->frame_size > context->bytes_transferred)
        {
            /* Sends out the next byte of the buffer. */
            context->tmp_data = *context->frame_data;
            context->frame_data++;
            mtb_lin_uart_write_tx_data((l_u32)context->tmp_data, context);
            context->bytes_transferred++;
        }
        else    /* All data bytes are sent - compute and transmit the checksum. */
        {
            /* Computes and sends out the checksum byte */
            mtb_lin_uart_write_tx_data((((l_u8)context->interim_checksum) ^ (l_u32)0xFFU),
                                       context);
            context->bytes_transferred = 0U;

            /* Switches to the checksum state */
            context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_4_CHS;
        }
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_3_rx
****************************************************************************//**
*
*  Handles data receive.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_3_rx(mtb_stc_lin_context_t* context)
{
    /* Sets Response error only if the error appears during receive */
    if (0U != (MTB_LIN_FSM_FRAMING_ERROR_FLAG & context->uart_fsm_flags))
    {
        /* This is a workaround to suppress Response Error when only Break header + PID are
           received. */
        if ((0U == context->bytes_transferred) && (!context->frame_error_after_pid))
        {
            /* Reads the erroneous byte from FIFO. */
            (void)Cy_SCB_UART_Get(context->scb_uart_obj.base);

            /* Bypasses the first Framing error after PID */
            context->frame_error_after_pid = true;
        }
        else
        {
            /* Sets a response error */
            context->ifc_comm_status |= MTB_LIN_IFC_STS_ERROR_IN_RESPONSE;

            l_bool_wr(context->config->resp_error_signal_handle, true, context);

            /* Clear the FE  flag and DATA_RECEIVE flags anyway */
            context->uart_fsm_flags &= (l_u8)(~MTB_LIN_FSM_DATA_RECEIVE);

            /* Finishes frame processing */
            mtb_lin_finalize_frame(MTB_LIN_HANDLING_DONT_SAVE_PID, context);
        }
        /* Clear FE  flag and DATA_RECEIVE flags anyway */
        context->uart_fsm_flags &= (l_u8)(~MTB_LIN_FSM_FRAMING_ERROR_FLAG);
    }
    else
    {
        /* Saves the received byte */
        context->tmp_data = (l_u8)(Cy_SCB_UART_Get(context->scb_uart_obj.base));

        *context->frame_data = context->tmp_data;
        context->frame_data++;
        context->bytes_transferred++;

        /* Sets the response active flag */
        context->status |= MTB_LIN_STATUS_RESPONSE_ACTIVE;

        /* One or more data bytes are received */
        context->uart_fsm_flags |= MTB_LIN_FSM_DATA_RECEIVE;

        /* Adds the received byte to  the interim checksum. */
        context->interim_checksum += context->tmp_data;

        if (context->interim_checksum >= 256U)
        {
            context->interim_checksum -= 255U;
        }

        /* Checks if the data section completed */
        if (context->frame_size > context->bytes_transferred)
        {
            /* There is data to be received. */
        }
        else
        {
            /* There is no data to be received. */
            context->bytes_transferred = 0U;

            /* Switches to the checksum state. */
            context->uart_fsm_state = MTB_LIN_UART_ISR_STATE_4_CHS;
        }
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_4_chs
****************************************************************************//**
*
*  Processes the frame's checksum.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_4_chs(mtb_stc_lin_context_t* context)
{
    l_u32 interrupt_state;

    /* Previously transmitted and read back bytes are not equal */
    if (((((l_u8)context->interim_checksum) ^ 0xFFU) !=
         (l_u8)Cy_SCB_UART_Get(context->scb_uart_obj.base))
        || (0U != (context->uart_fsm_flags & MTB_LIN_FSM_FRAMING_ERROR_FLAG)))
    {
        l_ifc_rx_fsm_4_chs_err(context);
    }
    else
    {
        /* Clears all error bits in the interface status. */

        /* Clears the framing error and data receive flags */
        context->uart_fsm_flags &=
            ((l_u8) ~(MTB_LIN_FSM_FRAMING_ERROR_FLAG | MTB_LIN_FSM_DATA_RECEIVE));

        /* Sets the successful transfer interface flag */
        context->ifc_comm_status |= MTB_LIN_IFC_STS_SUCCESSFUL_TRANSFER;

        /* Saves the last processed on bus PID to the status variable */
        context->ifc_comm_status &= ((l_u16) ~MTB_LIN_IFC_STS_PID_MASK);
        context->ifc_comm_status |= ((l_u16)(((l_u16)context->current_frame_pid) << 8U));

        /* Sets the overrun interface flag */
        if (0U != (MTB_LIN_FSM_OVERRUN & context->uart_fsm_flags))
        {
            context->ifc_comm_status |= MTB_LIN_IFC_STS_OVERRUN;
        }

        /* Sets the Overrun flag */
        context->uart_fsm_flags |= MTB_LIN_FSM_OVERRUN;

        /* Clears response error signal if the frame contains RESPONSE ERROR signal */
        if (context->config->signals[context->config->resp_error_signal_handle].
            frame_index == context->current_frame_index)
        {
            l_bool_wr(context->config->resp_error_signal_handle, false, context);
        }

        if ((!context->config->tl_enabled) || ((context->config->tl_enabled) &&
                                               (!((MTB_LIN_FRAME_PID_MRF ==
                                                   context->current_frame_pid) ||
                                                  (MTB_LIN_FRAME_PID_SRF ==
                                                   context->current_frame_pid)))))
        {
            /* This frame is event-triggered */
            if (mtb_is_current_frame_event_triggered(context))
            {
                /* Clears event-triggered flags */
                context->config->frames_context[context->current_frame_index].et_flag = false;

                /* Resets UART State Machine */
                mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
            }

            /* Sets associated with the current frame flags */
            mtb_lin_update_flags(context);
        }

        if (context->config->tl_enabled)
        {
            l_ifc_rx_fsm_4_chs_tl(context);
        }
        else
        {
            /* RX response (subscribe action) is requested by Master */
            if (MTB_LIN_INVALID_FRAME_PID != context->current_frame_index)
            {
                if (MTB_LIN_FRAME_DIR_SUBSCRIBE ==
                    context->config->frames[context->current_frame_index].direction)
                {
                    interrupt_state = Cy_SysLib_EnterCriticalSection();

                    /* Copies the received data from the temporary buffer to the frame buffer */
                    for (l_u8 i = 0U; i < context->frame_size; i++)
                    {
                        CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 18.4', 1,
                                                'Advisory. Usage of arithmetic on pointer. This approach gives flexibility in data access. Checked manually, no possible issues expected.');
                        *(context->config->frames[context->current_frame_index].data +
                          i) = context->tmp_rx_frame_data[i];
                        CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.4');
                    }

                    Cy_SysLib_ExitCriticalSection(interrupt_state);
                }
            }

            /* Resets the UART state machine */
            mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
        }
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_4_chs_tl
****************************************************************************//**
*
*  Processes the frame's checksum on the transport layer.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_4_chs_tl(mtb_stc_lin_context_t* context)
{
    l_u32 interrupt_state;

    /* Checks if the received data is a "master request frame" */
    if (MTB_LIN_FRAME_PID_MRF == context->current_frame_pid)
    {
        l_bool nad_accepted;

        /* Check NAD */
        nad_accepted =
            (l_bool)((context->configured_nad ==
                      context->mrf_buffer[MTB_LIN_PDU_NAD_IDX]) ||
                     (MTB_LIN_NAD_BROADCAST ==
                      context->mrf_buffer[MTB_LIN_PDU_NAD_IDX]) ||
                     (context->config->initial_nad ==
                      context->mrf_buffer[MTB_LIN_PDU_NAD_IDX]));

        if (MTB_LIN_NAD_GO_TO_SLEEP == context->mrf_buffer[MTB_LIN_PDU_NAD_IDX])
        {
            context->ifc_comm_status |= MTB_LIN_IFC_STS_GO_TO_SLEEP;
        }
        else if (nad_accepted)
        {
            mtb_lin_update_node_state(MTB_LIN_NODE_STIMULUS_MRF_OWN_NAD, context);
        }
        else if (MTB_LIN_NAD_FUNCTIONAL == context->mrf_buffer[MTB_LIN_PDU_NAD_IDX])
        {
            mtb_lin_update_node_state(MTB_LIN_NODE_STIMULUS_MRF_FUNC_NAD, context);
        }
        else
        {
            mtb_lin_update_node_state(MTB_LIN_NODE_STIMULUS_MRF_ALIEN_NAD, context);
        }

        /* Clears the TL RX direction flag */
        context->tl_flags &= ((l_u8) ~MTB_LIN_TL_RX_DIRECTION);

        /* Resets the UART state machine */
        mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
    }
    else if (MTB_LIN_FRAME_PID_SRF == context->current_frame_pid)
    {
        mtb_lin_update_node_state(MTB_LIN_NODE_STIMULUS_SRF, context);

        /* Resets the UART state machine */
        mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
    }
    else
    {
        /* RX response (subscribe action) is requested by Master */
        if (MTB_LIN_INVALID_FRAME_PID != context->current_frame_index)
        {
            if (MTB_LIN_FRAME_DIR_SUBSCRIBE ==
                context->config->frames[context->current_frame_index].direction)
            {
                interrupt_state = Cy_SysLib_EnterCriticalSection();

                /* Copy the received data from the temporary buffer to the frame buffer */
                for (l_u8 i = 0U; i < context->frame_size; i++)
                {
                    CY_MISRA_FP_BLOCK_START('MISRA C-2012 Rule 18.4', 1,
                                            'Advisory. Usage of arithmetic on pointer. This approach gives flexibility in data access. Checked manually, no possible issues expected.');
                    *(context->config->frames[context->current_frame_index].data +
                      i) = context->tmp_rx_frame_data[i];
                    CY_MISRA_BLOCK_END('MISRA C-2012 Rule 18.4');
                }

                Cy_SysLib_ExitCriticalSection(interrupt_state);
            }
        }

        /* Resets the UART state machine */
        mtb_lin_finalize_frame(MTB_LIN_HANDLING_RESET_FSM_ERR, context);
    }
}


/*******************************************************************************
* Function Name: l_ifc_rx_fsm_4_chs_err
****************************************************************************//**
*
*  Handles the situation when the checksum is incorrect.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void l_ifc_rx_fsm_4_chs_err(mtb_stc_lin_context_t* context)
{
    /* Mismatch or Checksum Error */
    /* Sets a response error */
    context->ifc_comm_status |= MTB_LIN_IFC_STS_ERROR_IN_RESPONSE;

    l_bool_wr(context->config->resp_error_signal_handle, true, context);

    /* Checks for a framing error */
    if (0U == (context->uart_fsm_flags & MTB_LIN_FSM_FRAMING_ERROR_FLAG))
    {
        /* Saves the last processed on bus PID to the status variable. */
        context->ifc_comm_status &= ((l_u16) ~MTB_LIN_IFC_STS_PID_MASK);
        context->ifc_comm_status |=
            ((l_u16)(((l_u16)context->current_frame_pid) << 8U));
    }

    /* Resets the UART state machine with a checksum or mismatch error. */
    mtb_lin_finalize_frame(MTB_LIN_HANDLING_DONT_SAVE_PID, context);
}


/*******************************************************************************
* Function Name: mtb_lin_config_check_nad
****************************************************************************//**
*
* This function validates if preserved Node Address values
* are not used as NAD for Slave node.
* For more details about allowable NAD, see specification LIN 2.2A,
* chapter 4.2.3.2 NAD.
*
* \param nad
* This field defines Node Address for validation.
*
* \param config
* The pointer to the LIN configuration structure.
*
* \return
*  See \ref mtb_lin_status_t.
*
*******************************************************************************/
mtb_lin_status_t mtb_lin_config_check_nad(l_u8 nad, const mtb_stc_lin_config_t* config)
{
    mtb_lin_status_t status;

    if ((nad == MTB_LIN_NAD_GO_TO_SLEEP) || (nad == MTB_LIN_NAD_BROADCAST))
    {
        status = MTB_LIN_STATUS_INVALID_NAD;
    }
    else if ((nad == MTB_LIN_NAD_FUNCTIONAL) &&
             (config->spec == MTB_LIN_SPEC_2_2))
    {
        status = MTB_LIN_STATUS_INVALID_NAD;
    }
    else
    {
        status = MTB_LIN_STATUS_SUCCESS;
    }

    return status;
}


/* [] END OF FILE */
