/***************************************************************************//**
 * \file mtb_lin_sig.c
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

#include "mtb_lin_sig.h"

/*******************************************************************************
*            Internal API
*******************************************************************************/
__STATIC_INLINE void mtb_lin_set_et_flag(l_u16 signal_index, const mtb_stc_lin_context_t* context);

#define IS_FRAME_FLAG_HANDLE(h)         (CY_LO16((h)) == 0U)
#define GET_FRAME_INDEX(h)              (CY_HI16((h)) - 1U)
#define GET_SIGNAL_INDEX(h)             (CY_LO16((h)) - 1U)

/* Take the MRF and SRF is the transport layer is enabled. */
#define IS_FRAME_FLAG_HANDLE_VALID(h, c)  \
    ((CY_HI16((h)) <= ((c)->config->num_of_frames + (((c)->config->tl_enabled) ? 2U : 0U))) \
    && (CY_HI16(h) != 0U))

#define IS_SIGNAL_FLAG_HANDLE_VALID(h, c)  \
    ((CY_LO16((h)) <= (c)->config->num_of_signals) && (CY_LO16((h)) != 0U))

#define IS_FLAG_HANDLE_VALID(h, c)  \
    (IS_FRAME_FLAG_HANDLE((h)) ? \
    IS_FRAME_FLAG_HANDLE_VALID ((h), (c)) : \
    IS_SIGNAL_FLAG_HANDLE_VALID((h), (c)))


/*******************************************************************************
* Function Name: l_flg_tst
****************************************************************************//**
*
*  Returns a C boolean indicating the current state of the flag specified by the name fff.
*  i.e. returns zero if the flag is cleared, non-zero otherwise.
*
* \param fff
*  The name of the flag handle.
*  See \ref l_flag_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  Returns a C boolean indicating the current state of the flag specified by
*  the name fff:
*  "false" - if the flag is cleared.
*  "true" - if the flag is not cleared.
*
*******************************************************************************/
l_bool l_flg_tst(l_flag_handle fff, const mtb_stc_lin_context_t* context)
{
    l_bool result;

    CY_ASSERT_L2(IS_FLAG_HANDLE_VALID(fff, context));

    if (IS_FRAME_FLAG_HANDLE(fff))
    {
        result = context->config->frames_context[GET_FRAME_INDEX(fff)].flag;
    }
    else    /* Flag handle for the signal */
    {
        result = context->config->signals_context[GET_SIGNAL_INDEX(fff)].flag;
    }

    return result;
}


/*******************************************************************************
* Function Name: l_flg_clr
****************************************************************************//**
*
*  Sets the current value of the flag specified by the name fff to zero.
*
* \param fff
*  The name of the flag handle.
*  See \ref l_flag_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_flg_clr(l_flag_handle fff, const mtb_stc_lin_context_t* context)
{
    CY_ASSERT_L2(IS_FLAG_HANDLE_VALID(fff, context));

    if (IS_FRAME_FLAG_HANDLE(fff))
    {
        context->config->frames_context[GET_FRAME_INDEX(fff)].flag = false;
    }
    else    /* Flag handle for the signal */
    {
        context->config->signals_context[GET_SIGNAL_INDEX(fff)].flag = false;
    }
}


/*******************************************************************************
* Function Name: l_bool_rd
****************************************************************************//**
*
*  Reads and returns the current value of the signal for one-bit signals.
*  If an invalid signal handle is passed into the function, does nothing.
*
* \param sss
*  The signal handle of the signal to read.
*  See \ref l_signal_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  Returns the current value of a signal. Returns a non-zero value for an unknown signal.
*
*******************************************************************************/
l_bool l_bool_rd(l_signal_handle sss, const mtb_stc_lin_context_t* context)
{
    l_u32          interrupt_state;
    l_u8           index;
    const volatile l_u8* data;
    l_u8           msk;

    /* Will be returned if no signal with the specified handle exists */
    l_bool result = false;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);

    /* Save the current global interrupt enable and disable it */
    interrupt_state = Cy_SysLib_EnterCriticalSection();

    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            l_u8 bit_offset = sig.bit_offset % 8U;
            l_u8 byte_offset = sig.bit_offset / 8U;

            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_SCALAR_BOOL == sig.type);

            index = sig.frame_index;
            data  = &context->config->frames[index].data[byte_offset];
            msk   = (l_u8)(1U << bit_offset);

            result = (0U != (*data & msk)) ? true : false;
            break;
        }
    }

    /* Restore the global interrupt enable state */
    Cy_SysLib_ExitCriticalSection(interrupt_state);

    return (result);
}


/*******************************************************************************
* Function Name: l_bool_wr
****************************************************************************//**
*
*  Sets the current value of the signal for one-bit signals to "v".
*  If an invalid signal handle is passed into the function, does nothing
*
* \param sss
*  The signal handle of the signal to write.
*  See \ref l_signal_handle.
*
* \param v
*  The value of the signal to be set.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_bool_wr(l_signal_handle sss, l_bool v, const mtb_stc_lin_context_t* context)
{
    l_u8           index;
    volatile l_u8* data;
    l_u8           msk;
    l_u32          interrupt_state;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);


    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            l_u8 bit_offset = sig.bit_offset % 8U;
            l_u8 byte_offset = sig.bit_offset / 8U;

            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_SCALAR_BOOL == sig.type);

            index = sig.frame_index;
            data  = &context->config->frames[index].data[byte_offset];
            msk   = (l_u8)(1U << bit_offset);

            interrupt_state = Cy_SysLib_EnterCriticalSection();

            if (v)
            {
                *data |= msk;
            }
            else
            {
                *data &= ~msk;
            }

            mtb_lin_set_et_flag(i, context);

            Cy_SysLib_ExitCriticalSection(interrupt_state);
        }
    }
}


/*******************************************************************************
* Function Name: l_u8_rd
****************************************************************************//**
*
*  Reads and returns the current value of the signal for 2-8-bit size signals.
*  If an invalid signal handle is passed into the function, does nothing.
*
* \param sss
*  The signal handle of the signal to read.
*  See \ref l_signal_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  Returns current value of the signal.
*
*******************************************************************************/
l_u8 l_u8_rd(l_signal_handle sss, const mtb_stc_lin_context_t* context)
{
    l_u8           index;
    l_u8           byte_offset;
    l_u8           bit_offset;
    l_u8           size;
    l_u8           mask;
    const volatile l_u8* data_0;
    const volatile l_u8* data_1;
    l_u32          interrupt_state;

    /* Will be returned if no signal with the specified handle exists */
    l_u8 result = 0U;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);


    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_SCALAR_U8 == sig.type);

            index      = sig.frame_index;
            size       = sig.size;
            byte_offset = sig.bit_offset / 8U;
            bit_offset  = sig.bit_offset % 8U;

            interrupt_state = Cy_SysLib_EnterCriticalSection();

            if ((size == 8U) && (bit_offset == 0U))
            {
                result = context->config->frames[index].data[byte_offset];
            }
            else if (bit_offset == 0U)
            {
                mask        = 0xFFU >> (8U - size);
                result = context->config->frames[index].data[byte_offset] & mask;
            }
            else if ((bit_offset + size) <= 8U)
            {
                mask = (l_u8)((0xFFU >> (8U - size - bit_offset)) << (8U - size)) >>
                       (8U - size - bit_offset);

                result = (context->config->frames[index].data[byte_offset] & mask) >> bit_offset;
            }
            else
            {
                mask   = 0xFFU >> (8U - size);
                data_0 = &context->config->frames[index].data[byte_offset];
                data_1 = &context->config->frames[index].data[byte_offset + 1U];

                result  = *data_1 << (8U - bit_offset);
                result |= (*data_0 >> bit_offset);
                result &= mask;
            }

            Cy_SysLib_ExitCriticalSection(interrupt_state);
            break;
        }
    }


    return result;
}


/*******************************************************************************
* Function Name: l_u8_wr
****************************************************************************//**
*
*  Sets the current value of the signal for 2-8-bit size signals to v.
*
* \param sss
*  The signal handle of the signal to write.
*  See \ref l_signal_handle.
*
* \param v
*  The value of the signal to be set.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_u8_wr(l_signal_handle sss, l_u8 v, const mtb_stc_lin_context_t* context)
{
    l_u8           index;
    l_u8           byte_offset;
    l_u8           bit_offset;
    l_u8           size;
    l_u8           mask_0;
    l_u8           mask_1;
    volatile l_u8* data_0;
    volatile l_u8* data_1;
    l_u32          interrupt_state;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);


    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_SCALAR_U8 == sig.type);

            index      = sig.frame_index;
            size       = sig.size;
            byte_offset = sig.bit_offset / 8U;
            bit_offset  = sig.bit_offset % 8U;

            interrupt_state = Cy_SysLib_EnterCriticalSection();

            if ((size == 8U) && (bit_offset == 0U))
            {
                context->config->frames[index].data[byte_offset] = v;
            }
            else if (bit_offset == 0U)
            {
                mask_0                                           = 0xFFU >> (8U - size);
                context->config->frames[index].data[byte_offset] &= (l_u8) ~mask_0;
                context->config->frames[index].data[byte_offset] |= (v & mask_0);
            }
            else if ((bit_offset + size) <= 8U)
            {
                mask_0 = (l_u8)((0xFFU >> (8U - size - bit_offset)) << (8U - size)) >>
                         (8U - size - bit_offset);

                context->config->frames[index].data[byte_offset] &= (l_u8) ~mask_0;
                context->config->frames[index].data[byte_offset] |= ((v << bit_offset) & mask_0);
            }
            else
            {
                mask_0 = 0xFFU << bit_offset;
                mask_1 = 0xFFU >> (16U - bit_offset - size);

                data_0 = &context->config->frames[index].data[byte_offset];
                data_1 = &context->config->frames[index].data[byte_offset + 1U];

                /* Write the LSB part of the value: clear LSB and OR with the left-shifted and
                 * masked value.
                 */
                *data_0 = (*data_0 & ((l_u8) ~mask_0)) | (((l_u8)(v << bit_offset)) & mask_0);

                /* Write MSB part of value: clear MSB and OR with right shifted and masked value  */
                *data_1 = (*data_1 & ((l_u8) ~mask_1)) |
                          (((l_u8)(v >> (8u - bit_offset))) & mask_1);
            }

            mtb_lin_set_et_flag(i, context);
            Cy_SysLib_ExitCriticalSection(interrupt_state);
        }
    }
}


/*******************************************************************************
* Function Name: l_u16_rd
****************************************************************************//**
*
*  Reads and returns the current value of the signal for 9-16-bit size signals.
*  If an invalid signal handle is passed into the function, does nothing.
*
* \param sss
*  The signal handle of the signal to read.
*  See \ref l_signal_handle.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
* \return
*  Returns the current value of the signal.
*
*******************************************************************************/
l_u16 l_u16_rd(l_signal_handle sss, const mtb_stc_lin_context_t* context)
{
    l_u8  index;
    l_u8  byte_offset;
    l_u8  bit_offset;
    l_u8  size;
    l_u8  mask_0;
    l_u8  data_0;
    l_u8  data_1;
    l_u32 interrupt_state;

    /* Will be returned if no signal with the specified handle exists */
    l_u16 result = 0U;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);


    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_SCALAR_U16 == sig.type);

            index      = sig.frame_index;
            size       = sig.size;
            byte_offset = sig.bit_offset / 8U;
            bit_offset  = sig.bit_offset % 8U;

            interrupt_state = Cy_SysLib_EnterCriticalSection();

            data_0 = context->config->frames[index].data[byte_offset];
            data_1 = context->config->frames[index].data[byte_offset + 1U];

            if ((size == 16U) && (bit_offset == 0U))
            {
                result = UNITE_2xU8_TO_U16(data_1, data_0);
            }
            else if (bit_offset == 0U)
            {
                mask_0      = 0xFFU >> (16U - size);
                data_1     &= mask_0;
                result = UNITE_2xU8_TO_U16(data_1, data_0);
            }
            else if ((bit_offset + size) <= 16U)
            {
                mask_0      = 0xFFU >> (16U - size);
                result = (UNITE_2xU8_TO_U16(data_1, data_0) >>  bit_offset) &
                         ((l_u16)((l_u16)mask_0 << 8u) | 0x00FFu);
            }
            else
            {
                l_u8  data_2;
                l_u16 tmp_1;
                l_u8  tmp_2;

                l_u8 shift_0 = 24U - bit_offset - size;
                l_u8 shift_1 = bit_offset - (8U - shift_0);

                mask_0 = 0xFFU >> (16U - size);
                data_2 = context->config->frames[index].data[byte_offset + 2U];

                /* Swap MSB and ISB. Shift it to the left. */
                tmp_1 = UNITE_2xU8_TO_U16(data_2, data_1) << shift_0;

                /* Mask LO8(tmp1) and OR with shifted to the right LSB */
                tmp_2 = ((CY_LO8(tmp_1)) & ((l_u8) ~mask_0)) | (data_0 >> (8u - shift_0));

                result =
                    UNITE_2xU8_TO_U16(CY_HI8(tmp_1), tmp_2) >> shift_1;
            }
            Cy_SysLib_ExitCriticalSection(interrupt_state);
            break;
        }
    }


    return result;
}


/*******************************************************************************
* Function Name: l_u16_wr
****************************************************************************//**
*
*  Writes the value v to the signal.
*  If an invalid signal handle is passed into the function, does nothing.
*
* \param sss
*  The signal handle of the signal to write.
*  See \ref l_signal_handle.
*
* \param v
*  The value of the signal to be set.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_u16_wr(l_signal_handle sss, l_u16 v, const mtb_stc_lin_context_t* context)
{
    l_u8           index;
    l_u8           byte_offset;
    l_u8           bit_offset;
    l_u8           size;
    l_u8           mask_0;
    l_u8           mask_1;
    volatile l_u8* data_0;
    volatile l_u8* data_1;
    l_u32          interrupt_state;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);


    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_SCALAR_U16 == sig.type);

            index      = sig.frame_index;
            size       = sig.size;
            byte_offset = sig.bit_offset / 8U;
            bit_offset  = sig.bit_offset % 8U;

            interrupt_state = Cy_SysLib_EnterCriticalSection();

            data_0 = &context->config->frames[index].data[byte_offset];
            data_1 = &context->config->frames[index].data[byte_offset + 1U];

            if ((size == 16U) && (bit_offset == 0U))
            {
                *data_0 = CY_LO8(v);
                *data_1 = CY_HI8(v);
            }
            else if (bit_offset == 0U)
            {
                mask_0 = 0xFFU >> (16U - size);

                /* Write LSB */
                *data_0 = CY_LO8(v);

                /* Clear the bits to be set */
                *data_1 &= ((l_u8) ~mask_0);

                /* OR masked value */
                *data_1 |= (CY_HI8(v) & mask_0);
            }
            else if ((bit_offset + size) <= 16U)
            {
                mask_0 = 0xFFU << bit_offset;
                mask_1 = 0xFFU >> (16U - bit_offset - size);

                /* Clear MSB with a mask */
                *data_1 &= ((l_u8) ~mask_1);

                /* Write MSB */
                *data_1 |= (l_u8)(CY_LO8((l_u16)(v >> (8U - bit_offset))) & mask_1);

                /* Clear LSB with a mask */
                *data_0 &= ((l_u8) ~mask_0);

                /* Write a new LSB value */
                *data_0 |= CY_HI8((l_u16)(v << (8U + bit_offset)));
            }
            else
            {
                volatile l_u8* data_2;

                data_2 = &context->config->frames[index].data[byte_offset + 2U];

                mask_0 = 0xFFU << bit_offset;
                mask_1 = 0xFFU >> (24U - bit_offset - size);

                *data_0 &= ((l_u8) ~mask_0);
                *data_0 |= CY_LO8((l_u16)(v << bit_offset));

                *data_1 = CY_HI8((l_u16)(v << bit_offset));

                *data_2 &= ((l_u8) ~mask_1);
                *data_2 |=
                    (l_u8)(CY_HI8((l_u16)(((l_u16)(CY_HI8(v))) <<
                                          bit_offset)) & mask_1);
            }

            mtb_lin_set_et_flag(i, context);
            Cy_SysLib_ExitCriticalSection(interrupt_state);
        }
    }
}


/*******************************************************************************
* Function Name: l_bytes_rd
****************************************************************************//**
*
*  Reads and returns the current values of the selected bytes in the signal.
*  The sum of the "start" and "count" parameters must never be greater than the
*  length of the byte array. Note that when the sum of "start" and "count" is
*  greater than the length of the signal byte array then an accidental data is
*  read.
*
*  If an invalid signal handle is passed into the function, no action is done.
*  Assume that a byte array is 8 bytes long, numbered 0 to 7. Reading bytes from
*  2 to 6 from user selected array requires start to be 2 (skipping byte 0 and 1)
*  and count to be 5. In this case byte 2 is written to user_selected_array[0]
*  and all consecutive bytes are written into user_selected_array in ascending
*  order.
*
* \param sss
*  The signal handle of the signal to read.
*  See \ref l_signal_handle.
*
* \param start_byte
*  The first byte to read from.
*
* \param count
*  The number of bytes to read.
*
* \param pData
*  The pointer to the array, in which the data read from the signal is stored.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_bytes_rd(l_signal_handle sss, l_u8 start_byte, l_u8 count, l_u8* pData,
                const mtb_stc_lin_context_t* context)
{
    l_u8  index;
    l_u32 interrupt_state;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);


    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            l_u8 byte_offset = sig.bit_offset / 8U;
            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_BYTE_ARRAY == sig.type);
            CY_ASSERT_L2((count * 8U) <= sig.size);

            index = sig.frame_index;
            interrupt_state = Cy_SysLib_EnterCriticalSection();

            for (l_u8 j = 0U; j < count; j++)
            {
                pData[j] =
                    context->config->frames[index].data[byte_offset + start_byte + j];
            }
            Cy_SysLib_ExitCriticalSection(interrupt_state);
            break;
        }
    }
}


/*******************************************************************************
* Function Name: l_bytes_wr
****************************************************************************//**
*
*  Writes the current value of the selected bytes to the signal specified by the
*   name "sss". The sum of "start" and "count" must never be greater than
*  the length of the byte array, although the device driver may choose not to
*  enforce this in runtime. Note that when the sum of "start" and "count" is
*  greater than the length of the signal byte array then an accidental memory
*  area is to be affected.
*
*  If an invalid signal handle is passed into the function, no action is done.
*  Assume that a byte array signal is 8 bytes long, numbered 0 to 7. Writing
*  byte 3 and 4 of this array requires "start" to be 3 (skipping byte 0, 1 and
*  2) and "count" to be 2. In this case byte 3 of the byte array signal is
*  written from user_selected_array[0] and byte 4 is written from
*  user_selected_array[1].
*
* \param sss
*  The signal handle of the signal to write.
*  See \ref l_signal_handle.
*
* \param start_byte
*  The first byte to write to.
*
* \param count
*  The number of bytes to write.
*
* \param pData
*  The pointer to the array, in which the data to transmit to LIN master is located.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
void l_bytes_wr(l_signal_handle sss, l_u8 start_byte, l_u8 count, const l_u8* const pData,
                const mtb_stc_lin_context_t* context)
{
    l_u16 index;
    l_u32 interrupt_state;

    CY_ASSERT_L2(sss < context->config->num_of_signal_handles);


    for (l_u16 i = 0U; i < context->config->num_of_signals; i++)
    {
        mtb_stc_lin_signal_t sig = context->config->signals[i];

        if (sig.handle == sss)
        {
            l_u8 byte_offset = sig.bit_offset / 8U;

            CY_ASSERT_L2(MTB_LIN_SIGNAL_TYPE_BYTE_ARRAY == sig.type);
            CY_ASSERT_L2((count * 8U) <= sig.size);

            index = sig.frame_index;

            interrupt_state = Cy_SysLib_EnterCriticalSection();

            for (l_u8 j = 0U; j < count; j++)
            {
                context->config->frames[index].data[byte_offset + start_byte + j] = pData[j];
            }

            mtb_lin_set_et_flag(i, context);
            Cy_SysLib_ExitCriticalSection(interrupt_state);
        }
    }
}


/*******************************************************************************
* Function Name: mtb_lin_set_et_flag
****************************************************************************//**
*
* Sets the flag for the event-triggered frame that is associated with the
* unconditional frame where the specified signal is placed.
*
* \param signal_index
*  The signal index.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void mtb_lin_set_et_flag(l_u16 signal_index, const mtb_stc_lin_context_t* context)
{
    volatile mtb_stc_lin_frame_context_t* frame =
        context->config->signals_context[signal_index].associated_frame;

    if (NULL != frame)
    {
        (*frame).et_flag = true;
    }
}


/* [] END OF FILE */
