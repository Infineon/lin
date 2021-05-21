/***************************************************************************//**
 * \file mtb_lin_sig.h
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

#if !defined(MTB_LIN_SIG_H)
#define MTB_LIN_SIG_H

#include "mtb_lin_types.h"

/*******************************************************************************
*  Supplemental macros
*******************************************************************************/
#define UNITE_2xU8_TO_U16(x, y)    (l_u16) ((l_u16) (((l_u16) (x)) << 8U) | (l_u16) (y))


/*******************************************************************************
*            Signal Interaction API
*******************************************************************************/
/**
 * \addtogroup group_lin_core_api_notification_function
 * \{
 */
/* Core API Functions: Notification */
l_bool l_flg_tst(l_flag_handle fff, const mtb_stc_lin_context_t* context);
void l_flg_clr(l_flag_handle fff, const mtb_stc_lin_context_t* context);
/** \} group_lin_core_api_notification_function */

/**
 * \addtogroup group_lin_core_api_signal_interaction_function
 * \{
 */
l_bool l_bool_rd(l_signal_handle sss, const mtb_stc_lin_context_t* context);
void l_bool_wr(l_signal_handle sss, l_bool v, const mtb_stc_lin_context_t* context);

l_u8 l_u8_rd(l_signal_handle sss, const mtb_stc_lin_context_t* context);
void l_u8_wr(l_signal_handle sss, l_u8 v, const mtb_stc_lin_context_t* context);

l_u16 l_u16_rd(l_signal_handle sss, const mtb_stc_lin_context_t* context);
void l_u16_wr(l_signal_handle sss, l_u16 v, const mtb_stc_lin_context_t* context);

void l_bytes_rd(l_signal_handle sss, l_u8 start_byte, l_u8 count, l_u8* pData,
                const mtb_stc_lin_context_t* context);
void l_bytes_wr(l_signal_handle sss, l_u8 start_byte, l_u8 count, const l_u8* const pData,
                const mtb_stc_lin_context_t* context);
/** \} group_lin_core_api_signal_interaction_function */


#endif /* MTB_LIN_IFC_H */

/* [] END OF FILE */
