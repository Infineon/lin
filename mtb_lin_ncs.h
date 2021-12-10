/***************************************************************************//**
 * \file mtb_lin_ncs.h
 * \version 1.10
 *
 * \brief
 * Provides the LIN middleware node configuration services API
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

#if !defined(MTB_LIN_NCS_H)
#define MTB_LIN_NCS_H

#include "mtb_lin_types.h"

/*******************************************************************************
*            Node Configuration Services Internal API
*******************************************************************************/
/**
 * \addtogroup group_lin_functions
 * \{
 */
/** \cond INTERNAL */
void mtb_lin_service_assign_nad(mtb_stc_lin_context_t* context);
void mtb_lin_service_read_by_id(mtb_stc_lin_context_t* context);
void mtb_lin_service_cond_change_nad(mtb_stc_lin_context_t* context);
void mtb_lin_service_save_config(mtb_stc_lin_context_t* context);
void mtb_lin_service_frame_id_range(mtb_stc_lin_context_t* context);
/** \endcond */
/** \} group_lin_functions */

/* Positive responses for Node Configuration Services requests */
#define MTB_LIN_NCS_POS_RESP_ASSIGN_NAD             (0xF0U)
#define MTB_LIN_NCS_POS_RESP_ASSIGN_FRAME_ID        (0xF1U) /* Used only in LIN 2.0 */
#define MTB_LIN_NCS_POS_RESP_READ_BY_ID             (0xF2U)
#define MTB_LIN_NCS_POS_RESP_COND_CHANGE_NAD        (0xF3U)
#define MTB_LIN_NCS_POS_RESP_DATA_DUMP              (0xF4U) /* Not supported */
#define MTB_LIN_NCS_POS_RESP_ASSIGN_NAD_SNPD        (0xF5U) /* Not supported */
#define MTB_LIN_NCS_POS_RESP_SAVE_CONFIG            (0xF6U)
#define MTB_LIN_NCS_POS_RESP_ASSIGN_FRAME_ID_RANGE  (0xF7U)

#endif /* MTB_LIN_NCS_H */

/* [] END OF FILE */
