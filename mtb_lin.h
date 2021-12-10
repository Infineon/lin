/***************************************************************************//**
 * \file mtb_lin.h
 * \version 1.10
 *
 * \brief
 * Provides the LIN middleware common API declarations.
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


/**
 *******************************************************************************
 * \mainpage LIN Middleware
 *
 * The LIN middleware implements LIN 2.2 Slave node for the PSoC 4 devices.
 * Options for the ISO 17987 is also available. The LIN Configurator tool makes
 * it easy to configure the middleware.
 *
 * <b>Features:</b>
 * * Full LIN 2.2 Slave Node implementation
 * * Compliance with ISO 17987 specification
 * * Full transport layer support
 * * Automatic baud rate synchronization
 * * Automatic detection of bus inactivity
 * * Automatic configuration services
 * * Fully implements a Diagnostic Class I Slave Node
 * * Supports implementation of Diagnostic Class II and III Slave Node
 * * Full error detection
 * * \ref section_lin_configurator
 *
 *******************************************************************************
 * \section section_lin_general_description General Description
 *******************************************************************************
 *
 * Include mtb_lin.h to get access to all the functions and other declarations in
 * this middleware. See the \ref section_lin_quick_start to start using the
 * middleware.
 *
 * Refer to the release notes for compatibility information.
 *
 * Refer to the \ref section_lin_spec_conformance section for
 * the API deviation from the specification.
 *
 * The \ref section_lin_configuration_considerations section provides
 * the guidance for all operation modes and use cases.
 *
 *******************************************************************************
 * \section section_lin_quick_start Quick Start Guide
 *******************************************************************************
 *
 * LIN middleware can be used in various Development Environments.
 * Refer to the release notes.
 *
 * This quick start guide is for an environment with the configured:
 * * CAT2 Peripheral Driver Library (mtb-pdl-cat2)
 * * ModusToolbox Hardware Abstraction Layer (mtb-hal-cat2)
 * * Core Library (core-lib)
 *
 * The below steps describe the simplest way of enabling the LIN Slave.
 *
 * \subsection subsection_qsg_step1 STEP 1: Enable the LIN middleware.
 *
 * Launch ModusToolbox Library Manager and enable the LIN middleware.
 * This step is required only if the ModusToolbox IDE is used. Otherwise, ensure
 * the LIN Middleware is included in your project.
 *
 * \subsection subsection_qsg_step2 STEP 2: Generate initialization code.
 *
 * The following code snippet shows the mtbcfg_lin.h header file with the
 * configuration generated by the LIN configurator:
 * \snippet snippet/main.c LIN_CFG_H
 *
 * The following code snippet shows the mtbcfg_lin.c source file with the
 * configuration generated by the LIN configurator:
 * \snippet snippet/main.c LIN_CFG_C
 *
 * \subsection subsection_qsg_step3 STEP 3: Update main.c
 *
 * The following code snippet shows the LIN initialization:
 * \snippet snippet/main.c LIN_INIT
 *
 * \subsection subsection_qsg_step4 STEP 4: Hardware Configuration
 *
 * Use LIN hardware like CY8CKIT-026-CAN and Lin Shield Kit and CY8CKIT-041S-MAX
 * PSoC 4100S Max Pioneer kit.
 *
 * \subsection subsection_qsg_step5 STEP 5: Build and program the device.
 *
 * Connect the device to the LIN Master. Configure LIN Master to send frames and
 * observe updated signals in the LIN Slave application.
 *
 *******************************************************************************
 * \section section_lin_configuration_considerations Configuration Considerations
 *******************************************************************************
 *
 * This section consists of different guides and instruction of how to enable,
 * configure, and use the LIN middleware in a design.
 * As you can see from the \ref section_lin_quick_start section,
 * the settings of the LIN middleware are controlled with
 * the \ref mtb_stc_lin_config_t structure. Please see its description
 * to learn about the parameters and values.
 *
 * The following is a description of the most common use cases along with the
 * configuration structure examples and code snippets.
 * The list of sections under Configuration Considerations:
 *
 * * \ref section_lin_operating_modes
 * * \ref section_lin_interrupt
 * * \ref section_lin_spec_conformance
 * * \ref section_lin_resources
 *
 ********************************************************************************
 * \subsection section_lin_configurator LIN Configurator
 ********************************************************************************
 *
 * The standalone LIN Configurator tool helps configure the middleware. The tool
 * output are generated source and header files. Generated files are mandatory
 * for the middleware operation and must be added to your project. The header
 * file provides access to instances of the LIN configuration structure
 * \ref mtb_stc_lin_config_t.
 *
 * The LIN Configurator tool provides the User Guide, which can be found in the
 * documentation.
 *
 *******************************************************************************
 * \subsection section_lin_operating_modes Operating Modes
 *******************************************************************************
 *
 * The settings of the LIN middleware are controlled by
 * the \ref mtb_stc_lin_config_t structure. See its description
 * to learn about the parameters and values.
 *
 *******************************************************************************
 * \subsection section_lin_interrupt Interrupts
 *******************************************************************************
 *
 * The interrupt is mandatory for LIN middleware operation. The LIN middleware
 * requires two interrupt sources for the proper operation: one for UART operation,
 * other for SysTick timer operation.
 *
 * The Bus Inactivity Timer is based on the SysTick timer. It is configured by
 * default at Reset time to interrupt a period of 1 millisecond. The LIN middleware
 * uses one callback handler (out of five) from the SysTick timer for one instance.
 * The SysTick timer is configured and enabled automatically during initialization.
 *
 * The UART interrupt has the configurable priority. Ensure that the UART
 * priority has a level sufficient for a timely response to events on the bus.
 *
 *******************************************************************************
 * \subsection section_lin_resources Resources
 *******************************************************************************
 *
 * This section describes the software and hardware resources used by the
 * middleware.
 *
 * * For the PSoC 4 devices, the middleware is based on the SCB block.
 *
 * * If Bus Inactivity is enabled, the middleware registers a callback from
 *   PDL SysTick driver.
 *
 * * The project described in the \ref section_lin_quick_start section, compiled
 *   with the GCC compiler's -Og option for CY8C4149AZI-S598 consumes around
 *   33 KB of flash memory and around 800 bytes of RAM memory. The consumption
 *   reported for RAM memory is for the .data and .bss sections, without the
 *   heap, stack and RAM vectors sections.
 *
 *******************************************************************************
 * \subsection section_lin_spec_conformance Specification Conformance
 *******************************************************************************
 *
 * This section describes the deviations from the API defined by the LIN
 * specifications.
 *
 * * The following deviations are applicable to all functions:
 *   - There are no static functions available. The middleware provides only the
 *     dynamic prototypes.
 *   - The interface of each function accepts the pointer to
 *     \ref mtb_stc_lin_context_t as a parameter. The context holds the state of
 *     the LIN interfaces.
 *   - The interface handle parameter of the Interface Management functions is
 *     ignored. The context is used to differentiate between the interfaces instead.
 *
 * * The following functions have the following interface changes:
 *   - The list of parameters and return value of the \ref l_sys_init() function.
 *   - The list of the \ref l_ifc_init() function parameters extended with the
 *     RX and TX pin configuration.
 *   - The return type of the \ref l_ifc_ioctl() function changed from l_u16 to
 *     \ref mtb_lin_status_t for the enhanced status reporting.
 *   - The data length parameter added to the \ref ld_read_by_id_callout()
 *     function.
 *
 * * The deviations from the ISO 17987 specification are as follows:
 *   - The optional big-endian signal encoding variant is not supported.
 *   - The optional SID B8 is not supported.
 *
 *******************************************************************************
 * \subsection section_lin_migration API Changes between PSoC Creator Component and Middleware
 *******************************************************************************
 *
 * This section describes the differences between PSoC Creator LIN component and
 * the LIN middleware API.
 *
 * <table class="doxtable">
 *   <tr>
 *     <th>#</th>
 *     <th>PSoC Creator LIN component</th>
 *     <th>LIN middleware</th>
 *   </tr>
 *   <tr>
 *     <td>1</td>
 *     <td>
 * The result of the \ref MTB_LIN_IOCTL_READ_STATUS operation of the
 * \ref l_ifc_ioctl() is returned by the function.
 *</td>
 *     <td>
 * The result of the \ref MTB_LIN_IOCTL_READ_STATUS operation of the
 * \ref l_ifc_ioctl() is returned via the pointer specified by the pv parameter.
 * The function returns \ref MTB_LIN_STATUS_SUCCESS.
 * </td>
 *   </tr>
 * <tr>
 *     <td>2</td>
 *     <td>
 * No l_sys_irq_disable() and l_sys_irq_restore() implemented.
 *</td>
 *     <td>
 * Added implementation of the \ref l_sys_irq_disable() and \ref l_sys_irq_restore().
 * </td>
 *   </tr>
 * <tr>
 *     <td>3</td>
 *     <td>
 * All the interrupt service routines are initialized inside of the component - no
 * any actions required on the application level.
 *</td>
 *     <td>
 * The communication and inactivity interrupt service routines should be implemented
 * on the application level. Refer to the \ref subsection_qsg_step3 section for
 * the more details.
 * </td>
 *   </tr>
 * </table>
 *
 *******************************************************************************
 * \section section_lin_MISRA MISRA-C Compliance
 *******************************************************************************
 *
 * This section describes MISRA-C:2012 compliance and deviations for the LIN.
 *
 * MISRA stands for Motor Industry Software Reliability Association. The MISRA
 * specification covers a set of 10 mandatory rules, 110 required rules, and
 * 39 advisory rules that apply to the firmware design and has been put together
 * by the Automotive Industry to enhance the quality and robustness of
 * the firmware code embedded in automotive devices.
 *
 * The MISRA specification defines two categories of deviations (see section 5.4
 * of the MISRA-C:2012 specification):
 * * Project Deviations - applicable for a particular class of deviations.
 * * Specific Deviations - applicable for a single instance in a single file.
 *
 * Project Deviations and Specific Deviations are documented later in this section.
 *
 * Specific deviations are also documented in the source code, close to the
 * deviation occurrence. For each deviation, a special macro identifies the
 * relevant rule or directive number, and reason.
 *
 * <h2>Verification Environment</h2>
 * This section provides a MISRA compliance analysis environment description.
 *
 * <table class="doxtable">
 *   <tr>
 *     <th>Component</th>
 *     <th>Name</th>
 *     <th>Version</th>
 *   </tr>
 *   <tr>
 *     <td>Test Specification</td>
 *     <td>MISRA-C:2012 Guidelines for the use of the C language in critical systems</td>
 *     <td>March 2013</td>
 *   </tr>
 *   <tr>
 *     <td rowspan="2">MISRA Checking Tool</td>
 *     <td>Coverity Static Analysis Tool</td>
 *     <td>2020.03</td>
 *   </tr>
 * </table>
 *
 * <h2>Project and Specific Deviations</h2>
 * The list of deviated rules is provided in the table below.
 *
 * <table class="doxtable">
 *   <tr>
 *     <th>MISRA Rule</th>
 *     <th>Required/Advisory</th>
 *     <th>Rule Description</th>
 *     <th>Description of Deviation(s)</th>
 *   </tr>
 *   <tr>
 *     <td>1.2</td>
 *     <td>A</td>
 *     <td>Language extensions are not used</td>
 *     <td>The middleware library supports ISO:C99 standard.</td>
 *   </tr>
 *   <tr>
 *     <td>2.5</td>
 *     <td>A</td>
 *     <td>A project does not contain unused macro definitions</td>
 *     <td>The middleware library provides an API to the hardware.
 *     The macro is part of the API, which is defined for the application-level only.</td>
 *   </tr>
 *   <tr>
 *     <td>3.1</td>
 *     <td>R</td>
 *     <td>The special comment symbols is not used within a comment</td>
 *     <td>Special comment symbols are required for Doxygen comment support,
 *     it does not have any impact.</td>
 *   </tr>
 *   <tr>
 *     <td>8.7</td>
 *     <td>A</td>
 *     <td>Functions and objects are not defined with external linkage
 *     if they are referenced in only one translation unit</td>
 *     <td>The middleware library provides an API to the hardware.
 *     The functions and objects with external linkage are part of the API,
 *     which are defined for application-level only.</td>
 *   </tr>
 *   <tr>
 *     <td>11.4</td>
 *     <td>A</td>
 *     <td>A conversion are not performed between the pointer to an object and an integer type</td>
 *     <td>The cast from an unsigned integer to the pointer does not have any unintended effect,
 *     as it is a consequence of the definition of a structure based on hardware registers.</td>
 *   </tr>
 *   <tr>
 *     <td>11.5</td>
 *     <td>A</td>
 *     <td>A conversion is not performed from the pointer to void into the pointer to an object</td>
 *     <td>See specific deviations documented in the source code, close to the deviation
 * occurrence.</td>
 *   </tr>
 *   <tr>
 *     <td>18.4</td>
 *     <td>A</td>
 *     <td>The +, -, += and -= operators are not applied to an expression of the pointer type</td>
 *     <td>See specific deviations documented in the source code, close to the deviation
 * occurrence.</td>
 *   </tr>
 *   <tr>
 *     <td>Directive 4.8</td>
 *     <td>A</td>
 *     <td>If a pointer to the structure or union is never dereferenced within a translation unit,
 *     then the implementation of the object is hidden</td>
 *     <td>Deviated, since the object provided by HAL is used without dereferencing for the own
 *         needs of LIN middleware.</td>
 *   </tr>
 *   <tr>
 *     <td>Directive 4.9</td>
 *     <td>A</td>
 *     <td>A function is used in preference to a function-like macro where they are
 * interchangeable</td>
 *     <td>Deviated, since function-like macros are used to allow more efficient code.</td>
 *   </tr>
 * </table>
 *
 *******************************************************************************
 * \section section_lin_changelog Changelog
 *******************************************************************************
 *
 * <table class="doxtable">
 *   <tr><th>Version</th><th>Changes</th><th>Reason for Change</th></tr>
 *   <tr>
 *     <td>1.10</td>
 *     <td>Added CAT2 Hardware Abstraction Layers (mtb-hal-cat2) 2.x support./td>
 *     <td></td>
 *   </tr>
 *   <tr>
 *     <td>1.0</td>
 *     <td>Initial Version</td>
 *     <td></td>
 *   </tr>
 * </table>
 *
 *******************************************************************************
 * \section section_lin_more_information More Information
 *******************************************************************************
 *
 * For more information, refer to the following documents:
 *
 * * <a href="https://www.cypress.com/products/modustoolbox-software-environment">
 *      <b>ModusToolbox Software Environment, Quick Start Guide, Documentation,
 *         and Videos</b>
 *   </a>
 *
 * * <a
 * href="https://cypresssemiconductorco.github.io/mtb-pdl-cat2/pdl_api_reference_manual/html/index.html">
 *   <b>CAT2 Peripheral Driver Library</b>
 *   </a>
 *
 * * <a href="https://cypresssemiconductorco.github.io/mtb-hal-cat2/html/modules.html">
 *   <b>ModusToolbox Hardware Abstraction Layer (mtb-hal-cat2)</b>
 *   </a>
 *
 * * <a
 * href="https://www.cypress.com/documentation/development-kitsboards/psoc-4100s-max-pioneer-kit-cy8ckit-041s-max">
 *   <b>PSoC 4100S Max Pioneer Kit (CY8CKIT-041S-Max)</b>
 *   </a>
 *
 * * <a
 * href="https://www.cypress.com/documentation/development-kitsboards/cy8ckit-026-can-and-lin-shield-kit">
 *   <b>CY8CKIT-026 CAN and LIN Shield Kit</b>
 *   </a>
 *
 * * <a href="http://www.cypress.com">
 *      <b>Cypress Semiconductor</b>
 *   </a>
 *
 * \note
 * The links to the other software component's documentation (middleware and PDL)
 * point to GitHub to the latest available version of the software.
 * To get documentation of the specified version, download from GitHub and unzip
 * the component archive. The documentation is available in
 * the <i>docs</i> folder.
 *
 *******************************************************************************
 *
 * \defgroup group_lin_macros Macros
 *
 * \defgroup group_lin_functions Functions
 * \brief
 * This section describes the LIN Slave Function Prototypes.
 * \{
 * \defgroup group_lin_core_API_function Core API function
 * \{
 * \defgroup group_lin_core_api_initialization_function Initialization
 * \defgroup group_lin_core_api_signal_interaction_function Signal Interaction
 * \defgroup group_lin_core_api_notification_function Notification
 * \defgroup group_lin_core_api_interface_management_function Interface Management
 * \defgroup group_lin_core_api_user_callouts User Provided Call-Outs
 * \}
 * \defgroup group_lin_node_configuration_function Node Configuration
 * \defgroup group_lin_transport_layer_function Transport Layer function
 * \{
 * \defgroup group_lin_transport_layer_initialization_function Initialization
 * \defgroup group_lin_transport_layer_raw_function Raw Transport Layer
 * \defgroup group_lin_transport_layer_cooked_function Cooked Transport Layer
 * \}
 * \}
 * \defgroup group_lin_data_structures Data Structures
 * \brief
 * This section describes the data structures defined by the LIN Slave.
 *
 * \defgroup group_lin_enums Enumerated Types
 * \brief
 * This section describes the enumeration types defined by the LIN Slave.
 *
 */


#if !defined(MTB_LIN_H)
#define MTB_LIN_H

#include "cy_device.h"
#include "cy_device_headers.h"
#include "cy_syslib.h"
#include "cy_sysint.h"
#include "cy_scb_uart.h"
#include "cyhal.h"

#include "mtb_lin_types.h"
#include "mtb_lin_ld.h"
#include "mtb_lin_ifc.h"
#include "mtb_lin_ncs.h"
#include "mtb_lin_sig.h"


/*******************************************************************************
*                          Middleware Version
*******************************************************************************/

/** LIN middleware major version */
#define MTB_LIN_MW_VERSION_MAJOR                (1)

/** LIN middleware minor version */
#define MTB_LIN_MW_VERSION_MINOR                (10)

/** LIN middleware version */
#define MTB_LIN_MW_VERSION                      (110)

/* The SCB driver version 3.0 or above required */
#define MTB_LIN_SCB_DRV_VERSION_MAJOR_REQUIRED  (3)

/* Check used driver version */
#if (MTB_LIN_SCB_DRV_VERSION_MAJOR_REQUIRED > CY_SCB_DRV_VERSION_MAJOR)
    #error The LIN Middleware requires the newer version of the PDL. Update the PDL in your project.
#endif


/*******************************************************************************
*  Function Prototypes
*******************************************************************************/
/**
 * \addtogroup group_lin_core_api_initialization_function
 * \{
 */
mtb_lin_status_t l_sys_init(const mtb_stc_lin_config_t* config, mtb_stc_lin_context_t* context,
                            cy_israddress comm_isr, l_u32 comm_isr_priority,
                            cy_israddress inactivity_isr);
/** \} group_lin_core_api_initialization_function */


/**
 * \addtogroup group_lin_node_configuration_function
 * \{
 */
/*******************************************************************************
* Function Name: mtb_lin_register_read_by_id_callback
****************************************************************************//**
*
* Registers a callback function for \ref ld_read_by_id_callout().
* To remove the callback function, pass a NULL as the function pointer.
*
* \param callback
* The pointer to a callback function.
*
* \param context
*  The pointer to the context structure \ref mtb_stc_lin_context_t
*  allocated by the user. This structure is used during LIN Slave operation
*  for internal configuration and data retention. The user must not modify
*  anything in this structure.
*
*******************************************************************************/
__STATIC_INLINE void mtb_lin_register_read_by_id_callback(mtb_cb_lin_read_by_id_t callback,
                                                          mtb_stc_lin_context_t* context)
{
    context->read_by_id_callback = callback;
}


/** \} group_lin_node_configuration_function */

/**
 * \addtogroup group_lin_core_api_user_callouts
 * \{
 */
l_irqmask l_sys_irq_disable(const mtb_stc_lin_context_t* context);
void l_sys_irq_restore(l_irqmask previous, const mtb_stc_lin_context_t* context);
/** \} group_lin_core_api_user_callouts */


/** \cond INTERNAL */
/* Communication IRQ state used by \ref l_sys_irq_disable() and \ref l_sys_irq_restore(). */
#define MTB_LIN_COMMUNICATION_IRQ_ENABLED   (0x01U)
#define MTB_LIN_INACTIVITY_IRQ_ENABLED      (0x02U)
/** \endcond */

#endif /* MTB_LIN_H */

/* [] END OF FILE */
