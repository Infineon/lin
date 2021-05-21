/***************************************************************************//**
 * \file mtb_lin_types.h
 * \version 1.0
 *
 * \brief
 * Provides the LIN middleware common API data type declarations.
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


#if !defined(MTB_LIN_TYPES_H)
#define MTB_LIN_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#include "cyhal.h"  /* cyhal_gpio_t, cyhal_gpio_t, cy_stc_sysint_t, and cyhal_uart_t */
#include "cy_result.h"


/*******************************************************************************
*                        LIN Core Types
*******************************************************************************/
/**
 * \addtogroup group_lin_enums
 * \{
 */

/** The status of initialization of LIN middleware */
typedef enum
{
    /** Operation
     * completed successfully */
    MTB_LIN_STATUS_SUCCESS            = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 0U),

    /** One or more input parameters
     * are invalid */
    MTB_LIN_STATUS_BAD_PARAM          = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 1U),

    /** The size of the signals
     * with the same handle ID
     * is not equal */
    MTB_LIN_STATUS_SIGNAL_HANDLE      = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 2U),

    /** The value of frame_index field
     * in signals array is greater or equal
     * to number of frames */
    MTB_LIN_STATUS_SIGNAL_FRAME_ID    = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 3U),

    /** The value of the bit_offset field
     * in signals array is above 64 */
    MTB_LIN_STATUS_SIGNAL_BIT_OFFSET  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 4U),

    /** The value of the size field
     * in the signals array is incorrect
     * for corresponding signal type:
     * for MTB_LIN_SIGNAL_TYPE_SCALAR_BOOL must be 1 bit;
     * for MTB_LIN_SIGNAL_TYPE_SCALAR_U8:   must be 2 - 8 bits;
     * for MTB_LIN_SIGNAL_TYPE_SCALAR_BOOL: must be 9 - 16 bits;
     * for MTB_LIN_SIGNAL_TYPE_SCALAR_BOOL: must be 1 - 8 bytes
     * */
    MTB_LIN_STATUS_SIGNAL_SIZE        = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 5U),

    /** The mapping of the signals into the frame
     * is incorrect, please check the frame size or
     * signals overlap conditions */
    MTB_LIN_STATUS_SIGNAL_PLACING     = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 6U),

    /** The MTB_LIN_IOCTL_SLEEP command of the l_ifc_ioctl() fails */
    MTB_LIN_STATUS_IFC_IOCTL_WAKEUP  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                      CY_RSLT_MODULE_MIDDLEWARE_LIN, 7U),

    /** The auto baud rate sync is disabled. */
    MTB_LIN_STATUS_AUTO_BAUD_RATE_SYNC_DISABLED  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                                  CY_RSLT_MODULE_MIDDLEWARE_LIN,
                                                                  8U),

    /** The specified l_ifc_ioctl()'s command is not supported within current configuration. */
    MTB_LIN_STATUS_IFC_IOCTL_UNSUPPORTED_COMMAND  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                                   CY_RSLT_MODULE_MIDDLEWARE_LIN,
                                                                   9U),

    /** The invalid NAD was specified. */
    MTB_LIN_STATUS_INVALID_NAD  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_MIDDLEWARE_LIN,
                                                 10U),

    /** The invalid PID index was specified to the MTB_LIN_IOCTL_GET_FRAME_PID command of the \ref
       l_ifc_ioctl() function */
    MTB_LIN_STATUS_INVALID_PID_INDEX  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 11U),

    /** The invalid message ID was specified. */
    MTB_LIN_STATUS_INVALID_MESSAGE_ID  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                        CY_RSLT_MODULE_MIDDLEWARE_LIN, 12U),

    /** The frequency of the high-frequency clock for LIN operation is too low. Returns the status
       of
     * \ref l_sys_init().
     */
    MTB_LIN_STATUS_TOO_LOW_FREQUENCY  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_LIN, 13U),

    /** A combination of frequency of the high-frequency clock and LIN baud rate does not allow
       achieving
     * acceptable tolerance of the baudrate. Returns the status of \ref l_sys_init().
     */
    MTB_LIN_STATUS_BAD_TOLERANCE  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                   CY_RSLT_MODULE_MIDDLEWARE_LIN, 14U),

    /** The specified baud rate is out of range. */
    MTB_LIN_STATUS_BAD_BAUDRATE  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_MIDDLEWARE_LIN,
                                                  15U),

    /** An incorrect combination of protocol specifications. */
    MTB_LIN_STATUS_PROTOCOL_SPECIFICATION  = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                            CY_RSLT_MODULE_MIDDLEWARE_LIN, 16U)
} mtb_lin_status_t;

/** Slave Node states as defined in LIN 2.2a, section 5.5 */
typedef enum
{
    /** In this state, Slave node neither receives nor transmits any
     *  messages in the cluster. It is consequently available for any incoming
     * request from Master node. It will not respond to Slave response
     * frames.
     */
    MTB_LIN_NODE_STATE_IDLE = 0U,

    /** In this state, Slave node receives a transmission from Master
     * node. It receives and processes the transport layer frames as
     * received from Master node. Slave node will ignore any
     * interleaved functional addressed transmission from Master node.
     */
    MTB_LIN_NODE_STATE_RX_PHY_REQUEST = 1U,

    /** In this state, Slave node is currently still processing the previously
     * received request, is ready to transmit a physical response or is actually
     * transmitting the response to the previously received request. Slave
     * node will not receive nor process interleaved functional addressed
     * (NAD 0x7E) transmissions from Master node. Physical transmissions
     * will be received and will make Slave node discard the current
     * request data or response data. If the request is addressed to Slave
     * node the request will be received and processed.
     */
    MTB_LIN_NODE_STATE_TX_PHY_RESPONSE = 2U
} mtb_stc_lin_node_state_t;


/** The protocol specification */
typedef enum
{
    MTB_LIN_SPEC_2_2 = 0U    /**< LIN 2.2 Compatibility */
} mtb_stc_lin_spec_t;

/** The frame direction */
typedef enum
{
    MTB_LIN_FRAME_DIR_SUBSCRIBE = 0U,        /**< Subscribe direction */
    MTB_LIN_FRAME_DIR_PUBLISH   = 1U         /**< Publish direction */
} mtb_stc_lin_frame_direction_t;

/** The type of the LIN frame */
typedef enum
{
    MTB_LIN_FRAME_TYPE_UNCONDITIONAL = 0U,   /**< Unconditional frame */
    MTB_LIN_FRAME_TYPE_EVENT_TRIGGERED      = 1U    /**< Event-Triggered frame */
} mtb_stc_lin_frame_type_t;

/** The type of the signal */
typedef enum
{
    MTB_LIN_SIGNAL_TYPE_SCALAR_BOOL = 0U,    /**< One-bit signal */
    MTB_LIN_SIGNAL_TYPE_SCALAR_U8   = 1U,    /**< Scalar signal with length 2 - 8 bits */
    MTB_LIN_SIGNAL_TYPE_SCALAR_U16  = 2U,    /**< Scalar signal with length 9 - 16 bits */
    MTB_LIN_SIGNAL_TYPE_BYTE_ARRAY  = 3U     /**< ByteArray signal with length 1-8 bytes */
} mtb_stc_lin_signal_type_t;

/** The transport layer API format */
typedef enum
{
    MTB_LIN_TL_FORMAT_COOKED = 0U,   /**< Cooked Transport Layer API */
    MTB_LIN_TL_FORMAT_RAW    = 1U    /**< Raw Transport Layer API */
} mtb_stc_lin_tl_format_t;

/** The type of import file for LIN Slave Node configuration */
typedef enum
{
    MTB_LIN_FILE_VERSION_SOURCE_LDF = 1U,  /**< LIN Description File */
    MTB_LIN_FILE_VERSION_SOURCE_NCF = 2U   /**< Node Capability File */
} mtb_stc_lin_file_version_source_t;


/** Slave Node states as defined in LIN 2.2a, section 5.5 */
typedef enum
{
    /** A transport layer error occurred or a master request frame with an
     * NAD different from Slave node's own NAD was received.
     */
    MTB_LIN_NODE_STIMULUS_MRF_ALIEN_NAD = 1U,

    /** A master request frame has been received with the NAD matching the lave
     * node's own NAD.
     */
    MTB_LIN_NODE_STIMULUS_MRF_OWN_NAD = 2U,

    /** A master request frame with the NAD parameter set to the functional NAD
     * has been received.
     */
    MTB_LIN_NODE_STIMULUS_MRF_FUNC_NAD = 3U,

    /** The time until reception of the next Consecutive Frame (CF) (N_CR) timeout
     * expired.
     */
    MTB_LIN_NODE_STIMULUS_RX_TIMEOUT = 4U,

    /** The time for transmission of the LIN frame (MRF or SRF) on the transmitter
     * side (N_AS) timeout expired.
     */
    MTB_LIN_NODE_STIMULUS_TX_TIMEOUT = 5U,

    /** A slave response frame was received. */
    MTB_LIN_NODE_STIMULUS_SRF = 6U
} mtb_stc_lin_node_state_stimulus_t;
/** \} group_lin_enums */

/**
 * \addtogroup group_lin_data_structures
 * \{
 */

/**
 * \addtogroup group_lin_data_structures_data_types Data types
 * \{
 */
/** Provides the typedef for the bool */
typedef bool     l_bool;
/** Provides the typedef for the unsigned 8-bit integer */
typedef uint8_t  l_u8;
/** Provides the typedef for the unsigned 16-bit integer */
typedef uint16_t l_u16;
/** Provides the typedef for the unsigned 32-bit integer */
typedef uint32_t l_u32;
/** Provides the typedef for \ref l_ifc_ioctl() function operation */
typedef uint8_t  l_ioctl_op;
/** Provides the typedef for the signal handle */
typedef uint8_t  l_signal_handle;
/** Provides the typedef for the interface handle */
typedef uint8_t  l_ifc_handle;
/** Provides the typedef for the flag handle */
typedef l_u32 l_flag_handle;
/** Provides the typedef for the interrupt state */
typedef l_u32 l_irqmask;
/** \} group_lin_data_structures_data_types */


/** Frame config structure.
 * The MRF and SRF do not require instances of the structure defined.
 */
typedef struct
{
    /** Specifies the default frame PID that the frame will use before any
     * configuration requests by the master.
     */
    l_u8 initial_pid;

    /** Specifies the direction of the frame */
    mtb_stc_lin_frame_direction_t direction;

    /** Sets the size of the frame in bytes */
    l_u8 size;

    /** Specifies the type of the frame */
    mtb_stc_lin_frame_type_t type;

    /** The pointer to the array, in which the frame data is located */
    volatile l_u8* data;

    /** Associate unconditional frames with event-triggered frames.
     *
     * An event-triggered frame must have at least one unconditional frame that
     * is associated with it, according to the LIN specification. Only one
     * unconditional frame can be associated with an event-triggered frame. An
     * event-triggered frame that is associated with an unconditional frame must
     * have the same length and direction as the unconditional frame with which
     * it is associated.
     *
     * \note Only applicable for event-triggered frames.
     */
    l_u8 associated_frame_index;

    /** Sets the initial message ID value.
     * \note Only applicable for LIN 2.0 specification.
     */
    l_u16 initial_message_id;
} mtb_stc_lin_frame_t;


/** Frame context structure
 * All fields for the context structure are internal. Firmware never reads or
 * writes these values. Firmware allocates the structure and provides the
 * address of the structure to the driver in function calls. Firmware must
 * ensure that the defined instance of this structure remains in scope
 * while the drive is in use.
 *
 * Refer to the frames_context description for the more details.
 */
typedef struct
{
    /** \cond INTERNAL */
    /** Hold the run time value of the message ID value.
     * \note Only applicable for LIN 2.0 specification.
     */
    l_u16 configured_message_id;

    /** Holds the run time value of the frame PID. Initialized by
     * \ref l_ifc_init() with the initialPID.
     */
    l_u8 configured_pid;

    /** The event-triggered flag. */
    l_bool et_flag;

    /** The flag for the frame. If any of the signals on frame are updated, the
     * flag is raised. For clearing the flag use \ref l_flg_clr. */
    l_bool flag;
    /** \endcond */
} mtb_stc_lin_frame_context_t;


/** Signal config structure */
typedef struct
{
    /** Specifies the signal handle. The same signal packed into multiple frames
     * have the same handle value. Used as an input for the signal interaction
     * function.
     */
    l_u8 handle;

    /** Specifies the frame in which a signal will be located */
    l_u8 frame_index;

    /** Defined the position of the signal in the frame (number of bits in
     * offset from the first bit in the frame). For a byte array, both size and
     * offset must be multiples of eight.
     */
    l_u8 bit_offset;

    /** Specifies the type of the signal. Refer to \ref mtb_stc_lin_signal_type_t */
    mtb_stc_lin_signal_type_t type;

    /** Sets the size of signal. The allowable signal size depends on the
     * signal type \ref mtb_stc_lin_signal_type_t */
    l_u8 size;
} mtb_stc_lin_signal_t;


/** Signal context structure
 * All fields for the context structure are internal. Firmware never reads or
 * writes these values. Firmware allocates the structure and provides the
 * address of the structure to the driver in function calls. Firmware must
 * ensure that the defined instance of this structure remains in scope
 * while the drive is in use.
 */
typedef struct
{
    /** \cond INTERNAL */

    /** A flag is set when the frame/signal is considered to be transmitted
     * respectively received, see Section 2.2.4 of the LIN API Specification
     * Revision 2.2A. Used by signals/frames notifications functions
     * \ref l_flg_clr() and \ref \ref l_flg_tst(). */
    l_bool flag;

    /** The pointer to the event-triggered frame that is associated with the
     * unconditional frame where the specified signal is placed. Set to NULL if
     * no such frame exists. Initialized in \ref l_ifc_init().
     */
    volatile mtb_stc_lin_frame_context_t*  associated_frame;

    /** \endcond */
} mtb_stc_lin_signal_context_t;


/** NCF/LDF file version */
typedef struct
{
    l_u8 major; /**< The Major Version of LDF/NFC definition ID */
    l_u8 minor; /**< The Minor Version of LDF/NFC definition ID */
    l_u8 sub;   /**< The Sub Version of LDF/NFC definition ID */

    /** The type of an import file for LIN Slave Node configuration */
    mtb_stc_lin_file_version_source_t source;
} mtb_stc_lin_file_version_t;

/** LIN Slave config structure */
typedef struct
{
    /** Specifies the protocol specification \ref mtb_stc_lin_spec_t */
    mtb_stc_lin_spec_t spec;

    /** Enable compliant with ISO 17987.
     * \note Only applicable for LIN 2.2 specification.
     */
    bool iso17987;

    /** Enable the bus inactivity feature. After a specified time of bus inactivity,
     * the corresponding status bit is set. The value of this bit can be obtained
     * by the \ref MTB_LIN_IOCTL_READ_STATUS of the \ref l_ifc_ioctl().
     */
    bool inactivity_enabled;

    /** Sets the minimal timeout value.
     * \note The minimal timeout value in this field is restricted to 4000 milliseconds,
     * as defined in the ISO 17987 specification.
     */
    l_u32 inactivity_threshold;

    /** Sets the Slave node break detection threshold. See section 5.2.2.3. of the ISO
     * 17987-3 specification for more information about break detection threshold
     * selection criteria.
     *
     * \note For the PSoC 4000S, PSoC 4100S, PSoC 4100S Plus, and PSoC 4500S series,
     * the actual threshold may be up to one bit less than the specified threshold.
     * For example, if the value of the break_threshold is 11, the actual threshold
     * will be from 10 to 11 bits.
     */
    l_u32 break_threshold;

    /** Enable or disable the automatic baud rate synchronization. If this option
     * is enabled, the LIN middleware measures the exact baud rate of the
     * bus from the sync byte field of each LIN frame header.
     *
     * If this option is disabled, the LIN middleware does not measure the
     * baud rate from the sync byte field. Instead, it receives the sync byte field
     * as a 0x55 data byte.
     *
     * As required by the ISO 17987-4 specification, LIN Slave nodes with a frequency
     * deviation of +/-1.5 percent or less do not need to use the automatic baud rate
     * synchronization to measure the sync byte field of each frame. However, if the
     * frequency deviation of the LIN Slave node is more than +/-1.5 percent, then
     * Slave node must use automatic baud rate synchronization to measure the sync
     * byte field of each frame.
     *
     * Therefore, frequency deviation specifications must be checked for the clock
     * source which connected to SCB block.
     */
    bool auto_baud_rate_sync;

    /** Sets the nominal LIN bus baud rate at which this LIN Slave node must operate. */
    l_u32 baud_rate;

    /** Enable Transport Layer */
    bool tl_enabled;

    /** This field is used to select the Network Address (NAD) of Slave node. The
     * NAD is used in MRF and SRF frames to address one particular Slave node in a cluster.
     * Note that this field is used to select the Initial NAD for the node. The NAD of a
     * Slave node can change at run time.
     *
     * By default, the Initial NAD value can be in the range from 0x01 to 0xFF. The NAD
     * value of 0x00 is reserved for a "Go To Sleep" command. The NAD value of 0x7E is
     * reserved as a "Functional NAD" which is used for diagnostic services. The NAD
     * value of 0x7F is reserved as a "wildcard" NAD. Therefore, the next value is not
     * allowed: 0x00, 0x7E, or 0x7F.
     */
    l_u8 initial_nad;

    /** Sets the format for the Transport Layer API functions.
     *
     * The cooked format is used to send and receive Transport Layer messages using just
     * one API function for each message \ref group_lin_transport_layer_cooked_function.
     * The raw format is used to send or receive each frame that makes up a Transport
     * Layer message using one API function call for each frame
     * \ref group_lin_transport_layer_raw_function.
     */
    mtb_stc_lin_tl_format_t tl_api_format;

    /** Maximum Message Length
     *
     * Sets the maximum Transport Layer message length that this Slave node supports.
     * The minimum value is 6, because there are up to six Transport Layer message data
     * bytes in messages that use only one frame. This middleware only supports
     * Transport Layer messages with lengths up to 4095 bytes.
     *
     * \note Note that the actual Transport Layer message buffer is located in
     * the application code of the node.
     */
    l_u32 tl_buf_len_max;

    /** TX Queue Length.
     *
     * When using the raw API format, there is a message queue that buffers the
     * frame response data that is being sent or received. If the slave cannot
     * update the queues very quickly, then the queue lengths must be made
     * longer.
     *
     * If Slave can update the queues very quickly, then the queues can be
     * made shorter to decrease RAM memory use. The middleware supports queue
     * lengths from 8 to 2048 with 8-byte steps.
     *
     * \note Only applicable when the Raw Transport Layer API format is selected.
     */
    l_u32 tl_tx_queue_len;

    /** RX Queue Length.
     *
     * Refer to TX Queue Length description above.
     *
     * \note Only applicable when the Raw Transport Layer API format is selected.
     */
    l_u32 tl_rx_queue_len;

    /** The Master Request Frame (MRF) buffer.
     * The buffer size is defined by \ref tl_rx_queue_len.
     * \note Only applicable when the Raw Transport Layer API format is selected.
     */
    volatile l_u8* tl_raw_rx_queue;

    /** The Slave Response Frame (SRF) buffer.
     * The size is defined by \ref tl_tx_queue_len.
     * \note Only applicable when the Raw Transport Layer API format is selected.
     */
    volatile l_u8* tl_raw_tx_queue;

    /** Automatic Configuration Request Handling
     *
     * The middleware is designed so that it automatically handles configuration
     * service requests. In other words, you do not have to use any API or
     * application code to service these requests from Master. However, you
     * can disable this automatic handling and handle these requests with your
     * own custom application code.
     *
     * If the option is set to "true", any service that is enabled is
     * automatically handled by the middleware. Whenever any of these
     * automatically handled requests occur during LIN bus operation,
     * the corresponding MRF and SRF frames will not be available to the
     * application through the Transport Layer API.
     *
     * If a service request is not automatically handled (the auto_config_request_handling is
     * set to false), then the corresponding MRF and SRF frames of the
     * configuration service request must be received or sent by the application
     * using the Transport Layer API.
     */
    bool auto_config_request_handling;

    /** Sets the Supplier ID.
     *
     * The Supplier ID is a 16-bit value, but its valid range is from 0x0000 to
     * 0x7FFE. Used in the configuration service requests to differentiate
     * between the different slave nodes in a LIN cluster.
     *
     * \note Used if Automatic Configuration Request Handling is enabled.
     */
    l_u16 supplier_id;

    /** Sets the Function ID.
     *
     * The Function ID is also 16 bits, and its valid range is 0x0000 to 0xFFFE.
     * Used in the configuration service requests to differentiate between the
     * different slave nodes in a LIN cluster.
     *
     * \note Used if Automatic Configuration Request Handling is enabled.
     */
    l_u16 function_id;

    /** Sets the Variant
     *
     * The Variant is 8 bits and its valid range is from 0x00 to 0xFF. Used in
     * the configuration service requests to differentiate between the different
     * slave nodes in a LIN cluster.
     *
     * \note Used if Automatic Configuration Request Handling is enabled.
     */
    l_u8 variant;


    /** Enable Service 0xB0 - "Assign NAD"
     *
     * This is an optional service in the ISO 17987 specification. This is a service
     * request where a new NAD value is assigned to the slave node.
     */
    bool service_assign_nad;

    /** Enable Service 0xB1 - "Assign Frame Identifier"
     *
     * This is an obsolete service in the ISO 17987 specification.
     *
     * This configuration service request is used to change the frame ID value for
     * a frame to which this slave node responds.
     *
     * \note Only applicable for LIN 2.0 specification.
     */
    bool service_assign_frame_id;

    /** Enable Service 0xB2 - "Read by identifier"
     *
     * This configuration service request is mandatory according to the ISO 17987
     * specification. This request is used to allow the LIN master to read
     * Slave's identification information (Supplier ID, Function ID, Variant).
     *
     * LIN middleware supports:
     * * The LIN Product Identification version
     * * Serial number
     * * Optional NCF/LDF version for ISO 17987 nodes
     * * "Message ID" parameter for LIN 2.0 nodes of this request.
     */
    bool service_read_by_id;

    /** Enable Service 0xB3 - "Conditional Change NAD"
     *
     * This is an optional service in the LIN 2.2 and ISO 17987 specifications.
     *
     * This is very similar to the Assign NAD configuration service. One major
     * difference is that this service uses Slave's current (volatile) NAD
     * instead of the initial (nonvolatile) NAD. When this request occurs,
     * Slave does some logic processing on the data bytes received from the master
     * and only updates its current (volatile) NAD if the result of the processing
     * is zero.
     */
    bool service_cond_change_nad;

    /** Enable Service 0xB4 - "Data Dump"
     *
     * This service is reserved for initial configuration of a slave node by
     * Slave node supplier and the format of this message is application specific
     * and is not supported by this LIN middleware. Received data is
     * transferred to the application with the transport layer.
     */
    bool service_data_dump;

    /** Enable Service 0xB5 - "Assign NAD via SNPD" (Targeted Reset) */
    bool service_target_reset;

    /** Enable Service 0xB6 - "Save Configuration"
     *
     * This is an optional service request in the ISO 17987 specification.
     *
     * The Slave device can save its configuration data (NAD value and PID values)
     * in nonvolatile memory (flash). However, the application code must implement
     * the actual flash writing operations.
     *
     * When this configuration service request occurs, the Save Configuration flag
     * in the status returned by the \ref l_ifc_read_status() API function is set.
     * This lets the application know that it must save its current LIN slave node
     * configuration information to nonvolatile memory (flash).
     */
    bool service_save_config;

    /** Enable Service 0xB7 - "Assign frame identifier range"
     *
     * This is a mandatory configuration service request in the ISO 17987 specification.
     *
     * This service allows the LIN master to change the volatile frame PID values for
     * Slave's frames.
     */
    bool service_assign_frame_id_range;

    /** Specifies the number of frames */
    l_u8 num_of_frames;

    /** Specifies the number of the signals. Matches to the number of the
     * signals in the array pointed by \ref mtb_stc_lin_signal_t.  */
    l_u16 num_of_signals;

    /** Specifies the number of the signal handles. Matches the
     *  \ref num_of_signals if no signals with the same name defined. The signals
     *  with the same name has different index in the array of the signals (for
     *  example, mtb_lin_0_signals), but same handle defined. If no signals with
     *  the same name defined, this value should match \ref num_of_signals.
     */
    l_u16 num_of_signal_handles;

    /** Sets the response error signal handler */
    l_signal_handle resp_error_signal_handle;

    /** The pointer to the array of the frames. */
    volatile const mtb_stc_lin_frame_t* frames;

    /** The pointer to the array of the signals. */
    volatile const mtb_stc_lin_signal_t* signals;

    /** The pointer to the frames' context. The size of the context equals the
     * number of the supported frames. If the Transport layer is enabled, the MRF and
     * SRF frames are taken into account. */
    volatile mtb_stc_lin_frame_context_t* frames_context;

    /** The pointer to the array of the signals. */
    volatile mtb_stc_lin_signal_context_t* signals_context;

    /** "true" if LDF/NCF file version is specified. See ISO 17987-2, Table 19.
     * If no file version defined, this member is set to "false".
     */
    bool file_rev_defined;

    /** The pointer to the structure that holds LDF/NCF file version */
    const mtb_stc_lin_file_version_t* file_rev;
} mtb_stc_lin_config_t;


/**
 * \addtogroup group_lin_data_structures_function_pointers Function Pointers
 * \{
 */
/**
 * The type is used to define a callback for \ref ld_read_by_id_callout().
 */
typedef l_u8 (* mtb_cb_lin_read_by_id_t)(l_ifc_handle iii, l_u8 id,
                                         l_u8* frame_data, l_u8* data_length);
/** \} group_lin_data_structures_function_pointers */


/** LIN Slave context structure.
 * All fields for the context structure are internal. Firmware never reads or
 * writes these values. Firmware allocates the structure and provides the
 * address of the structure to the driver in function calls. Firmware must
 * ensure that the defined instance of this structure remains in scope
 * while the drive is in use.
 */
typedef struct
{
    /** \cond INTERNAL */
    const mtb_stc_lin_config_t* config;  /**< LIN config structure */
    cy_israddress              comm_isr;     /**< Interrupt service routine for SCB block */
    l_u32                      comm_isr_priority; /**< Interrupt priority for UART */
    cy_israddress              inactivity_isr; /**< Interrupt service routine for SysTick timer */

    /** The clock divider value. Calculated as system clock frequency divided by
     * the specified nominal baud rate (baud_rate member) and oversampling rate that is
     * hardcoded to 16.
     */
    l_u32 initial_clock_divider;

    /** The run time value of the clock divider value.
     */
    l_u32 configured_clock_divider;

    /** The run time value of the Network Address (NAD).
     */
    l_u8 configured_nad;

    cy_stc_sysint_t comm_isr_config;     /**< Communication interrupt configuration */
    cyhal_uart_t    scb_uart_obj; /**< HAL UART structure */
    cyhal_clock_t   scb_uart_clk; /**< HAL clock structure for UART */
    cyhal_gpio_t    tx_pin;      /**< The TX pin name */
    cyhal_gpio_t    rx_pin;      /**< The RX pin name */

    l_u32 period_counter;        /**< Free-running timer */

    l_u8  time_counts_max_deviation;   /**< Maximum deviation for LIN transfer */
    l_u32 corrected_clock_divider;    /**< Corrected the clock divider */

    l_u8  status;               /**< Internal Status */
    l_u16 sync_counts;           /**< Sync Field Timer Counts */
    l_u16 break_to_sync_counts;    /**< Break to Sync timeout counter */

    l_u16 break_to_sync_counts_1;   /**< Baud detect counts on timeout + 1ms */
    l_u16 break_to_sync_counts_2;   /**< Baud detect counts on timeout + 1ms */
    l_u8  aux_status;            /**< Internal AUX ISR shadow status */
    l_u16 ifc_ioctl_status;          /**< Status used by l_ifc_ioctl() */
    l_u16 ifc_comm_status;            /**< Interface communication status */
    l_u8  uart_fsm_state;         /**< Current state of the UART ISR */
    l_u8  uart_fsm_flags;             /**< Current flags of the UART ISR  */

    l_u8 mrf_buffer[8U];    /**< The buffer used for the Master request frame storage */
    l_u8 srf_buffer[8U];    /**< The buffer used for the Slave response frame storage */

    l_u8* serial_number; /**< Serial Number */

    l_u8 tl_tx_status;    /**< Transport Layer RX Status */
    l_u8 tl_rx_status;    /**< Transport Layer TX Status */

    l_u8 tl_flags;   /**< Flags that are used for the Transport Layer */

    /** Slave Node states as defined in LIN 2.2a, section 5.5 */
    mtb_stc_lin_node_state_t node_state;

    /** The internal variable used to store the PCI of the previously transmitted frame */
    l_u8 tl_tx_prev_pci;

    /** The internal variable used to store the PCI of the previously received frame */
    l_u8 tl_rx_prev_pci;

    l_u16 tl_tx_message_length;   /**< TX message length */
    l_u16 tl_rx_message_length;   /**< RX message length */
    l_u8  tl_tx_frame_counter;    /**< TX frames counter */
    l_u8  tl_rx_frame_counter;    /**< RX frames counter */
    l_u32 tl_timeout_counter;      /**< Timeout counter */

    /* Cooked TL API */
    const   l_u8* tl_tx_data_pointer;   /**< Cooked TL TX pointer  */
    l_u16         tl_tx_data_count;     /**< Cooked TL RX pointer  */
    l_u8*         tl_rx_data_pointer;   /**< The pointer to the size of RX data */
    l_u8*         tl_rx_initial_data_pointer; /**< Initial cooked TL pointer */
    l_u8*         tl_nad_pointer;      /**< The pointer to the initial NAD. */
    l_u16*        tl_length_pointer;   /**< Cooked TL length pointer */

    /**
     * The callback for the \ref ld_read_by_id_callout() function.
     * Registered by \ref mtb_lin_register_read_by_id_callback().
     *
     */
    mtb_cb_lin_read_by_id_t read_by_id_callback;

    /* Raw TL API */
    l_u8 tl_raw_tx_buf_depth;     /**< TX buffer depth index */
    l_u8 tl_raw_rx_buf_depth;     /**< RX buffer depth index */

    l_u16 tl_raw_tx_write_index;     /**< TX write buffer index */
    l_u16 tl_raw_tx_read_index;     /**< TX read buffer index */

    l_u16 tl_raw_rx_write_index;     /**< RX write buffer index */
    l_u16 tl_raw_rx_read_index;     /**< RX read buffer index */

    l_u16 interim_checksum;      /**< Holds the interim checksum value */
    l_u8  current_frame_pid;      /**< PID of the current frame */
    l_u8  frame_size;            /**< The size of the frame being processed */
    l_u8  bytes_transferred;     /**< The number of transferred bytes */
    l_u8  tmp_data;              /**< Used to store the transmitted byte */
    l_u8  current_frame_index;    /**< Index in pidInfoTable */
    l_u8  tmp_rx_frame_data[8U];   /**< RXed data before checksum checked */
    l_u8  tmp_tx_frame_data[8U];   /**< TX data buffer for signal-carrying frames */
    volatile l_u8* frame_data;   /**< The pointer to frame data, points to byte to be sent */
    l_bool  frame_error_after_pid;   /**< True for first framing error after PID */
    /** \endcond */
} mtb_stc_lin_context_t;
/** \} group_lin_data_structures */


/******************************************************************************
*     Data Types Definitions
******************************************************************************/

/** Defines the type of the l_ifc_ioctl() parameter for the
 * \ref MTB_LIN_IOCTL_GET_FRAME_PID and \ref MTB_LIN_IOCTL_SET_FRAME_PID commands. */
typedef struct
{
    /* Used as the second parameter for l_ifc_ioctl() */
    /** The index of the frame in the array of \ref mtb_stc_lin_frame_t */
    l_u8 index;
    /** Protected Identifier of the frame with the specified index. */
    l_u8 pid;
} mtb_stc_lin_pid_t;


#endif /* MTB_LIN_TYPES_H */

/* [] END OF FILE */
